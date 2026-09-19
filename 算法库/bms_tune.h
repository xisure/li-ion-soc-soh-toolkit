/*********************************************************************************
 * File Name          : bms_tune.h
 * Description        : 电池 SOC / SOH 算法库 —— 在线调参协议层 (通道无关)
 *
 * 解决什么问题:
 *   标定表 / 容量 / R0 / SOH 学习结果 这些参数原先只能改 bms_config.h 重新
 *   烧录。做实验时一天要烧十几次, 而且不同电芯的 OCV 表没法现场切换。
 *   本模块把它们做成"运行时可读写", 只要有个能收发字节的通道 (串口/USB/
 *   CAN/蓝牙) 就能改, 不用重新烧程序。
 *
 * 为什么库里没有 UART 相关的一行代码:
 *   库是平台无关的。一旦 #include 了某个厂家的 UART 头文件, 这个库就只能
 *   在那一家上用。所以本模块只定义"字节流怎么成帧、成帧后调哪个接口",
 *   收发字节这件事交给工程侧:
 *
 *       中断里:  只往环形缓冲塞字节, 绝不调本模块的任何函数
 *       主循环:  while(环形缓冲非空) BMS_TUNE_Feed(取一个字节);
 *       发送:    BMS_TUNE_SetTx(你的发一字节函数);
 *
 *   **中断里不能调 BMS_TUNE_Feed**: 主循环的 SOC_AhUpdate() 正在读标定表,
 *   中断里改表会读到"一半新一半旧"。同理 BMS_NVM_Save() 也不能进中断
 *   (擦页要关中断 10~20ms)。
 *
 * ---------------------------------------------------------------------
 * 帧格式 (上行下行同构)
 *
 *   偏移  长度  字段
 *   0     1     SOF0   0xAA
 *   1     1     SOF1   0x55
 *   2     2     LEN    payload 字节数 (u16 小端)
 *   4     1     CMD    命令字
 *   5     LEN   PAYLOAD
 *   5+LEN 2     CRC16  u16 小端, 覆盖偏移 2 .. 4+LEN (即 LEN+CMD+PAYLOAD)
 *
 *   帧总长 = 7 + LEN
 *
 *   CRC16-CCITT: poly 0x1021, init 0xFFFF, 输入/输出均不取反。
 *   与 bms_nvm.c 落盘用的 CRC **同一套参数**, 所以上位机只需一份 CRC 实现。
 *   (实现上各写了一份 static 函数 —— 为 20 行代码多拉一个文件不划算,
 *    但两边参数必须一致, 改一处要改两处。)
 *
 *   为什么 LEN 用 u16 而不是 u8: 标定表整包是 3 x 4 x 11 x 2 = 264 字节,
 *   超过 255, u8 装不下。用 u16 也顺便让将来加温度点不用改帧格式。
 *
 *   应答帧: 同样的格式, CMD 改成 (CMD | 0x80)。
 *   **所有应答的 payload 第 0 字节都是状态码 rc** (读命令 rc=0 时后面跟数据)。
 *
 *   PC 端拼帧 (Python):
 *     import struct
 *     def frame(cmd, pay=b''):
 *         body = struct.pack('<HB', len(pay), cmd) + pay
 *         return b'\xAA\x55' + body + struct.pack('<H', crc16(body))
 *
 * ---------------------------------------------------------------------
 * 安全须知 (重要, 别跳过)
 *
 *   本模块**不做任何鉴权**。谁能往 BMS_TUNE_Feed() 里喂字节, 谁就能改
 *   标定表、改容量、清 SOH 学习结果 —— 改错了不会让固件崩溃 (所有越界
 *   都被挡住), 但会让 SOC 算错。是否开放、什么时候开放由工程侧决定,
 *   典型做法: 上电后 N 秒内允许调参 / 检测到某个握手序列才允许 / 物理
 *   按键按下时才把串口数据接到本模块上。
 * ---------------------------------------------------------------------
 *
 * 源码编码 UTF-8, 换行 LF。
 *
 * 许可: MIT (见仓库根 LICENSE)。Copyright (c) 2026 锂电池 SOC/SOH 算法库 贡献者。
 *******************************************************************************/

#ifndef __BMS_TUNE_H
#define __BMS_TUNE_H

#include <stdint.h>
#include "bms_config.h"
#include "bms_cal.h"      /* BMS_CAL_BYTES / BMS_CAL_NTBL / BMS_CAL_NSOC / BMS_CAL_TEMP_N */

/* 协议版本: 帧格式或命令语义变了就 +1, 上位机靠它判断能不能对话。
 * ※ bms_config.h 里还有一处同名 #ifndef 兜底, 两者必须同值 (见该处注释)。 */
#ifndef BMS_TUNE_PROTO_VER
#define BMS_TUNE_PROTO_VER      2u
#endif

/* 接收缓冲能装的最大 payload。
 * 默认跟随标定表整包大小 (264B) 再加 8B 余量; RAM 极紧时可以调小, 代价是
 * 超过这个长度的命令会被判 "长度不符" 并丢弃 (单点读写命令不受影响)。 */
#ifndef BMS_TUNE_RX_MAX
#define BMS_TUNE_RX_MAX         (BMS_CAL_BYTES + 8u)
#endif

/* 帧定界字节 */
#define BMS_TUNE_SOF0           0xAAu
#define BMS_TUNE_SOF1           0x55u
#define BMS_TUNE_ACK_FLAG       0x80u   /* 应答帧 = CMD | 本值 */

/* =====================================================================
 * 命令字
 *
 * 下面注释里 "上:" 后面的字节数 = **应答 payload 总字节数 (含 rc)**,
 * 与《在线调参协议.md》§6.0 的"上行"列同一个口径。
 * ===================================================================== */

/* ---- 链路 / 自检 ---- */
#define BMS_TUNE_CMD_PING           0x00u   /* 下: 空            上: rc + 2B = 3B */
#define BMS_TUNE_CMD_INFO           0x01u   /* 下: 空            上: rc + 15B = 16B */

/* ---- 标定表 (出厂基准, 不含老化量) ---- */
#define BMS_TUNE_CMD_RD_CAL_POINT   0x02u   /* 下: ti,tbl,idx    上: rc + 2B = 3B */
#define BMS_TUNE_CMD_WR_CAL_POINT   0x03u   /* 下: ti,tbl,idx,v  上: rc (1B) */
#define BMS_TUNE_CMD_RD_CAL_ALL     0x04u   /* 下: 空            上: rc + 264B = 265B */
#define BMS_TUNE_CMD_WR_CAL_ALL     0x05u   /* 下: 264B 整包     上: rc */
#define BMS_TUNE_CMD_CAL_RESTORE    0x06u   /* 下: 空            上: rc  (恢复出厂) */
#define BMS_TUNE_CMD_RD_CAL_TEMPS   0x07u   /* 下: 空            上: rc + NTEMP x i16 */
#define BMS_TUNE_CMD_RD_TEMP        0x08u   /* 下: 空            上: rc + temp_dc(i16) */
#define BMS_TUNE_CMD_WR_TEMP        0x09u   /* 下: temp_dc(i16)  上: rc  (强制切温度) */
#define BMS_TUNE_CMD_RD_CAL_ACTIVE  0x0Au   /* 下: 空            上: rc + 4x11x u16 = 89B */

/* ---- R0 活跃值 (出厂基准 + 老化增量, 算法真正用的那张) ---- */
#define BMS_TUNE_CMD_RD_R0          0x10u   /* 下: idx           上: rc + v(u16) */
#define BMS_TUNE_CMD_WR_R0          0x11u   /* 下: idx,v(u16)    上: rc */
#define BMS_TUNE_CMD_RD_R0_ALL      0x12u   /* 下: 空            上: rc + 22B = 23B */
#define BMS_TUNE_CMD_WR_R0_ALL      0x13u   /* 下: 22B           上: rc */

/* ---- 容量 ---- */
#define BMS_TUNE_CMD_RD_CAP         0x14u   /* 下: 空            上: rc + mAh(u32) */
#define BMS_TUNE_CMD_WR_CAP         0x15u   /* 下: mAh(u32)      上: rc */

/* ---- SOC 运行态与 EKF 参数 (在线调参的主力) ----
 * EKF 参数 8 个整组读写, 顺序 = soc_ekf_param_t (soc.h):
 *   q_soc / q_vrc / r_v / p0_soc / p0_vrc / res_max_mv / s_min / p_min
 *
 * **不落盘**: 与标定表同理, 权威源在 PC (tools/test/kalman_tune.py) 和
 * bms_config.h —— 掉电回默认值。定下来就把值写进 bms_config.h 重编。
 *
 * WR_SOC 是"人工把一个已知 SOC 对齐进去", 只重建积分基准 + 直接置 EKF 的
 * SOC (不按端压查表, 带载时端压查表值系统性偏低)。SOH 学习不受影响 ——
 * 容量学习的两个 OCV 锚点记的是 SOC_VoltageToSoc01(静置端压), 与估计器的
 * SOC 无关, 所以不需要作废进行中的静置窗口。 */
#define BMS_TUNE_CMD_RD_SOC         0x16u   /* 下: 空            上: rc + 21B = 22B */
#define BMS_TUNE_CMD_WR_SOC         0x17u   /* 下: soc01(u16)    上: rc  (强制校准) */
#define BMS_TUNE_CMD_RD_EKF         0x18u   /* 下: 空            上: rc + 8xf32 = 33B */
#define BMS_TUNE_CMD_WR_EKF         0x19u   /* 下: 8 x f32 (32B) 上: rc  (一个越界整组不写) */
#define BMS_TUNE_CMD_EKF_RESET      0x1Au   /* 下: 空            上: rc  (重置 P, SOC 不动) */

/* ---- SOH 学习结果 ---- */
#define BMS_TUNE_CMD_RD_SOH_BLOB    0x20u   /* 下: 空            上: rc + 120B 参数块 = 121B */
#define BMS_TUNE_CMD_WR_SOH_BLOB    0x21u   /* 下: 120B 参数块   上: rc */
#define BMS_TUNE_CMD_RD_SOH_SUM     0x22u   /* 下: 空            上: rc + 14B = 15B 摘要 */
#define BMS_TUNE_CMD_SOH_RESET      0x23u   /* 下: 空            上: rc  (清学习值) */
#define BMS_TUNE_CMD_RD_SOH_CNT     0x24u   /* 下: 空            上: rc + 15B = 16B 累计与循环 */
#define BMS_TUNE_CMD_SOH_CNT_RESET  0x25u   /* 下: 空            上: rc  (只清计数) */
#define BMS_TUNE_CMD_RD_R0_CHG_ALL  0x26u   /* 下: 空            上: rc + 22B = 23B 充方向 R0 */
#define BMS_TUNE_CMD_WR_R0_CHG_ALL  0x27u   /* 下: 22B           上: rc  (充方向 R0 整表) */
#define BMS_TUNE_CMD_RD_CORR        0x28u   /* 下: 空            上: rc + 14B = 15B 工况折算 */
/*
 * 上面 0x24~0x28 五条是为"温度修正 / 倍率修正 / 循环次数 / 累计充放电 /
 * 充电方向 R0"这五项功能配的, 逐字节布局见《在线调参协议.md》§6。
 *
 * 两条容易踩的:
 *   RD_SOH_CNT 的 5 个字段都是"从学到现在"的累计量, 不是瞬时值 ——
 *     半循环计数 (摆幅法) 与等效满循环是两个互补口径 (见 bms_config.h [10]),
 *     别把哪个当唯一真值。
 *   SOH_CNT_RESET 只清计数, **不动**学到的容量与 R0; 要连学习值一起清
 *     应该用 SOH_RESET (0x23)。换电芯时两个都要发。
 */

/* ---- 掉电保持 ---- */
#define BMS_TUNE_CMD_NVM_SAVE       0x30u   /* 下: 空            上: rc */
#define BMS_TUNE_CMD_NVM_INFO       0x31u   /* 下: 空            上: rc + 7B = 8B */

/* ---- 实时量 (调试用) ---- */
#define BMS_TUNE_CMD_RD_LIVE        0x40u   /* 下: 空            上: rc + 12B = 13B */

/* =====================================================================
 * 状态码 —— 所有应答的 payload[0]
 * ===================================================================== */
#define BMS_TUNE_OK             0u   /* 成功 */
#define BMS_TUNE_EBADARG        1u   /* 参数越界 (温度点/表号/SOC 点序号不对) */
#define BMS_TUNE_EBADLEN        2u   /* 下行 payload 长度不是该命令期望的值 */
#define BMS_TUNE_ENOSUP         3u   /* 该功能未编译进库 (如 BMS_USE_NVM=0 时的落盘) */

/* 下面两个只用于库内部统计, 不会作为 rc 发出去 */
#define BMS_TUNE_ECRC           4u   /* CRC 校验失败 (帧被丢弃, 不产生应答) */

/* =====================================================================
 * 外部接口 —— 只有 5 个函数
 * ===================================================================== */

/*********************************************************************
 * @fn      BMS_TUNE_SetTx
 *
 * @brief   注册"发送一个字节"的回调
 *
 * @param   tx   你的发送函数; 传 NULL 表示丢弃所有应答 (只写不读的场景)
 *
 * @note    本模块在 BMS_TUNE_Feed() 的调用上下文里同步调 tx(), 所以
 *          tx 里不要做太重的事 (不要 while 等发送完成中断标志等几百 ms)。
 *          通常就是: 查询发送寄存器空 -> 写入; 或者塞进发送环形缓冲。
 */
typedef void (*bms_tune_tx_fn)(uint8_t b);
void BMS_TUNE_SetTx(bms_tune_tx_fn tx);

/*********************************************************************
 * @fn      BMS_TUNE_Feed
 *
 * @brief   喂入一个收到的字节; 凑满一帧且 CRC 通过就执行并回应答
 *
 * @note    **只能在主循环 (非中断) 上下文调用**。
 *          内部是状态机, 帧同步靠 SOF0/SOF1: 连续两个字节不是 AA 55 就
 *          一直丢弃, 所以中途插垃圾数据 (比如 MCU 自己的打印) 不会卡死,
 *          最多是丢一帧。
 *
 *          一个字节的调用开销: 非 payload 阶段只有一次 switch; payload 阶段
 *          是"存一个字节 + 计数器自增"。整帧的 CRC 只在收完时算一次。
 */
void BMS_TUNE_Feed(uint8_t b);

/*********************************************************************
 * @fn      BMS_TUNE_Reset
 *
 * @brief   强制回到"等 SOF0"状态, 丢弃半截帧
 *
 * @note    一般不用调。帧同步本身能自恢复; 只有当你想主动打断 (比如
 *          检测到长时间没收到完整帧) 时才用。
 */
void BMS_TUNE_Reset(void);

/* ---- 诊断 ---- */
uint16_t BMS_TUNE_GetErrCount(void);    /* CRC 错 / 超长帧 累计 */
uint16_t BMS_TUNE_GetCmdCount(void);    /* 成功执行并应答的命令数 */

#endif /* __BMS_TUNE_H */
