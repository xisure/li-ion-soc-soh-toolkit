/*********************************************************************************
 * File Name          : bms_nvm.h
 * Description        : 电池 SOC / SOH 算法库 —— 掉电保持 (非易失存储) 接口
 *
 * 解决什么问题: SOH 在线学到的容量 Q / 11 格 R0 表 / 标定基线快照 / 样本计数
 * 全在 RAM 里, 掉电就回 100%。本模块把它们做成"装上就长期记住"。
 *
 * 这个头文件是平台无关的一半: 序列化 / CRC 校验 / 版本 / 磨损均衡 / 落盘去抖
 * 都在 bms_nvm.c 里; 介质相关的"擦 + 写"只有 3 个函数, 见下面的移植层。
 *
 * ---------------------------------------------------------------------
 * 介质选型 (优先顺序, 对应"有外部就用外部"):
 *   1) 外部 EEPROM / FRAM (I2C / SPI) —— 首选
 *      字节写、不需要擦除、不占程序 Flash、寿命 10 万次(EEPROM)~无限(FRAM)
 *   2) STM32 的 VBAT 备份寄存器 —— 写次数无限、无需擦除
 *      但容量小 (F4 有 20 x 32b = 80B, 刚好放一份); 需 VBAT 供电才保持
 *   3) 内部 Flash 末尾页 —— 通用兜底
 *      必须靠多槽轮转 + 去抖保寿命 (见 bms_nvm.c 的磨损均衡说明)
 * 大扇区 MCU (STM32F4/F7/H7, 最小扇区 16~128KB) 用内部 Flash 存 80B 代价
 * 太高 (整扇区报废), 强烈建议走 1)。
 *
 * 换介质只需换一个 移植示例/port_nvm_*.c 文件, 算法库一行不用改。
 * ---------------------------------------------------------------------
 *
 * 移植层需要实现 (3 个函数, 全部返回 1 = 成功 / 0 = 失败):
 *   BMS_NVM_SlotCount()               有几个独立擦除单元可用 (>= 1)
 *   BMS_NVM_SlotRead (slot, buf, len) 读一个槽
 *   BMS_NVM_SlotWrite(slot, buf, len) 擦除 + 写入一个槽
 * 现成示例: 移植示例/port_nvm_stm32f1.c / _stm32f4.c / _stm32g0.c /
 *           _ch32x035.c / _i2c_eeprom.c
 *
 * 应用代码需要做的 (2 行):
 *   上电初始化处:  BMS_NVM_Init();     <- 必须在 SOH_Init() 之后
 *   主循环里:      BMS_NVM_Task();
 *
 * 源码编码 UTF-8, 换行 LF。
 *
 * 许可: MIT (见仓库根 LICENSE)。Copyright (c) 2026 锂电池 SOC/SOH 算法库 贡献者。
 *******************************************************************************/

#ifndef __BMS_NVM_H
#define __BMS_NVM_H

#include <stdint.h>
#include "bms_config.h"
#include "soh.h"        /* soh_param_t */

/* =====================================================================
 * 落盘块格式 —— 显式小端字节流, 与编译器/字节序无关
 *
 * 为什么不用 memcpy(结构体): 结构体的 padding 与字段偏移随编译器、
 * 目标位宽、字节序变化 (16 位 MCU 的 u32 对齐可能是 2, 大端 MCU 整串反)。
 * 所以这里规定"第几字节是什么", 任何平台写出来的 132 字节逐字节相同。
 *
 * 头部 12B (BMS_NVM_HDR_LEN):
 *   偏移  长度  字段
 *   0     4     magic  'BNM1' (固定字节序: 0x42 0x4E 0x4D 0x31)
 *   4     2     ver    格式版本 (小端 u16) —— 结构改了必须 +1
 *   6     2     seq    落盘序号 (小端 u16) —— 单调递增, 用于判新旧
 *   8     2     crc    CRC16-CCITT (小端 u16), 覆盖 payload 的 len 字节
 *   10    2     len    payload 有效长度 (小端 u16)
 *
 * 载荷 120B (BMS_NVM_PAYLOAD_LEN, 布局与 soh_param_t 一一对应):
 *   0     4     cap_mah        学到的容量 (mAh, u32, 参考工况)
 *   4     22    r0[11]         学到的 R0 表 (mΩ, 11 x u16, **放电方向**)
 *   26    22    base[11]       R0 标定基线快照 (SOH_R 的分母, 两方向共用)
 *   48    11    cnt[11]        放电方向各格有效样本数 (11 x u8)
 *   59    1     q_n            容量学习次数 (u8)
 *   60    1     r0_any         放电方向 R0 已有有效样本 (u8)
 *   61    1     q_any          容量已学到过 (u8)
 *   62    2     temp_dc        落盘时的电芯温度 (0.1C, 有符号)
 *   64    4     kf_p           卡尔曼协方差 (IEEE-754 float32, 小端)
 *   ---- 以上 68B 与 ver 2 逐字节相同 (老工具只读前 68B 也能用) ----
 *   68    22    r0_chg[11]     学到的 R0 表 (mΩ, 11 x u16, **充电方向**)
 *   90    11    cnt_chg[11]    充电方向各格有效样本数 (11 x u8)
 *   101   1     r0_chg_any     充电方向 R0 已有有效样本 (u8)
 *   102   1     cnt_any        计数字段有效 (至少累计过一次充放, u8)
 *   103   1     pad0           对齐填充, 写 0
 *   104   4     cum_chg_mah    累计充入电量 (mAh, u32)
 *   108   4     cum_dis_mah    累计放出电量 (mAh, u32)
 *   112   2     half_cycle     半循环计数 (摆幅法, u16)
 *   114   2     pad1           预留, 写 0
 *   116   4     pad2           预留, 写 0
 *
 *   版本沿革:
 *     ver 1 -> 2: 62..63 从"保留, 写 0"的对齐填充改成 temp_dc。长度不变,
 *                 所以 PAYLOAD_LEN 仍是 68。
 *     ver 2 -> 3: 追加充电方向 R0 (r0_chg / cnt_chg / r0_chg_any) 与累计
 *                 充放电 (cum_chg_mah / cum_dis_mah / half_cycle / cnt_any)。
 *                 **前 68 字节一个都没动** —— PC 侧"68B 参数块"那套老流程
 *                 (bms_gui 参数块面板 / bms_tune_proto.SOH_BLOB_BYTES)
 *                 照旧可用, 只是看不到新字段。
 *   版本号不符的数据一律判无效并回退标称值, 不会读串 (见 nvm_block_ok)。
 *
 * PC 端解析 (Python, 兼容 ver 2 与 ver 3):
 *   FMT68 = '<IHHHH' + 'I 11H 11H 11B B B B h f'      # 12B 头 + 前 68B 载荷
 *   FMT3X = '11H 11B B B x I I H H I'                  # ver 3 追加的 52B
 *   magic, ver, seq, crc, ln, cap = d[0:6] ...
 *   ('2x' 改成 'h': 62..63 现在是有意义的温度, 不能再跳过)
 *
 * 选 120 而不是 struct 的 sizeof: 就是上面的固定值, 换平台不会变。
 * (soh_param_t 里那几个 pad 字段就是为了让 sizeof 也正好 120, 见 soh.h)
 * ===================================================================== */
#define BMS_NVM_MAGIC_B0       0x42u   /* 'B' */
#define BMS_NVM_MAGIC_B1       0x4Eu   /* 'N' */
#define BMS_NVM_MAGIC_B2       0x4Du   /* 'M' */
#define BMS_NVM_MAGIC_B3       0x31u   /* '1' */

#define BMS_NVM_HDR_LEN        12u
#define BMS_NVM_PAYLOAD_LEN    120u
#define BMS_NVM_BLOCK_LEN      (BMS_NVM_HDR_LEN + BMS_NVM_PAYLOAD_LEN)  /* = 132 */

/* 编译期确认 soh_param_t 的尺寸假设成立 (字段偏移见 soh.h)。
 * 表达式是整型常量, 不触发 -Wvariably-modified。 */
typedef char bms_nvm_param_size_check_t[(sizeof(soh_param_t) == BMS_NVM_PAYLOAD_LEN) ? 1 : -1];

/* =====================================================================
 * 移植层 —— 下面 3 个函数由目标平台的 port_nvm_*.c 实现
 * ===================================================================== */

/*********************************************************************
 * @fn      BMS_NVM_SlotCount
 *
 * @brief   可用槽数。1 = 单槽 (无磨损均衡); >=2 自动启用轮转 + 乒乓
 *
 * @note    槽 = 一个独立擦除单元 (Flash 页 / 扇区, 或 EEPROM 里的一段)。
 *          同一擦除单元不能拆成两个槽, 否则擦一个会连带毁掉另一个。
 *          返回 0 视为持久化功能不可用, 库内部退化为"不上电不落盘"。
 *
 *          寿命 = 单页擦写次数 x 槽数。所以内部 Flash 上给到 2~4 个槽
 *          是"尽量保证寿命"的主要手段 (见 README 持久化章节)。
 */
uint8_t BMS_NVM_SlotCount(void);

/*********************************************************************
 * @fn      BMS_NVM_SlotRead
 *
 * @brief   读一个槽的 len 字节到 buf
 *
 * @param   slot  槽号, 0 ~ BMS_NVM_SlotCount()-1
 * @param   buf   目标缓冲 (调用方提供, 至少 len 字节)
 * @param   len   字节数, 库内固定传 BMS_NVM_BLOCK_LEN (132)
 *
 * @return  1 = 成功, 0 = 失败 (调用方视为"该槽无有效数据", 不是致命错误)
 *
 * @note    读操作不该有副作用 (不能顺手擦掉什么)。空白槽返回的应是
 *          全 0xFF 或全 0x00, 两种都会被上层的 magic/CRC 判为无效。
 */
uint8_t BMS_NVM_SlotRead(uint8_t slot, void *buf, uint32_t len);

/*********************************************************************
 * @fn      BMS_NVM_SlotWrite
 *
 * @brief   擦除并写入一个槽 (整块 len 字节)
 *
 * @param   slot  槽号
 * @param   buf   源数据, 至少 len 字节
 * @param   len   字节数, 固定 BMS_NVM_BLOCK_LEN (132)
 *
 * @return  1 = 成功, 0 = 失败
 *
 * @note    这一层要吸收的差异 (实现时逐个确认):
 *          - 擦除粒度: Flash 只能整页/整扇区擦 (1 字节也擦一整块),
 *            EEPROM / FRAM / 备份寄存器则不需要擦
 *          - 写粒度与对齐: STM32F1 只能半字(2B)写、G0/L4 要求双字(8B)对齐、
 *            CH32X035 4B、F4 可以字节写
 *          - 写前必须擦: Flash 只能 1->0, 不擦就写会得到旧值与新值的"与"
 *          - 写缓冲: 内部 Flash 需要一份 RAM 缓冲把不足一页的部分填 0xFF
 *            (库给的是 80B, 页大小由 port 自己补), 建议 static 不占栈
 *          - 擦写期间能否取指: CH32X035 / RP2040 等擦写时 CPU 读不到 Flash,
 *            必须关中断, 且只能在主循环上下文调用 (不能在中断里)
 *          - 失败必须返回 0: 上层靠返回值决定要不要清脏标记, 别吞掉错误
 */
uint8_t BMS_NVM_SlotWrite(uint8_t slot, const void *buf, uint32_t len);

/* =====================================================================
 * 平台无关接口 —— 应用代码调用
 * ===================================================================== */

/*********************************************************************
 * @fn      BMS_NVM_Init
 *
 * @brief   上电初始化一次: 扫描所有槽, 取最新且校验通过的一份导入
 *
 * @return  1 = 成功读回并导入; 0 = 没有有效数据 (保持标称值)
 *
 * @note    **必须在 SOH_Init() 之后调用**。SOH_Init() 会用当前 R0 表
 *          快照标定基线, 早于它导入会让基线变成"学过的值", SOH_R 恒 100%。
 *          本函数不依赖时间戳, 所以可以先于主循环。
 *          返回 0 不是错误 —— 新板子第一次上电就是这样。
 */
uint8_t BMS_NVM_Init(void);

/*********************************************************************
 * @fn      BMS_NVM_Save
 *
 * @brief   立即落盘一次 (一般不需要手动调用, 交给 BMS_NVM_Task)
 *
 * @return  1 = 成功; 0 = 失败或无需写 (内容与上次落盘完全相同时跳过)
 *
 * @note    内部按槽轮转选下一个槽, 所以连续调用会均匀磨损所有槽。
 *          只能在主循环 / 非中断上下文调用。
 */
uint8_t BMS_NVM_Save(void);

/*********************************************************************
 * @fn      BMS_NVM_Task
 *
 * @brief   主循环周期调用: 有未落盘的学习结果且距上次落盘超过
 *          BMS_NVM_DEBOUNCE_MS 时写一次
 *
 * @note    去抖的意义: 一次放电里 SOH 可能学习多次 (每格 R0 / 每次容量),
 *          不去抖就会连续擦写同一批槽。60s 去抖把一轮放电合并成一次写。
 *
 *          擦写是阻塞的 (内部 Flash 上可能还要关中断); 放在主循环的
 *          低频分支里, 不要放进高频采样路径。
 */
void BMS_NVM_Task(void);

/*********************************************************************
 * @fn      BMS_NVM_IsDirty / BMS_NVM_MarkDirty
 *
 * @brief   有无未落盘的学习结果
 *
 * @note    soh.c 学习成功后自动置脏, 应用一般不用管。MarkDirty 供外部
 *          改了参数 (比如通过串口下发新 OCV 表) 后手动置脏。
 */
uint8_t BMS_NVM_IsDirty(void);
void    BMS_NVM_MarkDirty(void);

/* =====================================================================
 * SOH 参数块序列化 —— 落盘与串口调参共用同一份字节布局
 *
 * 字节偏移见表头"载荷 120B"。bms_tune 的"读/写 SOH 参数块"命令直接调
 * 这两个函数, 不再自己写一份序列化 —— 两处各写一份必然会有一次忘了同步,
 * 而参数错位在板子上极难查。
 *
 * 只在 BMS_USE_NVM = 1 时存在 (整个 bms_nvm.c 受该开关门控), 所以
 * bms_tune 的那两条命令在关闭持久化时返回 BMS_TUNE_ENOSUP。这是有意的:
 * 字节布局归本模块管, 没道理为了"关掉持久化也能用"而破坏该性质。
 * ===================================================================== */
#if BMS_USE_NVM
uint16_t BMS_NVM_ParamToBytes(const soh_param_t *p, uint8_t *buf, uint16_t cap);
uint8_t  BMS_NVM_BytesToParam(soh_param_t *p, const uint8_t *buf, uint16_t len);
#endif

/* ---- 诊断 ---- */
uint32_t BMS_NVM_GetSaveCount(void);    /* 累计成功落盘次数 (只计写入, 不含上电读回) */
int8_t   BMS_NVM_GetActiveSlot(void);   /* 当前最新数据所在槽, -1 = 无 */
uint16_t BMS_NVM_GetSeq(void);          /* 当前数据的落盘序号 */

#endif /* __BMS_NVM_H */
