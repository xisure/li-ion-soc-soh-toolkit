/*********************************************************************************
 * File Name          : bms_cal.h
 * Description        : 电池 SOC / SOH 算法库 —— 标定表管理 (多温度 + 温度插值)
 *
 * 解决什么问题:
 *   电池模型的四个参数 (OCV / R0 / R1 / 时间常数) 都随温度变。低温内阻成倍
 *   抬高、OCV 曲线整体下移, 用 25C 的表去算 5C 的电池, 端压预测会系统性
 *   偏高一两百毫伏, EKF 的新息长期为负, SOC 被拖着走偏。
 *   本模块把"一张表"升级成"三个温度点各一张", 运行时按当前温度线性插值
 *   出一张"当前温度的表"给算法用。
 *
 * 设计取舍 (三条):
 *
 *  1) 插值只在标定的温度点之间做, 端点平延不外推。
 *     三个点撑不起二次拟合; 而 5C 以下 / 45C 以上没有数据, 外推出的数没有
 *     物理依据, 宁可平延。
 *
 *  2) 老化量与温度量必须解耦, 否则温度一变就把 SOH 学到的东西冲掉。
 *     本模块只管"出厂基准随温度怎么变"; SOH 在线学到的老化量由 soc.c 用
 *     增量 (delta) 单独保存: 活跃 R0 = 本模块给的基准 + delta。
 *     温度变化只改基准, delta 不动 —— 学习成果自动跟着温度走。
 *     (若让 SOH 直接改写"表", 那每次温度刷新都会把学习值抹回出厂值。)
 *
 *  3) 温度不变就不重算。温度传感器读数总有噪声, 每帧重算既浪费 CPU 又会让
 *     R0 基准抖动。用 BMS_CAL_TEMP_HYST_DC (默认 2C) 做滞回。
 *
 * ---------------------------------------------------------------------
 * 现在只有 25C 的实测数据。5C / 45C 在 bms_config.h 里直接引用 25C 的值
 * 占位, 所以三个点相同 -> 插值结果恒等于 25C 的值 -> **行为与加温度维度
 * 之前完全一致**。拿到实测后改 bms_config.h, 或运行时用 BMS_CAL_SetPoint()
 * 写入 (串口改表的接口已留好: SetPoint/GetPoint + Serialize/Deserialize)。
 * ---------------------------------------------------------------------
 *
 * 调用方式 (main.c 主循环, 与 SOC_AhUpdate 平级, 数据流保持显式):
 *     BMS_CAL_Update(d.temp_dc);     <- 温度变了就刷新表
 *     SOC_AhUpdate(d.bus_mv, d.cur_ma);
 *     SOH_Update(d.bus_mv, d.cur_ma, d.temp_dc);
 *
 * 源码编码 UTF-8, 换行 LF。
 *
 * 许可: MIT (见仓库根 LICENSE)。Copyright (c) 2026 锂电池 SOC/SOH 算法库 贡献者。
 *******************************************************************************/

#ifndef __BMS_CAL_H
#define __BMS_CAL_H

#include <stdint.h>
#include "bms_config.h"

/* 表编号 (BMS_CAL_* ), 传给 GetBase / GetPoint / SetPoint */
#define BMS_CAL_OCV         0u      /* 开路电压 (mV) */
#define BMS_CAL_R0          1u      /* 欧姆内阻 (mΩ) */
#define BMS_CAL_R1          2u      /* 极化内阻 (mΩ) */
#define BMS_CAL_TAU         3u      /* 极化时间常数 (s) */
#define BMS_CAL_NTBL        4u

/* 每张表的点数: SOC = i * 10%, 0~100% 共 11 点 */
#define BMS_CAL_NSOC        11u

/* 一张表一个温度点的字节数 (序列化用) */
#define BMS_CAL_BYTES       (BMS_CAL_TEMP_N * BMS_CAL_NTBL * BMS_CAL_NSOC * 2u)

/* =====================================================================
 * 初始化 / 温度刷新
 * ===================================================================== */

/*********************************************************************
 * @fn      BMS_CAL_Init
 *
 * @brief   上电初始化: 用默认温度 (中间那个标定点, 即 25C) 刷新一次活跃表
 *
 * @note    必须在任何 SOC_* 之前调用 (soc.c 的查表依赖这里的活跃表)。
 *          不依赖时间戳, 可以放在最前面。
 */
void BMS_CAL_Init(void);

/*********************************************************************
 * @fn      BMS_CAL_Update
 *
 * @brief   按当前温度刷新活跃表; 温度变化不到阈值就直接返回
 *
 * @param   temp_dc   电芯温度, 0.1 摄氏度 (25.0C -> 250)
 *
 * @note    每帧调用即可, 内部有滞回 (BMS_CAL_TEMP_HYST_DC), 不会每帧重算。
 *          放在主循环周期任务里、SOC_AhUpdate 之前。
 */
void BMS_CAL_Update(int16_t temp_dc);

/*********************************************************************
 * @fn      BMS_CAL_SetTempDc
 *
 * @brief   强制把活跃表刷新到指定温度 (无视滞回)
 *
 * @note    给"上电导入落盘数据"用: 落盘时的温度往往和当前温度不同,
 *          必须先把基准设回落盘时的温度, 再导入那时学到的 R0,
 *          算出来的老化增量才是干净的 (否则会混进温度差)。
 */
void BMS_CAL_SetTempDc(int16_t temp_dc);

/*********************************************************************
 * @fn      BMS_CAL_GetTempDc
 *
 * @brief   当前活跃表对应的温度 (0.1 摄氏度)
 */
int16_t BMS_CAL_GetTempDc(void);

/*********************************************************************
 * @fn      BMS_CAL_GetTempAt
 *
 * @brief   第 ti 个标定点本身的温度 (0.1 摄氏度), ti 越界返回 0
 *
 * @note    GetTempDc 给的是"当前插值出来的温度", 本函数给的是"这张表
 *          是在哪几个温度下标的"。上位机下发整包标定表前必须先知道这个,
 *          否则不知道第 ti 块数据该对应多少度。
 */
int16_t BMS_CAL_GetTempAt(uint8_t ti);

/* =====================================================================
 * 读 —— 算法 (soc.c) 用
 * ===================================================================== */

/*********************************************************************
 * @fn      BMS_CAL_GetBase
 *
 * @brief   取当前温度下、某张表、第 idx 个 SOC 点的基准值
 *
 * @param   tbl   BMS_CAL_OCV / R0 / R1 / TAU
 * @param   idx   0~10, 对应 SOC = idx*10%
 *
 * @return  基准值 (mV 或 mΩ 或 s, 取决于表); 参数越界返回 0
 *
 * @note    这是"出厂值", 不含老化量。R0 的活跃值 = 本值 + soc.c 里的 delta。
 */
uint16_t BMS_CAL_GetBase(uint8_t tbl, uint8_t idx);

/* =====================================================================
 * 工况折算系数 —— 容量与 SOC 的温度 / 倍率修正 (bms_config.h [10])
 *
 * 两个系数都归一化到"参考工况" (BMS_REF_TEMP_DC / BMS_REF_CUR_MA):
 *   1.0 = 处于参考工况, 不需要折算。
 * 可用容量  Q_eff = Q_ref × f_T × f_I
 * 反折算    Q_ref = ΔQ / (ΔSOC × f_T × f_I)
 *
 * 默认参数 (SOC_CAP_TEMP_PCT 全 100 / SOC_PEUKERT_K = 1.0) 下两者恒等于 1.0,
 * 调用方算出来的结果与不折算**逐位相同** —— 所以这两条路径可以一直挂着。
 * ===================================================================== */

/*********************************************************************
 * @fn      BMS_CAL_TempFactorF
 *
 * @brief   容量温度系数 f_T: 当前温度下的可用容量占参考温度容量的比例
 *
 * @param   temp_dc   电芯温度 (0.1 摄氏度)
 *
 * @return  f_T (1.0 = 参考温度)
 *
 * @note    按 BMS_CAL_TEMP_DC 三点对 SOC_CAP_TEMP_PCT 线性插值,
 *          超出范围端点平延 (与标定表同一套取舍: 不外推)。
 *          查表用 lo 前推法, 与 refresh() 里那段同一个写法与同一个坑
 *          (见 bms_cal.c 的注释)。
 */
float    BMS_CAL_TempFactorF(int16_t temp_dc);

/*********************************************************************
 * @fn      BMS_CAL_RateFactorF
 *
 * @brief   倍率系数 f_I: Peukert 修正, f_I = (|I|/I_ref)^(1-k)
 *
 * @param   cur_ma   电流 (mA, 放电为正; 只取绝对值)
 *
 * @return  f_I (1.0 = 参考倍率 / 电流过小 / k = 1)
 *
 * @note    两处短路: k == 1.0 (未标定) 直接返回 1.0; |I| < SOC_RATE_MIN_CUR_MA
 *          (静置 / 漏电流) 也返回 1.0 —— 小电流不产生倍率效应。
 *          电流比值被钳到 [SOC_RATE_RATIO_LO, SOC_RATE_RATIO_HI]。
 *          内部用自带级数算 ln / exp, 不用 libm, 结果跨平台一致。
 */
float    BMS_CAL_RateFactorF(int32_t cur_ma);

/*********************************************************************
 * @fn      BMS_CAL_GetCapTempPct
 *
 * @brief   第 ti 个温度点的容量温度系数 (%) 原值, 越界返回 0
 *
 * @note    给上位机看"这条曲线是怎么定的"; 算法用 BMS_CAL_TempFactorF。
 */
uint16_t BMS_CAL_GetCapTempPct(uint8_t ti);

/*********************************************************************
 * @fn      BMS_CAL_GetRefTempDc / BMS_CAL_GetRefCurMa
 *
 * @brief   参考工况 (与 bms_config.h [10] 一致), 供上位机显示与自检
 */
int16_t  BMS_CAL_GetRefTempDc(void);
int32_t  BMS_CAL_GetRefCurMa(void);

/* =====================================================================
 * 读写标定点 —— 串口改表 / 标工具用 (本轮只留接口, 串口协议后续再加)
 * ===================================================================== */

/*********************************************************************
 * @fn      BMS_CAL_GetPoint / BMS_CAL_SetPoint
 *
 * @brief   直接读写"某个温度点、某张表、某个 SOC 点"的标定值
 *
 * @param   ti    温度点序号 0 ~ BMS_CAL_TEMP_N-1 (0=5C, 1=25C, 2=45C)
 * @param   tbl   BMS_CAL_OCV / R0 / R1 / TAU
 * @param   idx   SOC 点序号 0~10
 *
 * @note    SetPoint 要在运行时改标定基准, 所以本模块的基准表不能放 Flash
 *          (不能是 const), 只能放 RAM —— 见 bms_cal.c 的实现取舍第 1 条。
 *          改完会置脏标记并立刻重算当前温度的活跃表。
 *
 *          越界调用直接返回, 不写任何东西 (串口下发的脏数据打不死固件)。
 */
uint16_t BMS_CAL_GetPoint(uint8_t ti, uint8_t tbl, uint8_t idx);
void     BMS_CAL_SetPoint(uint8_t ti, uint8_t tbl, uint8_t idx, uint16_t v);

/*********************************************************************
 * @fn      BMS_CAL_LoadDefault
 *
 * @brief   把所有标定点恢复成 bms_config.h 里的出厂值
 */
void BMS_CAL_LoadDefault(void);

/*********************************************************************
 * @fn      BMS_CAL_IsDirty / BMS_CAL_ClearDirty
 *
 * @brief   标定表被改过 (供后续落盘用)
 */
uint8_t BMS_CAL_IsDirty(void);
void    BMS_CAL_ClearDirty(void);

/*********************************************************************
 * @fn      BMS_CAL_Serialize / BMS_CAL_Deserialize
 *
 * @brief   全部标定点 <-> 小端字节流 (供后续串口收发 / 落盘用)
 *
 * @param   buf   缓冲区 (Serialize 至少 BMS_CAL_BYTES 字节)
 * @param   cap   buf 容量
 * @param   len   Deserialize 的字节数
 *
 * @return  Serialize: 写入的字节数 (容量不够返回 0)
 *          Deserialize: 1 = 成功, 0 = 失败 (长度不符)
 *
 * @note    布局: 温度点优先 -> 表 -> SOC 点, 每点 2 字节小端
 *          [ti][tbl][idx], 共 3 x 4 x 11 x 2 = 264 字节。
 *          PC 端: struct.unpack('<264H', buf) 后按 ti*44 + tbl*11 + idx 取。
 */
uint16_t BMS_CAL_Serialize(uint8_t *buf, uint16_t cap);
uint8_t  BMS_CAL_Deserialize(const uint8_t *buf, uint16_t len);

#endif /* __BMS_CAL_H */
