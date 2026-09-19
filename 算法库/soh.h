/*********************************************************************************
 * File Name          : soh.h
 * Description        : SOH (健康度) 在线学习 = 容量 Q 学习 + 欧姆内阻 R0 学习
 *
 * 与 ../tools/soh_learn 一一对应 (判据常量必须两边同步改!):
 *   R0: 电流阶跃 ΔV/ΔI 事件触发, 11 格 SOC 独立滑动更新 (自适应 α)。
 *       充 / 放方向**各维护一张表** (同一 SOC 点上两个方向的表观内阻不等,
 *       混在一格里做平滑会两个方向都不准): 方向在进入带载段时按电流符号定,
 *       融合进对应方向的表, EKF 按当前电流方向取用。
 *   Q : 静置 OCV 锚点法, Q = ΔQ / ΔSOC, 标量卡尔曼融合 (正向新息压上跳)。
 *       融合前先把实测容量**折回参考工况** (25C / 0.2C):
 *         Q_ref = Q_meas / (f_T × f_I)     (见 bms_config.h [10])
 *       —— 否则一次低温或大倍率试验会被当成"电池老化了"永久学进 SOH。
 *   计数: 累计充入 / 累计放出 (mAh) + 摆幅法半循环计数 + 等效满循环。
 *   SOH: SOH_Q = Q/Q_nom;
 *        SOH_R = (SOH_R_EOL_RATIO - R0_avg/R0_fresh)/(SOH_R_EOL_RATIO-1)*100
 *                (默认内阻翻倍 = EOL; 均值取 SOH_R_AVG_LO~SOH_R_AVG_HI 网格)
 *
 * 用法 (主循环, 在 SOC_AhUpdate 之后调用, 每 BMS_UPDATE_PERIOD_MS 一次):
 *   SOH_Init() 一次 (上电初始化); 此后每周期 SOH_Update(bus_mv, cur_ma, temp_dc)。
 *   传感器掉线帧不要调用。
 *   学到的新容量经 SOC_SetCapacityMAh 回写 soc.c; 新 R0 经 SOC_SetR0DirAt 落表
 *   (按方向), EKF 量测方程即时生效。soh.c 只依赖 soc.c 的公开接口, 无反向依赖。
 *
 * 学习判据常量 (SOH_* / SOC_*) 全部在 bms_config.h。
 * 移植方法见同目录 README.md。
 *
 * 源码编码 UTF-8, 换行 LF。
 *
 * 许可: MIT (见仓库根 LICENSE)。Copyright (c) 2026 锂电池 SOC/SOH 算法库 贡献者。
 *******************************************************************************/
#ifndef __SOH_H
#define __SOH_H

#include <stdint.h>
#include "bms_config.h"   /* 学习判据常量集中在这里 */

/*********************************************************************
 * @fn      SOH_Init
 *
 * @brief   上电初始化: 快照 R0 标定基线 (SOH_R 的分母) + 清学习状态
 *
 * @note    必须在 SOC_AhUpdate 第一次被调用之前调用 (至少要早于任何
 *          可能改写 R0 表的学习动作), 否则分母会被学习结果污染。
 *          也可以在 SOH 掉电参数恢复之后调用, 把恢复值当基线。
 */
void SOH_Init(void);

/*********************************************************************
 * @fn      SOH_Update
 *
 * @brief   每 BMS_UPDATE_PERIOD_MS 调用一次 (在 SOC_AhUpdate 之后)
 *
 * @param   bus_mv   电池电压 mV
 * @param   cur_ma   电流 mA, 放电为正
 * @param   temp_dc  温度 0.1摄氏度 (如 253 = 25.3°C); R0 学习只在
 *                   SOH_TEMP_LO_C ~ SOH_TEMP_HI_C 窗口内取样本
 *
 * @note    传感器掉线帧不要调用。
 */
void SOH_Update(int32_t bus_mv, int32_t cur_ma, int32_t temp_dc);

/* ---- 结果查询 ---- */
int32_t  SOH_GetPercent01(void);          /* 综合 SOH, 0.01% 单位 (0~10000) */
uint8_t  SOH_GetPercent(void);            /* 综合 SOH, 整数 % */
uint8_t  SOH_GetCapacityPercent(void);    /* SOH_Q = Q/Q_nom, 整数 % */
uint8_t  SOH_GetResistancePercent(void);  /* SOH_R, 整数 % */
uint32_t SOH_GetLearnedCapacityMAh(void); /* 当前(学到)的容量 mAh (参考工况) */
uint16_t SOH_GetR0AvgMOhm(void);          /* 放电方向 R0 网格均值 (mΩ, 默认 10~90%) */
uint8_t  SOH_IsCapacityValid(void);       /* 容量已至少学到 1 次 */
uint8_t  SOH_IsR0Valid(void);             /* 放电方向 R0 已有有效样本 (SOH_R 的门槛) */

/* ---- 充电方向 R0 (与放电表独立维护) ---- */
uint16_t SOH_GetR0ChgAvgMOhm(void);       /* 充电方向 R0 网格均值 (mΩ) */
uint8_t  SOH_IsR0ChgValid(void);          /* 充电方向 R0 已有有效样本 */
uint8_t  SOH_GetResistancePercentChg(void);
/*
 * 充电方向的 SOH_R。**分母与 SOH_R 共用同一张出厂基线** —— bms_cal 只有
 * 一张 R0 表 (出厂时没分方向), 所以刚出厂时它可能不是 100% 而是一个偏移值。
 * 它的用途是看**趋势**: 老化会让它往下走, 涨得比 SOH_R 快就说明充电方向的
 * 内阻恶化更快。绝对值不要与 SOH_R 直接比。
 */

/* ---- 计数量: 累计充放电与循环次数 (bms_config.h [10]) ---- */

/*********************************************************************
 * @fn      SOH_GetCumDischargeMAh / SOH_GetCumChargeMAh
 *
 * @brief   累计放出 / 充入的电量 (mAh)。只在真的在充放时累计 ——
 *          静置态的电流噪声 (±19mA) 不进长期统计。
 */
uint32_t SOH_GetCumDischargeMAh(void);
uint32_t SOH_GetCumChargeMAh(void);

/*********************************************************************
 * @fn      SOH_GetHalfCycleCount / SOH_GetCycleMilli
 *
 * @brief   半循环计数 (摆幅法) 与等效满循环数 (x1000)
 *
 * @note    两个口径互补 (见 bms_config.h [10] 的说明):
 *            半循环     SOC 摆动超过 SOH_CYCLE_SWING_01 记半个;
 *            等效满循环 = 累计放出电量 / 标称容量 (与 DOD 无关)
 *          返回 1000 倍是为了让 0.001 个循环的分辨率能塞进整数。
 */
uint16_t SOH_GetHalfCycleCount(void);
uint32_t SOH_GetCycleMilli(void);

uint8_t  SOH_IsCountValid(void);      /* 计数有效 (至少累计过一次) */

/*********************************************************************
 * @fn      SOH_ResetCounters
 *
 * @brief   清零累计充放电与循环计数 (换电池 / 换电芯后手工归零)
 *
 * @note    只动计数, 不动学习到的容量与 R0。
 */
void     SOH_ResetCounters(void);

/* =====================================================================
 * 掉电保持支撑接口 (供 bms_nvm.c 调用, 应用代码一般不用直接调)
 *
 * 结构体只是"内存里的搬运载体": 落盘时 bms_nvm.c 会按固定偏移逐字段写
 * 小端字节流, 所以这里的 padding / 对齐 不影响落盘格式。但 sizeof 必须
 * 等于 BMS_NVM_PAYLOAD_LEN (120), 否则 bms_nvm.h 里的编译期断言会报错
 * (那说明字段被改过, 记得同步改落盘协议和 PC 解析)。
 *
 * **字段顺序 = 落盘字节顺序**, 而且**前 68 字节与 ver 2 逐字节相同**:
 * PC 侧"68 B 参数块"那套老流程 (bms_gui 的参数块面板 / bms_tune_proto 的
 * SOH_BLOB_BYTES) 照旧只读前 68 字节也能用, 新增字段全部追加在后面。
 * 改字段时不要动前 68 字节的顺序 —— 那会让老工具静默读错位。
 *
 * 几个不能漏的字段:
 *   base[11]   必须一起存, 这是最容易漏的一条: SOH_Init() 用"当前 R0 表"
 *              快照当 SOH_R 的分母。只恢复学过的 R0 而不恢复 base, 重启后
 *              分母变成"学过的值", SOH_R 就永远显示 100%。
 *   temp_dc    R0 是按"当前温度基准 + 老化增量"存的, 所以上电导入时必须
 *              **先把标定表切回落盘时的温度**, 再导入那时学到的 R0, 算出来
 *              的增量才干净; 否则会混进"落盘温度到当前温度"的基准差 (低温下
 *              可能有十几 mΩ)。这一步由 bms_nvm.c 在 SOH_ImportParam() 之前做。
 *   cum_*      累计充放电。**每帧都在变**, 所以它自己带一套落盘阈值
 *              (SOH_COUNT_SAVE_PCT), 不是每帧置脏 —— 见 soh.c 的说明。
 * ===================================================================== */
typedef struct
{
    /* ---- 以下 68 字节 = ver 2 的完整载荷, 顺序不能动 ---- */
    uint32_t cap_mah;      /* 学到的容量 Q (mAh, 参考工况 25C/0.2C) */
    uint16_t r0[11];       /* 学到的 R0 表 (mΩ), **放电方向**, SOC = i*10% */
    uint16_t base[11];     /* R0 标定基线快照 (SOH_R 的分母, 两个方向共用) */
    uint8_t  cnt[11];      /* 放电方向各格有效样本数 (续接自适应 α) */
    uint8_t  q_n;          /* 容量学习次数 */
    uint8_t  r0_any;       /* 放电方向 R0 已至少 1 个有效样本 */
    uint8_t  q_any;        /* 容量已至少学到 1 次 */
    int16_t  temp_dc;      /* 落盘时的电芯温度 (0.1 摄氏度) */
    float    kf_p;         /* 卡尔曼协方差 (续接滤波状态) */
    /* ---- 以下为 ver 3 新增 (68 之后追加) ---- */
    uint16_t r0_chg[11];   /* 学到的 R0 表 (mΩ), **充电方向** */
    uint8_t  cnt_chg[11];  /* 充电方向各格有效样本数 */
    uint8_t  r0_chg_any;   /* 充电方向 R0 已至少 1 个有效样本 */
    uint8_t  cnt_any;      /* 计数字段有效 (至少累计过一次充放) */
    uint8_t  pad0;         /* 对齐填充, 写 0 */
    uint32_t cum_chg_mah;  /* 累计充入电量 (mAh) */
    uint32_t cum_dis_mah;  /* 累计放出电量 (mAh) */
    uint16_t half_cycle;   /* 半循环计数 (摆幅法) */
    uint16_t pad1;         /* 预留, 写 0 */
    uint32_t pad2;         /* 预留, 写 0 */
} soh_param_t;

void    SOH_ExportParam(soh_param_t *p);        /* 内部状态 -> 结构体 */
void    SOH_ImportParam(const soh_param_t *p);  /* 结构体 -> 内部状态 */
uint8_t SOH_IsDirty(void);                      /* 有未落盘的学习结果 */
void    SOH_ClearDirty(void);

#endif /* __SOH_H */
