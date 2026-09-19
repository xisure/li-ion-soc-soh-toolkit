/*********************************************************************************
 * File Name          : soc.h
 * Description        : SOC 估计模块接口 = 静置 OCV 查表 + 带载安时积分 混合
 *
 * 用法 (主循环, 每 BMS_UPDATE_PERIOD_MS 一次):
 *   读电压电流 -> if(数据有效) SOC_AhUpdate(bus_mv, cur_ma);
 *   显示/上报取 SOC_GetPercent() (整数%) 或 SOC_GetPercent01() (0.01%)。
 *   数据无效帧 (传感器掉线) 不要调用 SOC_AhUpdate, SOC 保持上次值。
 *
 * 算法:
 *   带载(非静止) -> 安时积分: SOC = 基准 - Σ(I×Δt)/容量 (放电为正)
 *     其中容量是"折算到参考工况"的可用容量: 低温/大倍率下可用容量变小,
 *     同一安时数对应更大的 ΔSOC。口径见 bms_config.h [10]。
 *   静止(|I|<SOC_IDLE_CUR_MA 满 SOC_IDLE_MS) -> 向 OCV 目标平滑校正:
 *   首次直接覆盖, 此后每次收敛偏差一半(半衰, 显示无感),
 *   大偏差封顶 ±SOC_RESYNC_MAX_01
 *   上电首次 -> OCV 查表建基准 (尽量上电静止, 否则首次静置后自动修正)
 *
 * 全部可调参数 (电芯表 / 判据 / 周期 / EKF 参数) 见 bms_config.h。
 * 其中 EKF 的 8 个参数可在运行期由在线调参协议改写 (不落盘, 见文件末尾)。
 * 移植方法见同目录 README.md。
 *
 * 源码编码 UTF-8, 换行 LF。
 *
 * 许可: MIT (见仓库根 LICENSE)。Copyright (c) 2026 锂电池 SOC/SOH 算法库 贡献者。
 *******************************************************************************/

#ifndef __SOC_H
#define __SOC_H

#include <stdint.h>
#include "bms_config.h"   /* 电芯参数与判据常量集中在这里 */

/*********************************************************************
 * @fn      SOC_AhUpdate
 *
 * @brief   安时积分 + 静置 OCV 重同步 + EKF 一步 (详见 soc.c 注释)
 *
 * @param   bus_mv  电池电压 mV
 * @param   cur_ma  电流 mA, 放电为正
 *
 * @note    调用周期应为 BMS_UPDATE_PERIOD_MS。内部用真实时间戳算 Δt
 *          (BMS_USE_TICK 时), 所以偶发丢帧不会让积分失真。
 *          采样无效帧 (传感器掉线 / 读数过期) 不要调用本函数。
 */
void SOC_AhUpdate(int32_t bus_mv, int32_t cur_ma);

/*********************************************************************
 * @fn      SOC_GetPercent
 *
 * @brief   当前 SOC (整数 %, 四舍五入), 钳位 0~100
 */
uint8_t SOC_GetPercent(void);

/*********************************************************************
 * @fn      SOC_GetPercent01
 *
 * @brief   当前 SOC (0.01% 精度, 0~10000)
 *
 * @note    与 SOC_GetPercent 同源: EKF 建立后取 EKF soc (带载平滑 +
 *          SOC_IDLE_MS 静置 OCV 强同步), 否则回退 "基准 + 安时积分" (0.01%)。
 *          供上位机记录后做标定 / 滤波参数离线对比。
 */
int32_t SOC_GetPercent01(void);

/*********************************************************************
 * @fn      SOC_R1MOhm / SOC_R0MOhm / SOC_TauS
 *
 * @brief   查标定表 (SOC 0~100% 每 10% 网格, 最近网格点取值), 供外部
 *          模型查询。EKF 内部用的是同一张表。
 */
uint16_t SOC_R1MOhm(uint8_t soc_pct);   /* 极化电阻 R1 (mΩ) */
uint16_t SOC_R0MOhm(uint8_t soc_pct);   /* 欧姆内阻 R0 (mΩ), 放电方向 */
uint16_t SOC_TauS(uint8_t soc_pct);     /* 极化时间常数 τ (秒) */

/* ---- 充/放方向 ----
 * R0 按"充"与"放"两张独立的表维护: 同一个 SOC 点上两个方向的表观内阻
 * 并不相等 (充放电过电位不对称; R0 在 200ms 尺度上量, 已含一部分快的
 * 电荷转移阻抗)。EKF 的量测方程 h = OCV - I*R0 - vrc 于是按当前电流
 * 方向取用, soh.c 也按方向各自学习、各自计数。
 * 索引 0 = 放电 (SOH_R 与老接口 SOC_R0MOhm / SOC_GetR0At 用的就是它),
 * 1 = 充电。 */
#define SOC_DIR_DISCHARGE   0u
#define SOC_DIR_CHARGE      1u

/*********************************************************************
 * @fn      SOC_GetR0MOhmDir / SOC_GetR0DirAt
 *
 * @brief   按方向查 R0: 前者按整数 SOC(%), 后者按 10% 网格索引 (0~10)
 */
uint16_t SOC_GetR0MOhmDir(uint8_t dir, uint8_t soc_pct);
uint16_t SOC_GetR0DirAt(uint8_t dir, uint8_t idx);

/*********************************************************************
 * @fn      SOC_SetR0ChgAt / SOC_GetR0ChgAt
 *
 * @brief   读/写**充电方向**的 R0 表 (mΩ)。放电方向的入口仍是
 *          SOC_SetR0At / SOC_GetR0At (语义没变, 供既有命令与 SOH_R 用)。
 */
void     SOC_SetR0ChgAt(uint8_t idx, uint16_t mohm);
uint16_t SOC_GetR0ChgAt(uint8_t idx);
void     SOC_SetR0DirAt(uint8_t dir, uint8_t idx, uint16_t mohm);

/* =================================================================
 * 在线学习支撑接口 (供 soh.c 使用)
 * ================================================================= */

/*********************************************************************
 * @fn      SOC_SetCapacityMAh / SOC_GetCapacityMAh
 *
 * @brief   运行时容量 mAh 读写。soh.c 学到新容量后写回, SOC 的安时积分
 *          与 EKF 的 dsoc 立刻按新容量换算。
 *
 * @note    写入值被钳位到 [SOC_CAP_LO_PCT%, SOC_CAP_HI_PCT%] × 标称容量
 *          (见 bms_config.h [6]), 防止单次离谱估计把 SOC 算飞 (老化到
 *          50% 容量已远超 EOL, 110% 上限挡住过估计)。
 *          写入成功时会把当前 SOC 冻结成新的积分基准并清零累加器, 否则
 *          已累计的电荷会按新容量重新换算, 显示会瞬间跳变。
 */
void     SOC_SetCapacityMAh(uint32_t cap_mah);
uint32_t SOC_GetCapacityMAh(void);

/*********************************************************************
 * @fn      SOC_VoltageToSoc01
 *
 * @brief   电压(mV) -> SOC (0.01% 单位, 0~10000)
 *
 * @note    与内部 SOC_FromVoltage 同一张 OCV 表, 但精度从整数 % 提到
 *          0.01%。soh.c 的容量学习要用: ΔSOC 只有 30% 量级时, 1% 量化
 *          会让 Q 估计引入 3.3% 系统误差 (实测需求 <1%)。
 */
int32_t  SOC_VoltageToSoc01(int32_t mv);

/*********************************************************************
 * @fn      SOC_SetR0At / SOC_GetR0At
 *
 * @brief   按 10% 网格索引 (0~10) 读写 R0 表 (mΩ)。
 *          soh.c 的在线学习把新测值融合进表, EKF 量测方程即时生效。
 */
void     SOC_SetR0At(uint8_t idx, uint16_t mohm);   /* 放电方向表 */
uint16_t SOC_GetR0At(uint8_t idx);                 /* 放电方向表 */

/* =================================================================
 * 在线调参支撑接口 (供 bms_tune.c 在线读写 SOC 运行态与 EKF 参数)
 *
 *   EKF 的 8 个参数原来是 bms_config.h 里的编译期宏 —— 想试一组新参数
 *   就得重编固件。调 EKF 是"试几组看曲线"的活, 所以这里把它们变成
 *   运行期变量 (初值仍取宏), 由协议层读写。
 *
 *   **EKF 参数不落盘**: 与标定表同理, 权威源在 PC (tools/test/kalman_tune.py)
 *   和 bms_config.h —— 掉电回到固件默认值。定下来了就把值写进
 *   bms_config.h 重编, 而不是指望它自己记住。
 * ================================================================= */

/* EKF 可调参数的索引。顺序即协议线上顺序, 改这里必须同步:
 *   在线调参协议.md §6        (逐字节布局)
 *   脚本/bms_tune_proto.py    (EKF_PARAM 列表与命令表) */
typedef enum
{
    SOC_EKF_Q_SOC = 0,      /* 过程噪声: SOC (%^2/帧) */
    SOC_EKF_Q_VRC,          /* 过程噪声: 极化电压 (mV^2/帧) */
    SOC_EKF_R_V,            /* 量测噪声: 端压 (mV^2) */
    SOC_EKF_P0_SOC,         /* 初值协方差: SOC */
    SOC_EKF_P0_VRC,         /* 初值协方差: vrc */
    SOC_EKF_RES_MAX_MV,     /* 新息野值门限 (mV) */
    SOC_EKF_S_MIN,          /* 新息方差下限 (mV^2) */
    SOC_EKF_P_MIN,          /* 协方差下限 */
    SOC_EKF_N               /* 参数个数 (= 8) */
} soc_ekf_param_t;

/*********************************************************************
 * @fn      SOC_GetBase01 / SOC_GetCoulombMAh
 *
 * @brief   读积分基准 (0.01%) 与自基准起累计的电荷 (mAh, 放电为正)
 *
 * @note    这两项配上 SOC_GetPercent01 就能把 SOC 拆成"基准 + 积分"两半:
 *          调参时用来区分偏差是"重同步没做"还是"积分漂了"。
 */
int32_t SOC_GetBase01(void);
int32_t SOC_GetCoulombMAh(void);

/*********************************************************************
 * @fn      SOC_ForceSetPercent01
 *
 * @brief   强制把 SOC 设成给定值 (0.01% 单位), 现场标定对齐用
 *
 * @note    做四件事: 重建积分基准 / 清零累加器 / 把 EKF 的 SOC 直接置成
 *          该值 (不按端电压查表 —— 带载时端压查表值系统性偏低) /
 *          重新开始静置计时 (给一个完整的观察窗口)。
 *
 *          **不改变"静置久了以 OCV 为准"的既有设计**: 设完之后若真的
 *          静置满 SOC_IDLE_MS, 重同步仍会把它拉回 OCV 表读数。这是有意
 *          的 (静置态 OCV 是唯一真值), 不是命令没生效。
 */
void SOC_ForceSetPercent01(int32_t soc01);

/* =================================================================
 * 工况折算查询 (bms_config.h [10])
 * ================================================================= */

/*********************************************************************
 * @fn      SOC_GetCorrPpm / SOC_GetEffectiveCapacityMAh
 *
 * @brief   当前工况折算系数 K = f_T × f_I (ppm, 1e6 = 1.0)
 *          与折算后的可用容量 (mAh) = 学到的容量 × K
 *
 * @note    K 由 SOC_AhUpdate 每帧按当前温度 (标定表温度) 与电流刷新,
 *          所以调用前必须已经跑过至少一帧。未跑过时 K = 1e6。
 *          显示用途: 把"当前还剩多少可用 mAh"和"额定多少"摆在一起看。
 */
int32_t  SOC_GetCorrPpm(void);
uint32_t SOC_GetEffectiveCapacityMAh(void);

/*********************************************************************
 * @fn      SOC_IsEkfReady / SOC_GetEkfVrc / SOC_GetEkfP11 / SOC_GetEkfP22
 *
 * @brief   读 EKF 内部量: 是否已建立、极化电压 (mV)、两个协方差
 *
 * @note    调 q/r 时看"滤波器是收得住还是发散了"。EKF 未建立时几个
 *          读取函数返回 0。
 */
uint8_t SOC_IsEkfReady(void);
float   SOC_GetEkfVrc(void);
float   SOC_GetEkfP11(void);
float   SOC_GetEkfP22(void);

/*********************************************************************
 * @fn      SOC_EkfParamOk / SOC_EkfSetParam / SOC_EkfGetParam / SOC_EkfParamCount
 *
 * @brief   EKF 可调参数的在线读写 (索引见 soc_ekf_param_t)
 *
 * @note    返回 0 = 通过 / 已写入, 1 = 被拒 (协议层回 EBADARG)。
 *          只挡"明显写错"的值: 非正数 / NaN / 量级离谱 (比如 R 填 1e30)。
 *          好坏不在这里判 —— 那是一组参数跑一遍 PC 端回放的事。
 *
 *          SOC_EkfParamOk 只判不写: 协议层写整组时先用它把所有值过一遍,
 *          有一个越界就整组不写 —— 不留"写进去一半"的参数组。
 *          写入立即生效, 下一帧 EKF 就用新参数, 不用复位。
 */
uint8_t SOC_EkfParamOk(uint8_t idx, float v);
uint8_t SOC_EkfSetParam(uint8_t idx, float v);
float   SOC_EkfGetParam(uint8_t idx);
uint8_t SOC_EkfParamCount(void);

/*********************************************************************
 * @fn      SOC_EkfReset
 *
 * @brief   把滤波器打回"刚建立"的状态: 极化电压与协方差回初值
 *
 * @note    **保留当前 SOC 估计, 不重猜**: 调参的人要的是"用新参数重新
 *          收敛", 不是"按当前(多半带载的)端电压重算 SOC"。按 OCV 重新
 *          对齐 SOC 是静置 SOC_IDLE_MS 重同步的职责, 不在这里重复。
 *          EKF 尚未建立时, 用当前"基准 + 积分"值当起点 (与显示一致)。
 */
void SOC_EkfReset(void);

#endif /* __SOC_H */
