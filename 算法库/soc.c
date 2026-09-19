/*********************************************************************************
 * File Name          : soc.c
 * Description        : SOC 估计模块 = 静置 OCV 查表 + 带载安时积分 混合
 *
 * 算法 (三部分, 互相配合):
 *   安时积分   带载时 SOC = 基准 - Σ(I×Δt)/容量, 放电为正 -> SOC 降
 *   OCV 重同步 静止(|I|<SOC_IDLE_CUR_MA 持续 SOC_IDLE_MS) 时向 OCV 查表值
 *              平滑校正, 消积分漂移
 *   一阶 RC EKF 端压平滑 + 极化电压观测 + SOC 微修正, 让带载显示不毛刺
 *   上电首次   以当时电压查 OCV 建基准 (尽量上电静止, 否则首次静置后自动修正)
 *
 * 调用: 主循环每 BMS_UPDATE_PERIOD_MS 调一次 SOC_AhUpdate(bus_mv, cur_ma)。
 *       传感器掉线帧不要调用, SOC 保持不动。
 *
 * 精度设计:
 *   积分 Δt 用真实时间戳差 (BMS_NowMs) 而非固定周期 —— 任务耗时波动、
 *   改周期、临时插入任务时都不必回头改这里, 积分永不失真。
 *   内部 0.01% 定点 (10000 = 100.00%), int64 累加器无溢出
 *   换算系数 = 360×容量 = 1,206,000 mA·ms/0.01% (3350mAh 时)
 *   累加器存的是"折算到参考工况的等效电量" (Q16 定点), 见下面 s_coul_eq_q16
 *   的说明 —— 温度/倍率修正只作用在电流上, 读的时候除一个恒定分母
 *   整数除法截断误差 < 1 LSB = 0.01%, 不随积分时间累积
 *
 * 静置消抖:
 *   电流 LSB=0.5mA, 静置时实测 ±19mA 抖动 (无负载纯噪声)。
 *   若单帧 |I|<SOC_IDLE_CUR_MA 就判静置, 会 (a) 把静置期间的抖动当
 *   "小电流"积分, 长时间静置漂移几个百分点; (b) 一次电流毛刺就能让
 *   SOC_IDLE_MS 计时器从头开始。
 *   因此要求连续 SOC_QUIET_DEBOUNCE 帧 (~1s) |I|<门槛 才算进入静置, 进入后
 *   本帧跳过积分; 任何一帧 |I|>=门槛 把消抖计数器和静置计时器同时清零。
 *
 * 电芯参数 / 判据常量 / 调用周期全部在 bms_config.h, 本文件不定义可调参数。
 * 源码编码 UTF-8, 换行 LF。
 *
 * 许可: MIT (见仓库根 LICENSE)。Copyright (c) 2026 锂电池 SOC/SOH 算法库 贡献者。
 *******************************************************************************/

#include "soc.h"
#include "bms_port.h"    /* BMS_NowMs: 平台时间戳统一出口 */
#include "bms_cal.h"     /* 标定表: 多温度点 + 按当前温度插值 */

#if BMS_USE_LIBM_EXPF
#include <math.h>        /* expf: EKF 的 RC 状态转移系数 */
#endif

/* EKF 步长 (秒): 直接由调用周期换算, 改 BMS_UPDATE_PERIOD_MS 时自动跟随。
 * 默认 200ms -> 0.2f, 与历史版本硬编码的 0.2f 完全一致 (编译期常量折叠)。 */
#define KF_DT   (BMS_UPDATE_PERIOD_MS / 1000.0f)

/* =====================================================================
 * 标定表 (内容在 bms_config.h, 换电芯只改那一处)
 * ===================================================================== */

/* 标定表不再由本文件持有 —— bms_cal.c 统一管理: 三个温度点各一张,
 * 按当前温度线性插值出"当前温度的活跃表"。表的内容在 bms_config.h。
 * 下面三个宏按 SOC 点号取当前温度的出厂值 (i = 0~10, 对应 SOC = i*10%)。 */
#define OCV_AT(i)   BMS_CAL_GetBase(BMS_CAL_OCV, (uint8_t)(i))
#define R1_AT(i)    BMS_CAL_GetBase(BMS_CAL_R1,  (uint8_t)(i))
#define TAU_AT(i)   BMS_CAL_GetBase(BMS_CAL_TAU, (uint8_t)(i))

/* R0 的老化增量 (mΩ)。加温度维度后必须把两个量拆开:
 *     R0 活跃值 = 当前温度的标定基准 (bms_cal 给) + 本表的老化增量 (SOH 学)
 * 拆开的理由: 温度一变, 基准就得重算。若让 SOH 直接改写"表", 每次温度
 * 刷新都会把学到的老化量抹回出厂值 —— 一轮放电学出来的东西全丢。
 * 拆开之后 delta 不动、只换基准, 学习成果自动跟着温度走。
 * 初值全 0 = 出厂未老化状态。
 *
 * 2026-09-14 起再按**充/放方向分开**: 同一个 SOC 点上, 充电方向的表观内阻
 * 与放电方向不等 (充放电过电位不对称, 而 R0 是在 200ms 尺度上量的, 里面
 * 已经含了一部分快的电荷转移阻抗)。混在一格里做指数平滑的结果是两个方向
 * 都不准 —— 尤其 EKF 的量测方程 h = OCV - I*R0 - vrc 在充电段会系统性偏。
 * 分开之后 EKF 按当前电流方向取用, soh.c 也按方向各自学习。
 * 索引 0 = 放电 (SOH_R 与老接口用的就是它), 1 = 充电。 */
static int16_t s_r0_delta[2][11];

/* 取第 dir 方向、第 idx 格的活跃 R0: 基准 + 老化增量, 再钳到表的合法范围 */
static uint16_t r0_at_dir(uint8_t dir, uint8_t idx)
{
    int32_t v;

    if(idx > 10) idx = 10;
    if(dir > 1u)  dir = 0u;

    v = (int32_t)BMS_CAL_GetBase(BMS_CAL_R0, idx) + (int32_t)s_r0_delta[dir][idx];

    if(v < (int32_t)SOC_R0_TABLE_MIN_MOHM) v = (int32_t)SOC_R0_TABLE_MIN_MOHM;
    if(v > (int32_t)SOC_R0_TABLE_MAX_MOHM) v = (int32_t)SOC_R0_TABLE_MAX_MOHM;

    return (uint16_t)v;
}

/* 放电方向快捷方式 (SOH_R 与老接口用的就是它) */
static uint16_t r0_at(uint8_t idx)
{
    return r0_at_dir(SOC_DIR_DISCHARGE, idx);
}

/* 安时积分内部状态 (0.01% 定点精度: 10000 = 100.00%) */
static int32_t  s_soc_base_01;   /* 积分基准 SOC: 上电/静置重同步时由 OCV 表刷新 */

/* 电荷累加器。**存的是"折算到参考工况的等效电量", 不是原始电荷**:
 *     s_coul_eq_q16 = Σ (I × Δt × 65536 / K),  K = f_T × f_I (bms_config.h [10])
 * 这么做的理由: 折算系数逐帧在变, 而"用当前可用容量去换算电荷"这条路走不通
 * —— 每帧一次整数除法会把不足 1 LSB 的部分截掉 (2000mA×200ms/(360×3350)
 * = 0.33 个 LSB, 每帧都截成 0), 积分会**完全失效**。改成把折算做在电流上,
 * 累加器本身仍严格线性, 读的时候只除一个恒定分母。
 * K = 1 时 s_coul_eq_q16 = 65536 × Σ I×Δt, 与改造前逐位等价。 */
static int64_t  s_coul_eq_q16;

/* 当前工况折算系数 K (Q16 定点), 每帧由 soc_corr_update() 刷新。 */
static uint32_t s_corr_q16 = 65536u;
#define SOC_COUL_EQ_DIV     65536LL
#define SOC_Q16             65536LL
static uint32_t s_last_ms;       /* 上次积分时刻 (真实时间戳, 消除主循环抖动) */
static uint8_t  s_soc_inited;    /* 基准已建立 */
static uint8_t  s_quiet_cnt;     /* 静置消抖: 连续 |I|<门槛 帧数, <SOC_QUIET_DEBOUNCE 不算静置 */
static uint8_t  s_idle_flag;     /* 已进入静置态(消抖通过), SOC_IDLE_MS 计时中 */
static uint32_t s_idle_since_ms; /* 进入静置的时刻 */
static uint8_t  s_first_resync;  /* 首次重同步未做: 首次直接覆盖纠偏(见重同步分支) */

/* 运行时容量 mAh: 初值 = 标称 (bms_config.h), soh.c 在线学习后改写。
 * 安时积分与 EKF 的 dsoc 全部按它换算 —— 容量学到 80% 后, 同样放出
 * 100mAh 对应 ΔSOC 从 2.99% 变成 3.73%, SOC 才不会在老化后虚高。 */
static uint32_t s_cap_mah = SOC_CAPACITY_NOMINAL_MAH;

/* =====================================================================
 * 一阶 RC EKF 状态
 *   x = [soc(%), vrc(mV)];  量测 = 端压
 *   soc' = soc - I*dt/(cap*3600)*100          (放电为正)
 *   vrc' = a*vrc + I*R1/1000*(1-a), a = exp(-dt/tau)
 *   h    = OCV(soc) - I*R0/1000 - vrc;  H = [dOCV/dsoc, -1]
 * 调参结果 (全量 49072 帧): q_soc=1e-3, q_vrc=0.5, R=10
 *   带载段 99.98% 单调下降, 各静置末与安时积分参考 RMS 偏差 0.72%
 * EKF 负责端压平滑 + 极化电压观测 + SOC 微修正; SOC 主体仍由上面的
 * "基准 + 安时积分"给出 (容量准), 900s 静置 OCV 重同步照常覆盖基准。
 * 带载"端压反推"由 EKF 的 H 自然完成, 不再需要外挂 Rdc 补偿。
 *
 * 默认值 (过程噪声 / 量测噪声 / 初值 / 数值保护) 在 bms_config.h 的
 * [4] EKF 参数; 下面把它们复制成**运行期变量** —— 在线调参协议要能在板子上
 * 试参数, 不能每试一组就重编固件 (见文件末尾的在线调参支撑接口)。
 * ------------------------------------------------------------------ */
static float   s_kf_soc;    /* EKF SOC (%) */
static float   s_kf_vrc;    /* EKF 极化电压 (mV) */
static float   s_kf_p11;    /* 协方差: SOC */
static float   s_kf_p22;    /* 协方差: vrc */
static float   s_kf_p12;    /* 协方差: 交叉 */
static uint8_t s_kf_init;   /* EKF 已建立 */

/* ---- 运行期 EKF 参数 (初值 = bms_config.h [4], 可被在线调参改写) ----
 * 顺序与 soc_ekf_param_t 索引一一对应 (soc.h)。
 * 写入立即生效: 下面 SOC_KF_Step 每帧直接读这些变量, 不用复位滤波器。
 * **不落盘** —— 掉电回这里的初值。 */
static float s_kf_q_soc    = BMS_EKF_Q_SOC;     /* 过程噪声: SOC */
static float s_kf_q_vrc    = BMS_EKF_Q_VRC;     /* 过程噪声: vrc */
static float s_kf_r_v      = BMS_EKF_R_V;       /* 量测噪声: 端压 */
static float s_kf_p0_soc   = BMS_EKF_P0_SOC;    /* 初值协方差: SOC */
static float s_kf_p0_vrc   = BMS_EKF_P0_VRC;    /* 初值协方差: vrc */
static float s_kf_res_max  = BMS_EKF_RES_MAX_MV;/* 新息野值门限 */
static float s_kf_s_min    = BMS_EKF_S_MIN;     /* 新息方差下限 */
static float s_kf_p_min    = BMS_EKF_P_MIN;     /* 协方差下限 */

/* =====================================================================
 * exp(-x) 的两个来源 (BMS_USE_LIBM_EXPF 切换, 见 bms_config.h)
 *
 * 注意: BMS_EXPNEG(x) 的语义是 exp(-x) —— 负号在这里加, 调用方传正的
 *       KF_DT/tau 即可。libm 分支写成 expf(x) 是错的 (a 会 >1, RC 状态
 *       反向发散), 已修。两个分支对同一输入必须给出同一个值。
 * ===================================================================== */
#if BMS_USE_LIBM_EXPF
#define BMS_EXPNEG(x)   expf(-(x))
#else
/*********************************************************************
 * @fn      SOC_ExpNeg
 *
 * @brief   不依赖 libm 的 exp(-x) 近似: 范围缩减 + 6 阶泰勒 + 平方还原
 *
 * @note    EKF 里 x = KF_DT/τ。6 阶泰勒在 |x|<=1 时相对误差 < 2e-4;
 *          x>1 时先折半 k 次 (k<=6, 覆盖 x<=64), 泰勒算完再平方 k 次,
 *          误差被平方放大 2^k 倍 —— x<=4 时总相对误差 < 8e-4, 对 RC 状态
 *          递推的衰减系数完全够用。
 *          无 FPU 平台上本近似约 5 次乘加, 比 expf 的库调用便宜一个量级。
 */
static float SOC_ExpNeg(float x)
{
    float y;
    int   k = 0;

    if(x <= 0.0f) return 1.0f;

    while(x > 1.0f && k < 6)   /* 范围缩减: x = x0 * 2^k, x0 <= 1 */
    {
        x *= 0.5f;
        k++;
    }

    /* exp(-x0) 的 6 阶泰勒 (Horner): 1 - x + x^2/2 - x^3/6 + x^4/24 - x^5/120 + x^6/720 */
    y = 1.0f + x * (-1.0f + x * (0.5f + x * (-1.6666667e-1f +
              x * (4.1666667e-2f + x * (-8.3333333e-3f + x * 1.3888889e-3f)))));

    while(k-- > 0) y *= y;     /* 平方还原 */
    return y;
}
#define BMS_EXPNEG(x)   SOC_ExpNeg(x)
#endif /* BMS_USE_LIBM_EXPF */

/*********************************************************************
 * @fn      SOC_FromVoltage
 *
 * @brief   由电压(mV)查 OCV 表插值得到 SOC (整数 0~100%)
 *
 * @note    表是"开路电压", 带载时端电压含内阻压降, SOC 会偏低
 *          (小电流放电偏差小; 准确标定靠重标 OCV 表)。
 *          平台区 (3.6~3.9V) 每 10% SOC 只差 70~100mV, 电压采样误差
 *          10mV ~ 1% SOC, 1.25mV 分辨率的电压计足够。
 */
static uint8_t SOC_FromVoltage(int32_t mv)
{
    int8_t i;

    if(mv >= (int32_t)OCV_AT(10))
    {
        return 100;
    }
    if(mv <= (int32_t)OCV_AT(0))
    {
        return 0;
    }
    for(i = 9; i >= 0; i--)
    {
        if(mv > (int32_t)OCV_AT(i))
        {
            /* 落在 [i, i+1] 区间: 线性插值 */
            return (uint8_t)(i * 10 +
                   (mv - (int32_t)OCV_AT(i)) * 10 /
                   (int32_t)(OCV_AT(i + 1) - OCV_AT(i)));
        }
    }
    return 0;
}

/*********************************************************************
 * @fn      SOC_R1MOhm
 *
 * @brief   查标定极化电阻表: 输入 SOC%(整数), 返回最近网格点的 R1 (mΩ)
 */
uint16_t SOC_R1MOhm(uint8_t soc_pct)
{
    uint8_t idx = (uint8_t)(soc_pct / 10);

    if(idx > 10)
    {
        idx = 10;
    }
    return R1_AT(idx);
}

/*********************************************************************
 * @fn      SOC_R0MOhm
 *
 * @brief   查标定欧姆内阻表 R0 (mΩ), 最近网格点取值; EKF 即时压降项用
 */
uint16_t SOC_R0MOhm(uint8_t soc_pct)          /* 放电方向 (老语义) */
{
    uint8_t idx = (uint8_t)(soc_pct / 10);

    if(idx > 10)
    {
        idx = 10;
    }
    return r0_at(idx);
}

uint16_t SOC_GetR0MOhmDir(uint8_t dir, uint8_t soc_pct)
{
    uint8_t idx = (uint8_t)(soc_pct / 10);

    if(idx > 10) idx = 10;
    return r0_at_dir(dir, idx);
}

/*********************************************************************
 * @fn      SOC_TauS
 *
 * @brief   查标定极化时间常数表 τ (秒), 最近网格点取值; RC 状态递推用
 */
uint16_t SOC_TauS(uint8_t soc_pct)
{
    uint8_t idx = (uint8_t)(soc_pct / 10);

    if(idx > 10)
    {
        idx = 10;
    }
    return TAU_AT(idx);
}

/* ---------------- EKF 辅助: OCV 浮点查表/斜率 (与 PC 端 kalman_tune 一致) ---- */
static float SOC_OCV_MvF(float soc)
{
    int i;

    if(soc >= 100.0f) return (float)OCV_AT(10);
    if(soc <= 0.0f)   return (float)OCV_AT(0);
    i = (int)(soc / 10.0f);
    if(i > 9) i = 9;
    return (float)OCV_AT(i) +
           (soc - i * 10.0f) * (float)(OCV_AT(i + 1) - OCV_AT(i)) / 10.0f;
}

static float SOC_OCV_SlopeF(float soc)     /* dOCV/dSOC, mV/%, 当前表段斜率 */
{
    int i = (int)(soc / 10.0f);

    if(i > 9) i = 9;
    if(i < 0) i = 0;
    return (float)(OCV_AT(i + 1) - OCV_AT(i)) / 10.0f;
}

/*********************************************************************
 * @fn      SOC_KF_Reset
 *
 * @brief   以当前电压查表复位 EKF (静置 SOC_IDLE_MS 强同步 / 上电首次用)
 */
/*********************************************************************
 * @fn      SOC_KF_Seed
 *
 * @brief   以给定 SOC 值建/重置 EKF (极化电压与协方差回初值)
 *
 * @note    强制设 SOC 时用这个而不是 SOC_KF_Reset —— 带载时端电压查表值
 *          系统性偏低, 会把刚设的值立刻带偏。
 */
static void SOC_KF_Seed(float soc_pct)
{
    s_kf_soc  = soc_pct;
    s_kf_vrc  = 0.0f;
    s_kf_p11  = s_kf_p0_soc;
    s_kf_p22  = s_kf_p0_vrc;
    s_kf_p12  = 0.0f;
    s_kf_init = 1;
}

static void SOC_KF_Reset(int32_t bus_mv)
{
    SOC_KF_Seed((float)SOC_FromVoltage(bus_mv));
}

/*********************************************************************
 * @fn      SOC_KF_Step  (每 BMS_UPDATE_PERIOD_MS, 在 SOC_AhUpdate 内调用)
 *
 * @brief   一阶 RC EKF 一步: 预测(积分+RC) + 端压量测更新
 *
 * @note    SOC 主体由安时积分(基准 s_soc_base + coul)给出, EKF 提供
 *          端压平滑与极化电压 vrc 观测, 静置 SOC_IDLE_MS 时由外层 Reset 强同步。
 *          野值(校准/接触弹跳)残差超过 s_kf_res_max 丢弃量测。
 */
static void SOC_KF_Step(int32_t bus_mv, int32_t cur_ma)
{
    float   I    = (float)cur_ma;               /* mA */
    float   V    = (float)bus_mv;               /* mV */
    float   a, r0, r1, tau, dsoc, h, h1, h2, s, k1, k2, res, soc_c;
    uint8_t soc8;
    float   p11, p22, p12;

    soc_c = s_kf_soc;                           /* 先钳位再查表(防负->uint8 UB) */
    if(soc_c < 0.0f)   soc_c = 0.0f;
    if(soc_c > 100.0f) soc_c = 100.0f;
    soc8  = (uint8_t)(soc_c + 0.5f);
    /* R0 按当前电流方向取表: 充/放的表观内阻不同, 用错方向会让量测方程
     * h = OCV - I*R0 - vrc 在充电段系统性偏。|I| 很小时两个方向都无所谓
     * (钳位后接近出厂基准), 放电方向作默认。 */
    r0  = (float)SOC_GetR0MOhmDir((cur_ma < 0) ? SOC_DIR_CHARGE
                                              : SOC_DIR_DISCHARGE, soc8);
    r1  = (float)SOC_R1MOhm(soc8);
    tau = (float)SOC_TauS(soc8);
    if(tau < 1.0f) tau = 1.0f;

    /* --- 预测 --- */
    dsoc = I * KF_DT / ((float)s_cap_mah * 3600.0f) * 100.0f;
    a = BMS_EXPNEG(KF_DT / tau);
    s_kf_soc = s_kf_soc - dsoc;
    s_kf_vrc = s_kf_vrc * a + I * r1 / 1000.0f * (1.0f - a);
    p11 = s_kf_p11 + s_kf_q_soc;                     /* F = diag(1, a) */
    p22 = a * a * s_kf_p22 + s_kf_q_vrc;
    p12 = a * s_kf_p12;

    /* --- 量测更新 (h = OCV - I*R0/1000 - vrc) --- */
    h  = SOC_OCV_MvF(s_kf_soc) - I * r0 / 1000.0f - s_kf_vrc;
    h1 = SOC_OCV_SlopeF(s_kf_soc);
    h2 = -1.0f;
    s  = h1 * (h1 * p11 + h2 * p12) + h2 * (h1 * p12 + h2 * p22) + s_kf_r_v;
    if(s < s_kf_s_min) s = s_kf_s_min;
    k1 = (h1 * p11 + h2 * p12) / s;
    k2 = (h1 * p12 + h2 * p22) / s;
    res = V - h;
    if(res > -s_kf_res_max && res < s_kf_res_max)     /* 野值丢弃 */
    {
        s_kf_soc = s_kf_soc + k1 * res;
        s_kf_vrc = s_kf_vrc + k2 * res;
        if(s_kf_soc < 0.0f)   s_kf_soc = 0.0f;
        if(s_kf_soc > 100.0f) s_kf_soc = 100.0f;
        s_kf_p11 = p11 * (1.0f - k1 * h1) - p12 * k1 * h2;
        s_kf_p22 = p22 * (1.0f - k2 * h2) - p12 * k2 * h1;
        s_kf_p12 = p12 * (1.0f - k2 * h2) - p11 * k2 * h1;
        if(s_kf_p11 < s_kf_p_min) s_kf_p11 = s_kf_p_min;
        if(s_kf_p22 < s_kf_p_min) s_kf_p22 = s_kf_p_min;
    }
    else
    {
        s_kf_p11 = p11;
        s_kf_p22 = p22;
        s_kf_p12 = p12;
    }
}

/* =====================================================================
 * 工况折算 (bms_config.h [10])
 * ===================================================================== */

/*********************************************************************
 * @fn      soc_corr_update
 *
 * @brief   刷新折算系数 K = f_T(当前温度) × f_I(当前电流), 钳位后存 Q16
 *
 * @note    温度取的是标定表的活跃温度 (BMS_CAL_GetTempDc) —— 与 OCV/R0 表
 *          用的是同一个温度, 不会出现"表按 30C 插值、容量按 25C 折算"。
 *          主循环里 BMS_CAL_Update 排在 SOC_AhUpdate 之前, 所以这里的温度
 *          就是本帧温度。
 *          参数未标定时 f_T ≡ 1、f_I ≡ 1 -> K = 65536, 全程恒等。
 */
static void soc_corr_update(int32_t cur_ma)
{
    float k = BMS_CAL_TempFactorF(BMS_CAL_GetTempDc())
            * BMS_CAL_RateFactorF(cur_ma);

    if(k < SOC_CORR_MIN) k = SOC_CORR_MIN;
    if(k > SOC_CORR_MAX) k = SOC_CORR_MAX;

    s_corr_q16 = (uint32_t)(k * 65536.0f + 0.5f);
    if(s_corr_q16 == 0u) s_corr_q16 = 1u;      /* 防除零 */
}

/*********************************************************************
 * @fn      soc_from_coul
 *
 * @brief   由"基准 + 等效电量"换算当前 SOC (0.01% 单位), 不钳位
 *
 * @note    分母 = 65536 × 360 × 容量, 容量是**参考工况容量** s_cap_mah;
 *          折算已经做在累加器里, 这里不再乘 K。
 */
static int32_t soc_from_coul(void)
{
    return s_soc_base_01 -
           (int32_t)(s_coul_eq_q16 /
                     (SOC_COUL_EQ_DIV * 360LL * (int64_t)s_cap_mah));
}

/*********************************************************************
 * @fn      SOC_AhInit
 *
 * @brief   建立积分基准: SOC = OCV 表查表值 (仅静止时准确)
 */
static void SOC_AhInit(int32_t bus_mv)
{
    s_soc_base_01 = (int32_t)SOC_FromVoltage(bus_mv) * 100;
    s_coul_eq_q16 = 0;
    s_last_ms     = BMS_NowMs();
    s_soc_inited  = 1;
    s_first_resync = 1;   /* 首次重同步等待触发 */
    SOC_KF_Reset(bus_mv);
}

/*********************************************************************
 * @fn      SOC_AhUpdate  (每 BMS_UPDATE_PERIOD_MS)
 *
 * @brief   安时积分:  SOC = 基准 - ∫I·dt / 容量  (放电为正 -> SOC 下降)
 *          静止重同步: |I|<SOC_IDLE_CUR_MA 持续 SOC_IDLE_MS 时向 OCV 目标
 *          校正——首次直接覆盖(纠上电带载的基准错), 此后每次收敛偏差的
 *          一半(半衰平滑, 整数%显示无感), 大偏差封顶 ±SOC_RESYNC_MAX_01
 *
 * @note    时间间隔用真实时间戳差 (BMS_NowMs) 而非固定周期:
 *          任务耗时波动或将来改周期时都不必回改, 积分永不失真。
 *          传感器掉线帧调用方不调用本函数, SOC 保持不动。
 *          静置期间 (消抖通过后) 本帧不积分, 避免 ±19mA 抖动积分漂移。
 */
void SOC_AhUpdate(int32_t bus_mv, int32_t cur_ma)
{
    uint32_t now = BMS_NowMs();
    int32_t  dt_ms;
    uint8_t  is_quiet;          /* 本帧 |I| < SOC_IDLE_CUR_MA */

    if(!s_soc_inited)
    {
        SOC_AhInit(bus_mv);   /* 首次: 上电通常静止, bus_mv 即 OCV */
    }

    /* 工况折算系数先刷新: 本帧的重同步换算与新积分都要用它 */
    soc_corr_update(cur_ma);

    /* --- 静置检测 + 消抖 --- */
    is_quiet = (cur_ma > -SOC_IDLE_CUR_MA && cur_ma < SOC_IDLE_CUR_MA) ? 1 : 0;
    if(is_quiet)
    {
        if(s_quiet_cnt < SOC_QUIET_DEBOUNCE)
        {
            s_quiet_cnt++;                       /* 累积; 饱和不再 ++ */
        }
    }
    else
    {
        s_quiet_cnt = 0;                         /* 任何一次毛刺即清零 */
    }

    /* --- OCV 重同步 (消抖通过后才计时; 900s 期间也允许 |I|>=20mA 直接打断) --- */
    if(s_quiet_cnt >= SOC_QUIET_DEBOUNCE)
    {
        if(!s_idle_flag)
        {
            s_idle_flag     = 1;
            s_idle_since_ms = now;
        }
        else if((uint32_t)(now - s_idle_since_ms) >= SOC_IDLE_MS)
        {
            int32_t target_01 = (int32_t)SOC_FromVoltage(bus_mv) * 100;

            if(s_first_resync)
            {
                /* 首次重同步: 直接覆盖纠偏. 上电若带载, 初始基准是
                 * 带载电压查表值(偏低), 一次性拉回比渐进更快; 之后
                 * 积分基于已校准基准, 偏差只会是小漂移, 才用限幅 */
                s_soc_base_01 = target_01;
                s_first_resync = 0;
            }
            else
            {
                /* 后续重同步: 平滑校正——每次只收敛偏差的一半
                 * (指数收敛, 整数%显示几乎无感), 大偏差封顶 ±SOC_RESYNC_MAX_01.
                 * 直接覆盖/线性限幅在 OCV 表/容量不准时显示仍会跳
                 * 1 个数字; 半衰后小漂移(1~2%)两次静置内就无感,
                 * 大偏差由后续静置周期继续逼近 */
                int32_t cur_01  = soc_from_coul();
                int32_t diff_01 = target_01 - cur_01;

                diff_01 /= 2;    /* 半衰: C 向零截断, 1 LSB(0.01%)差不动 */
                if(diff_01 >  SOC_RESYNC_MAX_01) diff_01 =  SOC_RESYNC_MAX_01;
                if(diff_01 < -SOC_RESYNC_MAX_01) diff_01 = -SOC_RESYNC_MAX_01;
                s_soc_base_01 += diff_01;
            }
            s_coul_eq_q16 = 0;
            s_last_ms     = now;
            s_idle_flag   = 0;    /* 重新计时, 持续静置则每 SOC_IDLE_MS 逼近一次 */
            SOC_KF_Reset(bus_mv); /* EKF 同步到 OCV 表 (极化已消) */
        }
    }
    else
    {
        s_idle_flag = 0;          /* 消抖未过/有毛刺, 离开静置 */
    }

    /* --- 安时积分: ΔQ = I × Δt ---
     * 静置期间 (消抖通过后) 跳过积分, 堵死 ±19mA 抖动造成的累计漂移。
     * 静置尾段 OCV 重同步会清零积分累加器, 此处跳过不会让电量失真 */
    dt_ms = (int32_t)(now - s_last_ms);        /* 无符号相减, 回绕安全 */
    if(dt_ms < 0 || dt_ms > BMS_DT_MAX_MS)
    {
        dt_ms = BMS_UPDATE_PERIOD_MS;          /* 异常间隔兜底 */
    }
    s_last_ms = now;

    if(cur_ma != 0 && !s_idle_flag)            /* 静置态: 不积分 */
    {
        /* 折算做在电流上: 等效电流 = I / K, 再乘 65536 落成 Q16 等效电量。
         *
         * **两个 SOC_Q16 都要, 少一个整条积分就废** —— 这不是笔误:
         *   K 本身是 Q16 (s_corr_q16 = K × 65536), 所以
         *     65536 / K = 65536 * 65536 / s_corr_q16
         *   第一个 65536 把 I/K 定标成 Q16 等效电流, 第二个把电荷也定标成
         *   Q16。读的时候 (soc_from_coul / SOC_GetCoulombMAh) 只除恒定分母,
         *   累加过程保持严格线性 —— 若改成逐帧用"当前可用容量"折算,
         *   不足 1 LSB 的部分会被整数除法截掉, 积分直接失效。
         * K = 1 时 s_corr_q16 == SOC_Q16, 两级相消 -> 65536 × I × dt,
         * 恰好是改造前 s_coul_mAms 的固定倍数, 换算结果逐位相同;
         * 只写一个 65536 的话累加器退化成 ΣI·dt, 而读取端仍按 Q16 除,
         * 结果被压掉 65536 倍 —— 表现为"库仑电量恒为 0, SOC 不动"。
         *
         * 中间量量级: |I|<=20000mA, dt<=2000ms, 65536*65536=4.295e9
         *   -> 1.72e17, int64 上限 9.22e18, 余量 50 倍以上。 */
        s_coul_eq_q16 += (int64_t)cur_ma * dt_ms * SOC_Q16 * SOC_Q16
                       / (int64_t)s_corr_q16;
    }

    /* --- EKF 一步: 端压平滑 + vrc 观测 + SOC 微修 (900s 重同步时先 Reset) --- */
    SOC_KF_Step(bus_mv, cur_ma);
}

/*********************************************************************
 * @fn      SOC_GetPercent
 *
 * @brief   当前 SOC (整数 %), 由基准 + 积分换算, 钳位 0~100
 *
 * @note    ΔSOC(0.01%) = ΣI·dt / (3.6e6 * 容量) * 10000
 *                       = ΣI·dt / (360 * 容量)
 *          容量取运行时 s_cap_mah (SOH 学习会改写它; 标称 3350mAh 时
 *          每 0.01% 对应 360*3350 = 1,206,000 mA·ms ≈ 0.335 mAh);
 *          整数除法截断误差 < 1 LSB = 0.01%, 不随积分时间累积.
 *          充电时累加器为负, C 向零截断除法 -> 偏大 0.01%, 可忽略.
 */
uint8_t SOC_GetPercent(void)
{
    int32_t soc_01;

    if(s_kf_init)
    {
        return (uint8_t)(s_kf_soc + 0.5f);     /* EKF soc 已钳位 0~100 */
    }
    soc_01 = soc_from_coul();
    if(soc_01 < 0)      soc_01 = 0;
    if(soc_01 > 10000)  soc_01 = 10000;
    return (uint8_t)((soc_01 + 50) / 100);     /* 四舍五入到整数 % */
}

/*********************************************************************
 * @fn      SOC_GetPercent01
 *
 * @brief   当前 SOC (0.01% 精度整数 0~10000)
 *
 * @note    EKF 建立(s_kf_init)后返回 EKF soc 的 0.01% 值 —— 它就是
 *          SOC_GetPercent() 的量化源, 多 100 倍分辨率供离线对比迭代;
 *          未建立时与 SOC_GetPercent 同一套 "基准+积分" 的 soc_01.
 */
int32_t SOC_GetPercent01(void)
{
    int32_t soc_01;

    if(s_kf_init)
    {
        soc_01 = (int32_t)(s_kf_soc * 100.0f + 0.5f);  /* EKF soc 已钳位 0~100 */
        if(soc_01 < 0)      soc_01 = 0;
        if(soc_01 > 10000)  soc_01 = 10000;
        return soc_01;
    }
    soc_01 = soc_from_coul();
    if(soc_01 < 0)      soc_01 = 0;
    if(soc_01 > 10000)  soc_01 = 10000;
    return soc_01;
}


/* =====================================================================
 * 在线学习支撑接口实现
 *   容量/内阻变成可学状态量后, soh.c 通过下面这几个函数读写它们;
 *   soc.c 不依赖 soh.c, 无反向调用。
 * ===================================================================== */

/*********************************************************************
 * @fn      SOC_VoltageToSoc01
 *
 * @brief   电压(mV) -> SOC (0.01% 单位, 0~10000)
 *
 * @note    与 SOC_FromVoltage 同一张 OCV 表, 只是把整数 % 的分辨率
 *          提到 0.01%。soh.c 的容量学习用: ΔSOC 只有 30% 量级时,
 *          1% 的量化会让 Q 估计带 3.3% 系统误差, 0.01% 可忽略。
 */
int32_t SOC_VoltageToSoc01(int32_t mv)
{
    int8_t i;

    if(mv >= (int32_t)OCV_AT(10))
    {
        return 10000;
    }
    if(mv <= (int32_t)OCV_AT(0))
    {
        return 0;
    }
    for(i = 9; i >= 0; i--)
    {
        if(mv > (int32_t)OCV_AT(i))
        {
            /* 落在 [i, i+1] 区间: 线性插值, 每段 10% -> 1000 个 0.01% */
            return (int32_t)(i * 1000 +
                   (mv - (int32_t)OCV_AT(i)) * 1000 /
                   (int32_t)(OCV_AT(i + 1) - OCV_AT(i)));
        }
    }
    return 0;
}

/*********************************************************************
 * @fn      SOC_SetCapacityMAh / SOC_GetCapacityMAh
 *
 * @brief   运行时容量 mAh 读写 (SOH 在线学习结果的落点)
 *
 * @note    写入值被钳位到 [SOC_CAP_LO_PCT%, SOC_CAP_HI_PCT%] × 标称容量
 *          (见 bms_config.h [6])。换容量前先把当前 SOC 冻结成新的积分
 *          基准并清零累加器 —— 否则已累计的 s_coul_eq_q16 会按新容量重新
 *          换算, SOC 显示会瞬间跳变。
 */
void SOC_SetCapacityMAh(uint32_t cap_mah)
{
    /* 钳位范围见 bms_config.h [6] (默认 50%~110% 标称) */
    uint32_t lo = SOC_CAPACITY_NOMINAL_MAH * SOC_CAP_LO_PCT / 100;
    uint32_t hi = SOC_CAPACITY_NOMINAL_MAH * SOC_CAP_HI_PCT / 100;
    int32_t  soc_01;

    if(cap_mah < lo) cap_mah = lo;
    if(cap_mah > hi) cap_mah = hi;
    if(cap_mah == s_cap_mah)
    {
        return;
    }
    if(s_soc_inited)
    {
        soc_01 = soc_from_coul();  /* 用**旧**容量换算出当前 SOC */
        if(soc_01 < 0)     soc_01 = 0;
        if(soc_01 > 10000) soc_01 = 10000;
        s_soc_base_01 = soc_01;    /* 冻结当前 SOC 为新基准 */
        s_coul_eq_q16 = 0;
        s_last_ms     = BMS_NowMs();
    }
    s_cap_mah = cap_mah;
}

uint32_t SOC_GetCapacityMAh(void)
{
    return s_cap_mah;
}

/*********************************************************************
 * @fn      SOC_SetR0At / SOC_GetR0At
 *
 * @brief   按 10% 网格索引 (0~10) 读写 R0 表 (mΩ), SOH 学习融合后落表
 */
/* 写一个方向的 R0 表 (绝对 mΩ), 内部存成"相对当前温度基准的增量" */
static void soc_set_r0(uint8_t dir, uint8_t idx, uint16_t mohm)
{
    int32_t d;

    if(idx > 10 || dir > 1u) return;

    /* 存储范围防呆见 bms_config.h [6] (默认 20~500 mΩ) */
    if(mohm < SOC_R0_TABLE_MIN_MOHM) mohm = SOC_R0_TABLE_MIN_MOHM;
    if(mohm > SOC_R0_TABLE_MAX_MOHM) mohm = SOC_R0_TABLE_MAX_MOHM;

    /* 存"相对当前温度基准的增量"而不是绝对值 —— 这样温度变了基准跟着换,
     * 学到的老化量还留在 delta 里, 不会被温度刷新抹掉。
     * 调用方 (soh.c) 传进来的仍是绝对 mΩ, 语义没变。
     * 方向分了之后这条性质对两个方向各自成立 (基准是同一张出厂表)。 */
    d = (int32_t)mohm - (int32_t)BMS_CAL_GetBase(BMS_CAL_R0, idx);
    if(d >  32767) d =  32767;
    if(d < -32768) d = -32768;

    s_r0_delta[dir][idx] = (int16_t)d;
}

void SOC_SetR0At(uint8_t idx, uint16_t mohm)     /* 放电方向 (老语义) */
{
    soc_set_r0(SOC_DIR_DISCHARGE, idx, mohm);
}

void SOC_SetR0ChgAt(uint8_t idx, uint16_t mohm)  /* 充电方向 */
{
    soc_set_r0(SOC_DIR_CHARGE, idx, mohm);
}

void SOC_SetR0DirAt(uint8_t dir, uint8_t idx, uint16_t mohm)  /* 按方向 */
{
    soc_set_r0(dir, idx, mohm);
}

uint16_t SOC_GetR0At(uint8_t idx)                /* 放电方向 (老语义) */
{
    if(idx > 10) idx = 10;
    return r0_at(idx);
}

uint16_t SOC_GetR0ChgAt(uint8_t idx)             /* 充电方向 */
{
    if(idx > 10) idx = 10;
    return r0_at_dir(SOC_DIR_CHARGE, idx);
}

uint16_t SOC_GetR0DirAt(uint8_t dir, uint8_t idx)/* 按方向 */
{
    if(idx > 10) idx = 10;
    return r0_at_dir(dir, idx);
}


/* =====================================================================
 * 在线调参支撑接口实现
 *   bms_tune.c 通过下面这些函数在线读写 SOC 运行态与 EKF 参数。
 *   和"在线学习支撑接口"一样遵循单向依赖: bms_tune.c 调 soc.c, 不反向。
 * ===================================================================== */

int32_t SOC_GetBase01(void)
{
    return s_soc_base_01;
}

int32_t SOC_GetCoulombMAh(void)
{
    /* s_coul_eq_q16 是 Q16 的"等效电量" (mA·ms 已按 K 折算过),
     * 除 65536 得 mA·ms, 再除 3.6e6 得 mAh。截断方向与 SOC 换算一致
     * (都往零截), 所以"基准 + 这两项"能对上 SOC。
     * 注意语义: 是**折算到参考工况的等效电量**, 低温/大倍率下它比真实放出的
     * 电荷小 (K<1) —— 这正是"可用容量变少"在数字上的体现。 */
    return (int32_t)(s_coul_eq_q16 / (SOC_Q16 * 3600000LL));
}

uint8_t SOC_IsEkfReady(void)
{
    return s_kf_init;
}

float SOC_GetEkfVrc(void)   { return s_kf_init ? s_kf_vrc : 0.0f; }
float SOC_GetEkfP11(void)   { return s_kf_init ? s_kf_p11 : 0.0f; }
float SOC_GetEkfP22(void)   { return s_kf_init ? s_kf_p22 : 0.0f; }

/*********************************************************************
 * @fn      SOC_ForceSetPercent01
 *
 * @brief   强制设定 SOC (现场标定对齐)
 */
void SOC_ForceSetPercent01(int32_t soc01)
{
    if(soc01 < 0)     soc01 = 0;
    if(soc01 > 10000) soc01 = 10000;

    s_soc_base_01 = soc01;      /* 新基准 */
    s_coul_eq_q16 = 0;          /* 旧累计是按旧基准算的, 清掉 */
    s_last_ms     = BMS_NowMs();
    s_soc_inited  = 1;          /* 上电首帧还没跑过也算已初始化 */
    SOC_KF_Seed((float)soc01 / 100.0f);

    /* 人工对齐等价于"首次重同步已经做过": 之后静置重同步走半衰限幅
     * (每轮最多 SOC_RESYNC_MAX_01), 不会一次性把刚设的值跳回 OCV 值;
     * 留着 s_first_resync=1 的话, 下一次静置会直接覆盖 → 看起来像命令没生效。 */
    s_first_resync = 0;
    s_idle_flag    = 0;         /* 重新开始静置计时: 给一个完整观察窗口 */
}

/*********************************************************************
 * @fn      SOC_EkfReset
 *
 * @brief   把滤波器打回"刚建立"的状态 (保留 SOC 估计)
 */
void SOC_EkfReset(void)
{
    if(!s_kf_init)
    {
        /* 还没建立: 用当前"基准 + 积分"值当起点 —— 与 SOC_GetPercent01
         * 的回退路径同一个值, 免得复位前后显示跳一下。 */
        int32_t soc01 = soc_from_coul();

        if(soc01 < 0)     soc01 = 0;
        if(soc01 > 10000) soc01 = 10000;
        s_kf_soc = (float)soc01 / 100.0f;
    }
    s_kf_vrc  = 0.0f;
    s_kf_p11  = s_kf_p0_soc;
    s_kf_p22  = s_kf_p0_vrc;
    s_kf_p12  = 0.0f;
    s_kf_init = 1;
}

uint8_t SOC_EkfParamCount(void)
{
    return (uint8_t)SOC_EKF_N;
}

float SOC_EkfGetParam(uint8_t idx)
{
    switch(idx)
    {
    case SOC_EKF_Q_SOC:      return s_kf_q_soc;
    case SOC_EKF_Q_VRC:      return s_kf_q_vrc;
    case SOC_EKF_R_V:        return s_kf_r_v;
    case SOC_EKF_P0_SOC:     return s_kf_p0_soc;
    case SOC_EKF_P0_VRC:     return s_kf_p0_vrc;
    case SOC_EKF_RES_MAX_MV: return s_kf_res_max;
    case SOC_EKF_S_MIN:      return s_kf_s_min;
    case SOC_EKF_P_MIN:      return s_kf_p_min;
    default:                 return 0.0f;
    }
}

/*********************************************************************
 * @fn      SOC_EkfParamOk / SOC_EkfSetParam (后者用前者判, 再写)
 *
 * @brief   在线写一个 EKF 参数, 返回 0 = 已写入 / 1 = 被拒
 *
 * @note    只挡"明显写错"的值: 非正数 / NaN / 量级离谱。
 *          `!(v > lo && v < hi)` 这个写法把 NaN 一并挡掉 (NaN 的任何比较
 *          都是假), 所以不用 isnan, 也就不给 BMS_USE_LIBM_EXPF=0 的配置
 *          添 libm 符号。
 *          上界给得宽是有意的: 只挡量级错误 (比如 R 填成 1e30 会让卡尔曼
 *          增益恒为 0)。参数好坏不在固件里判 —— 那是一组参数跑一遍
 *          PC 端回放 (tools/test/kalman_tune.py) 的事。
 */
static uint8_t ekf_param_ok(uint8_t idx, float v)
{
    /* 索引顺序 = soc_ekf_param_t */
    static const float lo[SOC_EKF_N] = { 1e-9f,  1e-9f,  1e-9f,  1e-9f,
                                         1e-9f,  1e-6f,  1e-12f, 1e-12f };
    static const float hi[SOC_EKF_N] = { 1.0f,   1e4f,   1e6f,   1e4f,
                                         1e6f,   1e5f,   1.0f,   1e4f };

    if(idx >= (uint8_t)SOC_EKF_N) return 1;
    return (!(v > lo[idx] && v < hi[idx])) ? 1u : 0u;
}

uint8_t SOC_EkfParamOk(uint8_t idx, float v)
{
    return ekf_param_ok(idx, v);
}

uint8_t SOC_EkfSetParam(uint8_t idx, float v)
{
    if(ekf_param_ok(idx, v) != 0u) return 1;

    switch(idx)
    {
    case SOC_EKF_Q_SOC:      s_kf_q_soc   = v; break;
    case SOC_EKF_Q_VRC:      s_kf_q_vrc   = v; break;
    case SOC_EKF_R_V:        s_kf_r_v     = v; break;
    case SOC_EKF_P0_SOC:     s_kf_p0_soc  = v; break;
    case SOC_EKF_P0_VRC:     s_kf_p0_vrc  = v; break;
    case SOC_EKF_RES_MAX_MV: s_kf_res_max = v; break;
    case SOC_EKF_S_MIN:      s_kf_s_min   = v; break;
    case SOC_EKF_P_MIN:      s_kf_p_min   = v; break;
    default:                 return 1;
    }
    return 0;
}

/* =====================================================================
 * 工况折算查询实现 (bms_config.h [10])
 * ===================================================================== */

int32_t SOC_GetCorrPpm(void)
{
    /* Q16 -> ppm: ×1000000/65536。用 int64 中间量, 免得 98304×1e6 溢出 int32 */
    return (int32_t)((int64_t)s_corr_q16 * 1000000LL / SOC_Q16);
}

uint32_t SOC_GetEffectiveCapacityMAh(void)
{
    return (uint32_t)(((uint64_t)s_cap_mah * (uint64_t)s_corr_q16) >> 16);
}
