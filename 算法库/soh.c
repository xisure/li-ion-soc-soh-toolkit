/*********************************************************************************
 * File Name          : soh.c
 * Description        : SOH 在线学习 = 容量 Q 学习 + 欧姆内阻 R0 学习
 *
 * 调用: 主循环每 BMS_UPDATE_PERIOD_MS 调一次 SOH_Update(bus_mv, cur_ma, temp_dc),
 *       位置在 SOC_AhUpdate() 之后; 传感器掉线帧不调用。
 *
 * 算法与判据 1:1 复刻 ../tools/soh_learn (全量回放验证:
 * 单次 R0 测量与 PC 拟合吻合 ±1mΩ; 合成老化 -10%/-20% 的容量学习误差
 * 0.12%/0.15%)。判据常量在 bms_config.h, 改一处必须两边同步改。
 *
 * -- R0 学习 (事件触发 ΔV/ΔI) -------------------------------------
 *   状态机: 静置 --|I|>SOH_I_ON_MA--> 带载 --|I|<SOH_I_OFF_MA--> 断电首帧
 *           -> 次帧确认
 *     R0 = (V_断电首帧 - V_带载末N帧均值) / I_带载均值
 *   要点 1: 必须先判"退出带载"再决定是否累加 —— 退出帧 (I<SOH_I_OFF_MA,
 *           电压已回跳 I·R0) 绝不能计入 v_load, 否则均值被抬高 I·R0/N,
 *           R0 系统性低估约 20%。这是整个状态机最容易写错的一处。
 *   要点 2: 带载时长 < SOH_MIN_PULSE_MS 的段丢弃 (接触弹跳 / 校准尖峰)。
 *   要点 3: 只接受 SOH_TEMP_LO_C~SOH_TEMP_HI_C 的样本, 且 R0 落在
 *           SOH_R0_LO_MOHM~SOH_R0_HI_MOHM (R0 对温度敏感)。
 *   要点 4: 判据用 |ΔI|, 充/放电脉冲都参与, 但**两个方向各记一张表**:
 *           方向在进入带载段时按电流符号定 (cur>0 放电 / cur<0 充电),
 *           融合进对应方向的表, EKF 按当前电流方向取用。
 *           (修订 2026-09-14: 旧版其实也接受充电样本, 但把两个方向**混在
 *            同一格里**做指数平滑 —— 结果是两个方向都不准。现在是分开维护。)
 *   融合: 每个方向各 11 格 SOC 独立滑动更新, 自适应遗忘因子
 *         max(SOH_R0_ALPHA, 1/(n+SOH_R0_ALPHA_N0)); 前 SOH_R0_MIN_SAMPLE 个
 *         样本单次限幅 ±SOH_R0_LIMIT_RATIO, 防单帧离谱值把表带跑。
 *   单次测量噪声 σ≈0.6% (2A 阶跃 ΔV≈124mV, 电压 LSB 1.25mV)。
 *
 * -- Q 学习 (OCV 锚点法) ------------------------------------------
 *   静置 |I|<SOH_IDLE_MA 消抖后连续满 SOH_MIN_REST_MS 记一个 OCV 锚点
 *   (电压 -> SOC, 0.01%)。
 *   维护 SOC 最高/最低两个锚点, 跨度同时满足 SOH_Q_MIN_DSOC_01 与
 *   SOH_Q_MIN_DQ_MAH 时:  Q_est = ΔQ / (ΔSOC/100)
 *   融合: 标量卡尔曼滤波 (状态 Q 随机游走, 量测 Q_est 方差 R = sigma^2,
 *         正向新息把 R 放大 SOH_Q_KF_RUP 倍以压住 SOH 上跳),
 *         再做 [SOC_CAP_LO_PCT%, SOC_CAP_HI_PCT%]×标称 绝对钳位,
 *         然后 SOC_SetCapacityMAh 回写。
 *   为什么用卡尔曼而不是死区: 死区按 σ 标定, 一旦真实 σ 变小 (比如重标 OCV
 *   表之后), 死区会变成 3~6 倍真实误差的门槛 -> 滤波器直接冻结不再更新;
 *   卡尔曼在 σ 估错 4 倍时仍在跟踪, 且稳态偏差几乎与 σ 无关 (可一次性标定)。
 *   注意: OCV 表误差是绝对精度的天花板 —— 实测 Q_est 的单次散布约 ±4~5%,
 *   滤波器只能把数字做稳, 不能做准; 且这份数据无法区分"随机噪声"和
 *   "OCV 表系统偏差"。看趋势请用 ../tools/soh_learn 里的 SOH_rel = Q/Q_ref
 *   (比值里 OCV 误差抵消)。
 *
 * -- SOH ---------------------------------------------------------
 *   SOH_Q = Q_learned / Q_nom × 100   (Q_learned 已折回参考工况)
 *   SOH_R = (SOH_R_EOL_RATIO - R0_avg/R0_fresh)/(SOH_R_EOL_RATIO - 1) × 100
 *           (R0_avg 取 SOH_R_AVG_LO~SOH_R_AVG_HI 网格均值)
 *   综合 SOH = min(SOH_Q, SOH_R) —— 取劣, 与 ../tools/soh_learn 一致
 *
 * -- 累计充放电与循环计数 (bms_config.h [10]) ----------------------
 *   累计: 只在真的在充放时累加 (|I| > SOH_IDLE_MA)。静置态的电流噪声
 *         (约 ±19mA) 不进长期统计, 否则一台放了半年的机器也会"累计"
 *         出几十 Ah。注意 R0 学习仍接受 0.5A 以上的脉冲, 两者判据不同。
 *   循环: 两个口径互补 ——
 *     摆幅法半循环: SOC 自本段极值回升/回落超过 SOH_CYCLE_SWING_01 记半
 *                   个, 与电池手册的循环寿命同口径 (都按 DOD 记), 但
 *                   受 DOD 影响;
 *     等效满循环  = 累计放出电量 / 标称容量 (与 DOD 无关, 看趋势更干净)。
 *   落盘: 累计量**每帧都在变**, 若每帧置脏会把 bms_nvm 的 60s 去抖一直
 *         续期, 退化成"每 60s 一写" (每天 1440 次), 十几天磨穿 2 槽
 *         1 万次的介质。所以它自己带阈值: 总吞吐量涨够
 *         SOH_COUNT_SAVE_PCT% × 标称容量才置脏。
 *
 * 全部判据常量在 bms_config.h 的 [3]/[5]/[6]/[10], 本文件不定义可调参数。
 * 源码编码 UTF-8, 换行 LF。
 *
 * 许可: MIT (见仓库根 LICENSE)。Copyright (c) 2026 锂电池 SOC/SOH 算法库 贡献者。
 *******************************************************************************/

#include "soh.h"
#include "soc.h"        /* SOC_VoltageToSoc01 / Set|GetCapacityMAh / Set|GetR0At */
#include "bms_port.h"   /* BMS_NowMs: 平台时间戳统一出口 */
#include "bms_cal.h"    /* BMS_CAL_GetTempDc: 落盘温度取当前标定表温度 */

/* =====================================================================
 * R0 学习状态
 * ===================================================================== */
static uint8_t  s_r0_in_load;      /* 处于带载段 */
static uint8_t  s_r0_pend;         /* 已退出带载, 等断电后第一帧确认 */
static uint32_t s_r0_fcnt;         /* 带载段帧数 */
static int64_t  s_r0_isum;         /* 带载段电流累加 (mA·帧, 供求均值) */
/* 带载末 N 帧电压环形缓冲 (N = SOH_R0_TAIL_N, 见 bms_config.h [5]) */
static int32_t  s_r0_tail[SOH_R0_TAIL_N];
static uint8_t  s_r0_tail_i;
static int32_t  s_r0_pend_v;       /* 断电后第一帧电压 (mV) */
static uint8_t  s_r0_dir;          /* 本带载段的方向 (SOC_DIR_*), 进段时定 */
static uint8_t  s_r0_cnt[2][11];   /* [方向] 各格有效样本数 (自适应 α 用) */
static uint16_t s_r0_base[11];     /* 上电快照的标定基线 (SOH_R 的分母,
                                     * 用放电表; bms_cal 只有一张出厂 R0 表) */
static uint8_t  s_r0_any[2];       /* [方向] 已有至少 1 个有效 R0 样本 */

/* =====================================================================
 * 静置锚点检测状态 (Q 学习)
 * ===================================================================== */
static uint8_t  s_rest_quies;      /* |I|<SOH_IDLE_MA 连续帧数 (消抖) */
static uint8_t  s_rest_flag;       /* 已进入静置段 */
static uint32_t s_rest_since_ms;
static uint8_t  s_rest_done;       /* 本静置段已产出锚点 (每段只出一次) */

/* =====================================================================
 * Q 学习状态
 * ===================================================================== */
static int64_t  s_coul_mAms;       /* 自己的库仑累计 (mA·ms, 放电为正) */
static uint32_t s_last_ms;
static uint8_t  s_q_time_ok;       /* 时间戳已初始化 */
static uint8_t  s_q_hi_ok, s_q_lo_ok;
static int32_t  s_q_hi_soc01, s_q_lo_soc01;
static int64_t  s_q_hi_coul,  s_q_lo_coul;
/* 锚点上的工况累计量快照 (温度 / 倍率折算要用):
 *   mean|I| = (absq_hi - absq_lo) / (tms_hi - tms_lo)
 *   mean T  = (tdc_hi - tdc_lo) / (tms_hi - tms_lo)
 * 存"锚点时的累计量"而不是"区间均值", 因为学习用的 hi/lo 两个锚点不一定
 * 相邻 (中间可能夹着好几次静置), 只有两个端点的累计量差值才对应真实区间。 */
static int64_t  s_iv_absq;        /* Σ|I|·Δt (mA·ms) */
static int64_t  s_iv_tms;         /* ΣΔt (ms) */
static int64_t  s_iv_tdc;         /* ΣT·Δt (0.1C·ms) */
static int64_t  s_q_hi_absq, s_q_lo_absq;
static int64_t  s_q_hi_tms,  s_q_lo_tms;
static int64_t  s_q_hi_tdc,  s_q_lo_tdc;
static float    s_kf_p;            /* 卡尔曼: 容量估计的方差 (相对值, 无量纲) */
static uint8_t  s_q_n;             /* 已学习次数 */
static uint8_t  s_q_any;           /* 容量已至少学到 1 次 */

/* =====================================================================
 * 掉电保持: 脏标记
 * 学习成功 (R0 融合 / 容量融合) 置 1, 由 bms_nvm.c 落盘成功后清 0。
 * 不启用持久化 (BMS_USE_NVM = 0) 时它只是多一个字节的 RAM, 无副作用。
 * ===================================================================== */
static uint8_t  s_dirty;           /* 有学习结果尚未落盘 */

/* =====================================================================
 * 计数量: 累计充放电 + 循环次数 (bms_config.h [10])
 *
 * 累计器用 mA·ms 的 int64 (不直接存 mAh), 因为一个 200ms 帧在 20mA 下只有
 * 4 mA·ms = 1.1e-6 mAh, 直接以 mAh 为最小单位累计会把每一帧都截成 0。
 * ===================================================================== */
static int64_t  s_cum_chg_mAms;    /* 累计充入 (mA·ms) */
static int64_t  s_cum_dis_mAms;    /* 累计放出 (mA·ms) */
static uint32_t s_cnt_base_mah;    /* 上次落盘时的总吞吐量 (mAh) */
static uint16_t s_half_cycle;      /* 半循环计数 (摆幅法) */
static int32_t  s_cyc_hi01;        /* 摆幅法: 本段 SOC 最高点 (0.01%) */
static int32_t  s_cyc_lo01;        /* 摆幅法: 本段 SOC 最低点 */
static uint8_t  s_cyc_ok;          /* 极值已初始化 */
static uint8_t  s_cnt_any;         /* 计数有效 (至少累计过一次充放) */
static uint8_t  s_cnt_dirty;       /* 累计器涨够阈值, 待落盘 */

/* 最近一次 SOH_Update 的电芯温度 (0.1 摄氏度)。
 * 加温度维度后必须随参数一起落盘: 上电导入时要先把标定表切回这个温度,
 * 再导入那时学到的 R0, 得到的老化增量才干净 (详见 soh.h 的说明)。 */
static int16_t  s_temp_dc;

/* =====================================================================
 * 内部函数
 * ===================================================================== */

/*********************************************************************
 * @fn      soh_r0_fuse
 *
 * @brief   把一个有效 R0 测量融合进 idx 格 (自适应 α + 限幅 + 防呆)
 *
 * @param   dir     方向 (SOC_DIR_DISCHARGE / SOC_DIR_CHARGE)
 * @param   idx     SOC 网格索引 0~10 (10% 步进)
 * @param   r0      本次测量 R0 (mΩ)
 * @param   temp_c  温度 (°C, 整数)
 */
static void soh_r0_fuse(uint8_t dir, uint8_t idx, float r0, int32_t temp_c)
{
    float old, dev, cap, a_eff, lim, inv;

    if(dir > 1u)  dir = 0u;
    if(idx > 10u) return;

    if(r0 < (float)SOH_R0_LO_MOHM || r0 > (float)SOH_R0_HI_MOHM) return;
    if(temp_c < SOH_TEMP_LO_C || temp_c > SOH_TEMP_HI_C)          return;

    old = (float)SOC_GetR0DirAt(dir, idx);

    /* 自适应遗忘因子: 前几次快速收敛(≈递推平均), 之后保持 SOH_R0_ALPHA 的
     * 跟踪能力 —— 老化是慢变量, 快收敛比快跟踪重要。
     * 分母偏置 SOH_R0_ALPHA_N0 见 bms_config.h [5] (默认 2)。
     * 样本数**按方向各自计**, 两个方向互不干扰 (充电方向的样本不该让
     * 放电方向提前放开限幅)。 */
    a_eff = SOH_R0_ALPHA;
    inv   = 1.0f / (float)(s_r0_cnt[dir][idx] + SOH_R0_ALPHA_N0);
    if(inv > a_eff) a_eff = inv;

    /* 样本少时限幅 (比例见 bms_config.h [5]): 防单帧离谱值把表带跑 */
    lim = (s_r0_cnt[dir][idx] < SOH_R0_MIN_SAMPLE) ? SOH_R0_LIMIT_RATIO : 1.00f;
    dev = r0 - old;
    cap = old * lim;
    if(cap < 0.0f) cap = -cap;
    if(dev >  cap) dev =  cap;
    if(dev < -cap) dev = -cap;

    old = old + a_eff * dev;
    if(old < (float)SOH_R0_LO_MOHM) old = (float)SOH_R0_LO_MOHM;
    if(old > (float)SOH_R0_HI_MOHM) old = (float)SOH_R0_HI_MOHM;

    SOC_SetR0DirAt(dir, idx, (uint16_t)(old + 0.5f));  /* 回写: EKF 即时生效 */
    if(s_r0_cnt[dir][idx] < 255) s_r0_cnt[dir][idx]++;
    s_r0_any[dir] = 1;
    s_dirty       = 1;                          /* 有新学习结果待落盘 */
}

/*********************************************************************
 * @fn      soh_r0_frame
 *
 * @brief   R0 学习状态机 (每 BMS_UPDATE_PERIOD_MS 一帧, 与 PC learn_r0 逐帧对应)
 */
static void soh_r0_frame(int32_t bus_mv, int32_t cur_ma, int32_t temp_dc)
{
    int32_t a = (cur_ma < 0) ? -cur_ma : cur_ma;
    int32_t v_load, i_avg, dv;
    float   r0;
    uint8_t idx, i;

    if(!s_r0_in_load && !s_r0_pend)
    {
        if(a > SOH_I_ON_MA)
        {
            /* 进入带载段。方向在这里定一次, 整段沿用 —— 段中途若有电流换向
             * (HPPC 的双向脉冲串), 用段的起点符号比逐帧取符号稳:
             * 段内符号抖动会让同一次测量被写进两张表。 */
            s_r0_in_load = 1;
            s_r0_dir     = (cur_ma > 0) ? SOC_DIR_DISCHARGE : SOC_DIR_CHARGE;
            s_r0_fcnt    = 0;
            s_r0_isum    = 0;
            s_r0_tail_i  = 0;
            for(i = 0; i < SOH_R0_TAIL_N; i++) s_r0_tail[i] = 0;
        }
    }
    else if(s_r0_in_load)
    {
        /* 先判退出再累加: 退出帧电压已回跳 I·R0, 若计入 v_load 会把均值
         * 抬高 I·R0/5, R0 被系统性低估约 20% (见文件头"要点 1") */
        if(a < SOH_I_OFF_MA)
        {
            s_r0_in_load = 0;
            s_r0_pend    = 1;
            s_r0_pend_v  = bus_mv;      /* 断电后第一帧电压 */
        }
        else
        {
            s_r0_fcnt++;
            s_r0_isum += cur_ma;
            s_r0_tail[s_r0_tail_i] = bus_mv;
            s_r0_tail_i = (uint8_t)((s_r0_tail_i + 1) % SOH_R0_TAIL_N);
        }
    }
    else    /* s_r0_pend: 次帧确认电流仍低才算真的断开 (滤关断瞬态/毛刺) */
    {
        if(a < SOH_I_OFF_MA)
        {
            /* 带载段够长才用: 帧数 × 调用周期 >= SOH_MIN_PULSE_MS
             * (用 BMS_UPDATE_PERIOD_MS 而不是写死的 200, 改周期自动跟随) */
            if(s_r0_fcnt > 0 && (s_r0_fcnt * BMS_UPDATE_PERIOD_MS) >= SOH_MIN_PULSE_MS)
            {
                v_load = 0;
                for(i = 0; i < SOH_R0_TAIL_N; i++) v_load += s_r0_tail[i];
                v_load /= SOH_R0_TAIL_N;
                i_avg  = (int32_t)(s_r0_isum / (int64_t)s_r0_fcnt);
                if(i_avg != 0)
                {
                    dv = s_r0_pend_v - v_load;               /* mV */
                    r0 = (float)dv * 1000.0f / (float)i_avg; /* mV/mA = mΩ */
                    idx = (uint8_t)(SOC_VoltageToSoc01(s_r0_pend_v) / 1000);
                    if(idx > 10) idx = 10;
                    soh_r0_fuse(s_r0_dir, idx, r0, temp_dc / 10);  /* 0.1°C -> °C */
                }
            }
        }
        s_r0_pend = 0;
    }
}

/*********************************************************************
 * @fn      soh_anchor
 *
 * @brief   一个 OCV 静置锚点 (静置满 SOH_MIN_REST_MS): 更新 hi/lo 锚点, 跨度够就学容量
 */
static void soh_anchor(int32_t bus_mv)
{
    int32_t soc01 = SOC_VoltageToSoc01(bus_mv);
    int64_t coul  = s_coul_mAms;
    int32_t dsoc01;
    int64_t dcoul;
    float   dq_mah, q_est, q, d, sig, R, K, q_f, p_new;
    float   mean_i, mean_tdc, f_k;
    int64_t d_absq, d_tms, d_tdc;
    int32_t q_new;

    /* 维护 SOC 最高 / 最低两个锚点。连同当时的三项工况累计量一起存 ——
     * 学容量时要用"两个锚点之间"的平均电流与平均温度做折算。 */
    if(!s_q_hi_ok || soc01 > s_q_hi_soc01)
    {
        s_q_hi_ok = 1; s_q_hi_soc01 = soc01; s_q_hi_coul = coul;
        s_q_hi_absq = s_iv_absq; s_q_hi_tms = s_iv_tms; s_q_hi_tdc = s_iv_tdc;
    }
    if(!s_q_lo_ok || soc01 < s_q_lo_soc01)
    {
        s_q_lo_ok = 1; s_q_lo_soc01 = soc01; s_q_lo_coul = coul;
        s_q_lo_absq = s_iv_absq; s_q_lo_tms = s_iv_tms; s_q_lo_tdc = s_iv_tdc;
    }
    if(!s_q_hi_ok || !s_q_lo_ok) return;

    dsoc01 = s_q_hi_soc01 - s_q_lo_soc01;
    dcoul  = s_q_lo_coul - s_q_hi_coul;          /* 放电为正 */
    if(dsoc01 < SOH_Q_MIN_DSOC_01 || dcoul <= 0) return;

    dq_mah = (float)dcoul / 3600000.0f;          /* mA·ms -> mAh */
    if(dq_mah < (float)SOH_Q_MIN_DQ_MAH) return;

    q_est = dq_mah * 10000.0f / (float)dsoc01;   /* Q = ΔQ / (ΔSOC/10000) */

    /* ---- 折回参考工况 (bms_config.h [10]) ----
     * 上面这个 Q_est 是在**本次试验的工况下**测出来的可用容量: 低温可用
     * 容量变小、大倍率下 Peukert 效应也让可用容量变小, 两者都让 Q_est 偏小。
     * 若直接学进去, 一次冬天或大电流的试验就被当成"电池老化了"永久留在
     * SOH 里 —— 而且再也回不来 (卡尔曼只往下学得快, 往上被压)。
     * 所以先按区间平均温度与平均电流算出折算系数 K = f_T × f_I, 再除回去:
     *     Q_ref = Q_est / (f_T × f_I)
     * 默认参数 (全 100% / k = 1) 下 K ≡ 1, 与不折算逐位相同。
     *
     * 区间平均值用两个锚点上的累计量差值算。为什么存"锚点时的累计量"
     * 而不是"区间均值": hi/lo 两个锚点未必相邻 (中间可能夹着好几次静置),
     * 只有两个端点的累计量差值才对应真实的那一段。 */
    d_tms = s_q_hi_tms - s_q_lo_tms;
    if(d_tms >= 0)
    {
        d_absq = s_q_hi_absq - s_q_lo_absq;
        d_tdc  = s_q_hi_tdc  - s_q_lo_tdc;
    }
    else
    {
        d_tms  = -d_tms;
        d_absq = s_q_lo_absq - s_q_hi_absq;
        d_tdc  = s_q_lo_tdc  - s_q_hi_tdc;
    }
    if(d_tms > 0)
    {
        mean_i   = (float)d_absq / (float)d_tms;   /* 区间平均 |I| (mA) */
        mean_tdc = (float)d_tdc  / (float)d_tms;   /* 区间平均温度 (0.1C) */
    }
    else
    {
        mean_i   = 0.0f;
        mean_tdc = (float)BMS_CAL_GetRefTempDc();  /* 退化: 当成参考温度 */
    }
    f_k = BMS_CAL_TempFactorF((int16_t)((mean_tdc >= 0.0f)
                                        ? (mean_tdc + 0.5f) : (mean_tdc - 0.5f)))
        * BMS_CAL_RateFactorF((int32_t)(mean_i + 0.5f));
    if(f_k < SOC_CORR_MIN) f_k = SOC_CORR_MIN;
    if(f_k > SOC_CORR_MAX) f_k = SOC_CORR_MAX;
    if(f_k > 0.0f) q_est /= f_k;

    q = (float)SOC_GetCapacityMAh();
    /* ---- 标量卡尔曼融合 ----
     *   sigma = SOH_Q_SIG_K / ΔSOC(%)   (相对值, 小数)
     *   R     = sigma^2                  (量测噪声方差, 随跨度缩放)
     *   R_eff = R × SOH_Q_KF_RUP         (仅新息为正时 -> 压住 SOH 上跳)
     *   K     = P / (P + R_eff)           (卡尔曼增益)
     *   Q    += K × 新息;  P = (1-K)P + SOH_Q_KF_Q^2
     * P / R 都用"相对容量"单位 (与 Q 归一化同构), 所以 K 无量纲, 新息用 mAh 即可。
     * 单次最多动多少由 K 自然决定, 不需要额外的限幅常数。 */
    sig = SOH_Q_SIG_K / (dsoc01 / 100.0f) / 100.0f;   /* 相对 sigma (小数) */
    R = sig * sig;
    d = q_est - q;                                    /* 新息 (mAh) */
    if(d > 0.0f)
    {
        R *= SOH_Q_KF_RUP;                            /* 正向新息更不可信 */
    }
    K = s_kf_p / (s_kf_p + R);
    q_f   = q + K * d;
    p_new = (1.0f - K) * s_kf_p + SOH_Q_KF_Q * SOH_Q_KF_Q;

    q_new = (int32_t)(q_f + 0.5f);
    /* 绝对范围钳位: [SOC_CAP_LO_PCT%, SOC_CAP_HI_PCT%] × 标称 (bms_config.h [6]) */
    if(q_new < (int32_t)(SOC_CAPACITY_NOMINAL_MAH * SOC_CAP_LO_PCT / 100))
    {
        q_new = (int32_t)(SOC_CAPACITY_NOMINAL_MAH * SOC_CAP_LO_PCT / 100);
    }
    if(q_new > (int32_t)(SOC_CAPACITY_NOMINAL_MAH * SOC_CAP_HI_PCT / 100))
    {
        q_new = (int32_t)(SOC_CAPACITY_NOMINAL_MAH * SOC_CAP_HI_PCT / 100);
    }

    SOC_SetCapacityMAh((uint32_t)q_new);   /* 回写: 安时积分/EKF 立刻按新容量换算 */
    s_kf_p = p_new;                        /* 卡尔曼协方差递推 */
    if(s_q_n < 255) s_q_n++;
    s_q_any = 1;
    s_dirty = 1;                           /* 有新学习结果待落盘 */

    /* 重置: 以当前锚点重新累积跨度 (工况累计量也要跟着走, 否则下一次
     * 学习会把这一次的路程也算进"区间平均") */
    s_q_hi_soc01 = soc01; s_q_hi_coul = coul;
    s_q_lo_soc01 = soc01; s_q_lo_coul = coul;
    s_q_hi_absq = s_iv_absq; s_q_hi_tms = s_iv_tms; s_q_hi_tdc = s_iv_tdc;
    s_q_lo_absq = s_iv_absq; s_q_lo_tms = s_iv_tms; s_q_lo_tdc = s_iv_tdc;
}

/* SOH_Q (%), 浮点 */
static float soh_q_pct(void)
{
    return (float)SOC_GetCapacityMAh() / (float)SOC_CAPACITY_NOMINAL_MAH * 100.0f;
}

/* SOH_R (%), 浮点: 10~90% 网格均值 / 标定基线, 内阻翻倍 -> 0% */
static float soh_r_pct(void)
{
    uint32_t sum = 0, base = 0;
    uint8_t  i;
    float    ratio;

    for(i = SOH_R_AVG_LO; i <= SOH_R_AVG_HI; i++)
    {
        sum  += SOC_GetR0At(i);
        base += s_r0_base[i];
    }
    if(base == 0) return 100.0f;

    ratio = (float)sum / (float)base;
    return (SOH_R_EOL_RATIO - ratio) / (SOH_R_EOL_RATIO - 1.0f) * 100.0f;
}

/* 充电方向的 SOH_R (%), 公式与 soh_r_pct 完全相同; **分母共用同一张
 * 出厂基线** (bms_cal 只有一张 R0 表, 出厂时没分方向, 所以刚出厂时它
 * 也是 100%)。它的用途是看**趋势**: 老化会让它往下走, 若它掉得比
 * SOH_R 快, 说明充电方向的内阻恶化更快。 */
static float soh_r_chg_pct(void)
{
    uint32_t sum = 0, base = 0;
    uint8_t  i;
    float    ratio;

    for(i = SOH_R_AVG_LO; i <= SOH_R_AVG_HI; i++)
    {
        sum  += SOC_GetR0ChgAt(i);
        base += s_r0_base[i];
    }
    if(base == 0) return 100.0f;

    ratio = (float)sum / (float)base;
    return (SOH_R_EOL_RATIO - ratio) / (SOH_R_EOL_RATIO - 1.0f) * 100.0f;
}

static float soh_clamp100(float v)
{
    if(v < 0.0f)   v = 0.0f;
    if(v > 100.0f) v = 100.0f;
    return v;
}

/*********************************************************************
 * @fn      soh_count_track
 *
 * @brief   摆幅法半循环计数 + 累计量落盘阈值 (bms_config.h [10])
 *
 * @note    落盘阈值为什么必须单独设: 累计量**每一帧都在变**。若跟着
 *          "有变化就置脏"走, bms_nvm 的 60s 去抖会被一直续期, 退化成
 *          "每 60s 落盘一次" (每天 1440 次擦写), 2 槽 1 万次的介质十几天
 *          就磨穿了。这里只在总吞吐量涨够 SOH_COUNT_SAVE_PCT% × 标称
 *          容量时才置脏 —— 3350mAh 的电池约 33.5 Ah 才写一次。
 */
static void soh_count_track(void)
{
    int32_t  soc01 = SOC_GetPercent01();
    uint32_t tot   = (uint32_t)((s_cum_chg_mAms + s_cum_dis_mAms) / 3600000LL);
    uint32_t step  = (uint32_t)((uint32_t)SOC_CAPACITY_NOMINAL_MAH
                                * (uint32_t)SOH_COUNT_SAVE_PCT / 100u);

    /* ---- 摆幅法半循环 ----
     * 维护"自上次计数以来的 SOC 极值", 摆幅够大就记半个, 再从当前点重新
     * 起算。上下各走一趟 = 两个半循环 = 一个满循环。 */
    if(!s_cyc_ok)
    {
        s_cyc_hi01 = soc01;
        s_cyc_lo01 = soc01;
        s_cyc_ok   = 1;
    }
    else
    {
        if(soc01 > s_cyc_hi01) s_cyc_hi01 = soc01;
        if(soc01 < s_cyc_lo01) s_cyc_lo01 = soc01;
        if((s_cyc_hi01 - s_cyc_lo01) >= SOH_CYCLE_SWING_01)
        {
            if(s_half_cycle < 65535u) s_half_cycle++;
            s_cyc_hi01 = soc01;      /* 从当前点重新起算 */
            s_cyc_lo01 = soc01;
            s_cnt_dirty = 1;
        }
    }

    /* ---- 累计量落盘阈值 ---- */
    if(step > 0u && tot >= (s_cnt_base_mah + step)) s_cnt_dirty = 1;
}

/* =====================================================================
 * 公开接口
 * ===================================================================== */

void SOH_Init(void)
{
    uint8_t i;

    for(i = 0; i < 11; i++)
    {
        s_r0_base[i]   = SOC_GetR0At(i); /* 学习前快照 = 标定表 (SOH_R 的分母) */
        s_r0_cnt[0][i] = 0;
        s_r0_cnt[1][i] = 0;
    }
    s_r0_in_load = 0;
    s_r0_pend    = 0;
    s_r0_fcnt    = 0;
    s_r0_isum    = 0;
    s_r0_tail_i  = 0;
    s_r0_dir     = SOC_DIR_DISCHARGE;
    s_r0_any[0]  = 0;
    s_r0_any[1]  = 0;

    s_rest_quies = 0;
    s_rest_flag  = 0;
    s_rest_done  = 0;

    s_coul_mAms  = 0;
    s_last_ms    = BMS_NowMs();
    s_q_time_ok  = 0;
    s_q_hi_ok    = 0;
    s_q_lo_ok    = 0;
    s_kf_p       = SOH_Q_KF_P0 * SOH_Q_KF_P0;   /* 卡尔曼协方差初值 */
    s_q_n        = 0;
    s_q_any      = 0;

    s_iv_absq    = 0;
    s_iv_tms     = 0;
    s_iv_tdc     = 0;

    s_cum_chg_mAms = 0;
    s_cum_dis_mAms = 0;
    s_cnt_base_mah = 0;
    s_half_cycle   = 0;
    s_cyc_hi01     = 0;
    s_cyc_lo01     = 0;
    s_cyc_ok       = 0;
    s_cnt_any      = 0;
    s_cnt_dirty    = 0;
    s_temp_dc    = BMS_CAL_GetTempDc();  /* 与标定表当前温度对齐; 首次
                                          * SOH_Update 后会被真实值覆盖 */
    s_dirty      = 0;
}

void SOH_Update(int32_t bus_mv, int32_t cur_ma, int32_t temp_dc)
{
    uint32_t now = BMS_NowMs();
    int32_t  dt_ms;
    int32_t  a = (cur_ma < 0) ? -cur_ma : cur_ma;

    /* 记下当前温度, 落盘时一起存 —— 上电导入要靠它把标定表切回同一温度,
     * 否则 R0 的老化增量会混进"两次不同温度"的基准差。 */
    s_temp_dc = (int16_t)temp_dc;

    if(!s_q_time_ok)
    {
        s_last_ms   = now;
        s_q_time_ok = 1;
    }

    /* ---- 库仑累计 (mA·ms, 放电为正; 独立于 soc.c 的累加器) ---- */
    dt_ms = (int32_t)(now - s_last_ms);
    if(dt_ms < 0 || dt_ms > BMS_DT_MAX_MS) dt_ms = BMS_UPDATE_PERIOD_MS;  /* 异常间隔兜底 */
    s_last_ms = now;
    s_coul_mAms += (int64_t)cur_ma * dt_ms;

    /* ---- 锚点工况累计 (学容量时折算用): Σ|I|·Δt / ΣΔt / ΣT·Δt ---- */
    s_iv_absq += (int64_t)a * dt_ms;
    s_iv_tms  += (int64_t)dt_ms;
    s_iv_tdc  += (int64_t)temp_dc * dt_ms;

    /* ---- 累计充放电 (bms_config.h [10]) ----
     * 只在真的在充放时累计: 静置的电流噪声不进长期统计, 否则一台放了
     * 半年的机器也会"累计"出几十 Ah。阈值用 SOH_IDLE_MA (与静置判据同
     * 一个常量), 不是 R0 学习的 SOH_I_ON_MA —— 前者是"算不算在充放",
     * 后者是"够不够格测内阻", 语义不同。 */
    if(cur_ma > SOH_IDLE_MA)
    {
        s_cum_dis_mAms += (int64_t)cur_ma * dt_ms;
        s_cnt_any       = 1;
    }
    else if(cur_ma < -SOH_IDLE_MA)
    {
        s_cum_chg_mAms += (-(int64_t)cur_ma) * dt_ms;
        s_cnt_any       = 1;
    }

    /* ---- 静置锚点检测: |I|<SOH_IDLE_MA 消抖通过后计时满 SOH_MIN_REST_MS 出一个锚点 ---- */
    if(a < SOH_IDLE_MA)
    {
        if(s_rest_quies < SOH_IDLE_DEBOUNCE) s_rest_quies++;
    }
    else
    {
        s_rest_quies = 0;              /* 任何一次毛刺即清零 (同 soc.c) */
    }

    if(s_rest_quies >= SOH_IDLE_DEBOUNCE)
    {
        if(!s_rest_flag)
        {
            s_rest_flag     = 1;
            s_rest_since_ms = now;
            s_rest_done     = 0;
        }
        else if(!s_rest_done &&
                (uint32_t)(now - s_rest_since_ms) >= SOH_MIN_REST_MS)
        {
            s_rest_done = 1;
            soh_anchor(bus_mv);        /* 静置末 OCV 锚点 -> 容量学习 */
        }
    }
    else
    {
        s_rest_flag = 0;
        s_rest_done = 0;
    }

    /* ---- 累计充放电的循环计数与落盘阈值 (bms_config.h [10]) ---- */
    soh_count_track();

    /* ---- R0 带载脉冲状态机 ---- */
    soh_r0_frame(bus_mv, cur_ma, temp_dc);
}

int32_t SOH_GetPercent01(void)
{
    float q = soh_clamp100(soh_q_pct());
    float r = soh_clamp100(soh_r_pct());
    float m = (q < r) ? q : r;          /* 取劣 */

    return (int32_t)(m * 100.0f + 0.5f);
}

uint8_t SOH_GetPercent(void)
{
    return (uint8_t)((SOH_GetPercent01() + 50) / 100);
}

uint8_t SOH_GetCapacityPercent(void)
{
    return (uint8_t)(soh_clamp100(soh_q_pct()) + 0.5f);
}

uint8_t SOH_GetResistancePercent(void)
{
    return (uint8_t)(soh_clamp100(soh_r_pct()) + 0.5f);
}

uint32_t SOH_GetLearnedCapacityMAh(void)
{
    return SOC_GetCapacityMAh();
}

uint16_t SOH_GetR0AvgMOhm(void)
{
    uint32_t sum = 0;
    uint8_t  i;

    for(i = SOH_R_AVG_LO; i <= SOH_R_AVG_HI; i++) sum += SOC_GetR0At(i);
    return (uint16_t)((sum + SOH_R_AVG_N / 2) / SOH_R_AVG_N);
}

uint8_t SOH_IsCapacityValid(void)
{
    return s_q_any;
}

uint8_t SOH_IsR0Valid(void)
{
    return s_r0_any[SOC_DIR_DISCHARGE];   /* 放电方向: SOH_R 的门槛 */
}

uint16_t SOH_GetR0ChgAvgMOhm(void)
{
    uint32_t sum = 0;
    uint8_t  i;

    for(i = SOH_R_AVG_LO; i <= SOH_R_AVG_HI; i++) sum += SOC_GetR0ChgAt(i);
    return (uint16_t)((sum + SOH_R_AVG_N / 2) / SOH_R_AVG_N);
}

uint8_t SOH_IsR0ChgValid(void)
{
    return s_r0_any[SOC_DIR_CHARGE];
}

uint8_t SOH_GetResistancePercentChg(void)
{
    return (uint8_t)(soh_clamp100(soh_r_chg_pct()) + 0.5f);
}

uint32_t SOH_GetCumDischargeMAh(void)
{
    return (uint32_t)(s_cum_dis_mAms / 3600000LL);
}

uint32_t SOH_GetCumChargeMAh(void)
{
    return (uint32_t)(s_cum_chg_mAms / 3600000LL);
}

uint16_t SOH_GetHalfCycleCount(void)
{
    return s_half_cycle;
}

/* 等效满循环 × 1000 = 累计放出 / 标称容量。先乘 1000 再除, 保留
 * 0.001 个循环的分辨率 (3350mAh 一格约合 3.35 mAh)。 */
uint32_t SOH_GetCycleMilli(void)
{
    if(SOC_CAPACITY_NOMINAL_MAH == 0u) return 0u;
    return (uint32_t)((s_cum_dis_mAms * 1000LL)
                      / (3600000LL * (int64_t)SOC_CAPACITY_NOMINAL_MAH));
}

uint8_t SOH_IsCountValid(void)
{
    return s_cnt_any;
}

void SOH_ResetCounters(void)
{
    s_cum_chg_mAms = 0;
    s_cum_dis_mAms = 0;
    s_half_cycle   = 0;
    s_cyc_hi01     = 0;
    s_cyc_lo01     = 0;
    s_cyc_ok       = 0;      /* 极值从当前 SOC 重新起算 */
    s_cnt_any      = 0;
    s_cnt_base_mah = 0;
    s_cnt_dirty    = 1;      /* 归零本身也是要落盘的状态, 否则重启又回来 */
}

/* =====================================================================
 * 掉电保持支撑接口 (bms_nvm.c 调用)
 *
 * 导出/导入的是"学习结果 + 续接滤波所需的状态":
 *   R0 表 (学习值) / base (SOH_R 的分母) / cnt (自适应 α 的样本数)
 *   容量 Q / q_n / 两个 valid 标志 / 卡尔曼协方差 P
 * 不导出的是"瞬态量"(静置计时、库仑累加、带载段状态) —— 那些上电重来。
 * ===================================================================== */

void SOH_ExportParam(soh_param_t *p)
{
    uint8_t i;

    p->cap_mah = SOC_GetCapacityMAh();
    for(i = 0; i < 11u; i++)
    {
        p->r0[i]      = SOC_GetR0At(i);        /* 放电方向 */
        p->r0_chg[i]  = SOC_GetR0ChgAt(i);     /* 充电方向 */
        p->base[i]    = s_r0_base[i];
        p->cnt[i]     = s_r0_cnt[SOC_DIR_DISCHARGE][i];
        p->cnt_chg[i] = s_r0_cnt[SOC_DIR_CHARGE][i];
    }
    p->q_n        = s_q_n;
    p->r0_any     = s_r0_any[SOC_DIR_DISCHARGE];
    p->r0_chg_any = s_r0_any[SOC_DIR_CHARGE];
    p->q_any      = s_q_any;
    p->temp_dc    = s_temp_dc;   /* 落盘温度, 上电导入时用它切标定表 (见 soh.h) */
    p->kf_p       = s_kf_p;

    /* ---- ver 3 新增: 累计量与循环计数 (只存整 mAh, 亚 mAh 零头丢掉) ---- */
    p->cnt_any     = s_cnt_any;
    p->pad0        = 0u;
    p->cum_chg_mah = (uint32_t)(s_cum_chg_mAms / 3600000LL);
    p->cum_dis_mah = (uint32_t)(s_cum_dis_mAms / 3600000LL);
    p->half_cycle  = s_half_cycle;
    p->pad1        = 0u;
    p->pad2        = 0u;
}

void SOH_ImportParam(const soh_param_t *p)
{
    uint8_t i;

    /* 先恢复 R0 表与基线, 再写容量: 容量写入会冻结积分基准
     * (见 SOC_SetCapacityMAh), 排在最后更直观。
     * 上电时调用是安全的 —— 那时 SOC 尚未建立基准, 直接赋值不会跳变。 */
    for(i = 0; i < 11u; i++)
    {
        SOC_SetR0DirAt(SOC_DIR_DISCHARGE, i, p->r0[i]);     /* 内部防呆钳位 */
        SOC_SetR0DirAt(SOC_DIR_CHARGE,    i, p->r0_chg[i]);
        s_r0_base[i]                   = p->base[i];
        s_r0_cnt[SOC_DIR_DISCHARGE][i] = p->cnt[i];
        s_r0_cnt[SOC_DIR_CHARGE][i]    = p->cnt_chg[i];
    }
    SOC_SetCapacityMAh(p->cap_mah);

    s_q_n     = p->q_n;
    s_r0_any[SOC_DIR_DISCHARGE] = p->r0_any;
    s_r0_any[SOC_DIR_CHARGE]    = p->r0_chg_any;
    s_q_any   = p->q_any;
    s_temp_dc = p->temp_dc;
    s_kf_p    = p->kf_p;

    /* 累计量: 只恢复整 mAh 的部分 —— 它本来就是统计量, 零头无所谓。
     * s_cnt_base_mah 必须跟着调到"当前总吞吐量", 否则上电瞬间就会
     * 因为 base=0 而立刻置脏白写一次。 */
    s_cum_chg_mAms = (int64_t)p->cum_chg_mah * 3600000LL;
    s_cum_dis_mAms = (int64_t)p->cum_dis_mah * 3600000LL;
    s_half_cycle   = p->half_cycle;
    s_cnt_any      = p->cnt_any;
    s_cyc_ok       = 0;    /* 摆幅极值从当前 SOC 重新起算 */
    s_cnt_base_mah = (uint32_t)((s_cum_chg_mAms + s_cum_dis_mAms) / 3600000LL);
    s_cnt_dirty    = 0;

    s_dirty  = 0;      /* 刚从介质恢复, 本身就不需要再落盘 */
}

uint8_t SOH_IsDirty(void)
{
    /* 学习结果 (s_dirty) 与累计量 (s_cnt_dirty) 各自置脏, 任一为真就要落盘。
     * 注意 s_dirty 是"有新学习结果", s_cnt_dirty 是"累计量涨够阈值" ——
     * 后者不每帧置脏, 否则会把 bms_nvm 的 60s 去抖退化成每 60s 一写。 */
    return (uint8_t)(s_dirty || s_cnt_dirty);
}

void SOH_ClearDirty(void)
{
    s_dirty     = 0;
    s_cnt_dirty = 0;
    /* 落盘成功 = 介质里的累计量与当前一致, 阈值基准跟着往前挪 */
    s_cnt_base_mah = (uint32_t)((s_cum_chg_mAms + s_cum_dis_mAms) / 3600000LL);
}
