#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
soh_learn.py - SOH 在线学习离线验证器 (Q 容量 + R0 欧姆内阻)

把学习算法搬进 MCU 的 soh.c 之前, 用 stair 实测数据回放验证精度。
判据与 soc_bench.py 的 find_pulses/analyze_seg/smooth_abs 保持一致:
    R0 = (断电后第一静置帧电压 - 带载末 5 帧均值电压) / 带载电流均值
    Q  = 两个 OCV 锚点之间的库仑计数 / OCV 查表得到的 ΔSOC

关键: 这里是"在线/因果"复刻 —— 只用当前帧及历史帧, 不用未来数据,
与 MCU 每 200ms 跑一次的状态机完全一致。soc_bench.py 的 stair 是事后全局
切段, 可以用未来数据; 本脚本刻意不用, 否则验证结果偏乐观。

--scale 合成老化: 按 scale 缩小真实容量重算 SOC 轨迹(电压随 OCV 表走),
验证算法能否学到衰减后的容量 —— 否则"学到 3350"可能只是自证。

用法:
    python soh_learn.py --csv stair1.csv
    python soh_learn.py --csv stair1.csv --scale 0.80     # 模拟容量衰减到 80%
    python soh_learn.py --csv stair1.csv --i-on 300       # 放宽电流阶跃门限
    python soh_learn.py --csv stair1.csv --emit-c         # 输出学习后的 C 数组
    python soh_learn.py --csv stair1.csv --plot           # 出图
"""

import argparse
import csv
import os
import sys

import numpy as np

# ================= 与 MCU soc.c / soh.c 必须同步的常数 =================
OCV_MV = [3050, 3188, 3375, 3524, 3629, 3748, 3845, 3938, 4027, 4096, 4161]
R0_CAL = [70, 69, 63, 60, 61, 61, 61, 61, 60, 62, 62]
R1_CAL = [70, 72, 70, 49, 38, 39, 38, 35, 36, 40, 40]
TAU_S = [36, 44, 109, 99, 117, 125, 109, 83, 78, 71, 70]
Q_NOM_MAH = 3350

# ---- 学习判据 (改这里必须同步改固件 ../firmware/bms_config.h) ----
I_ON_MA = 500        # 进入带载滞回上门限
I_OFF_MA = 200       # 退出带载滞回下门限
MIN_PULSE_S = 5.0    # 带载段最短时长 (滤毛刺)
IDLE_MA = 20         # 静置电流门限
IDLE_DEBOUNCE = 5    # 静置消抖帧数 (与 soc.c 一致)
MIN_REST_S = 600     # 锚点所需静置时长 (stair 档间实测 600s;
                     #   MCU 复用 soc.c 的 900s 重同步事件)
Q_MIN_DSOC = 15.0    # 触发容量学习的最小 ΔSOC (%)
Q_MIN_DQ = 200.0     # 触发容量学习的最小 ΔQ (mAh)
# ---- Q 融合: 标量卡尔曼滤波 ----
# 状态 Q 随机游走; 量测 Q_est 方差 R = sigma^2 (随跨度缩放);
# 正向新息把 R 放大 Q_KF_RUP 倍 -> SOH 上升被自然压住。
# 相比死区方案的好处: 没有"死区/阻尼/限幅"三个手调常数之间的耦合,
# 而且 OCV 表重标后 sigma 变小, 滤波器自动变灵敏, 不用重调参。
#
# 单次测量的相对 sigma: sigma_Q(%) = Q_SIG_K / ΔSOC(%)  (由 stair1.csv 标定)
#   注: 该数据无法区分"随机噪声"和"OCV 表系统偏差"(两者严格简并, 最小二乘
#   残差恒为 0), 所以这个 sigma 模型只是可用的工程近似, 不是统计意义上的
#   无偏估计。换个电芯/重标 OCV 表后建议重新标定 Q_SIG_K。
Q_SIG_K = 126.0      # sigma_Q(%) = Q_SIG_K / ΔSOC(%);  30% -> 4.2%
Q_KF_P0 = 0.02       # Q 初值相对不确定度 (标称 3350 可信到 ±2%)
Q_KF_Q  = 0.002      # 过程噪声: 两次测量之间容量可能的漂移 (相对值)
Q_KF_RUP = 8.0       # 正向新息的量测噪声放大倍数 (抑制 SOH 上跳)
                     # 参数扫描结果 (40 次 x 2500 组, 跨度 30~80% 随机):
                     #   上跳 0.23%  下跳 1.01%  稳态波动 0.71%
                     #   稳态偏差 -1.60%  老化滞后 +0.72%
                     # (上跳 <=0.30% 的候选里, 这是 |偏差| 较小的一档;
                     #  想更保守把 RUP 提到 10~15, 代价是稳态偏差变到 -1.8~-2.0%)
R0_ALPHA = 0.20      # R0 遗忘因子
R0_MIN_SAMPLE = 3    # 该格达到这个样本数后才允许超过 ±20% 偏移
R0_LO, R0_HI = 20.0, 200.0     # R0 合理范围 mΩ
TEMP_LO_C, TEMP_HI_C = 15.0, 35.0   # R0 样本温度窗口
R_EOL_RATIO = 2.0    # 内阻翻倍视为寿命终点


# ---------------- OCV 查表 (0.01% 精度, 复刻 MCU) ----------------
def soc01_from_mv(mv):
    """电压 mV -> SOC (0.01% 单位, 0~10000)。MCU 端 soh.c 需要同精度版本,
    soc.c 现有的 SOC_FromVoltage 只返回整数 %, 量化到 1% 会让 Q 学习
    在 ΔSOC=30% 时引入 3.3% 的系统误差, 不够用。"""
    if mv >= OCV_MV[10]:
        return 10000
    if mv <= OCV_MV[0]:
        return 0
    for i in range(9, -1, -1):
        if mv > OCV_MV[i]:
            return int(i * 1000 + (mv - OCV_MV[i]) * 1000 // (OCV_MV[i + 1] - OCV_MV[i]))
    return 0


def mv_from_soc01(s01):
    """SOC (0.01%) -> OCV mV, 线性插值"""
    if s01 >= 10000:
        return float(OCV_MV[10])
    if s01 <= 0:
        return float(OCV_MV[0])
    i = int(s01 // 1000)
    if i > 9:
        i = 9
    f = (s01 - i * 1000) / 1000.0
    return OCV_MV[i] + f * (OCV_MV[i + 1] - OCV_MV[i])


def load_csv(path):
    ts, vs, iv, tt, sc = [], [], [], [], []
    with open(path, newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            try:
                ts.append(float(row["elapsed_s"]))
                vs.append(float(row["V_mV"]))
                iv.append(float(row["I_mA"]))
            except (KeyError, ValueError, TypeError):
                continue
            try:
                tt.append(float(row["T_0p1C"]))
            except (KeyError, ValueError, TypeError):
                tt.append(250.0)
            try:
                sc.append(float(row["SOC_0p01"]))
            except (KeyError, ValueError, TypeError):
                sc.append(float("nan"))
    if len(ts) < 100:
        sys.exit(f"有效帧太少 ({len(ts)})")
    return (np.array(ts), np.array(vs), np.array(iv),
            np.array(tt), np.array(sc))


# =====================================================================
# R0 在线学习 (因果状态机, 1:1 对应 MCU soh.c)
# =====================================================================
def learn_r0(ts, vs, iv, tt, soc01, i_on, i_off, min_pulse_s, alpha,
             verbose=True):
    """在线 R0 学习复刻。

    状态机 (每帧一次):
      静置 --|I|>i_on--> 带载 (记 v_pre=静置末几帧均值, 累电流/帧数)
      带载 --|I|<i_off-> 待断电首帧 -> 下一帧取 v_1st 算 R0
      带载时长 < min_pulse_s 的段丢弃 (接触弹跳/INA226 校准尖峰)
    R0 = (v_1st - v_load) / i_avg,  v_load = 带载末 5 帧均值 (降噪 √5)
    """
    n = len(ts)
    v_hist = [0.0] * 8          # 最近 8 帧电压 (MCU: 环形缓冲)
    i_hist = [0.0] * 8
    in_load = False
    pend = False                # 已退出带载, 等断电后第一帧
    f_cnt = 0
    i_sum = 0.0
    v_tail5 = [0.0] * 5
    v_pre = 0.0
    quies = 0

    table = list(R0_CAL)        # 学习表, 初值 = 标定表
    cnt = [0] * 11
    samples = []                # (soc_idx, r0_meas, mv_drop, i_a, temp_c, t)
    rows = []

    for k in range(n):
        I = iv[k]
        V = vs[k]
        a = abs(I)

        # 环形缓冲 (过去 8 帧)
        v_hist[k % 8] = V
        i_hist[k % 8] = I

        # --- 静置消抖 (因果: 连续 IDLE_DEBOUNCE 帧 |I|<IDLE_MA) ---
        if a < IDLE_MA:
            quies = min(quies + 1, 255)
        else:
            quies = 0

        if not in_load and not pend:
            if a > i_on:
                # 进入带载: v_pre 取此前静置段末 4 帧均值 (消抖通过才算静置)
                if quies >= IDLE_DEBOUNCE:
                    pre = [v_hist[(k - j) % 8] for j in range(1, 5)]
                    v_pre = float(np.mean(pre))
                else:
                    v_pre = float("nan")
                in_load, f_cnt, i_sum = True, 0, 0.0
                v_tail5 = [0.0] * 5
        elif in_load:
            # 注意顺序: 先判退出, 再累加。退出帧(I<i_off, 电压已回跳 I*R0)
            # 绝不能计入 v_load, 否则 v_load 被抬高 I*R0/5, R0 被系统性低估
            # 20% —— 本脚本第一版就栽在这里 (64.4 -> 51.5 mΩ)。
            if a < i_off:
                # 退出带载: 本帧即断电后第一帧, 下一帧确认电流仍低再算 R0
                in_load = False
                pend = True
                pend_t0 = ts[k]
                pend_v = V
            else:
                f_cnt += 1
                i_sum += I
                v_tail5[f_cnt % 5] = V
        elif pend:
            # 第二帧确认: 电流仍 < i_off 才算真的断开 (滤关断瞬态/毛刺)
            if a < i_off:
                i_avg = i_sum / max(f_cnt, 1)
                v_load = float(np.mean(v_tail5))
                if f_cnt * 0.2 >= min_pulse_s and abs(i_avg) > 1e-6:
                    r0 = (pend_v - v_load) / (i_avg / 1000.0)   # mV/A = mΩ
                    dv = pend_v - v_load
                    soc_idx = int(min(max(soc01[k], 0), 9999) // 1000)
                    temp_c = tt[k] / 10.0
                    ok = (R0_LO <= r0 <= R0_HI
                          and TEMP_LO_C <= temp_c <= TEMP_HI_C)
                    samples.append((soc_idx, r0, dv, i_avg, temp_c, ts[k], ok))
                    if ok:
                        old = table[soc_idx]
                        # 自适应遗忘因子: 前几次快速收敛(≈递推平均), 之后
                        # 保持 0.10 的跟踪能力(老化是慢变量, 不需要快跟踪)
                        a_eff = max(alpha, 1.0 / (cnt[soc_idx] + 2))
                        # 样本少时限幅: 防单帧离谱值把表带跑
                        lim = 0.20 if cnt[soc_idx] < R0_MIN_SAMPLE else 1.00
                        dev = r0 - old
                        cap = abs(old) * lim
                        if dev > cap:
                            dev = cap
                        if dev < -cap:
                            dev = -cap
                        newv = old + a_eff * dev
                        table[soc_idx] = newv
                        cnt[soc_idx] += 1
                        rows.append((ts[k], soc_idx, r0, old, newv,
                                     cnt[soc_idx], dv, i_avg, temp_c))
            pend = False

    return table, cnt, samples, rows


# =====================================================================
# Q 在线学习 (OCV 锚点法)
# =====================================================================
def find_rest_anchors(ts, iv, min_rest_s):
    """因果检测静置段: 连续 |I|<IDLE_MA 达到 min_rest_s 即认为该段末尾是
    一个有效 OCV 锚点。返回 [(end_idx, start_idx), ...]"""
    n = len(ts)
    runs, s = [], None
    for k in range(n):
        if abs(iv[k]) < IDLE_MA:
            if s is None:
                s = k
        else:
            if s is not None:
                runs.append((s, k - 1))
                s = None
    if s is not None:
        runs.append((s, n - 1))
    return [(a, b) for a, b in runs if ts[b] - ts[a] >= min_rest_s]


def learn_q(ts, vs, iv, q_total_mah, anchors, min_dsoc, q0, verbose=True):
    """锚点法容量学习复刻 (标量卡尔曼滤波融合)。

    维护 hi/lo 两个锚点 (SOC 最高/最低), 跨度够就估一次 Q, 然后重置。
    Q_est = ΔQ / (ΔSOC/100),  ΔQ 取放电方向 (lo.q - hi.q > 0)
    """
    hi = None        # (soc01, q_mah)
    lo = None
    ests = []
    q = q0
    P = Q_KF_P0 ** 2      # Q 的估计方差 (卡尔曼状态)
    n_q = 0
    for (a, b) in anchors:
        # 锚点电压: 静置段末 30s 均值 (MCU 端单点采样, 这里同时给单点做对比)
        k = max(a, int(np.searchsorted(ts, ts[b] - 30.0)))
        v_ocv = float(np.mean(vs[k:b + 1]))
        soc01 = soc01_from_mv(v_ocv)
        qq = float(q_total_mah[b])

        if hi is None or soc01 > hi[0]:
            hi = (soc01, qq)
        if lo is None or soc01 < lo[0]:
            lo = (soc01, qq)

        dsoc = (hi[0] - lo[0]) / 100.0
        dq = lo[1] - hi[1]
        if dsoc >= min_dsoc and abs(dq) >= Q_MIN_DQ and dq > 0:
            q_est = dq / (dsoc / 100.0)
            # ---- 标量卡尔曼融合 ----
            #   R = sigma^2 (随跨度);  正向新息把 R 放大 Q_KF_RUP 倍
            #   K = P/(P+R_eff);  Q += K*新息;  P = (1-K)P + Q_KF_Q^2
            sig = Q_SIG_K / max(dsoc, 1.0) / 100.0    # 相对 sigma (小数)
            R = sig * sig
            d_raw = q_est - q
            R_eff = R * (Q_KF_RUP if d_raw > 0.0 else 1.0)
            K = P / (P + R_eff)
            q_new = q + K * d_raw
            P_new = (1.0 - K) * P + Q_KF_Q ** 2
            q_new = min(max(q_new, 0.5 * Q_NOM_MAH), 1.10 * Q_NOM_MAH)
            n_q += 1
            ests.append({
                "t": ts[b], "soc_hi": hi[0] / 100.0, "soc_lo": lo[0] / 100.0,
                "dsoc": dsoc, "dq": dq, "q_est": q_est, "q_new": q_new,
                "alpha": K, "n": n_q, "sig": sig * 100.0, "K": K,
                "v_hi": mv_from_soc01(hi[0]), "v_lo": mv_from_soc01(lo[0]),
            })
            if verbose:
                print(f"    t={ts[b]:8.0f}s  SOC {hi[0]/100:6.2f}% -> {lo[0]/100:6.2f}%  "
                      f"ΔSOC={dsoc:5.2f}%  ΔQ={dq:7.1f}mAh  "
                      f"Q_est={q_est:7.1f}  σ={sig*100:4.1f}%  K={K:.3f}  "
                      f"Q={q_new:7.1f}")
            q = q_new
            P = P_new
            q = q_new
            # 重置: 以当前点为新锚点, 重新累积跨度
            hi = (soc01, qq)
            lo = (soc01, qq)
    return q, ests


# =====================================================================
def main():
    ap = argparse.ArgumentParser(
        description="SOH 在线学习离线验证 (Q 容量 + R0 欧姆内阻)")
    ap.add_argument("--csv", required=True, help="soc_record.py 采集的 CSV")
    ap.add_argument("--refs", default=None,
                    help="stair1_stair.csv, 用于对比 PC 端拟合的 r0_mohm")
    ap.add_argument("--scale", type=float, default=1.0,
                    help="合成老化: 真实容量 = scale × 3350 (默认 1.0 = 新电池)")
    ap.add_argument("--i-on", type=float, default=I_ON_MA,
                    help=f"进入带载电流门限 mA (默认 {I_ON_MA})")
    ap.add_argument("--i-off", type=float, default=I_OFF_MA,
                    help=f"退出带载电流门限 mA (默认 {I_OFF_MA})")
    ap.add_argument("--min-pulse", type=float, default=MIN_PULSE_S,
                    help=f"最短带载时长 s (默认 {MIN_PULSE_S})")
    ap.add_argument("--min-dsoc", type=float, default=Q_MIN_DSOC,
                    help=f"触发容量学习的最小 ΔSOC %% (默认 {Q_MIN_DSOC})")
    ap.add_argument("--alpha-r0", type=float, default=R0_ALPHA)
    ap.add_argument("--emit-c", action="store_true", help="输出学习后的 C 数组")
    ap.add_argument("--plot", action="store_true", help="出图 (R0 vs SOC)")
    args = ap.parse_args()

    ts, vs, iv, tt, sc = load_csv(args.csv)
    fs = 1.0 / float(np.median(np.diff(ts)))
    print("=" * 74)
    print(f"SOH 在线学习离线验证   {os.path.basename(args.csv)}")
    print(f"  {len(ts)} 帧, {ts[-1]/3600:.2f} h, 采样 {fs:.1f} Hz, "
          f"容量真值 {args.scale*Q_NOM_MAH:.0f} mAh (scale={args.scale})")
    print("=" * 74)

    # ---------- 合成老化: 按 scale 重算 SOC 轨迹与电压 ----------
    soc01 = np.array([soc01_from_mv(v) for v in vs], dtype=float)
    if abs(args.scale - 1.0) > 1e-9:
        dt = np.diff(ts, prepend=ts[0])
        qc = np.cumsum(iv * dt) / 3600.0           # mAh, 放电为正
        q_true = args.scale * Q_NOM_MAH
        s0 = soc01[0] / 100.0                      # soc01 是 0.01% 单位 -> 转成 %
        soc01 = s0 - qc / q_true * 100.0           # % (0~100)
        soc01 = np.clip(soc01, 0.0, 100.0)
        # 电压: OCV(SOC) - I*R0 - Vrc (Vrc 用一阶近似, 够验证用)
        v_ocv = np.array([mv_from_soc01(x * 100) for x in soc01])
        idx = np.clip((soc01 // 10).astype(int), 0, 10)
        r0v = np.array([R0_CAL[i] for i in idx])
        r1v = np.array([R1_CAL[i] for i in idx])
        tuv = np.array([TAU_S[i] for i in idx])
        # 一阶 RC 递推
        vrc = np.zeros(len(ts))
        for k in range(1, len(ts)):
            a = np.exp(-dt[k] / max(tuv[k], 1.0))
            vrc[k] = vrc[k - 1] * a + iv[k] * r1v[k] / 1000.0 * (1 - a)
        vs = v_ocv - iv * r0v / 1000.0 - vrc
        vs = np.round(vs / 1.25) * 1.25            # VBUS LSB 1.25mV 量化
        soc01 = soc01 * 100.0                      # -> 0.01% 单位
        print(f"  [合成] 已按 scale={args.scale} 重算 SOC 轨迹与端压 "
              f"(含 R0/R1/τ 一阶模型 + 1.25mV 量化)")

    # 累计库仑 (用于 Q 学习)
    dt = np.diff(ts, prepend=ts[0])
    q_total = np.cumsum(iv * dt) / 3600.0

    # ================= R0 =================
    print("\n【1】R0 在线学习 (断电瞬跳 ΔV/ΔI, 与 soc_bench.py analyze_seg 同判据)")
    print(f"  门限: |I|>{args.i_on:.0f}mA 进入 / <{args.i_off:.0f}mA 退出, "
          f"带载>={args.min_pulse:.0f}s, 温度 {TEMP_LO_C:.0f}~{TEMP_HI_C:.0f}C, "
          f"α={args.alpha_r0}")
    table, cnt, samples, rows = learn_r0(
        ts, vs, iv, tt, soc01, args.i_on, args.i_off, args.min_pulse,
        args.alpha_r0)

    n_ok = sum(1 for s in samples if s[6])
    print(f"  检测到带载脉冲事件 {len(samples)} 次, 通过有效性检验 {n_ok} 次")
    if n_ok == 0:
        print("  [!] 无有效样本: 检查 --i-on / --min-pulse / 温度窗口")

    print(f"\n  {'SOC格':>6} {'样本':>4} {'标定':>7} {'学习值':>8} {'偏差':>8} "
          f"{'单次σ':>7} {'典型ΔV':>8}")
    print("  " + "-" * 56)
    per_grid = {}
    for s in samples:
        if s[6]:
            per_grid.setdefault(s[0], []).append(s[1])
    for i in range(11):
        arr = np.array(per_grid.get(i, []))
        if len(arr) == 0:
            print(f"  {i*10:4d}% {0:4d} {R0_CAL[i]:7d} {table[i]:8.1f} "
                  f"{'-':>8} {'-':>7} {'-':>8}")
            continue
        sd = float(np.std(arr, ddof=1)) if len(arr) > 1 else 0.0
        dvs = [s[2] for s in samples if s[6] and s[0] == i]
        print(f"  {i*10:4d}% {len(arr):4d} {R0_CAL[i]:7d} {table[i]:8.1f} "
              f"{table[i]-R0_CAL[i]:+8.1f} {sd:7.2f} {np.mean(dvs):8.1f}mV")

    # 与 PC 端 stair 拟合对比
    if args.refs and os.path.exists(args.refs):
        try:
            ref = {}
            with open(args.refs, newline="", encoding="utf-8") as f:
                for row in csv.DictReader(f):
                    try:
                        ref[int(row["seg"])] = (float(row["soc_mid"]),
                                                float(row["r0_mohm"]))
                    except (KeyError, ValueError):
                        continue
            if ref:
                print(f"\n  与 {os.path.basename(args.refs)} 的 PC 拟合 r0_mohm 对比:")
                print(f"  {'seg':>4} {'SOC_mid':>8} {'PC拟合':>8} {'在线学习':>9} {'差':>7}")
                print("  " + "-" * 44)
                for seg in sorted(ref):
                    sm, r0ref = ref[seg]
                    gi = int(min(max(sm, 0), 99.9) // 10)
                    print(f"  {seg:4d} {sm:8.2f} {r0ref:8.1f} {table[gi]:9.1f} "
                          f"{table[gi]-r0ref:+7.1f}")
        except Exception as e:
            print(f"  (refs 解析失败: {e})")

    # ================= Q =================
    print(f"\n【2】Q 容量学习 (OCV 锚点法 + 标量卡尔曼, ΔSOC>={args.min_dsoc:.0f}%, "
          f"P0={Q_KF_P0:.0%} q={Q_KF_Q:.1%} Rup={Q_KF_RUP:.0f})")
    anchors = find_rest_anchors(ts, iv, MIN_REST_S)
    print(f"  静置段 (|I|<{IDLE_MA}mA 持续>={MIN_REST_S}s): {len(anchors)} 个")
    if len(anchors) < 2:
        print("  [!] 有效锚点不足 2 个, 无法学习容量")
        q_final, ests = float(Q_NOM_MAH), []
    else:
        print("  锚点 (静置末 30s 均值 -> OCV 查表):")
        for (a, b) in anchors:
            k = max(a, int(np.searchsorted(ts, ts[b] - 30.0)))
            v = float(np.mean(vs[k:b + 1]))
            print(f"    t={ts[b]:8.0f}s  V={v:7.1f}mV  "
                  f"SOC={soc01_from_mv(v)/100:6.2f}%  Q累计={q_total[b]:8.1f}mAh")
        print("  学习触发:")
        q_final, ests = learn_q(ts, vs, iv, q_total, anchors,
                                args.min_dsoc, float(Q_NOM_MAH))
        q_true = args.scale * Q_NOM_MAH
        if ests:
            errs = np.array([e["q_est"] - q_true for e in ests])
            print(f"\n  单次 Q_est 与真值 {q_true:.0f}mAh 的偏差: "
                  f"mean {np.mean(errs):+.1f} / std {np.std(errs, ddof=1) if len(errs)>1 else 0:.1f} mAh")
        print(f"  最终 Q = {q_final:.1f} mAh  (真值 {q_true:.0f}, "
              f"误差 {q_final-q_true:+.1f} mAh = {(q_final/q_true-1)*100:+.2f}%)")

    # ================= SOH =================
    print("\n【3】SOH")
    q_true = args.scale * Q_NOM_MAH
    soh_q = q_final / Q_NOM_MAH * 100.0
    r0_now = float(np.mean([table[i] for i in range(1, 10)]))   # 10~90% 均值
    r0_fresh = float(np.mean(R0_CAL[1:10]))
    soh_r = (R_EOL_RATIO - r0_now / r0_fresh) / (R_EOL_RATIO - 1.0) * 100.0
    soh_r = min(max(soh_r, 0.0), 100.0)
    print(f"  SOH_Q (容量)   = {q_final:.0f}/{Q_NOM_MAH} = {soh_q:6.2f}%   "
          f"(真值 {q_true/Q_NOM_MAH*100:.2f}%)")
    print(f"  SOH_R (内阻)   = R0 {r0_now:.1f}/{r0_fresh:.1f} mΩ "
          f"(EOL=×{R_EOL_RATIO}) = {soh_r:6.2f}%")
    print(f"  SOH (取劣)     = {min(soh_q, soh_r):6.2f}%")
    if ests:
        q_ref = ests[0]["q_new"]
        soh_rel = q_final / q_ref * 100.0
        print(f"\n  SOH_rel (趋势) = Q/Q_ref = {q_final:.0f}/{q_ref:.0f} = {soh_rel:6.2f}%")
        print("    ↑ Q_ref = 首次学到的容量。OCV 表误差/电流增益误差在比值里抵消,")
        print("      只看老化趋势时用它, 绝对值用上面 SOH_Q (需先标定好 OCV 表)。")

    # ================= 输出 =================
    if args.emit_c:
        print("\n【4】C 数组 (替换 soc.c 的 SOC_R0_MOHM)")
        print("static uint16_t s_r0_mohm[11] = {")
        for i in range(11):
            print(f"    {int(round(table[i]))},    /* {i*10:3d}%  "
                  f"标定 {R0_CAL[i]}, 样本 {cnt[i]} */")
        print("};")
        print(f"\n/* 学到的容量: {q_final:.0f} mAh (标称 {Q_NOM_MAH}) */")
        print(f"#define SOC_CAPACITY_NOMINAL_MAH  {int(round(q_final))}")

    if args.plot:
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            fig, ax = plt.subplots(1, 2, figsize=(12, 4.5))
            g = np.arange(11) * 10
            ax[0].plot(g, R0_CAL, "o--", color="#888", label="calib")
            ax[0].plot(g, table, "s-", color="#C0392B", label="learned")
            if per_grid:
                xs, ys = [], []
                for i in range(11):
                    for v in per_grid.get(i, []):
                        xs.append(i * 10 + np.random.uniform(-2, 2))
                        ys.append(v)
                ax[0].scatter(xs, ys, s=14, alpha=0.4, color="#2E86C1",
                              label="samples")
            ax[0].set_xlabel("SOC (%)")
            ax[0].set_ylabel("R0 (mΩ)")
            ax[0].set_title("R0 online learning")
            ax[0].legend(fontsize=8)
            ax[0].grid(alpha=0.3)

            ax[1].plot(ts / 3600, vs, lw=0.6, color="#185FA5")
            ax[1].set_xlabel("time (h)")
            ax[1].set_ylabel("V (mV)")
            ax[1].set_title("terminal voltage")
            ax[1].grid(alpha=0.3)
            png = args.csv.rsplit(".", 1)[0] + "_soh.png"
            fig.savefig(png, dpi=110, bbox_inches="tight")
            print(f"\n图已存: {png}")
        except ImportError:
            print("(未装 matplotlib, 跳过出图)")

    print("\n" + "=" * 74)


if __name__ == "__main__":
    main()
