#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
soh_mcu_sim.py - 把 MCU 端 soh.c 的逻辑逐行搬到 PC, 用实测 CSV 回放, 验证移植无误

和 soh_learn.py 的分工:
    soh_learn.py  : 离线分析视角 (numpy 向量化, 事后扫静置段) -> 定算法
    soh_mcu_sim.py: MCU 运行视角 (逐帧状态机 / 同样的整数-浮点混合 / 同样的
                 自适应 α / 同样的限幅顺序) -> 定移植

判据常量从 soh.h 抄来, 脚本启动时与 soh_learn.py 的常量逐项比对, 不一致就报警
(防止两边改了一边)。输出 R0 表 / 学到的容量 / SOH, 与 soh_learn.py 可直接对照。

用法:
    python soh_mcu_sim.py --csv stair1.csv [--scale 0.8]
"""

import argparse
import csv
import importlib.machinery
import importlib.util
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))


def load_soh_learn():
    """把同目录的 soh_learn.py 当模块加载, 复用它的常量与 OCV 查表 (单一真源)"""
    path = os.path.join(HERE, "soh_learn.py")
    loader = importlib.machinery.SourceFileLoader("soh_learn_mod", path)
    spec = importlib.util.spec_from_loader("soh_learn_mod", loader)
    mod = importlib.util.module_from_spec(spec)
    loader.exec_module(mod)
    return mod


SL = load_soh_learn()

# =====================================================================
# 判据常量: 与固件侧一一对应 (改固件必须同步改这里)
#   固件位置: ../firmware/bms_config.h
# =====================================================================
SOH_I_ON_MA        = 500
SOH_I_OFF_MA       = 200
SOH_MIN_PULSE_MS   = 5000
SOH_IDLE_MA        = 20
SOH_IDLE_DEBOUNCE  = 5
SOH_MIN_REST_MS    = 600000
SOH_Q_MIN_DSOC_01  = 1500
SOH_Q_MIN_DQ_MAH   = 200
SOH_Q_SIG_K        = 126.0   # sigma_Q(%) = SOH_Q_SIG_K / dSOC(%);  30% -> 4.2%
SOH_Q_KF_P0        = 0.02    # Q 初值相对不确定度
SOH_Q_KF_Q         = 0.002   # 过程噪声 (两次测量间容量漂移, 相对值)
SOH_Q_KF_RUP       = 8.0     # 正向新息的量测噪声放大倍数 (抑制上跳)
SOH_R0_ALPHA       = 0.20
SOH_R0_MIN_SAMPLE  = 3
SOH_R0_LO_MOHM     = 20
SOH_R0_HI_MOHM     = 200
SOH_TEMP_LO_C      = 15
SOH_TEMP_HI_C      = 35
SOH_R_EOL_RATIO    = 2.0

Q_NOM_MAH = SL.Q_NOM_MAH


def check_constants():
    """脚本侧判据 vs MCU soh.h 判据: 逐项比对, 不一致给出醒目警告"""
    pairs = [
        ("I_ON_MA",       SOH_I_ON_MA,       SL.I_ON_MA),
        ("I_OFF_MA",      SOH_I_OFF_MA,      SL.I_OFF_MA),
        ("MIN_PULSE_S",   SOH_MIN_PULSE_MS / 1000.0, SL.MIN_PULSE_S),
        ("IDLE_MA",       SOH_IDLE_MA,       SL.IDLE_MA),
        ("IDLE_DEBOUNCE", SOH_IDLE_DEBOUNCE, SL.IDLE_DEBOUNCE),
        ("MIN_REST_S",    SOH_MIN_REST_MS / 1000.0, SL.MIN_REST_S),
        ("Q_MIN_DSOC",    SOH_Q_MIN_DSOC_01 / 100.0, SL.Q_MIN_DSOC),
        ("Q_MIN_DQ",      SOH_Q_MIN_DQ_MAH,  SL.Q_MIN_DQ),
        ("Q_SIG_K",       SOH_Q_SIG_K,       SL.Q_SIG_K),
        ("Q_KF_P0",       SOH_Q_KF_P0,       SL.Q_KF_P0),
        ("Q_KF_Q",        SOH_Q_KF_Q,        SL.Q_KF_Q),
        ("Q_KF_RUP",      SOH_Q_KF_RUP,      SL.Q_KF_RUP),
        ("R0_ALPHA",      SOH_R0_ALPHA,      SL.R0_ALPHA),
        ("R0_MIN_SAMPLE", SOH_R0_MIN_SAMPLE, SL.R0_MIN_SAMPLE),
        ("R0_LO",         SOH_R0_LO_MOHM,    SL.R0_LO),
        ("R0_HI",         SOH_R0_HI_MOHM,    SL.R0_HI),
        ("TEMP_LO",       SOH_TEMP_LO_C,     SL.TEMP_LO_C),
        ("TEMP_HI",       SOH_TEMP_HI_C,     SL.TEMP_HI_C),
        ("R_EOL_RATIO",   SOH_R_EOL_RATIO,   SL.R_EOL_RATIO),
    ]
    bad = [n for n, a, b in pairs if abs(float(a) - float(b)) > 1e-9]
    if bad:
        print("[!] 判据常量与 soh_learn.py 不一致: " + ", ".join(bad))
        print("    (MCU soh.h 与脚本必须同步, 请先对齐再跑)")
    else:
        print("[i] 判据常量与 soh_learn.py 全部一致 (%d 项)" % len(pairs))
    return not bad


# =====================================================================
# MCU 端 soh.c 的逐帧复刻
# =====================================================================
class SohMcu:
    """严格按 soh.c 实现: 状态机顺序 / 类型 / 限幅顺序都照抄"""

    def __init__(self, cap_mah):
        # --- soc.c 侧的运行时容量 (SOC_SetCapacityMAh 的钳位照抄) ---
        self.cap = cap_mah
        self.cap_lo = Q_NOM_MAH // 2
        self.cap_hi = Q_NOM_MAH + Q_NOM_MAH // 10

        # --- R0 学习状态 ---
        self.r0_table = [float(x) for x in SL.R0_CAL]
        self.r0_base = list(SL.R0_CAL)
        self.r0_cnt = [0] * 11
        self.r0_in_load = False
        self.r0_pend = False
        self.r0_fcnt = 0
        self.r0_isum = 0
        self.r0_tail = [0] * 5
        self.r0_tail_i = 0
        self.r0_pend_v = 0
        self.r0_any = False

        # --- 静置锚点 ---
        self.rest_quies = 0
        self.rest_flag = False
        self.rest_since_ms = 0
        self.rest_done = False

        # --- Q 学习 ---
        self.coul_mAms = 0
        self.last_ms = 0
        self.q_time_ok = False
        self.q_hi_ok = self.q_lo_ok = False
        self.q_hi_soc01 = self.q_lo_soc01 = 0
        self.q_hi_coul = self.q_lo_coul = 0
        self.q_n = 0
        self.q_any = False
        self.kf_P = SOH_Q_KF_P0 ** 2      # 卡尔曼: Q 的估计方差

        self.anchors = []       # (t_ms, soc01, coul_mAms)
        self.q_learns = []      # dict
        self.r0_events = []     # dict

    # ---------------- soc.c: SOC_SetCapacityMAh ----------------
    def set_capacity(self, cap_mah):
        c = int(cap_mah + 0.5)
        if c < self.cap_lo:
            c = self.cap_lo
        if c > self.cap_hi:
            c = self.cap_hi
        self.cap = c

    # ---------------- soh.c: soh_r0_fuse ----------------
    def r0_fuse(self, idx, r0, temp_c):
        if r0 < SOH_R0_LO_MOHM or r0 > SOH_R0_HI_MOHM:
            return
        if temp_c < SOH_TEMP_LO_C or temp_c > SOH_TEMP_HI_C:
            return
        old = self.r0_table[idx]
        a_eff = SOH_R0_ALPHA
        inv = 1.0 / (self.r0_cnt[idx] + 2)
        if inv > a_eff:
            a_eff = inv
        lim = 0.20 if self.r0_cnt[idx] < SOH_R0_MIN_SAMPLE else 1.00
        dev = r0 - old
        cap = old * lim
        if cap < 0:
            cap = -cap
        dev = max(-cap, min(cap, dev))
        old = old + a_eff * dev
        old = max(SOH_R0_LO_MOHM, min(SOH_R0_HI_MOHM, old))
        self.r0_table[idx] = old
        if self.r0_cnt[idx] < 255:
            self.r0_cnt[idx] += 1
        self.r0_any = True

    # ---------------- soh.c: soh_r0_frame ----------------
    def r0_frame(self, bus_mv, cur_ma, temp_dc, now_ms):
        a = abs(cur_ma)
        if not self.r0_in_load and not self.r0_pend:
            if a > SOH_I_ON_MA:
                self.r0_in_load = True
                self.r0_fcnt = 0
                self.r0_isum = 0
                self.r0_tail_i = 0
                self.r0_tail = [0] * 5
        elif self.r0_in_load:
            if a < SOH_I_OFF_MA:
                self.r0_in_load = False
                self.r0_pend = True
                self.r0_pend_v = bus_mv
            else:
                self.r0_fcnt += 1
                self.r0_isum += cur_ma
                self.r0_tail[self.r0_tail_i] = bus_mv
                self.r0_tail_i = (self.r0_tail_i + 1) % 5
        else:                                   # pend
            if a < SOH_I_OFF_MA:
                if self.r0_fcnt > 0 and self.r0_fcnt * 200 >= SOH_MIN_PULSE_MS:
                    v_load = sum(self.r0_tail) // 5
                    i_avg = int(self.r0_isum / self.r0_fcnt)
                    if i_avg != 0:
                        dv = self.r0_pend_v - v_load
                        r0 = dv * 1000.0 / i_avg
                        idx = min(SL.soc01_from_mv(self.r0_pend_v) // 1000, 10)
                        self.r0_events.append(
                            dict(t_ms=now_ms, idx=idx, r0=r0, dv=dv,
                                 i_avg=i_avg, temp_c=temp_dc // 10))
                        self.r0_fuse(idx, r0, temp_dc // 10)
            self.r0_pend = False

    # ---------------- soh.c: soh_anchor ----------------
    def anchor(self, bus_mv, now_ms):
        soc01 = SL.soc01_from_mv(bus_mv)
        coul = self.coul_mAms
        if (not self.q_hi_ok) or soc01 > self.q_hi_soc01:
            self.q_hi_ok = True
            self.q_hi_soc01 = soc01
            self.q_hi_coul = coul
        if (not self.q_lo_ok) or soc01 < self.q_lo_soc01:
            self.q_lo_ok = True
            self.q_lo_soc01 = soc01
            self.q_lo_coul = coul
        self.anchors.append((now_ms, soc01, coul))
        if not (self.q_hi_ok and self.q_lo_ok):
            return

        dsoc01 = self.q_hi_soc01 - self.q_lo_soc01
        dcoul = self.q_lo_coul - self.q_hi_coul
        if dsoc01 < SOH_Q_MIN_DSOC_01 or dcoul <= 0:
            return
        dq_mah = dcoul / 3600000.0
        if dq_mah < SOH_Q_MIN_DQ_MAH:
            return

        q_est = dq_mah * 10000.0 / dsoc01
        q = float(self.cap)
        dsoc_pct = dsoc01 / 100.0
        # ---- 标量卡尔曼融合 (与 soh_learn.py 同一套) ----
        sig = SOH_Q_SIG_K / max(dsoc_pct, 1.0) / 100.0    # 相对 sigma (小数)
        R = sig * sig
        d_raw = q_est - q
        R_eff = R * (SOH_Q_KF_RUP if d_raw > 0.0 else 1.0)
        K = self.kf_P / (self.kf_P + R_eff)
        q_new = q + K * d_raw
        P_new = (1.0 - K) * self.kf_P + SOH_Q_KF_Q ** 2
        self.set_capacity(q_new)
        self.kf_P = P_new
        if self.q_n < 255:
            self.q_n += 1
        self.q_any = True
        self.q_learns.append(dict(t_ms=now_ms, dsoc=dsoc_pct,
                                  dq=dq_mah, q_est=q_est, alpha=K,
                                  sig=sig * 100.0, K=K,
                                  q_new=float(self.cap)))
        # 重置锚点
        self.q_hi_soc01 = self.q_lo_soc01 = soc01
        self.q_hi_coul = self.q_lo_coul = coul

    # ---------------- soh.c: SOH_Update ----------------
    def update(self, bus_mv, cur_ma, temp_dc, now_ms):
        if not self.q_time_ok:
            self.last_ms = now_ms
            self.q_time_ok = True
        dt_ms = now_ms - self.last_ms
        if dt_ms < 0 or dt_ms > 2000:
            dt_ms = 200
        self.last_ms = now_ms
        self.coul_mAms += cur_ma * dt_ms

        a = abs(cur_ma)
        if a < SOH_IDLE_MA:
            if self.rest_quies < SOH_IDLE_DEBOUNCE:
                self.rest_quies += 1
        else:
            self.rest_quies = 0

        if self.rest_quies >= SOH_IDLE_DEBOUNCE:
            if not self.rest_flag:
                self.rest_flag = True
                self.rest_since_ms = now_ms
                self.rest_done = False
            elif (not self.rest_done) and (now_ms - self.rest_since_ms) >= SOH_MIN_REST_MS:
                self.rest_done = True
                self.anchor(bus_mv, now_ms)
        else:
            self.rest_flag = False
            self.rest_done = False

        self.r0_frame(bus_mv, cur_ma, temp_dc, now_ms)

    # ---------------- 结果 ----------------
    def soh_q(self):
        return self.cap / Q_NOM_MAH * 100.0

    def soh_r(self):
        s = sum(self.r0_table[1:10])
        b = sum(self.r0_base[1:10])
        if b == 0:
            return 100.0
        ratio = s / b
        v = (SOH_R_EOL_RATIO - ratio) / (SOH_R_EOL_RATIO - 1.0) * 100.0
        return max(0.0, min(100.0, v))

    def soh(self):
        return min(self.soh_q(), self.soh_r())


# =====================================================================
def main():
    ap = argparse.ArgumentParser(description="MCU soh.c 逐帧复刻回放验证")
    ap.add_argument("--csv", required=True)
    ap.add_argument("--scale", type=float, default=1.0,
                    help="合成老化: 真容量 = scale*3350 (与 soh_learn.py 同口径)")
    ap.add_argument("--refs", default=None, help="stair1_stair.csv 对比 R0")
    args = ap.parse_args()

    check_constants()

    ts, vs, iv, tt, sc = SL.load_csv(args.csv)
    print("=" * 74)
    print("MCU soh.c 逐帧复刻回放   %s" % os.path.basename(args.csv))
    print("  %d 帧, %.2f h, 容量真值 %.0f mAh (scale=%.2f)"
          % (len(ts), ts[-1] / 3600.0, args.scale * Q_NOM_MAH, args.scale))
    print("=" * 74)

    # ---- 合成老化: 与 soh_learn.py 完全相同的重算方式 ----
    if abs(args.scale - 1.0) > 1e-9:
        soc01 = np.array([SL.soc01_from_mv(v) for v in vs], dtype=float)
        dt = np.diff(ts, prepend=ts[0])
        qc = np.cumsum(iv * dt) / 3600.0
        q_true = args.scale * Q_NOM_MAH
        s0 = soc01[0] / 100.0
        soc01 = np.clip(s0 - qc / q_true * 100.0, 0.0, 100.0)
        v_ocv = np.array([SL.mv_from_soc01(x * 100) for x in soc01])
        idx = np.clip((soc01 // 10).astype(int), 0, 10)
        r0v = np.array([SL.R0_CAL[i] for i in idx])
        r1v = np.array([SL.R1_CAL[i] for i in idx])
        tuv = np.array([SL.TAU_S[i] for i in idx])
        vrc = np.zeros(len(ts))
        for k in range(1, len(ts)):
            a = np.exp(-dt[k] / max(tuv[k], 1.0))
            vrc[k] = vrc[k - 1] * a + iv[k] * r1v[k] / 1000.0 * (1 - a)
        vs = np.round((v_ocv - iv * r0v / 1000.0 - vrc) / 1.25) * 1.25
        print("  [合成] 已按 scale=%.2f 重算 SOC 轨迹与端压" % args.scale)

    # ---- 逐帧喂给 MCU 复刻 ----
    sim = SohMcu(cap_mah=Q_NOM_MAH)
    for k in range(len(ts)):
        sim.update(int(round(vs[k])), int(round(iv[k])),
                   int(round(tt[k])), int(round(ts[k] * 1000.0)))

    # ---- 输出 ----
    print("\n【1】R0 学习 (MCU 逐帧状态机)")
    print("  有效脉冲事件 %d 次, 有效样本 %d 个"
          % (len(sim.r0_events), sum(sim.r0_cnt)))
    print("  %6s %4s %7s %9s %8s" % ("SOC格", "样本", "标定", "MCU学习", "偏差"))
    print("  " + "-" * 40)
    for i in range(11):
        print("  %5d%% %4d %7d %9.1f %+8.1f"
              % (i * 10, sim.r0_cnt[i], SL.R0_CAL[i], sim.r0_table[i],
                 sim.r0_table[i] - SL.R0_CAL[i]))

    print("\n【2】Q 容量学习 (MCU 逐帧状态机)")
    print("  静置锚点 %d 个, 触发学习 %d 次" % (len(sim.anchors), len(sim.q_learns)))
    for e in sim.q_learns:
        print("    t=%8.0fs  ΔSOC=%5.2f%%  ΔQ=%7.1fmAh  Q_est=%7.1f  "
              "σ=%4.1f%%  K=%.3f  Q=%7.1f"
              % (e["t_ms"] / 1000.0, e["dsoc"], e["dq"], e["q_est"],
                 e["sig"], e["K"], e["q_new"]))
    q_true = args.scale * Q_NOM_MAH
    print("  MCU 学到 Q = %.1f mAh (真值 %.0f, 误差 %+.1f mAh = %+.2f%%)"
          % (sim.cap, q_true, sim.cap - q_true, (sim.cap / q_true - 1) * 100))

    print("\n【3】SOH")
    print("  SOH_Q = %.2f%%   SOH_R = %.2f%%   SOH = %.2f%%"
          % (sim.soh_q(), sim.soh_r(), sim.soh()))

    # ---- 与 soh_learn.py (离线 numpy 版) 对照 ----
    print("\n【4】与 soh_learn.py (离线版) 逐项对照")
    ref_table, ref_cnt, ref_samples, _ = SL.learn_r0(
        ts, vs, iv, tt, np.array([SL.soc01_from_mv(v) for v in vs], dtype=float),
        SL.I_ON_MA, SL.I_OFF_MA, SL.MIN_PULSE_S, SL.R0_ALPHA, verbose=False)
    print("  %6s %9s %9s %8s" % ("SOC格", "离线学习", "MCU学习", "差"))
    print("  " + "-" * 36)
    for i in range(11):
        print("  %5d%% %9.1f %9.1f %+8.1f"
              % (i * 10, ref_table[i], sim.r0_table[i],
                 sim.r0_table[i] - ref_table[i]))

    anchors = SL.find_rest_anchors(ts, iv, SL.MIN_REST_S)
    dt = np.diff(ts, prepend=ts[0])
    q_total = np.cumsum(iv * dt) / 3600.0
    q_ref, ests = (SL.learn_q(ts, vs, iv, q_total, anchors, SL.Q_MIN_DSOC,
                              float(Q_NOM_MAH), verbose=False)
                   if len(anchors) >= 2 else (float(Q_NOM_MAH), []))
    print("\n  离线 Q = %.1f mAh   MCU Q = %.1f mAh   差 %+.1f mAh"
          % (q_ref, sim.cap, sim.cap - q_ref))
    print("  离线静置锚点 %d 个, MCU 静置锚点 %d 个" % (len(anchors), len(sim.anchors)))

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
                print("\n  与 %s 的 PC 拟合 r0_mohm 对比:" % os.path.basename(args.refs))
                print("  %4s %8s %9s %9s %7s" % ("seg", "SOC_mid", "PC拟合", "MCU学习", "差"))
                print("  " + "-" * 42)
                for seg in sorted(ref):
                    sm, r0ref = ref[seg]
                    gi = int(min(max(sm, 0), 99.9) // 10)
                    print("  %4d %8.2f %9.1f %9.1f %+7.1f"
                          % (seg, sm, r0ref, sim.r0_table[gi],
                             sim.r0_table[gi] - r0ref))
        except Exception as e:
            print("  (refs 解析失败: %s)" % e)

    print("\n" + "=" * 74)


if __name__ == "__main__":
    main()
