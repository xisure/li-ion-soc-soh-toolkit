#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""一阶 RC EKF 离线参数迭代 + MCU 移植一致性校验

用途 (配合十档标定采集 CSV, 其中 SOC_0p01 列是 MCU 回传的 SOC):
  1. 网格调参: 在 stair 静置末参考点(安时积分)上评估一组 (q_soc, q_vrc, R)
     的误差, 选最优 —— 换电芯重标定后重跑;
  2. 同参一致性: --param 给定与固件一致的参数回放, 与 CSV 里 MCU 实跑
     的 SOC 轨迹逐帧对比 (mcu_rms≈0 说明固件 SOC 估计与 PC 模型一致);
  3. 参数预览: 新参数在 PC 上的轨迹偏差 (mcu_rms/mcu_max) 直接给出,
     不用烧板即可看改参数效果.

模型与固件 soc.c 落板实现逐行对齐 (改模型须两边同步):
  x = [soc(%), vrc(mV)]
  soc'   = soc - I*dt/(cap_mah*3600)*100            (放电为正)
  vrc'   = a*vrc + I*R1(soc)/1000*(1-a)             a=exp(-dt/tau(soc))
  z = V_m,  h = OCV(soc) - I*R0(soc)/1000 - vrc,  H = [dOCV/dsoc, -1]
  Q = diag(q_soc, q_vrc) (每帧), 量测噪声 R (mV^2), 野值 |res|>250mV 丢弃
评估基准优先级: refs(stair 静置末参考) > mcu(MCU 轨迹) > idle(端压自洽)

用法:
  python test/kalman_tune.py --csv stair2.csv --refs stair2_stair.csv
      # 网格调参, 以 stair 静置末参考点打分 (需 --refs 才有意义)
  python test/kalman_tune.py --csv stair2.csv --param 0.001 0.5 10
      # 只跑一组: 同参回放 vs MCU 轨迹(CSV 有 SOC_0p01 列时)一致性校验
  python test/kalman_tune.py --csv xxx.csv --param 0.01 1.0 5 --limit 30000
      # 取前 N 帧快速看
  python test/kalman_tune.py --csv stair2.csv --param 0.001 0.5 10 --plot ekf.png
      # 只跑一组并出轨迹图 (SOC 轨迹 / 误差 / 电流-电压)
  python test/kalman_tune.py --print-fw-default
      # 打印从固件 bms_config.h 读到的 EKF 三参数现值 (上位机同此来源)

表值须与 soc.c 的 SOC_OCV_MV / SOC_R0_MOHM / SOC_R1_MOHM / SOC_TAU_S 同步。
"""

import argparse
import csv
import math
import os
import re

import numpy as np

# ------- 查表 (与 soc.c 同步; 重标定/改 soc.c 后一起改) -------
OCV = [3050, 3188, 3375, 3524, 3629, 3748, 3845, 3938, 4027, 4096, 4161]
R0 = [70, 69, 63, 60, 61, 61, 61, 61, 60, 62, 62]
R1 = [70, 72, 70, 49, 38, 39, 38, 35, 36, 40, 40]
TAU = [36, 44, 109, 99, 117, 125, 109, 83, 78, 71, 70]
CAP = 3350.0
IDLE_MA = 20
DT = 0.2            # 回放步长 s (与 200ms 采样一致)

# 固件 EKF 三参数的宏名 -> 在 fw_defaults() 返回值里的下标
FW_KEYS = {"BMS_EKF_Q_SOC": 0, "BMS_EKF_Q_VRC": 1, "BMS_EKF_R_V": 2}
FW_BUILTIN = (0.001, 0.5, 10.0)     # 找不到 bms_config.h 时的兜底 (与 v1 固件同)


def _find_cfg():
    """沿目录逐级向上找固件的 bms_config.h。

    覆盖三种摆放: 脚本/test/ 往上找到仓库根; 仓库根下的算法库/ 或 firmware/;
    上位机 exe 把副本放在 tools/ 时, tools/ 这一层就能命中。
    """
    rel = ("bms_config.h",
           os.path.join("算法库", "bms_config.h"),
           os.path.join("firmware", "bms_config.h"),
           os.path.join("SOC", "SOC", "User", "bms_config.h"))
    d = os.path.dirname(os.path.abspath(__file__))
    for _ in range(5):
        for r in rel:
            p = os.path.join(d, r)
            if os.path.isfile(p):
                return p
        nd = os.path.dirname(d)
        if nd == d:
            break
        d = nd
    return None


def fw_defaults(path=None):
    """读固件 bms_config.h 当前生效的 EKF 三参数 -> (q_soc, q_vrc, r)。

    上位机「固件现值」按钮直接调它, 保证界面上填的永远是固件里正在跑的那组,
    而不是在脚本里另抄一份 (抄的迟早会漂)。头文件是 GBK 编码;
    找不到文件或宏就读不到, 退回 FW_BUILTIN。
    """
    if path is None:
        path = _find_cfg()
    vals = list(FW_BUILTIN)
    if path and os.path.isfile(path):
        with open(path, "rb") as f:
            src = f.read().decode("gbk", errors="replace")
        pat = (r"^\s*#define\s+(BMS_EKF_Q_SOC|BMS_EKF_Q_VRC|BMS_EKF_R_V)"
               r"\s+([-+0-9.eE]+)f?")
        for m in re.finditer(pat, src, re.M):
            try:
                vals[FW_KEYS[m.group(1)]] = float(m.group(2))
            except ValueError:
                pass
    return tuple(vals)


def fw_defaults_info():
    """返回 ((q_soc, q_vrc, r), 来源路径或 None) —— 上位机要用来源做提示。"""
    p = _find_cfg()
    return fw_defaults(p), p


def tab(a, soc):
    """与 C getter 一致: idx = floor(soc/10) clamp, 最近低网格点"""
    i = int(min(max(soc, 0.0), 100.0) / 10.0)
    return a[i]


def ocv_at(soc):
    """OCV 表线性插值 (mV)"""
    soc = min(max(soc, 0.0), 100.0)
    i = min(int(soc / 10.0), 9)
    f = (soc - i * 10) / 10.0
    return OCV[i] + f * (OCV[i + 1] - OCV[i])


def docv_dsoc(soc):
    """OCV 斜率 mV/% (卡尔曼线性化用, 该段表斜率)"""
    i = min(max(int(soc / 10.0), 0), 9)
    return (OCV[i + 1] - OCV[i]) / 10.0


def soc_from_voltage(v):
    """OCV(mV) -> SOC%, 分段线性反解 (同 C SOC_FromVoltage)"""
    if v >= OCV[10]:
        return 100.0
    if v <= OCV[0]:
        return 0.0
    for i in range(9, -1, -1):
        if v > OCV[i]:
            return i * 10 + (v - OCV[i]) * 10 / (OCV[i + 1] - OCV[i])
    return 0.0


def load(path):
    """读 record CSV -> (t, V, I, mcu_soc)。

    mcu_soc: CSV 的 SOC_0p01 列 (float %, MCU 实跑估计), 无该列(老固件录制的
    文件)或某帧无效 -> NaN。
    """
    ts, vs, iv, ms = [], [], [], []
    with open(path, newline="", encoding="utf-8") as f:
        rd = csv.DictReader(f)
        no_soc = "SOC_0p01" not in (rd.fieldnames or [])
        for r in rd:
            try:
                ts.append(float(r["elapsed_s"]))
                vs.append(float(r["V_mV"]))
                iv.append(float(r["I_mA"]))
            except (KeyError, ValueError, TypeError):
                continue
            if no_soc:
                ms.append(float("nan"))
            else:
                try:
                    ms.append(float(r["SOC_0p01"]) / 100.0)
                except (KeyError, ValueError, TypeError):
                    ms.append(float("nan"))
    if len(ts) < 50:
        raise SystemExit(f"有效帧太少 ({len(ts)}), 确认 CSV 列: elapsed_s,V_mV,I_mA")
    return (np.array(ts), np.array(vs), np.array(iv), np.array(ms))


def load_refs(path):
    """读 stair 明细 (*_stair.csv): (t_after_s -> soc_end) 静置末参考点

    soc_end 是 stair 的安时积分值(放电结束时刻), 独立于端压, 是调参主基准。
    新版 *_stair.csv 同时有 mcu_soc_end 列, 可在这里做 MCU vs 安时 的核对。
    """
    pts = []
    with open(path, newline="", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            pts.append((float(r["t_after_s"]), float(r["soc_end"])))
    return pts


class EKF:
    """与 soc.c 的 SOC_KF_Reset/Step 逐行对应 (常量/公式/野值剔除)"""

    def __init__(self, q_soc, q_vrc, r):
        self.qs, self.qv, self.r = q_soc, q_vrc, r
        self.soc = 50.0
        self.vrc = 0.0
        self.p11, self.p22, self.p12 = 10.0, 400.0, 0.0
        self.init = False

    def step(self, V, I, dt):
        if not self.init:
            self.soc = self._soc_from_v(V)      # 首帧按 OCV 建基 (C: KF_Reset)
            self.vrc = 0.0
            self.init = True
            return
        r0 = tab(R0, self.soc)
        r1 = tab(R1, self.soc)
        tau = tab(TAU, self.soc)
        # 预测 (放电为正)
        dsoc = I * dt / (CAP * 3600.0) * 100.0
        a = math.exp(-dt / tau) if tau > 1e-6 else 0.0
        self.soc -= dsoc
        self.vrc = self.vrc * a + I * r1 / 1000.0 * (1.0 - a)
        p11 = self.p11 + self.qs                 # F = diag(1, a)
        p22 = a * a * self.p22 + self.qv
        p12 = a * self.p12
        # 量测: h = OCV - I*R0/1000 - vrc
        h = ocv_at(self.soc) - I * r0 / 1000.0 - self.vrc
        h1 = docv_dsoc(self.soc)
        h2 = -1.0
        s = h1 * (h1 * p11 + h2 * p12) + h2 * (h1 * p12 + h2 * p22) + self.r
        s = max(s, 1e-6)
        k1 = (h1 * p11 + h2 * p12) / s
        k2 = (h1 * p12 + h2 * p22) / s
        res = V - h
        if abs(res) < 250.0:                     # 野值丢弃 (C 同)
            self.soc = min(max(self.soc + k1 * res, 0.0), 100.0)
            self.vrc += k2 * res
            self.p11 = max(p11 * (1.0 - k1 * h1) - p12 * k1 * h2, 1e-4)
            self.p22 = max(p22 * (1.0 - k2 * h2) - p12 * k2 * h1, 1e-4)
            self.p12 = p12 * (1.0 - k2 * h2) - p11 * k2 * h1
        else:
            self.p11, self.p22, self.p12 = p11, p22, p12

    @staticmethod
    def _soc_from_v(v):
        return soc_from_voltage(v)


def eval_windows(iv):
    """切静置段(|I|<20 连续>=60s), 取各段尾部 30% 采样点集合
    (端压基本稳定处: KF SOC 应贴近"端压查表", 弱指标仅参考)"""
    n = len(iv)
    ev = np.zeros(n, bool)
    a = np.abs(iv) < IDLE_MA
    segs = []
    s = None
    for k in range(n):
        if a[k] and s is None:
            s = k
        elif not a[k] and s is not None:
            if (k - s) * DT >= 60.0:
                segs.append((s, k))
            s = None
    if s is not None and (n - s) * DT >= 60.0:
        segs.append((s, n))
    for lo, hi in segs:
        ev[lo + int((hi - lo) * 0.7):hi] = True
    return ev, len(segs)


def run(ts, vs, iv, mcu, qs, qv, r, refs=None):
    """回放一组参数, 返回指标 dict。refs/mcu 缺失时对应指标置 1e9。"""
    ekf = EKF(qs, qv, r)
    ev, n_seg = eval_windows(iv)
    n = len(ts)
    socs = np.empty(n)
    err, ref_err, mcu_err = [], [], []
    for k in range(n):
        ekf.step(vs[k], iv[k], DT)
        socs[k] = ekf.soc
        if ev[k]:
            err.append(ekf.soc - soc_from_voltage(vs[k]))
        if not np.isnan(mcu[k]):
            mcu_err.append(ekf.soc - mcu[k])
    for t_r, soc_r in (refs or []):
        i = int(round(t_r / DT))
        if i < len(socs):
            ref_err.append(socs[i] - soc_r)
    return {
        "socs": socs,                # 逐帧轨迹 (出图 / 事后核对用)
        "soc_end": float(socs[-1]),
        "n_seg": n_seg,
        "idle_rms": _rms(err),
        "ref_rms": _rms(ref_err),
        "ref_max": _max(abs, ref_err),
        "mcu_rms": _rms(mcu_err),
        "mcu_max": _max(abs, mcu_err),
        "up_frac": float(np.mean(np.diff(socs) > 0.2)) * 100.0 if n > 1 else 0.0,
    }


def _rms(a):
    return float(np.sqrt(np.mean(np.asarray(a) ** 2))) if a else 1e9


def _max(f, a):
    return float(np.max(f(np.asarray(a)))) if a else 1e9


def fmt(tag, m, show_mcu):
    cols = (f"ref_rms={m['ref_rms']:6.2f}% ref_max={m['ref_max']:5.2f}% "
            if m["ref_rms"] < 1e8 else "")
    if show_mcu:
        cols += (f"mcu_rms={m['mcu_rms']:6.2f}% mcu_max={m['mcu_max']:6.2f}% "
                 if m["mcu_rms"] < 1e8 else "mcu 列不可用 ")
    cols += f"up={m['up_frac']:5.2f}% end={m['soc_end']:6.2f}%"
    return f"{tag:<20} {cols}"


def plot_trace(png, ts, vs, iv, mcu, socs, refs, qs, qv, r, met=None):
    """把一组参数的回放轨迹画成 PNG: SOC 轨迹 / 误差 / 电流-电压。

    只跑一组参数时用 (--param ... --plot x.png); 网格模式画最优那组。
    出图风格与 soc_bench 一致 (Agg + 英文轴标签, 不依赖中文字体)。
    """
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("(未装 matplotlib, 跳过出图)")
        return None

    th = ts / 60.0
    fig, (a1, a2, a3) = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    a1.plot(th, socs, lw=1.6, color="#185FA5", label="PC EKF replay")
    if np.any(~np.isnan(mcu)):
        a1.plot(th, mcu, lw=1.0, color="#D85A30", label="MCU on-board")
    if refs:
        a1.plot([p[0] / 60.0 for p in refs], [p[1] for p in refs],
                "o", ms=5, color="#0F6E56", zorder=6,
                label="stair rest-end (Ah ref)")
    a1.set_ylabel("SOC (%)")
    a1.grid(alpha=0.3)
    a1.legend(loc="best", fontsize=9)
    a1.set_title("EKF replay   q_soc=%g   q_vrc=%g   R=%g" % (qs, qv, r))

    if np.any(~np.isnan(mcu)):
        a2.plot(th, socs - mcu, lw=0.9, color="#A32D2D", label="EKF - MCU")
    ev, _ = eval_windows(iv)
    vq = np.full(len(ts), np.nan)
    for k in range(len(ts)):
        if ev[k]:
            vq[k] = socs[k] - soc_from_voltage(vs[k])
    if np.any(~np.isnan(vq)):
        a2.plot(th, vq, lw=0.7, color="#5F5E5A", alpha=0.85,
                label="EKF - V lookup (idle tail)")
    a2.axhline(0, color="#888888", lw=0.6)
    a2.set_ylabel("SOC error (%)")
    a2.grid(alpha=0.3)
    a2.legend(loc="best", fontsize=9)
    if met:
        a2.text(0.01, 0.04,
                "mcu_rms=%s   ref_rms=%s   end=%.2f%%"
                % (("%.2f%%" % met["mcu_rms"]) if met["mcu_rms"] < 1e8 else "n/a",
                   ("%.2f%%" % met["ref_rms"]) if met["ref_rms"] < 1e8 else "n/a",
                   met["soc_end"]),
                transform=a2.transAxes, fontsize=9, color="#333333")

    a3.plot(th, iv / 1000.0, lw=0.7, color="#5F5E5A")
    a3.set_ylabel("I (A)")
    a3.grid(alpha=0.3)
    a3b = a3.twinx()
    a3b.plot(th, vs / 1000.0, lw=0.7, color="#185FA5", alpha=0.7)
    a3b.set_ylabel("V (V)", color="#185FA5")
    a3.set_xlabel("time (min)")
    fig.tight_layout()
    fig.savefig(png, dpi=140)
    plt.close(fig)
    return png


def main():
    ap = argparse.ArgumentParser(
        description="一阶RC EKF 离线调参/一致性校验 (表值与 soc.c 同步)")
    ap.add_argument("--csv", default=None, help="record 采集 CSV (v4 含 SOC_0p01 列)")
    ap.add_argument("--refs", default=None,
                    help="stair 明细 *_stair.csv: 提供静置末安时参考点 (打分主基准)")
    ap.add_argument("--param", type=float, nargs=3, metavar=("Q_SOC", "Q_VRC", "R"),
                    help="只回放这一组 (固件同参一致性校验/单点预览)")
    ap.add_argument("--grid", default="1e-4,1e-3,1e-2,0.1 | 0.05,0.5,2.0 | 2.0,10.0,50.0",
                    help="网格: 'qs列表|qv列表|r列表', 各档逗号分隔 (默认 4x3x3)")
    ap.add_argument("--plot", default=None, metavar="PNG",
                    help="把回放轨迹画成 PNG (单组配 --param; 网格模式画最优那组)")
    ap.add_argument("--limit", type=int, default=0, help="只取前 N 帧 (快速调试)")
    ap.add_argument("--print-fw-default", action="store_true",
                    help="打印固件 bms_config.h 的 EKF 三参数现值后退出")
    a = ap.parse_args()

    if a.print_fw_default:
        qs, qv, r = fw_defaults()
        print("固件 EKF 现值: q_soc=%g  q_vrc=%g  r=%g" % (qs, qv, r))
        print("来源: %s" % (_find_cfg() or "(没找到 bms_config.h, 用内置兜底值)"))
        return
    if not a.csv:
        ap.error("--csv 必填 (只看固件现值请用 --print-fw-default)")

    ts, vs, iv, mcu = load(a.csv)
    if a.limit:
        ts, vs, iv, mcu = ts[:a.limit], vs[:a.limit], iv[:a.limit], mcu[:a.limit]
    refs = None
    if a.refs:
        refs = load_refs(a.refs)
        refs = [p for p in refs if p[0] < ts[-1]] if a.limit else refs
    has_mcu = bool(np.any(~np.isnan(mcu)))
    print(f"{len(ts)} 帧 / {ts[-1]/60:.0f} min"
          f"{' / MCU SOC 列可用' if has_mcu else ' / 无 MCU SOC 列(老v3)'}"
          f"{' / ' + str(len(refs)) + ' 个静置末参考' if refs else ''}")

    if a.param is not None:
        qs, qv, r = a.param
        m = run(ts, vs, iv, mcu, qs, qv, r, refs)
        print(fmt(f"qs={qs:g} qv={qv:g} r={r:g}", m, has_mcu))
        if a.plot:
            plot_trace(a.plot, ts, vs, iv, mcu, m["socs"], refs, qs, qv, r, m)
            print(f"[i] 轨迹图: {a.plot}")
        if has_mcu and m["mcu_rms"] < 0.05:
            print(">> MCU 列与 PC 同参回放一致 (mcu_rms<0.05%) -> soc.c 移植可信, "
                  "可在 PC 直接迭代参数")
        elif has_mcu:
            print(f">> MCU 实跑与 PC 回放差 mcu_rms={m['mcu_rms']:.2f}%: 若固件就是该参数, "
                  "检查移植/表/初始 SOC 是否对齐")
        return

    # 网格调参
    ql, vl, rl = (s.strip() for s in a.grid.split("|"))
    ql = [float(x) for x in ql.split(",")]
    vl = [float(x) for x in vl.split(",")]
    rl = [float(x) for x in rl.split(",")]
    best = None
    for qs in ql:
        for qv in vl:
            for r in rl:
                m = run(ts, vs, iv, mcu, qs, qv, r, refs)
                # 打分: 主基准 ref(静置末安时参考), 无 refs 则退 mcu, 再退 idle
                if m["ref_rms"] < 1e8:
                    score = m["ref_rms"] + 0.5 * m["up_frac"]
                    tag = f"qs={qs:g} qv={qv:g} r={r:g}"
                    print(f"{fmt(tag, m, has_mcu)}")
                elif m["mcu_rms"] < 1e8:
                    score = m["mcu_rms"] + 0.5 * m["up_frac"]
                    tag = f"qs={qs:g} qv={qv:g} r={r:g}"
                    print(f"{fmt(tag, m, has_mcu)}")
                else:
                    score = m["idle_rms"] + 0.5 * m["up_frac"]
                    tag = f"qs={qs:g} qv={qv:g} r={r:g}"
                    print(f"{fmt(tag, m, has_mcu)}")
                if best is None or score < best[0]:
                    best = (score, (qs, qv, r), m)
    score, (qs, qv, r), m = best
    print(f"\n最优: qs={qs:g} qv={qv:g} r={r:g}  "
          f"(score={score:.2f}, ref_rms={m['ref_rms']:.2f}%"
          + (f", mcu_rms={m['mcu_rms']:.2f}%" if m["mcu_rms"] < 1e8 else "")
          + ")")
    if a.plot:
        plot_trace(a.plot, ts, vs, iv, mcu, m["socs"], refs, qs, qv, r, m)
        print(f"[i] 轨迹图(最优组): {a.plot}")


if __name__ == "__main__":
    main()
