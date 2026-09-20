#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
soc_bench.py - 一阶 RC 电池模型参数提取 (HPPC 风格: 恒流放电 -> 断电静置)

从一段 "恒流放电(60~120s) -> 断开负载静置(>=10min)" 的串口采样里, 自动拟合:
    R0 : 欧姆内阻 (断电瞬间电压跳变 / 放电电流)
    R1 : 极化电阻 (静置段指数回弹幅度 / 放电电流)
    tau: 极化时间常数 (指数恢复的 ln 线性回归斜率)
    C1 : 极化电容 = tau / R1
模型: V(t) = OCV - I*R0 - V_rc,  静置恢复 V(t) = V_inf - I*R1*exp(-(t-t_off)/tau)

用法:
  0) 交互向导 (什么都不带直接跑): 先问"程控源载 / 手动负载", 再决定调 soc_load.py
     还是走 record/手动设档, 用于统一入口:
       python soc_bench.py
  1) 采样 (与 soc_record.py 同一套串口协议, Ctrl+C 结束):
       python soc_bench.py record --out pulse1.csv
     不传 --port/--baud 时启动前交互选择 COM 口和波特率;
     也可: python soc_bench.py record --port COM3 --baud 115200 --out pulse1.csv
     操作流程: 脚本开始记录后 ->
        a. 电芯充满电 (或目标 SOC), 静置到电压稳定
        b. 电子负载设恒流 (推荐 1C=3.35A 或 0.5C=1.68A), 接入放电 60~120s
        c. 断开负载 (瞬间!), 保持静置 >=10min (越长 V_inf 越准)
        d. Ctrl+C 停止
  2) 拟合 (自动找断电点):
       python soc_bench.py fit
         -> 不传 --csv 时扫描当前目录列出 CSV 供选择 (回车=最新)
       python soc_bench.py fit --csv pulse1.csv
  也可: python soc_bench.py fit --csv pulse1.csv --I 3.35   # 指定放电电流(A)防误判

  3) N 档周期 (电子负载自动跑, 脚本只负责识别+处理):
       python soc_bench.py stair --csv stair1.csv
   电子负载侧设好 "恒流放电 -> 断开静置" 重复 N 次(默认期望 10 档, 电流可各档不同),
   脚本自动切出每一档: 逐档拟合 R0/R1/C1/tau, 取各档前后 OCV,
   安时积分定 SOC -> 汇总成 OCV-SOC 表 + R(SOC) 表。
   从满充开始录:  --init-soc 100 (默认) ; 容量: --capacity 3350 (默认)
   输出 C 查表数组: 加 --emit-c

  4) 多轮充放循环 (老化/一致性, 自动交替调度 soc_load.py):
       python soc_bench.py cycle --cycles 5
   放电腿/充电腿各是一次完整的 soc_load.py 运行, 所以每腿都单独落一份计划 +
   档位时刻表 (cyc_c1_dis.json/.csv, cyc_c1_chg.json/.csv, ...), 某一腿出问题
   只影响那一腿。循环窗口默认 9%<->80%、两腿首尾相接(每轮窗口一致才谈得上可比),
   第一腿默认从 100% 放到 9%。先加 --dry-run 看排出来的腿序与预计总时长, 不碰设备。

依赖: pyserial(record), numpy, matplotlib
输出: 终端打印参数 + 拟合图 (同名 .png)
"""

import argparse
import csv
import json
import os
import subprocess
import sys
import datetime

import numpy as np
from numpy.lib.stride_tricks import sliding_window_view

# ---------------- 与 soc_record.py 一致的协议常量 ----------------
V_MIN_MV, V_MAX_MV = 1500, 5000
I_MIN_MA, I_MAX_MA = -30000, 30000

IDLE_MA = 20       # |I|<20mA 判静置 (与 MCU soc.c / soc_analyze 一致)
LOAD_MA = 200      # |I|>200mA 判带载

# ---- stair (多档周期) 默认参数 ----
STAIR_I_ON_MA     = 200     # 带载进入阈值 |I|>: 判定"脉冲开始"
STAIR_I_OFF_MA    = 50      # 带载退出阈值 |I|<: 判定"脉冲结束"(滞回, 防边缘抖动)
STAIR_MIN_PULSE_S = 2.0     # 短于此的带载段当毛刺丢弃
STAIR_TAIL_S      = 60.0    # OCV / V_inf 取静置段末尾多少秒均值
STAIR_EXPECT      = 10      # 期望档位数 (只提示, 不符不报错)
STAIR_CAPACITY    = 3350    # 标称容量 mAh (与 soc.h SOC_CAPACITY_MAH 一致)

# ---- cycle (多轮充放循环) 默认参数 ----
# 窗口两端刻意与 soc_load.py 的 充电默认 to_soc(80) / 放电默认 to_soc(9) 对齐:
# 这样"两腿首尾相接"天然成立, 两腿之间不需要做任何 SOC 换算。
CYCLE_N         = 3        # 循环轮数
CYCLE_SEGMENTS  = 10       # 每腿档位数
CYCLE_SOC_HI    = 80.0     # 循环窗口上沿 %
CYCLE_SOC_LO    = 9.0      # 循环窗口下沿 %
CYCLE_START_SOC = 100.0    # 第一腿(放电首)的起始 SOC %; 充电首则默认取窗口下沿
CYCLE_CUR_LOAD  = "2.0"    # 放电腿每档电流 A (单值 / 逗号序列 / 'a:b' 递变)
CYCLE_CUR_CHG   = "1.6"    # 充电腿每档电流 A
CYCLE_REST_S    = 480.0    # 档间静置 s
CYCLE_VLIM_LOAD = 2.8      # 放电端压底线 V
CYCLE_VLIM_CHG  = 4.15     # 充电端压上限 V (必须低于 CYCLE_VSET)
CYCLE_VSET      = 4.20     # 电源恒压 CV 上限 V
CYCLE_CAPACITY  = STAIR_CAPACITY
CYCLE_PREFIX    = "cyc"

# ---- 一腿怎么跑: 默认另起一个 python 进程; 上位机里换成"在本进程内调 soc_load" ----
# 上位机打包成 exe 之后 sys.executable 指向 exe 自己, 再给每腿另起进程只会又弹一个界面
# 出来。把"执行一条腿"抽成可替换的钩子, 腿序/窗口/互锁/停跑判据那一整套编排逻辑就还是
# 同一份 —— 命令行与上位机共用, 不会各自长歪。
LEG_RUNNER = None


def run_leg(cmd):
    """跑一腿。cmd = [解释器, soc_load.py, ...]（LEG_RUNNER 只用从第 3 项起的参数）。"""
    if LEG_RUNNER is not None:
        return LEG_RUNNER(cmd)
    return subprocess.call(cmd)

# 电芯 OCV 表 (mV, SOC 0%..100% 步长 10) —— 与 soc_load.py 的 DEFAULT_OCV_MV、
# 固件 bms_config.h 的 OCV 表**同源**, 改一处三处都要改。
# 这里只在"没有档位计划文件、用户又没给 --init-soc"时用它反查起始 SOC,
# 属于兜底路径; 有计划文件时一律以计划为准(那才是当时的真实意图)。
STAIR_OCV_MV = [3050, 3188, 3375, 3524, 3629, 3748, 3845, 3938, 4027, 4096, 4161]


def soc_from_ocv(mv, tbl=None):
    """OCV(mV) -> SOC% (分段线性反解, 端点平延不外推)。"""
    tbl = tbl or STAIR_OCV_MV
    if mv >= tbl[-1]:
        return 100.0
    if mv <= tbl[0]:
        return 0.0
    for i in range(10):
        if tbl[i] <= mv <= tbl[i + 1]:
            f = (mv - tbl[i]) / (tbl[i + 1] - tbl[i])
            return i * 10 + f * 10.0
    return float("nan")


def parse_line(line: str):
    parts = line.strip().split(",")
    if len(parts) != 4 or parts[0] != "D":
        return None
    try:
        v, i, t = (int(p) for p in parts[1:])
    except ValueError:
        return None
    if not (V_MIN_MV <= v <= V_MAX_MV and I_MIN_MA <= i <= I_MAX_MA):
        return None
    return [v, i, t]


# ---------------- 启动前交互: 选 COM 口 + 波特率 (record 子命令用) ----------------
BAUD_PRESETS = [9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600]


def select_com_port():
    """枚举系统可用串口并交互选择一个, 返回串口名 (COM3 / /dev/ttyUSB0)。

    - 一个串口: 直接采用 (打印提示)
    - 多个串口: 列编号菜单, 回车默认 1
    - 零个串口: sys.exit 提示排查
    """
    from serial.tools import list_ports
    ports = list(list_ports.comports())
    if not ports:
        sys.exit("未检测到可用串口。\n"
                 "检查: USB 转串口是否插入 / 驱动是否装好 / 是否被别的程序占用。")
    if len(ports) == 1:
        p = ports[0]
        print(f"[i] 检测到唯一串口: {p.device}  ({p.description})")
        return p.device
    print("检测到多个串口, 请选择要打开的串口:")
    for i, p in enumerate(ports, 1):
        print(f"  [{i}] {p.device:<10} {p.description}")
    while True:
        s = input(f"输入编号 1-{len(ports)} (回车=1): ").strip()
        if s == "":
            return ports[0].device
        if s.isdigit() and 1 <= int(s) <= len(ports):
            return ports[int(s) - 1].device
        print("输入无效, 请重新输入。")


def select_baud(default_baud=115200):
    """交互选择波特率, 回车用 default_baud。返回波特率整数。"""
    if default_baud in BAUD_PRESETS:
        presets = BAUD_PRESETS
        def_i = BAUD_PRESETS.index(default_baud) + 1
    else:
        presets = [default_baud] + [b for b in BAUD_PRESETS if b != default_baud]
        def_i = 1
    print("选择波特率:")
    for i, b in enumerate(presets, 1):
        tag = "  (默认)" if i == def_i else ""
        print(f"  [{i}] {b}{tag}")
    print("  [c] 自定义")
    while True:
        s = input(f"输入编号或 c (回车=默认 {presets[def_i-1]}): ").strip().lower()
        if s == "":
            return presets[def_i - 1]
        if s == "c":
            while True:
                v = input("输入波特率数值 (如 115200): ").strip()
                if v.isdigit() and int(v) > 0:
                    return int(v)
                print("无效, 请重新输入。")
        if s.isdigit() and 1 <= int(s) <= len(presets):
            return presets[int(s) - 1]
        print("输入无效, 请重新输入。")


def select_csv_file():
    """扫描当前目录的 *.csv, 交互选择一个供 fit 分析, 返回文件名。

    - 无 CSV: 提示先跑 record 采集
    - 有 CSV : 按修改时间倒序列菜单 (最新的一般是刚采完的, 回车默认选它)
    """
    import glob
    import os
    files = sorted(glob.glob("*.csv"),
                   key=lambda p: os.path.getmtime(p), reverse=True)
    if not files:
        sys.exit("当前目录没有 CSV 文件。\n"
                 "请先采集: python soc_bench.py record   (录完 Ctrl+C 会自动存成 CSV)\n"
                 "再拟合:   python soc_bench.py fit")
    if len(files) == 1:
        print(f"[i] 当前目录唯一 CSV: {files[0]}")
        return files[0]
    print("当前目录有多个 CSV (最新的排最前), 请选择要拟合的文件:")
    for i, f in enumerate(files, 1):
        size = os.path.getsize(f)
        tag = "  (最新)" if i == 1 else ""
        print(f"  [{i}] {f:<24} {size/1024:7.1f} KB{tag}")
    while True:
        s = input(f"输入编号 1-{len(files)} (回车=最新 {files[0]}): ").strip()
        if s == "":
            return files[0]
        if s.isdigit() and 1 <= int(s) <= len(files):
            return files[int(s) - 1]
        print("输入无效, 请重新输入。")


# ---------------- 子命令: record ----------------
def cmd_record(args):
    try:
        import serial
    except ImportError:
        sys.exit("缺少 pyserial: pip install pyserial")

    # 串口参数: 命令行已给 -> 直接用; 缺省 -> 启动前交互选择
    port = args.port
    baud = args.baud
    if port is None:
        port = select_com_port()
    if baud is None:
        baud = select_baud(default_baud=115200)

    try:
        ser = serial.Serial(port, baud, timeout=1.0)
    except serial.SerialException as e:
        sys.exit(f"无法打开 {port}: {e}\n"
                 f"检查: 端口号是否正确 / 是否被其他程序占用 / 是否已插上 USB 转串口")

    print(f"[i] 打开 {port} @ {baud} -> {args.out}")
    print("=" * 64)
    print("操作流程 (本次采样将做一阶 RC 参数标定):")
    print("  1. 电芯已静置稳定 (满电或目标 SOC)")
    print("  2. 现在开始记录... 请等 5s 让基线稳定")
    print("  3. 接入电子负载恒流放电 (1C=3.35A 推荐), 维持 60~120s")
    print("  4. 断开负载 (瞬间拔掉/关输出), 保持静置 >=10min")
    print("  5. Ctrl+C 停止")
    print("=" * 64)

    t0 = None
    n_ok = n_bad = 0
    with open(args.out, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["time_iso", "elapsed_s", "V_mV", "I_mA", "T_0p1C"])
        try:
            while True:
                raw = ser.readline()
                if not raw:
                    continue
                text = raw.decode("utf-8", errors="replace")
                vals = parse_line(text)
                now = datetime.datetime.now()
                if vals is None:
                    n_bad += 1
                    continue
                if t0 is None:
                    t0 = now
                el = (now - t0).total_seconds()
                w.writerow([now.isoformat(timespec="milliseconds"),
                            f"{el:.3f}"] + vals)
                f.flush()
                n_ok += 1
                v, i, t = vals
                sys.stdout.write(f"\r[{n_ok}帧|丢{n_bad}] V={v}mV I={i:+d}mA "
                                 f"t={el/60:6.1f}min   ")
                sys.stdout.flush()
        except KeyboardInterrupt:
            pass
    print(f"\n[i] 已保存 {n_ok} 帧 -> {args.out}")
    print("[i] 下一步拟合: python soc_bench.py fit --csv " + args.out)


# ---------------- 子命令: fit ----------------
def cmd_fit(args):
    if args.csv is None:
        args.csv = select_csv_file()
    ts, vs, iv = [], [], []
    with open(args.csv, newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            try:
                ts.append(float(row["elapsed_s"]))
                vs.append(float(row["V_mV"]))
                iv.append(float(row["I_mA"]))
            except (KeyError, ValueError):
                continue
    ts = np.array(ts); vs = np.array(vs); iv = np.array(iv)
    if len(ts) < 50:
        sys.exit(f"有效帧太少 ({len(ts)}), 请确认 CSV 列: elapsed_s,V_mV,I_mA")

    # 1) 定位放电段与断电点
    #    把连续 |I|>LOAD_MA 的帧合并成段, 丢弃 < MIN_LOAD_FRAMES 帧(1s)的短段
    #    (接触弹片/INA226 校准尖峰等毛刺), 取最后一个有效段做 HPPC 放电脉冲.
    #    旧实现取"最后一帧 loaded"直接回溯, 静置段里一个 >200mA 的瞬态毛刺
    #    就会被误当放电段, R0/R1/tau 全部算错.
    MIN_LOAD_FRAMES = 5
    loaded = np.abs(iv) > LOAD_MA
    ld = np.diff(loaded.astype(np.int8), prepend=0, append=0)
    l_starts = np.where(ld == 1)[0]
    l_ends   = np.where(ld == -1)[0]
    segs = [(int(s), int(e)) for s, e in zip(l_starts, l_ends)
            if e - s >= MIN_LOAD_FRAMES]           # [s, e) 半开
    if not segs:
        sys.exit(f"未找到带载段 (|I|>{LOAD_MA}mA 且持续>={MIN_LOAD_FRAMES/5:.0f}s)。"
                 "确认 CSV 里真的有放电过程?")
    seg_start, seg_end_ex = segs[-1]
    seg_end = seg_end_ex - 1                        # 最后一段带载的末帧下标

    # 方向由电流符号自动判: 全链路约定"放电为正、充电为负"(与 MCU/INA226 一致)。
    # 正负都能算 —— R0 = ΔV/I 在两种方向下分子分母同号, 结果都是正值。
    i_pulse = float(np.mean(iv[seg_start:seg_end + 1]))
    if abs(i_pulse) < 1e-6:
        sys.exit("最后一段平均电流≈0, 无法算内阻。确认 CSV 里真的有充/放电过程?")
    charge = i_pulse < 0

    # ---- 静置判据: 滑动中值滤波 + 自适应阈值 ----
    # 逐帧 |I|<IDLE_MA 会被单帧噪声打碎: 零漂 13mA + 白噪 ±8mA 时约 1/5 的静置帧
    # 越过 20mA 门槛, "最长连续静置段"就不再是最末端的稳态段, V_inf 会取到刚断电
    # 那一段 -> R1/tau 全错。零漂本身也可能逼近 IDLE_MA, 故阈值按基线自适应抬高。
    not_load = np.ones(len(iv), dtype=bool)
    not_load[seg_start:seg_end + 1] = False
    bias = float(np.median(np.abs(iv[not_load]))) if np.any(not_load) else 0.0
    idle_thr = max(IDLE_MA, 2.0 * bias)
    if idle_thr > IDLE_MA * 1.1:
        print(f"[i] 静置基线 {bias:+.1f}mA, 静置判据自动抬到 {idle_thr:.1f}mA "
              f"(带载判据 {LOAD_MA}mA)")
    idle_mask = smooth_abs(iv, ts) < idle_thr

    # 断电点: 放电段结束后第一个静置帧
    relax_idx = seg_end + 1
    while relax_idx < len(iv) and not idle_mask[relax_idx]:
        relax_idx += 1
    if relax_idx >= len(ts):
        sys.exit("放电后没有静置段 (需要断电后静置 >=10min)")
    t_off = ts[seg_end]            # 断电时刻(最后一帧带载)
    relax_ts = ts[relax_idx:]
    relax_vs = vs[relax_idx:]
    if not np.any(idle_mask[relax_idx:]):
        sys.exit("断电后未检测到静置帧, 请确认负载已断开")

    # 2) R0: 断电瞬间电压跳变 (末帧带载电压 -> 第一帧静置电压)
    seg5 = max(seg_start, seg_end - 4)               # 放电末 5 帧(~1s)
    v_load_avg = float(np.mean(vs[seg5:seg_end + 1]))
    v_relax_1st = float(vs[relax_idx])
    dv_r0 = v_relax_1st - v_load_avg                 # mV, 断电回跳
    r0_mohm = dv_r0 / i_pulse * 1000.0

    # 3) 回弹拟合: v_inf = 静置末尾 60s 实测均值, fit_relax 在其上做 ln 回归。
    #    不迭代不外推 —— 真实电池回弹含多阶/扩散, 尾部实测是 OCV 可靠下界。
    sel = idle_mask[relax_idx:]
    v_inf = tail_mean(ts, vs, relax_idx, len(ts), 60.0, idle_mask)
    ft = fit_relax(relax_ts[sel] - t_off, relax_vs[sel], v_inf, v_relax_1st)
    if ft is None:
        sys.exit("静置恢复段数据不足 (需要断电后静置 >=10min)")
    amp, tau, r2 = ft["amp_mv"], ft["tau_s"], ft["r2"]

    # 4) R1: 幅度/电流, 再按 RC 饱和系数修正。
    #    脉冲时长 T 内极化只发展到 I*R1*(1-exp(-T/tau)), 直接用 amp/I 会把 R1
    #    低估到 sat 倍 —— 90s 脉冲配 80s 的 tau, sat 只有 0.67, R1 会被低估 1/3。
    #    T >= 3*tau 时 sat>0.95, 修正可忽略。
    t_pulse = ts[seg_end] - ts[seg_start]
    sat = 1.0 - float(np.exp(-t_pulse / tau))
    r1_mohm = amp / abs(i_pulse) * 1000.0 / max(sat, 0.05)
    c1_farad = tau / (r1_mohm * 1e-3)

    print("=" * 66)
    print(f"脉冲电流 I      : {i_pulse/1000:+.2f} A  ({'充电' if charge else '放电'})"
          f"   (脉冲 {t_pulse:.0f}s = {t_pulse/tau:.2f}*tau)")
    print(f"断电时刻 t_off  : {t_off:.1f} s")
    print(f"放电末电压       : {v_load_avg:.1f} mV")
    print(f"断电后第1帧      : {v_relax_1st:.1f} mV  (跳变 {dv_r0:+.1f} mV)")
    print(f"静置末 V_inf     : {v_inf:.1f} mV   (回弹幅度 {amp:.1f} mV, 尾部60s实测)")
    print("-" * 66)
    print(f"R0  (欧姆内阻)   : {r0_mohm:7.1f} mΩ")
    print(f"R1  (极化电阻)   : {r1_mohm:7.1f} mΩ   (RC 饱和系数 sat={sat:.2f}, "
          f"已按 1/sat 修正)")
    print(f"tau (时间常数)   : {tau:7.1f} s   (拟合 R2={r2:.3f}, {ft['n']} 帧)")
    print(f"C1  (极化电容)   : {c1_farad:7.0f} F")
    print(f"R0+R1     (饱和) : {r0_mohm + r1_mohm:7.1f} mΩ")
    print(f"R0+R1*sat (本次) : {r0_mohm + r1_mohm*sat:7.1f} mΩ   <- 本次脉冲末端的实际压降比")
    print("=" * 66)
    print("注: R0 用断电后第1帧(5Hz=><=200ms), 已含少量RC恢复, 可能低估若干%;")
    print("    准确 R0 需要更高采样率或专门的脉冲设备。")
    if (ts[-1] - t_off) / tau < 5.0:
        print(f"[!] 静置 {(ts[-1]-t_off)/60:.0f}min 仅 {(ts[-1]-t_off)/tau:.1f}*tau, "
              f"残余极化未消完, OCV 与 tau 精度受限; 建议静置 >= 5*tau = {5*tau:.0f}s")
    if sat < 0.95:
        print(f"[!] 脉冲 {t_pulse:.0f}s 只到 {t_pulse/tau:.2f}*tau, RC 未饱和 (sat={sat:.2f}), "
              f"R1 修正后误差放大 {1/sat:.2f} 倍。")
        print(f"    想让该项修正可忽略, 脉冲需放到 >= 3*tau = {3*tau:.0f}s "
              f"(或改用 stair 一次跑多档)。")

    # 画图
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(10, 5))
        ax.plot(ts, vs, lw=0.8, color="#185FA5", label="V (mV)")
        ax.set_xlabel("time (s)")
        ax.set_ylabel("V (mV)", color="#185FA5")
        ax.axvline(t_off, color="#D85A30", ls="--", lw=1, label="load off")
        ax.plot(t_off, v_relax_1st, "o", color="#993C1D")
        ax.axhline(v_inf, color="#0F6E56", ls=":", lw=1)
        # 拟合曲线叠加
        tfit = np.linspace(0, np.min([120, float(ts[-1] - t_off)]), 200)
        # 用带符号的回弹幅度: 放电端压向 OCV 爬升, 充电向 OCV 回落, 同一式通吃
        vfit = v_inf - (v_inf - v_relax_1st) * np.exp(-tfit / tau)
        ax.plot(t_off + tfit, vfit, color="#0F6E56", lw=2,
                label=f"fit: R1={r1_mohm:.0f}mΩ tau={tau:.0f}s")
        ax2 = ax.twinx()
        ax2.plot(ts, iv / 1000.0, lw=0.6, color="#888888", alpha=0.7)
        ax2.set_ylabel("I (A)", color="#5F5E5A")
        ax.legend(loc="lower right", fontsize=9)
        ax.grid(alpha=0.3)
        png = args.csv.rsplit(".", 1)[0] + "_fit.png"
        fig.savefig(png, dpi=110, bbox_inches="tight")
        print(f"图已存: {png}")
    except ImportError:
        print("(未装 matplotlib, 跳过出图)")



# ================= 档位计划文件 (soc_load.py 的产物) =================
# 为什么需要它: stair 以前只能靠"电流台阶"猜档位, --capacity/--init-soc/--expect
# 全靠人记着当时设了什么再重打一遍。记错 --init-soc, 整套 SOC 标定就系统性平移,
# 而且数据本身看不出来。soc_load.py 现在会把当时的计划落成 csv+json, 这里把它
# 认出来, 既省输入, 也能拿"计划"和"实测"对拍。
PLAN_SUFFIXES = ("_plan.json", "_plan.csv", "_segments.json",
                 "_load_segments.csv", "_charge_segments.csv", "_segments.csv")

# 计划文件的版本标签。soc_load.py 落盘时写进 format 字段; 这里校验一遍
# —— 将来语义改了就把版本号往上抬, 老脚本看到不认识的版本会出声,
# 而不是闷头按旧语义硬解。
PLAN_FORMAT = 'soc-plan/1'


def find_plan_file(csv_path, explicit=None):
    """找档位计划文件。

    顺序: --plan 显式指定 > 与采集 CSV **同前缀**的常见命名 > 同目录下唯一的
    *_plan.json / *_segments.json。都不中返回 None(退回老路: 全靠电流台阶猜)。
    """
    if explicit:
        # 逃生口: 目录里躺着一份对不上的计划时, 允许显式关掉
        # (退回"全靠电流台阶猜"的老路, 容量/起始 SOC 自己给)
        if str(explicit).strip().lower() in ("none", "off", "-", "no"):
            print("[i] --plan %s: 关掉档位计划识别, 容量/起始 SOC 请自己给" % explicit)
            return None
        if not os.path.exists(explicit):
            sys.exit("--plan 指定的文件不存在: %s" % explicit)
        return explicit
    d = os.path.dirname(os.path.abspath(csv_path))
    stem = os.path.basename(csv_path)
    if "." in stem:
        stem = stem.rsplit(".", 1)[0]
    for suf in PLAN_SUFFIXES:
        p = os.path.join(d, stem + suf)
        if os.path.exists(p):
            return p
    import glob
    cands = sorted(glob.glob(os.path.join(d, "*_plan.json"))
                   + glob.glob(os.path.join(d, "*_segments.json")))
    if len(cands) == 1:
        # 兜底命中要出声: soc_load.py 的默认输出名(load_segments/charge_segments)
        # 与采集文件的前缀本来就不一样, 所以兜底是常用路径; 但万一同目录里躺着
        # 别的数据集的计划, 静默采用会把容量/起始 SOC 全带错 —— 必须让人看见。
        print("[!] 没找到与 %s 同前缀的计划文件, 但同目录有唯一的 %s, 先按它算;"
              " 下面的逐档对拍会校验对不对得上, 对不上请用 --plan 明确指定"
              % (stem, os.path.basename(cands[0])))
        return cands[0]
    return None


def load_plan(path):
    """读档位计划 -> dict, 读不了返回 None。

    支持两种: soc_load.py 新写的 .json(推荐, 字段全) 与老的 *_segments.csv
    (只有逐档 soc_from/soc_to/mah/i_set_a/pulse_s, 容量与起始 SOC 由它反推)。
    """
    if not path or not os.path.exists(path):
        return None
    segs_raw = []
    meta = {}
    if path.lower().endswith(".json"):
        try:
            d = json.load(open(path, encoding="utf-8"))
        except (ValueError, OSError):
            return None
        if not isinstance(d, dict) or not d.get("segments"):
            return None
        fmt = d.get("format")
        if fmt and fmt != PLAN_FORMAT:
            print("[!] %s 的计划版本是 %r, 本脚本按 %r 解析 —— 字段含义可能已"
                  "经变了, 结果请对拍后再用"
                  % (os.path.basename(path), fmt, PLAN_FORMAT))
        meta = d
        for s in d["segments"]:
            segs_raw.append({
                "seg": s.get("seg"),
                "soc_from": s.get("soc_from"), "soc_to": s.get("soc_to"),
                "mah": s.get("mah"), "i_set_a": s.get("i_set_a"),
                "pulse_s": s.get("pulse_s"), "done": bool(s.get("done", True)),
            })
    else:
        try:
            with open(path, newline="", encoding="utf-8") as f:
                for r in csv.DictReader(f):
                    try:
                        segs_raw.append({
                            "seg": int(float(r["seg"])),
                            "soc_from": float(r["soc_from"]),
                            "soc_to": float(r["soc_to"]),
                            "mah": float(r["mah"]),
                            "i_set_a": float(r["i_set_a"]),
                            "pulse_s": float(r["pulse_s"]),
                            "done": True,
                        })
                    except (KeyError, ValueError, TypeError):
                        continue
        except OSError:
            return None
        if not segs_raw:
            return None
        meta = {"mode": None,
                "init_soc": segs_raw[0]["soc_from"],
                "to_soc": segs_raw[-1]["soc_to"]}

    init = meta.get("init_soc", segs_raw[0]["soc_from"])
    to = meta.get("to_soc", segs_raw[-1]["soc_to"])
    if init is None:
        init = segs_raw[0]["soc_from"]
    if to is None:
        to = segs_raw[-1]["soc_to"]
    # 容量: 老 csv 没记, 用 Σmah / |ΔSOC| 反推(计划本身就是这么算出来的, 自洽)
    cap = meta.get("capacity_mah")
    if cap is None and init is not None and to is not None:
        span = abs(float(init) - float(to))
        if span > 1e-9:
            cap = sum(float(s["mah"] or 0) for s in segs_raw) / (span / 100.0)
    mode = meta.get("mode")
    if mode is None and init is not None and to is not None:
        mode = "charge" if float(to) > float(init) else "discharge"
    return {
        "path": path, "source": "json" if path.lower().endswith(".json") else "csv",
        "mode": mode, "capacity_mah": cap, "init_soc": init, "to_soc": to,
        "aborted": bool(meta.get("aborted")),
        "n_planned": int(meta.get("segments_planned") or len(segs_raw)),
        "n_done": sum(1 for s in segs_raw if s["done"]),
        "ocv_table_mv": meta.get("ocv_table_mv"),
        "segments": segs_raw,
    }


def guess_init_soc(ts, vs, iv, first_load_s, tail_s, idle_mask, tbl):
    """开录后**第一段静置**的末尾电压 -> OCV 表反查起始 SOC。

    只在没有计划文件时兜底用。返回 (soc_pct, ocv_mv) 或 (None, None)。
    """
    if first_load_s is None or first_load_s < 5:
        return None, None
    ocv0 = tail_mean(ts, vs, 0, first_load_s, tail_s, idle_mask)
    if ocv0 is None:
        return None, None
    return soc_from_ocv(ocv0, tbl), ocv0


def plan_xcheck(plan, segs, rows, args, ts, vs, iv, idle_mask, warn, L):
    """实测 vs 计划逐档对拍。不一致就进 warn(醒目), 一致就打一行汇总。

    这条检查的价值: 计划是"当时打算做什么", 实测是"实际发生了什么"。两者分家
    通常意味着 电流源没按设定出流 / 时序被截停 / 接线换过 —— 而这些光看拟合
    结果(R0/R1/tau)是看不出来的, 只会悄悄把参数带偏。
    """
    if not plan:
        return
    n_mine = len(segs)
    L.append("档位计划对拍 (计划 %d 档 / 标 done %d 档 / 实测识别 %d 档, 中止=%s):"
             % (plan["n_planned"], plan["n_done"], n_mine,
                "是" if plan["aborted"] else "否"))
    if plan["n_done"] != n_mine:
        if plan["n_done"] > n_mine:
            warn.append("计划跑完 %d 档, 实测只识别出 %d 档 —— 差额档位多半被保护"
                        "截停/时长太短被当毛刺滤掉, 可放宽 --min-pulse-s 或核对"
                        "--i-on" % (plan["n_done"], n_mine))
        else:
            warn.append("实测识别 %d 档 > 计划 %d 档 —— 电流台阶里混进了额外脉冲"
                        "(接线抖动/设备自身动作?), 核对 --i-on/--min-pulse-s"
                        % (n_mine, plan["n_done"]))
    # 比的是**幅值**: 计划里存的是用户输入的电流幅值(充电也是正的), 而实测
    # i_a / dq_mah 在充电时是负的(全链路约定"放电为正、充电为负")。按幅值比
    # 才不会把正常充电误报成 200% 偏差; 方向一致性由上面的 mode 单独比。
    by_seg = {r["seg"]: r for r in rows}
    worst_i = worst_t = 0.0
    for k, ps in enumerate(plan["segments"], 1):
        r = by_seg.get(k)
        if r is None:
            continue
        pi, pt, pm = ps.get("i_set_a"), ps.get("pulse_s"), ps.get("mah")
        pi = abs(pi) if pi else None
        pm = abs(pm) if pm else None
        ri, rm = abs(r["i_a"]), abs(r["dq_mah"])
        di = abs(ri - pi) / pi * 100.0 if pi else 0.0
        dt = abs(r["t_pulse_s"] - pt) / pt * 100.0 if pt else 0.0
        dm = abs(rm - pm) / pm * 100.0 if pm else 0.0
        worst_i, worst_t = max(worst_i, di), max(worst_t, dt)
        if di > 5.0:
            warn.append("第 %d 档实测电流幅值 %.3fA vs 计划 %.3fA (差 %+.1f%%): 电流源"
                        "没按设定出流? 该档的 mAh 与 SOC 跨度会跟着偏"
                        % (k, ri, pi, (ri - pi) / pi * 100.0))
        if dt > 5.0:
            warn.append("第 %d 档实测时长 %.0fs vs 计划 %.0fs (差 %+.1f%%): 时序被"
                        "截停或轮询粒度偏大" % (k, r["t_pulse_s"], pt,
                                              (r["t_pulse_s"] - pt) / pt * 100.0))
        if dm > 5.0:
            warn.append("第 %d 档实测容量幅值 %.0fmAh vs 计划 %.0fmAh (差 %+.1f%%)"
                        % (k, rm, pm, (rm - pm) / pm * 100.0))
        L.append("  档%-2d  电流幅值 计划%.3f/实测%.3f (%+5.1f%%)   "
                 "时长 计划%.0f/实测%.0f s (%+5.1f%%)   "
                 "容量幅值 计划%.0f/实测%.0f mAh (%+5.1f%%)"
                 % (k, pi, ri, (ri - pi) / pi * 100.0 if pi else 0,
                    pt, r["t_pulse_s"],
                    (r["t_pulse_s"] - pt) / pt * 100.0 if pt else 0,
                    pm, rm, (rm - pm) / pm * 100.0 if pm else 0))
    # 计划方向 vs 实测方向: 反了说明当时不是按这份计划跑的(拿错文件?)
    if plan.get("mode"):
        n_neg = sum(1 for r in rows if r["i_a"] < 0)
        mine = "charge" if n_neg > len(rows) / 2 else "discharge"
        if mine != plan["mode"]:
            warn.append("计划说是 %s, 但实测电流符号是 %s —— 大概率拿错了计划"
                        "文件, 下面的逐档对拍仅供参考"
                        % ("充电" if plan["mode"] == "charge" else "放电",
                           "充电(电流为负)" if mine == "charge" else "放电(电流为正)"))
    # 起始 SOC: 计划值 vs "开录首段静置电压反查" —— 这条最能说明整体平移
    soc_g, ocv_g = guess_init_soc(ts, vs, iv, segs[0][0], args.ocv_tail,
                                  idle_mask,
                                  plan.get("ocv_table_mv") or STAIR_OCV_MV)
    if soc_g is not None and plan.get("init_soc") is not None:
        d = plan["init_soc"] - soc_g
        L.append("  起始 SOC  计划 %.1f%% / 首段静置电压 %.1fmV 反查 %.1f%%  (差 %+.1f%%)"
                 % (plan["init_soc"], ocv_g, soc_g, d))
        if abs(d) > 3.0:
            warn.append("起始 SOC 计划 %.1f%% 但开录静置电压只对应 %.1f%% (差 %+.1f%%): "
                        "要么这次不是从计划的那个点开录的, 要么 OCV 表与电芯不符 —— "
                        "整条 SOC 曲线会整体平移 %.1f%%"
                        % (plan["init_soc"], soc_g, d, -d))
    if worst_i <= 5.0 and worst_t <= 5.0:
        L.append("  -> 实测与计划吻合 (电流最大偏差 %.1f%%, 时长最大偏差 %.1f%%)"
                 % (worst_i, worst_t))


# ================= 子命令: stair (N 档周期: 逐档 R1 + OCV + SOC) =================
# 电子负载侧自动跑 "恒流放电 T 秒 -> 断开静置 T 秒" 重复 N 次, 脚本不干预硬件,
# 只把录下来的 CSV 切成 N 档, 每档独立拟合一阶 RC, 并把各档前后 OCV 与安时积分
# 得到的 SOC 配成 OCV-SOC 表。

def load_series(path):
    """读 record 输出的 CSV -> (t, V, I, T) 四个 np.array。

    T 列缺失时填 0 并提示(老 CSV 兼容); 有效帧过少直接退出。
    """
    ts, vs, iv, tt = [], [], [], []
    no_temp = False
    with open(path, newline="", encoding="utf-8") as f:
        rd = csv.DictReader(f)
        if not rd.fieldnames or "T_0p1C" not in rd.fieldnames:
            no_temp = True
        for row in rd:
            try:
                ts.append(float(row["elapsed_s"]))
                vs.append(float(row["V_mV"]))
                iv.append(float(row["I_mA"]))
            except (KeyError, ValueError, TypeError):
                continue
            try:
                tt.append(float(row["T_0p1C"]))
            except (KeyError, ValueError, TypeError):
                tt.append(0.0)
    if len(ts) < 50:
        sys.exit(f"有效帧太少 ({len(ts)}), 确认 CSV 列: elapsed_s,V_mV,I_mA")
    if no_temp:
        print("[!] CSV 无 T_0p1C 列, 温度按 0 处理 (不影响 R/OCV/SOC)")
    return (np.array(ts), np.array(vs), np.array(iv), np.array(tt))


def load_series_mcu(path):
    """同 load_series, 额外读 MCU 回传的 SOC_0p01 列。

    返回 (t, V, I, T, mcu_soc): mcu_soc 为 float % 数组 —— 是 MCU 端自己的
    SOC 估计 (EKF/安时积分, 0.01% 精度), 供 stair 与安时积分参考并排对比,
    从而离线评估 MCU 滤波行为/迭代参数。CSV 无该列(老固件录制的文件)或某帧
    字段无效(空/非数)时对应帧为 NaN。
    """
    ts, vs, iv, tt, ms = [], [], [], [], []
    no_soc = False
    with open(path, newline="", encoding="utf-8") as f:
        rd = csv.DictReader(f)
        no_soc = "SOC_0p01" not in (rd.fieldnames or [])
        for row in rd:
            try:
                ts.append(float(row["elapsed_s"]))
                vs.append(float(row["V_mV"]))
                iv.append(float(row["I_mA"]))
            except (KeyError, ValueError, TypeError):
                continue
            try:
                tt.append(float(row["T_0p1C"]))
            except (KeyError, ValueError, TypeError):
                tt.append(0.0)
            if no_soc:
                ms.append(float("nan"))
            else:
                try:
                    ms.append(float(row["SOC_0p01"]) / 100.0)   # 0.01% -> %
                except (KeyError, ValueError, TypeError):
                    ms.append(float("nan"))
    if len(ts) < 50:
        sys.exit(f"有效帧太少 ({len(ts)}), 确认 CSV 列: elapsed_s,V_mV,I_mA")
    return (np.array(ts), np.array(vs), np.array(iv), np.array(tt), np.array(ms))


def mcu_at(mcu, i):
    """mcu(float%数组或None) 在索引 i 的值, 无效/越界 -> None。"""
    if mcu is None or i is None or i < 0 or i >= len(mcu):
        return None
    v = mcu[i]
    return None if np.isnan(v) else round(float(v), 2)


def soc_cmp_report(ts, vs, iv, soc, mcu_soc, args, out, load_mask):
    """MCU 回传 SOC vs 安时积分参考: 打印汇总 + 落盘逐帧明细 CSV。

    为什么要单独出这一份: stair 主表里 MCU 列只是每档 3 个点, 看不出全局精度。
    这里按帧算 RMSE/偏差/最差区间, 并额外给出「扣除初始偏移」后的漂移指标 ——
    初偏反映的是「起始 SOC 假设 / 上电 OCV 查表」的差异, 不是算法精度; 产品真正
    关心的是从起点往后漂了多少。

    返回 dict(打印用的指标), 无 MCU 列时返回 None。
    """
    if mcu_soc is None:
        return None
    m = ~np.isnan(mcu_soc)
    if not np.any(m):
        return None
    d = mcu_soc[m] - soc[m]
    t = ts[m]
    ad = np.abs(d)
    bias0 = float(mcu_soc[m][0] - soc[m][0])          # 初始偏移
    dr = d - bias0                                     # 去初偏后的误差序列

    def rms(x):
        return float(np.sqrt(np.mean(x * x)))

    load = load_mask                                   # 带载帧掩码(与档位识别同源)
    load_m, idle_m = load[m], ~load[m]
    buckets = [(">50%", soc[m] > 50), ("20~50%", (soc[m] >= 20) & (soc[m] <= 50)),
               ("<20%", soc[m] < 20)]
    k_worst = int(np.argmax(ad))
    # 抽稀成可读的对比表 (~36 行), 采样点是原始帧, 不是插值
    step_tbl = max(1, int(np.ceil(int(m.sum()) / 36.0)))
    idx_tbl = np.arange(0, len(ts), 1)[m][::step_tbl]
    tbl = []
    for i in idx_tbl:
        mv = mcu_soc[i]
        err = float(mv - soc[i])
        tbl.append({"t_s": ts[i], "v_mv": vs[i], "i_ma": iv[i],
                    "truth": float(soc[i]), "meas": float(mv),
                    "err": err, "err_nb": err - bias0,
                    "load": bool(load_mask[i])})
    lines = []
    lines.append("=" * 96)
    lines.append("SOC 对比: 真实 SOC (参考) vs 测量 SOC (MCU 回传)")
    lines.append(f"  参考(真值): 安时积分 init={args.init_soc:.1f}% "
                 f"cap={args.capacity:.0f}mAh")
    lines.append("  注意      : 这条基准的绝对精度完全取决于 init 取对了没有 "
                 "(见下方注)")
    lines.append(f"  测量      : MCU 自身估计 (EKF/积分, CSV SOC_0p01 列, 0.01% 精度)")
    lines.append(f"{'t(min)':>8} {'V(mV)':>7} {'I(mA)':>7} {'状态':>5} "
                 f"{'真实SOC%':>9} {'测量SOC%':>9} {'误差%':>8} {'去初偏%':>8}")
    for r in tbl:
        lines.append(f"{r['t_s']/60:>8.1f} {r['v_mv']:>7.0f} {r['i_ma']:>7.0f} "
                     f"{('带载' if r['load'] else '静置'):>5} "
                     f"{r['truth']:>9.2f} {r['meas']:>9.2f} "
                     f"{r['err']:>+8.2f} {r['err_nb']:>+8.2f}")
    lines.append("-" * 96)
    lines.append("SOC 精度汇总:")
    lines.append(f"  样本     : {int(m.sum())} 帧 / {t[-1]/60:.0f} min "
                 f"(带载 {int(load_m.sum())} 帧, 静置 {int(idle_m.sum())} 帧)")
    lines.append(f"  初始偏移 : MCU {mcu_soc[m][0]:.2f}% vs 参考 {soc[m][0]:.2f}% "
                 f"-> {bias0:+.2f}%  (上电赋值/查表差异, 非算法误差)")
    lines.append(f"  终点     : MCU {mcu_soc[m][-1]:.2f}% vs 参考 {soc[m][-1]:.2f}% "
                 f"-> {d[-1]:+.2f}%")
    lines.append("-" * 96)
    lines.append(f"  {'指标':<12}{'全部帧':>12}{'带载帧':>12}{'静置帧':>12}")
    for name, fn in (("RMSE", rms), ("平均偏差", lambda x: float(np.mean(x))),
                     ("最大|误差|", lambda x: float(np.max(np.abs(x))))):
        lines.append(f"  {name:<12}{fn(d):>12.2f}"
                     f"{(fn(d[load_m]) if load_m.any() else float('nan')):>12.2f}"
                     f"{(fn(d[idle_m]) if idle_m.any() else float('nan')):>12.2f}")
    lines.append("-" * 96)
    lines.append(f"  {'去初偏后':<12}{'RMSE':>12}{'平均':>12}{'最大':>12}")
    lines.append(f"  {'':<12}{rms(dr):>12.2f}{float(np.mean(dr)):>12.2f}"
                 f"{float(np.max(np.abs(dr))):>12.2f}"
                 "   <- 去掉起点赋值差之后的真实漂移")
    lines.append("-" * 96)
    line = "  SOC 区间  :"
    for name, bk in buckets:
        if bk.any():
            line += f"  {name}: RMSE {rms(d[bk]):5.2f}% (bias {float(np.mean(d[bk])):+5.2f})"
    lines.append(line)
    lines.append(f"  最差点   : 误差 {d[k_worst]:+.2f}% @ t={t[k_worst]/60:.1f}min, "
                 f"参考 SOC {soc[m][k_worst]:.1f}%")
    lines.append("  注: 安时积分参考本身有两大系统项 —— (1) 起始 SOC 假设; "
                 "(2) 静置段电流零漂累积")
    lines.append("      (运行时会提示); 两者的表现形式都是整条曲线平移, 只有 A/B 起点"
                 "一致才不误判 ——")
    lines.append("      所以评估算法要看「去初偏」这一行, 不要看 RMSE 绝对值。")
    lines.append("      只有做了满充锚定 (CV 截止 -> SOC=100%) 之后, 安时积分才是"
                 "可用的真值基准;")
    lines.append("      在那之前这里的差异主要反映「两套 assumed 起点不一致」, "
                 "不等同于算法误差。")
    if abs(bias0) > 0.5:
        lines.append(f"      建议: 想让参考与 MCU 同起点, 重跑加 "
                     f"--init-soc {float(mcu_soc[m][0]):.2f}")

    # 落盘: 每 ~5s 抽一帧, 便于 Excel 查看/画图
    dt_med = float(np.median(np.diff(ts))) if len(ts) > 1 else 1.0
    step = max(1, int(round(5.0 / max(dt_med, 1e-3))))
    p_csv = out + "_soc_cmp.csv"
    try:
        with open(p_csv, "w", newline="", encoding="utf-8") as f:
            f.write("elapsed_s,V_mV,I_mA,soc_ref_pct,soc_mcu_pct,err_pct,"
                    "err_nobias_pct,state\n")
            for i in range(0, len(ts), step):
                mv = mcu_soc[i]
                if np.isnan(mv):
                    continue
                err = float(mv - soc[i])
                f.write(f"{ts[i]:.1f},{vs[i]:.0f},{iv[i]:.0f},{soc[i]:.2f},"
                        f"{mv:.2f},{err:+.2f},{err - bias0:+.2f},"
                        f"{'load' if load[i] else 'idle'}\n")
    except OSError as e:
        warn_note = str(e)
        p_csv = None
    return {"lines": lines, "csv": p_csv, "bias0": bias0,
            "rmse": rms(d), "rmse_nb": rms(dr), "n": int(m.sum())}


def find_pulses(ts, iv, i_on, i_off, min_s):
    """滞回阈值切出带载脉冲: |I|>i_on 进入, |I|<i_off 退出, 时长>=min_s 才保留。

    返回 [(s, e), ...] 闭区间下标(s 和 e 都是带载帧)。滞回 + 时长判据把接触弹跳、
    INA226 校准尖峰等短毛刺和静置段里的孤立尖峰全部滤掉, 不会误判成档位。
    """
    a = np.abs(iv)
    segs, s, in_seg = [], 0, False
    for k in range(len(a)):
        if not in_seg and a[k] > i_on:
            in_seg, s = True, k
        elif in_seg and a[k] < i_off:
            in_seg = False
            e = k - 1                       # k 是第一个退出帧, 不属于带载段
            if ts[e] - ts[s] >= min_s:
                segs.append((s, e))
    if in_seg:
        e = len(a) - 1
        if ts[e] - ts[s] >= min_s:
            segs.append((s, e))
    return segs


def tail_mean(ts, vs, lo, hi, tail_s, mask=None):
    """[lo, hi) 内取末尾 tail_s 秒的电压均值; 段比 tail 短则取整段。

    mask 传入后只对 mask 为 True 的帧求均值 —— 尾部窗口里仍可能落下零星毛刺
    帧(电流尖峰往往带着电压尖峰), 不滤掉会把均值拉偏。过滤后帧太少(<5)视为
    该窗口不可用, 返回 None。
    """
    if hi <= lo:
        return None
    k = max(int(np.searchsorted(ts, ts[hi - 1] - tail_s)), lo)
    idx = np.arange(k, hi)
    if mask is not None:
        idx = idx[mask[idx]]
    if len(idx) < 5:
        return None
    return float(np.mean(vs[idx]))


def smooth_abs(iv, ts, win_s=1.0):
    """|I| 的滑动中值滤波(窗口约 win_s 秒, 取奇数帧)。

    逐帧阈值判静置会被单帧噪声打碎: 零漂 13mA + 白噪 ±8mA 时约 1/5 的静置帧
    越过 20mA 门槛, 连续静置段被切成碎片 -> 需按"末尾窗口"而非最长连续段取值。
    中值滤波把白噪压掉、保留加性零漂和带载台阶, 判定才稳定。
    """
    a = np.abs(np.asarray(iv, dtype=float))
    if len(a) < 3:
        return a
    dt_med = float(np.median(np.diff(ts))) if len(ts) > 1 else 0.2
    win = max(3, int(round(win_s / max(dt_med, 1e-6))))
    win = win + 1 if win % 2 == 0 else win          # 取奇数, 中值有明确中心
    if win >= len(a):
        return np.full_like(a, float(np.median(a)))
    pad = win // 2
    ap = np.pad(a, (pad, win - 1 - pad), mode="edge")
    return np.median(sliding_window_view(ap, win), axis=1)


def fit_relax(t_rel, v_rel, v_inf, v1st_mv):
    """ln 线性回归求 tau (一阶 RC 回弹), v_inf 由调用方给静置末尾实测均值。

    **方向无关**: 放电断电后极化消失, 端压从"低"往 OCV 爬升; 充电断电后也从
    "高"往 OCV 回落。两个方向都是 |v_inf - v(t)| 按 exp(-t/tau) 衰减, 所以下面
    一律取极化幅值(gap/amp 都取绝对值), 带符号的 amp 只用来定方向与画图。
    放电时 sign=+1, gap 与 amp 逐值等于旧写法 —— 老数据的结论一个数都不变。

    动态带截断: 头部 gap>0.98*amp (断电瞬态, 非纯 RC) 与尾部 gap<0.1*amp
    (接近 VBUS 量化噪声 1.25mV, ln 不再线性) 都排除, 只在中间线性区回归。

    特意不做"稳态外推/迭代补残余": 真实电池回弹含多阶 + 扩散分量, 窗口内的
    表观 tau 只代表主极化成份; 任何把 v_inf 当自由参数的外推都可能推出比实测
    尾部还低的"稳态"(实测尾部是 OCV 的可靠下界), 或迭代发散。静置不足时 v_inf
    偏离是事实, 由调用方按 静置时长/tau 警告量化, 而不是让模型去猜。
    """
    if v_inf is None or v1st_mv is None:
        return None
    amp_signed = v_inf - v1st_mv
    if abs(amp_signed) <= 0.5:               # 回弹幅度太小, 拟合没有意义
        return None
    sign = 1.0 if amp_signed > 0 else -1.0
    t = np.asarray(t_rel, dtype=float)
    v = np.asarray(v_rel, dtype=float)
    amp = abs(amp_signed)
    gap = (v_inf - v) * sign
    good = (gap >= 0.10 * amp) & (gap <= 0.98 * amp) & (gap > 0.5)
    n = int(np.sum(good))
    if n < 10:
        return None
    x = t[good]
    y = np.log(gap[good])
    k, b = np.polyfit(x, y, 1)               # ln(gap) = k*t + b, k = -1/tau
    if k >= 0:
        return None
    tau = -1.0 / k
    yhat = b + k * x
    ss_res = float(np.sum((y - yhat) ** 2))
    ss_tot = float(np.sum((y - y.mean()) ** 2))
    return {"vinf_mv": float(v_inf), "amp_mv": float(amp),
            "tau_s": float(tau), "r2": 1.0 - ss_res / max(ss_tot, 1e-12),
            "n": n, "polarity": int(sign)}


def analyze_seg(ts, vs, iv, tt, s, e, next_s, tail_s, idle_mask):
    """分析单档: 带载段 [s, e], 其回弹段右开界 next_s (下一档起点或文件尾)。

    idle_mask 是全局静置标记(平滑电流 + 自适应阈值算出来的), 这里只用它,
    不再逐帧判电流 —— 否则单帧噪声就能把回弹段切碎。
    返回 dict, ok=False 时带 err 说明。单位: 电流 A(放电正), 电阻 mΩ, 电容 F。
    """
    n = len(ts)
    next_s = min(next_s, n)
    core = iv[s + 1:e] if (e - s) >= 3 else iv[s:e + 1]
    i_a = float(np.median(core)) / 1000.0              # 中位数抗毛刺
    t_p = float(ts[e] - ts[s])                         # 带载时长 s
    v_load = float(np.mean(vs[max(s, e - 4):e + 1]))   # 带载末 5 帧均值

    # 断电后第一个"真静置"帧: 跳过关断瞬态与毛刺
    r0 = e + 1
    while r0 < next_s and not idle_mask[r0]:
        r0 += 1
    if r0 >= next_s:
        return {"ok": False, "err": "断电后无静置帧"}
    v_1st = float(vs[r0])                              # 断电瞬跳之后的第一帧

    # R0: 断电瞬间电压跳变 / 电流 (mV/A = mΩ)。放电 I>0 时 v_1st>v_load -> R0>0
    if abs(i_a) < 1e-6:
        return {"ok": False, "err": "电流接近 0"}
    r0_mohm = (v_1st - v_load) / i_a

    # 实测 OCV (尾部窗口均值) 作 OCV 表取值; 静置不足时偏低, 由调用方按
    # rest_s/tau < 5 警告量化 (见 cmd_stair), 模型不做外推猜测。
    v_inf = tail_mean(ts, vs, r0, next_s, tail_s, idle_mask)
    if v_inf is None:
        return {"ok": False, "err": "回弹段末尾静置帧不足"}

    # 回弹段静置帧 (供 ln 回归)
    m = idle_mask[r0:next_s]
    t_rel = ts[r0:next_s][m] - ts[e]
    v_rel = vs[r0:next_s][m]
    if len(t_rel) < 12:
        return {"ok": False, "err": "回弹段静置帧不足"}

    ft = fit_relax(t_rel, v_rel, v_inf, v_1st)
    if ft is None:
        return {"ok": False, "err": "回弹拟合失败 (幅度太小或数据不足)"}
    amp, tau, r2 = ft["amp_mv"], ft["tau_s"], ft["r2"]

    # RC 饱和修正: 脉冲时长 T 内极化只发展到 I*R1*(1-exp(-T/tau)),
    # 直接用 amp/I 会把 R1 低估到 sat 倍。T >= 3*tau 时 sat>0.95, 可忽略。
    sat = 1.0 - float(np.exp(-t_p / tau))
    if sat < 0.05:
        return {"ok": False, "err": f"脉冲过短 (T={t_p:.0f}s < 0.05*tau), R1 外推不可靠"}
    r1_mohm = amp / abs(i_a) / sat
    c1_f = tau / (r1_mohm * 1e-3)

    return {
        "ok": True,
        "i_a": i_a, "t_pulse_s": t_p, "v_load_mv": v_load,
        "v_1st_mv": v_1st, "v_inf_mv": v_inf, "amp_mv": amp,
        "r0_mohm": r0_mohm, "r1_mohm": r1_mohm, "tau_s": tau,
        "c1_f": c1_f, "sat": sat, "r2": r2, "n_fit": ft["n"],
        "rest_s": float(ts[next_s - 1] - ts[e]),
        "temp_c": float(np.mean(tt[s:e + 1])) / 10.0,
        "s": s, "e": e,
    }


def interp_grid(soc_pts, ocv_pts, grid):
    """把实测 (SOC, OCV) 点插值/外推到 rule 网格 grid 上。

    网格端点落在实测范围外时按最近两点局部斜率线性外推, 并统计外推点数
    (外推点的可信度远低于插值点, 调用方需提示用户)。
    """
    order = np.argsort(soc_pts)
    xs = np.asarray(soc_pts, dtype=float)[order]
    ys = np.asarray(ocv_pts, dtype=float)[order]
    out, n_ex = [], 0
    for g in grid:
        if g < xs[0] and len(xs) >= 2:
            k = (ys[1] - ys[0]) / (xs[1] - xs[0])
            out.append(ys[0] + k * (g - xs[0])); n_ex += 1
        elif g > xs[-1] and len(xs) >= 2:
            k = (ys[-1] - ys[-2]) / (xs[-1] - xs[-2])
            out.append(ys[-1] + k * (g - xs[-1])); n_ex += 1
        else:
            out.append(float(np.interp(g, xs, ys)))
    return out, n_ex


def cmd_stair(args):
    if args.csv is None:
        args.csv = select_csv_file()
    # 先找 soc_load.py 留下的档位计划; 找到就能省掉 --capacity/--init-soc/--expect
    plan = load_plan(find_plan_file(args.csv, args.plan))
    ts, vs, iv, tt, mcu_soc = load_series_mcu(args.csv)
    dt = np.diff(ts, prepend=ts[0])

    # 零漂补偿(可选): INA226 零点是加性偏置, 实测静置时也有 +11~14mA 读数。
    # 静置占比大(阶梯标定往往如此)时会积成可观的虚假放电量, 但它同样存在于
    # MCU 侧积分里, 所以默认不补偿, 保持两端一致。
    if args.zero_drift:
        m = np.abs(iv) < args.idle_ma
        if np.any(m):
            bias = float(np.average(iv[m], weights=dt[m]))
            iv = iv - bias
            print(f"[i] 零漂补偿: 静置段加权平均 {bias:+.1f}mA, 全序列已扣除")

    segs = find_pulses(ts, iv, args.i_on, args.i_off, args.min_pulse_s)
    if not segs:
        sys.exit(f"未识别到带载档位 (判据: |I|>{args.i_on}mA 且持续>={args.min_pulse_s}s)。\n"
                 f"确认电子负载在跑周期? 可放宽 --i-on 或 --min-pulse-s。")
    # expect 只依赖识别出的档数, 所以这里就定下来 —— 晚了会先打一句
    # "期望 None 档"再被后面的代码补上, 看着像出错。
    if args.expect is None:
        args.expect = plan["n_planned"] if plan else STAIR_EXPECT
    if len(segs) != args.expect:
        print(f"[!] 识别到 {len(segs)} 档, 期望 {args.expect} 档 "
              f"(可调 --i-on={args.i_on}mA / --min-pulse-s={args.min_pulse_s}s 后重跑)")

    # 静置判据自适应: 零漂(实测 +11~14mA)会逼近甚至超过 IDLE_MA, 固定阈值会把
    # 静置帧全判成"非静置" -> 取不到 V_inf。用带载段之外的电流中位数定基线,
    # 阈值抬到基线的 2 倍(至少 --idle-ma); 它与带载判据 i_on 之间仍差一个数量级,
    # 不会把带载误判成静置。判定本身用滑动中值滤波后的电流, 抗单帧噪声打碎。
    not_load = np.ones(len(iv), dtype=bool)
    for s_, e_ in segs:
        not_load[s_:e_ + 1] = False
    bias = float(np.median(np.abs(iv[not_load]))) if np.any(not_load) else 0.0
    idle_thr = max(args.idle_ma, 2.0 * bias)
    if idle_thr > args.idle_ma * 1.1:
        print(f"[i] 静置基线 {bias:+.1f}mA, 静置判据自动抬到 {idle_thr:.1f}mA "
              f"(带载判据 {args.i_on:.0f}mA, 两者仍差 {args.i_on/idle_thr:.0f} 倍)")
    idle_mask = smooth_abs(iv, ts) < idle_thr

    # ---- 计划文件 / 自动识别 补齐"人脑参数" ----
    # 三者优先序: 命令行显式给的 > 计划文件 > (容量)标称值 / (起始SOC)静置电压反查
    src_cap = "(标称)" if args.capacity is None else "(命令行)"
    src_soc = "(命令行)" if args.init_soc is not None else "(自动)"
    if plan:
        print("[i] 档位计划: %s  (%s, %s, 计划 %d 档/标 done %d 档%s)"
              % (plan["path"], plan["mode"] or "?",
                 ("容量 %.0fmAh" % plan["capacity_mah"]) if plan["capacity_mah"] else "容量?",
                 plan["n_planned"], plan["n_done"],
                 ", 有中止" if plan["aborted"] else ""))
        if args.capacity is None and plan["capacity_mah"]:
            args.capacity, src_cap = float(plan["capacity_mah"]), "计划文件"
        if args.init_soc is None and plan["init_soc"] is not None:
            args.init_soc, src_soc = float(plan["init_soc"]), "计划文件"
    if args.capacity is None:
        args.capacity = STAIR_CAPACITY
    if args.init_soc is None:
        # 兜底: 拿开录后第一段静置的末尾电压去 OCV 表反查起始 SOC。
        # 只有"没有计划文件"时才走到这里 —— 有计划的场合计划值才是权威。
        tbl = ([float(x) for x in str(args.ocv_table).split(",")]
               if args.ocv_table else STAIR_OCV_MV)
        g, ocv0 = guess_init_soc(ts, vs, iv, segs[0][0], args.ocv_tail,
                                 idle_mask, tbl)
        if g is None:
            sys.exit("无法确定起始 SOC: 没有档位计划文件, 开录后第一段静置也不足 "
                     "%g 秒。请显式给 --init-soc, 或用 soc_load.py 跑一次留下计划文件。"
                     % args.ocv_tail)
        args.init_soc, src_soc = g, "首段静置电压反查"
        print("[i] 起始 SOC 未指定 -> 由开录首段静置电压 %.1fmV 反查得 %.2f%% "
              "(来自内置 OCV 表, 表不准这条就不准; 有档位计划时一律以计划为准)"
              % (ocv0, g))
    print("[i] 容量 %.0fmAh (%s) | 起始 SOC %.2f%% (%s) | 期望 %d 档"
          % (args.capacity, src_cap, args.init_soc, src_soc, args.expect))

    # 安时积分 -> 全程 SOC 序列 (与 soc_analyze 同一套约定: 放电为正)
    q_mah = np.cumsum(iv * dt) / 3600.0
    soc = args.init_soc - q_mah / args.capacity * 100.0

    rows, warn = [], []
    _xchk = []
    for k, (s, e) in enumerate(segs, 1):
        next_s = segs[k][0] if k < len(segs) else len(ts)

        # 档前 OCV: 上一段回弹末尾 (第 1 档 = 开录后的初始静置段)
        prev_e = segs[k - 2][1] if k >= 2 else -1
        p_lo, p_hi = prev_e + 1, s
        ocv_before = None
        if p_hi - p_lo >= 5:
            ocv_before = tail_mean(ts, vs, p_lo, p_hi, args.ocv_tail, idle_mask)

        r = analyze_seg(ts, vs, iv, tt, s, e, next_s, args.ocv_tail, idle_mask)
        if not r["ok"]:
            warn.append(f"第 {k} 档跳过: {r['err']}")
            continue

        # R_dc: 表观直流内阻 = (档前 OCV - 带载末端电压) / I。
        # 它不是纯内阻 —— 放电期间 SOC 在降, 分子里混入了 OCV-SOC 曲线斜率项,
        # 恒有 R_dc = R0 + R1*sat + (OCV_before - V_inf)/I, 所以拿它和
        # R0+R1*sat 相减得不到任何独立信息(早期版本那列 dR 就是这么来的)。
        # 它真正的用处: MCU 端"带载电压反推 OCV"要用的正是这个系数。
        r_dc = None
        if ocv_before is not None:
            r_dc = (ocv_before - r["v_load_mv"]) / r["i_a"]

        row = {
            "seg": k,
            "i_a": round(r["i_a"], 3),
            "t_pulse_s": round(r["t_pulse_s"], 1),
            "soc_start": round(float(soc[s]), 2),
            "soc_end": round(float(soc[e]), 2),
            "soc_mid": round(float(soc[(s + e) // 2]), 2),
            "dq_mah": round(float(q_mah[e] - q_mah[s]), 2),
            "q_before_mah": round(float(q_mah[s]), 2),
            "q_after_mah": round(float(q_mah[e]), 2),
            "ocv_before_mv": None if ocv_before is None else round(ocv_before, 1),
            "ocv_after_mv": round(r["v_inf_mv"], 1),
            "t_before_s": round(float(ts[max(s - 1, 0)]), 1),
            "t_after_s": round(float(ts[next_s - 1]), 1),
            "t_0p1C_before": round(float(np.mean(tt[max(0, s - 5):max(s, 1)])), 1),
            "r0_mohm": round(r["r0_mohm"], 1),
            "r1_mohm": round(r["r1_mohm"], 1),
            "rdc_mohm": None if r_dc is None else round(r_dc, 1),
            "tau_s": round(r["tau_s"], 1),
            "c1_f": round(r["c1_f"], 0),
            "sat": round(r["sat"], 3),
            "r2": round(r["r2"], 4),
            "rest_s": round(r["rest_s"], 1),
            "rest_tau": round(r["rest_s"] / r["tau_s"], 2),
            "temp_c": round(r["temp_c"], 1),
            # MCU 回传 SOC (CSV SOC_0p01 列; 无则 None):
            #   start = 带载首帧, end = 带载末帧, tail = 本档静置末帧
            "mcu_soc_start": mcu_at(mcu_soc, s),
            "mcu_soc_end": mcu_at(mcu_soc, e),
            "mcu_soc_tail": mcu_at(mcu_soc, next_s - 1),
        }
        if r["sat"] < 0.95:
            warn.append(f"第 {k} 档脉冲 {r['t_pulse_s']:.0f}s = {r['t_pulse_s']/r['tau_s']:.2f}*tau, "
                        f"RC 未饱和 (sat={r['sat']:.2f}), R1 已按 1/sat 修正但误差放大 "
                        f"{1/r['sat']:.2f} 倍 -> 建议该档脉冲放到 >= 3*tau")
        if row["rest_tau"] < 5.0:
            warn.append(f"第 {k} 档静置 {r['rest_s']:.0f}s 仅 {row['rest_tau']:.1f}*tau, "
                        f"残余极化未消完: OCV_after 偏低(尾部窗口均值含残余), "
                        f"tau 也受牵连 -> 建议该档静置 >= 5*tau = {5*r['tau_s']:.0f}s "
                        f"(tau 为本次拟合值, 若真实更慢需相应加长)")
        if r["r2"] < 0.98:
            warn.append(f"第 {k} 档 tau 拟合 R2={r['r2']:.3f} (<0.98), 回弹幅度仅 "
                        f"{r['amp_mv']:.1f}mV, 量化噪声占比偏高 -> 该档加大电流或延长脉冲")
        rows.append(row)

    if not rows:
        sys.exit("没有任何一档拟合成功:\n  " + "\n  ".join(warn))

    # ---- OCV-SOC 表: 初始静置点 + 每档回弹后的稳态点 ----
    # 每点自带 q_disch_mah / t_s, 打印与绘图都直接取用, 不再按 SOC 反查时间。
    ocv_tbl = []
    if rows and rows[0]["ocv_before_mv"] is not None:
        ocv_tbl.append({"soc_pct": rows[0]["soc_start"],
                        "ocv_mv": rows[0]["ocv_before_mv"],
                        "q_disch_mah": rows[0]["q_before_mah"],
                        "t_s": rows[0]["t_before_s"],
                        "t_0p1C": round(rows[0]["t_0p1C_before"], 1),
                        "src": "init"})
    for r in rows:
        if r["ocv_after_mv"] is not None:
            ocv_tbl.append({"soc_pct": r["soc_end"], "ocv_mv": r["ocv_after_mv"],
                            "q_disch_mah": r["q_after_mah"], "t_s": r["t_after_s"],
                            "t_0p1C": round(r["temp_c"] * 10, 1),
                            "src": f"seg{r['seg']}"})

    # ---- 打印 ----
    out = args.out or args.csv.rsplit(".", 1)[0]
    q_tot = q_mah[-1]
    print("=" * 96)
    print(f"数据源: {args.csv}   {len(ts)} 帧 / {ts[-1]/60:.1f} min   "
          f"容量 {args.capacity:.0f} mAh   起始 SOC {args.init_soc:.1f}%")
    _dir = "充电" if (q_tot < 0) else "放电"
    print(f"识别档位: {len(segs)} 档 (拟合成功 {len(rows)})   方向 {_dir}   "
          f"累计{'放出' if q_tot >= 0 else '充入'} {abs(q_tot):.0f} mAh = "
          f"{abs(q_tot)/args.capacity*100:.1f}% 容量   "
          f"SOC 终点 {args.init_soc + (-1 if q_tot >= 0 else 1)*abs(q_tot)/args.capacity*100:.1f}%")
    plan_xcheck(plan, segs, rows, args, ts, vs, iv, idle_mask, warn, _xchk)
    for _ln in _xchk:
        print(_ln)
    print("-" * 96)
    print(f"{'档':>2} {'I(A)':>7} {'时长s':>6} {'SOC%中':>7} {'R0':>6} {'R1':>6} "
          f"{'tau':>6} {'C1(F)':>7} {'Rdc':>6} {'sat':>5} {'R2':>6} "
          f"{'静置/tau':>8} {'T(C)':>5}")
    for r in rows:
        print(f"{r['seg']:>2} {r['i_a']:>7.2f} {r['t_pulse_s']:>6.1f} {r['soc_mid']:>7.1f} "
              f"{r['r0_mohm']:>6.1f} {r['r1_mohm']:>6.1f} {r['tau_s']:>6.1f} "
              f"{r['c1_f']:>7.0f} "
              f"{(r['rdc_mohm'] if r['rdc_mohm'] is not None else float('nan')):>6.1f} "
              f"{r['sat']:>5.2f} {r['r2']:>6.3f} {r['rest_tau']:>8.1f} {r['temp_c']:>5.1f}")
    print("  注: R0/R1/Rdc 单位 mΩ; Rdc = (档前OCV - 带载末压)/I, 含 SOC 下降带来的 "
          "OCV 斜率项,")
    print("      不是纯内阻, 但正是 MCU 端\"带载电压反推 OCV\"要用的那个系数。")

    # MCU 回传 SOC vs 安时参考并排 —— 离线对比/迭代 EKF 的入口
    if mcu_soc is not None and np.any(~np.isnan(mcu_soc)):
        print("MCU 回传 SOC (EKF/积分) vs 安时参考, 各档 (差值=MCU-安时):")
        print(f"  {'档':>3} {'安时 start->end':>16} {'MCU start->end':>17} "
              f"{'Δ带载末':>8} {'Δ静置末':>8}")
        for r in rows:
            if r["mcu_soc_start"] is None:
                continue
            d_on = r["mcu_soc_end"] - r["soc_end"]
            d_tl = (r["mcu_soc_tail"] - r["soc_end"]
                    if r["mcu_soc_tail"] is not None else None)
            print(f"  {r['seg']:>3} {r['soc_start']:>7.2f}->{r['soc_end']:<8.2f} "
                  f"{r['mcu_soc_start']:>7.2f}->{r['mcu_soc_end']:<8.2f} "
                  f"{d_on:>+8.2f} "
                  f"{(f'{d_tl:+.2f}' if d_tl is not None else '--'):>8}")
        print("  注: MCU 值来自固件回传 (CSV SOC_0p01 列, 0.01% 精度); 静置末差"
              "反映 900s OCV 重同步与 EKF 收敛质量")
    print("-" * 96)
    print("OCV-SOC 表 (SOC 由安时积分定, OCV 取各段静置末尾 "
          f"{args.ocv_tail:.0f}s 均值):")
    print(f"  {'SOC%':>6} {'OCV(mV)':>9} {'累计放电(mAh)':>12} {'T(C)':>6}  来源")
    for p in ocv_tbl:
        print(f"  {p['soc_pct']:>6.1f} {p['ocv_mv']:>9.1f} {p['q_disch_mah']:>12.1f} "
              f"{p['t_0p1C']/10:>6.1f}  {p['src']}")
    # 按 SOC 升序排开再判单调: 放电时 SOC 递减, 按时间序比等价; 充电时 SOC 递增,
    # 按时间序比就会把完全正常的表误报成"非单调"。
    _pts = sorted(ocv_tbl, key=lambda p: p["soc_pct"])
    mono = all(_pts[i]["ocv_mv"] <= _pts[i + 1]["ocv_mv"] + 1e-9
               for i in range(len(_pts) - 1))
    if not mono:
        warn.append("OCV-SOC 表非单调 (按 SOC 升序 OCV 应单调不减), "
                    "检查静置是否充分 / 是否有反向电流段")

    # 静置零漂累积: 静置段占阶梯标定的大部分时间, 零漂乘上去就是虚假电量
    if not args.zero_drift:
        m = idle_mask
        if np.any(m):
            t_idle = float(np.sum(dt[m]))
            i_bias = float(np.average(iv[m], weights=dt[m]))
            q_drift = i_bias * t_idle / 3600.0
            if abs(q_drift) >= 0.002 * args.capacity:
                warn.append(f"静置段平均电流 {i_bias:+.1f}mA (INA226 零漂), 静置共 "
                            f"{t_idle/60:.0f}min -> 安时积分混进 {q_drift:+.1f} mAh = "
                            f"{q_drift/args.capacity*100:+.2f}% SOC 虚假电量, "
                            f"加 --zero-drift 可扣除 (MCU 侧积分同样受此影响)")
    print("=" * 96)
    for w in warn:
        print("[!] " + w)

    # MCU SOC 精度汇总 (打印在末尾, 避免被长输出压掉)
    cmp_res = soc_cmp_report(ts, vs, iv, soc, mcu_soc, args, out, ~idle_mask)
    if cmp_res:
        print()
        for ln in cmp_res["lines"]:
            print(ln)
        if cmp_res["csv"]:
            print(f"[i] 真实/测量 SOC 对比表 (每5s全量): {cmp_res['csv']}")

    # ---- 落盘 ----
    csv_path = out + "_stair.csv"
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader(); w.writerows(rows)
    ocv_path = out + "_stair_ocv.csv"
    with open(ocv_path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, extrasaction="ignore",
                           fieldnames=["soc_pct", "ocv_mv", "q_disch_mah",
                                       "t_0p1C", "src"])
        w.writeheader(); w.writerows(ocv_tbl)
    json_path = out + "_stair.json"
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump({"meta": {"csv": args.csv, "capacity_mah": args.capacity,
                            "init_soc": args.init_soc, "n_seg": len(segs),
                            "q_total_mah": round(float(q_tot), 1),
                            "i_on_ma": args.i_on, "i_off_ma": args.i_off,
                            "min_pulse_s": args.min_pulse_s,
                            "ocv_tail_s": args.ocv_tail},
                   "segs": rows, "ocv_table": ocv_tbl, "warn": warn},
                  f, indent=2, ensure_ascii=False)
    print(f"[i] 逐档参数: {csv_path}")
    print(f"[i] OCV 表  : {ocv_path}")
    print(f"[i] 汇总JSON: {json_path}")

    if args.emit_c and len(ocv_tbl) >= 2:
        grid = list(range(0, 101, 10))
        vals, n_ex = interp_grid([p["soc_pct"] for p in ocv_tbl],
                                 [p["ocv_mv"] for p in ocv_tbl], grid)
        print(f"\n[i] C 查表数组 (实测 SOC {min(p['soc_pct'] for p in ocv_tbl):.0f}~"
              f"{max(p['soc_pct'] for p in ocv_tbl):.0f}%, 其中 {n_ex} 点为外推):")
        print("static const uint16_t SOC_OCV_MV[%d] = {   "
              "/* SOC 0%%..100%%, step 10%% */" % len(grid))
        line = "   "
        for v in vals:
            line += f" {max(min(int(round(v)), 65535), 0):>5},"
        print(line)
        print("};")
        if n_ex:
            print(f"[!] 有 {n_ex} 个网格点在实测范围外, 是按端点局部斜率外推的, "
                  f"仅用于填表, 建议补测该区间实测点")

    if args.no_plot:
        return
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("(未装 matplotlib, 跳过出图)")
        return

    th = ts / 60.0
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    for k, (s, e) in enumerate(segs, 1):
        a1.axvspan(th[s], th[e], color="#D85A30", alpha=0.15, lw=0)
        a2.axvspan(th[s], th[e], color="#D85A30", alpha=0.15, lw=0)
    a1.plot(th, vs, lw=0.8, color="#185FA5", label="V (mV)")
    for p in ocv_tbl:
        idx = int(np.clip(np.searchsorted(ts, p["t_s"]), 0, len(ts) - 1))
        a1.plot(th[idx], p["ocv_mv"], "o", ms=5, color="#A32D2D", zorder=5)
    a1.set_ylabel("V (mV)"); a1.legend(loc="best", fontsize=8); a1.grid(alpha=0.3)
    a1.set_title("stair timeline (orange bands = load pulses, red dots = OCV points)")
    a2.plot(th, iv / 1000.0, lw=0.8, color="#5F5E5A")
    a2.set_ylabel("I (A)"); a2.set_xlabel("time (min)"); a2.grid(alpha=0.3)
    fig.tight_layout()
    png1 = out + "_stair_timeline.png"
    fig.savefig(png1, dpi=140); plt.close(fig)

    soc_r = [r["soc_mid"] for r in rows]
    has_mcu = mcu_soc is not None and np.any(~np.isnan(mcu_soc))
    if has_mcu:
        # 3 行: 第 3 行整幅留给 SOC 时间曲线 (真实 vs 测量)
        fig = plt.figure(figsize=(11, 9.5))
        gs = fig.add_gridspec(3, 2, hspace=0.42, wspace=0.25)
        a_r = fig.add_subplot(gs[0, 0]); a_tau = fig.add_subplot(gs[0, 1])
        a_dc = fig.add_subplot(gs[1, 0]); a_ocv = fig.add_subplot(gs[1, 1])
        a_soc = fig.add_subplot(gs[2, :])
    else:
        fig, ((a_r, a_tau), (a_dc, a_ocv)) = plt.subplots(2, 2, figsize=(11, 7))
    a_r.plot(soc_r, [r["r0_mohm"] for r in rows], "-o", ms=5, color="#993C1D", label="R0")
    a_r.plot(soc_r, [r["r1_mohm"] for r in rows], "-s", ms=5, color="#185FA5", label="R1")
    a_r.set_ylabel("mOhm"); a_r.legend(fontsize=8); a_r.grid(alpha=0.3)
    a_r.set_title("R0 / R1 vs SOC")
    a_tau.plot(soc_r, [r["tau_s"] for r in rows], "-o", ms=5, color="#0F6E56")
    a_tau.set_ylabel("s"); a_tau.grid(alpha=0.3); a_tau.set_title("tau vs SOC")
    a_dc.plot(soc_r, [r["r0_mohm"] + r["r1_mohm"] * r["sat"] for r in rows],
              "-o", ms=5, color="#3C3489", label="R0+R1*sat (pulse end)")
    a_dc.plot(soc_r, [r["rdc_mohm"] for r in rows], "--x", ms=5,
              color="#888780", label="R_dc (incl. dOCV/dSOC)")
    a_dc.set_ylabel("mOhm"); a_dc.legend(fontsize=8); a_dc.grid(alpha=0.3)
    a_dc.set_title("apparent DC resistance vs SOC")
    a_ocv.plot([p["soc_pct"] for p in ocv_tbl], [p["ocv_mv"] for p in ocv_tbl],
               "-o", ms=5, color="black")
    a_ocv.set_xlabel("SOC (%)"); a_ocv.set_ylabel("OCV (mV)")
    a_ocv.grid(alpha=0.3); a_ocv.set_title("OCV-SOC")
    for a in (a_r, a_tau, a_dc):
        a.set_xlabel("SOC (%)")

    if has_mcu:
        th = ts / 60.0
        for s_, e_ in segs:
            a_soc.axvspan(th[s_], th[e_], color="#D85A30", alpha=0.12, lw=0)
        a_soc.plot(th, soc, lw=1.2, color="black",
                   label=f"real SOC (coulomb, init={args.init_soc:.1f}%)")
        a_soc.plot(th, np.where(np.isnan(mcu_soc), np.nan, mcu_soc), lw=1.2,
                   color="#A32D2D", ls="--",
                   label="measured SOC (MCU report)")
        a_soc.set_xlabel("time (min)"); a_soc.set_ylabel("SOC (%)")
        a_soc.grid(alpha=0.3); a_soc.legend(loc="best", fontsize=8)
        err_rms = float(np.sqrt(np.nanmean(
            (mcu_soc - soc)[~np.isnan(mcu_soc)] ** 2)))
        a_soc.set_title(f"SOC tracking: real vs measured "
                        f"(RMSE {err_rms:.2f}%, RMSE w/o initial offset "
                        f"{cmp_res['rmse_nb']:.2f}%)")
    fig.tight_layout()
    png2 = out + "_stair.png"
    fig.savefig(png2, dpi=140); plt.close(fig)
    print(f"[i] 图: {png1}")
    print(f"[i] 图: {png2}")


# ================= 多轮充放循环 (cycle) =================
def cycle_legs(cycles, first, soc_hi, soc_lo, start_soc):
    """把"跑 N 轮充放"排成一条腿序: [(轮次, 方向, 起始SOC, 末点SOC), ...]。

    两腿交替, 而且**后一腿从前一腿的末点起步** —— 这正是"多轮循环"能首尾相接的
    原因:
        放电 100->9 | 充电 9->80 | 放电 80->9 | 充电 9->80 | ...
    只有第一腿可能宽一点(取决于 --start-soc 与窗口上沿), 从第二轮起每一轮的两腿都
    严格是 soc_lo <-> soc_hi: 窗口一样, 轮与轮之间才谈得上可比。
    """
    order = ("discharge", "charge") if first == "discharge" else ("charge", "discharge")
    legs, cur = [], float(start_soc)
    for k in range(1, int(cycles) + 1):
        for d in order:
            to = soc_lo if d == "discharge" else soc_hi
            legs.append((k, d, cur, to))
            cur = to
    return legs


def leg_prefix(prefix, k, direction):
    """这一腿的产物前缀: cyc_c1_dis / cyc_c1_chg ..."""
    return "%s_c%d_%s" % (prefix, k, "dis" if direction == "discharge" else "chg")


def leg_timeline(load_mod, soc_from, soc_to, seg_n, cap_mah, cur_text, rest_s):
    """这一腿的 (断点, 每档电流, 每档 mAh, 每档秒数, 合计分钟)。

    **不自己重写一遍切档与时长算法** —— 直接借 soc_load 的 build_breaks /
    currents_for。soc_load 怎么切档、怎么算时长, 这里的预览就必须一模一样, 否则
    "预览说 4.5 小时、真跑 6 小时"这种对不上的账要白搭一整天(还得盯着设备)。
    """
    currs = load_mod.currents_for(cur_text, seg_n)
    brk = load_mod.build_breaks(load_mod.DEFAULT_OCV_MV, soc_from, soc_to, seg_n, "ocv")
    mahs = [abs(brk[k + 1] - brk[k]) / 100.0 * cap_mah for k in range(seg_n)]
    pulses = [m / c * 3.6 for m, c in zip(mahs, currs)]
    return brk, currs, mahs, pulses, (sum(pulses) + seg_n * rest_s) / 60.0


def leg_cmd(load_py, args, direction, s_from, s_to, pfx):
    """拼这一腿的 soc_load.py 命令行。

    --yes 必须带: 它同时跳掉"方向提问"与"回车开始"两步, 并且**未指定的参数一律
    取 soc_load 自己的默认值** —— 默认值只在 soc_load 里维护一份, 这里不抄第二份,
    免得哪天改了默认值两边悄悄对不上。
    """
    cmd = [sys.executable, load_py, "--yes",
           "--capacity", "%g" % args.capacity,
           "--init-soc", "%g" % s_from, "--to-soc", "%g" % s_to,
           "--segments", "%d" % args.segments,
           "--rest-s", "%g" % args.rest_s,
           "--out", pfx]
    if direction == "charge":
        cmd += ["--charge", "--current", args.current_charge,
                "--vlimit", "%g" % args.vlimit_charge,
                "--vset", "%g" % args.vset]
        if args.port_psu:
            cmd += ["--port", args.port_psu]
        if args.port_load:
            # 互锁: 充电腿开跑前先关电子负载 (两台并接, 同一时刻只许一台导通)
            cmd += ["--peer-port", args.port_load]
    else:
        cmd += ["--current", args.current_load, "--vlimit", "%g" % args.vlimit_load]
        if args.port_load:
            cmd += ["--port", args.port_load]
        if args.port_psu:
            # 互锁: 放电腿开跑前先关可编程电源
            cmd += ["--peer-port", args.port_psu]
    # 方言按腿分开传: 电子负载与可编程电源可能是两个牌子, 各有一套管法。
    # 用 getattr 兜底是为了让手工拼 Namespace 的测试脚本也能直接调这个函数。
    over = getattr(args, "cmd_charge", None) if direction == "charge" \
        else getattr(args, "cmd_load", None)
    for one in (over or ()):
        if str(one).strip():
            cmd += ["--cmd", str(one).strip()]
    return cmd


def read_leg_plan(pfx):
    """读回这一腿落盘的计划 json; 没落盘或坏了都返回 None(只影响汇总, 不致命)。"""
    p = pfx + ".json"
    if not os.path.isfile(p):
        return None
    try:
        with open(p, encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return None


def cmd_cycle(args):
    """多轮充放循环: 反复调度 soc_load.py 跑"放电腿 / 充电腿"。

    为什么放在 soc_bench 而不是给 soc_load 加个 --cycles: 一次循环要几小时, 而且
    要跨两个设备(电子负载 + 可编程电源)。放在编排层之后, 每一腿都是一次**独立的**
    soc_load 运行 ——
      · 每腿各自落一份计划 json + 档位时刻表 csv, 哪一腿出问题只丢那一腿, 已经跑完
        的腿不受影响, 也不用为了接着跑而重头再来;
      · soc_load 那六道保护(电压门限 / 断点校验 / 开载确认 / 通信看门狗 / 兜底断开 /
        跨设备互锁) 一行都不用重写, 也不会因为多了一层编排而失效;
      · 两腿除"区间与电流"外的参数全部由 soc_load 的默认值兜底, 默认值只有一份。
      · 跨设备互锁由 soc_load 的 --peer-port 承担 —— 每腿开跑前先关另一台并复核电流。
        所以这里必须知道两台各自的口: 缺一个口时真跑直接拒绝, --no-interlock 才放行。
      · "一腿怎么跑"是**可替换的**: 命令行下每腿另起一个 python 进程(默认), 上位机打包成
        exe 后没有独立解释器, 就把 LEG_RUNNER 换成"在本进程里调 soc_load" —— 腿序/窗口/
        互锁/停跑判据这些编排逻辑一行没变, 只是执行方式换了实现。
    """
    here = os.path.dirname(os.path.abspath(__file__))
    load_py = os.path.join(here, "soc_load.py")
    if not os.path.isfile(load_py):
        sys.exit("[X] 找不到 soc_load.py (应与 soc_bench.py 同目录)")
    if here not in sys.path:
        sys.path.insert(0, here)
    try:
        import soc_load as _sl          # 只为复用 build_breaks / currents_for
    except Exception as e:
        sys.exit("[X] 导入 soc_load.py 失败(循环要借它算切档与时长): %s" % e)

    soc_hi = CYCLE_SOC_HI if args.soc_hi is None else args.soc_hi
    soc_lo = CYCLE_SOC_LO if args.soc_lo is None else args.soc_lo
    if args.cycles < 1:
        sys.exit("[X] --cycles 至少为 1")
    if not (0.0 <= soc_lo < soc_hi <= 100.0):
        sys.exit("[X] 要 0 <= --soc-lo < --soc-hi <= 100 (收到 lo=%g hi=%g)"
                 % (soc_lo, soc_hi))
    # 与 check_consts.py 里那条对拍同一条道理: 脚本中止门限必须**先于**电源 CV
    # 收手, 否则端压先顶到 CV、电流自己衰减, 门限永远不触发, 等于没有保护。
    if args.vlimit_charge >= args.vset:
        sys.exit("[X] 充电中止门限 %.2fV 必须低于电源 CV %.2fV —— 否则先撞 CV, "
                 "电流自衰减, 门限永不触发" % (args.vlimit_charge, args.vset))
    # ---- 互锁前提: 两台设备的口都得显式给 ----
    # 每一腿都靠 --peer-port 让 soc_load 先关另一台。缺一个口时那一腿只能自动探测自己
    # 的设备, 而"两台同插"场景下自动探测会挑错(两台都会应答 *IDN?) —— 互锁本来是防
    # 两台对打的, 这种配置下等于没有, 所以真跑直接拒绝。
    pl = (args.port_load or "").strip().upper()
    pp = (args.port_psu or "").strip().upper()
    if pl and pp and pl == pp:
        sys.exit("[X] --port-load 与 --port-psu 是同一个口 (%s): 两台设备不可能接在"
                 "同一个串口上" % args.port_load)
    interlocked = bool(pl and pp)
    if not interlocked and not args.no_interlock:
        msg = ("[X] 多轮循环是两台设备并接交替跑, 要显式给全两台的口才能互锁:\n"
               "     --port-load COM9 --port-psu COM8\n"
               "    缺一个口时, 那一腿的自动探测会在两台同插时挑错, 互锁无从建立。\n"
               "    确实不用互锁(或只跑 --dry-run)就加 --no-interlock 明确放行。")
        if args.dry_run:
            print(msg.replace("[X] ", "[!] ", 1))
            print("    (--dry-run 只打印不碰设备, 先放行; 去掉 --dry-run 会被拦下来)")
        else:
            sys.exit(msg)
    first = args.first
    if args.start_soc is None:
        start_soc = CYCLE_START_SOC if first == "discharge" else soc_lo
    else:
        start_soc = args.start_soc
    seg_n, cap = args.segments, args.capacity

    legs = cycle_legs(args.cycles, first, soc_hi, soc_lo, start_soc)

    # ---- 先把腿序与账算清再动设备: 跑一轮几小时, 先看后跑 ----
    print("=" * 72)
    print("多轮充放循环: %d 轮 / 每轮 2 腿 / 窗口 %g%% <-> %g%%"
          % (args.cycles, soc_lo, soc_hi))
    print("第一腿 %s 从 %g%% 起 | 容量 %g mAh | 每腿 %d 档 | 档间静置 %gs"
          % ("放电" if first == "discharge" else "充电", start_soc, cap, seg_n,
             args.rest_s))
    if args.no_interlock:
        print("[!] --no-interlock: 每腿开跑前**不会**先关另一台设备 —— 两台并接时"
              "必须自己确认另一台是断开的")
    elif interlocked:
        print("设备串口: 放电腿 %s / 充电腿 %s" % (args.port_load, args.port_psu))
        print("互锁: 每腿开跑前先连上另一台、下发它的断开命令并复核实测电流, 再驱动本腿"
              "设备 (确认不了就停下, 不会先开自己)")
    else:
        print("设备串口: 自动探测 | [!] 两个口没给全, 互锁建不起来")
    print("-" * 72)
    print("%-3s %-3s %-4s %-17s %9s %8s  %s"
          % ("轮", "腿", "方向", "SOC 区间", "计划mAh", "预计min", "产物前缀"))
    rows, total_min = [], 0.0
    for i, (k, d, s_from, s_to) in enumerate(legs):
        pfx = leg_prefix(args.prefix, k, d)
        cur_text = args.current_charge if d == "charge" else args.current_load
        _brk, _cur, _mahs, _pul, mins = leg_timeline(
            _sl, s_from, s_to, seg_n, cap, cur_text, args.rest_s)
        total_min += mins
        rows.append((k, i + 1, d, s_from, s_to, pfx))
        print("%-3d %-3d %-4s %5.1f%% -> %5.1f%% %9.0f %8.1f  %s"
              % (k, i + 1, "放电" if d == "discharge" else "充电",
                 s_from, s_to, sum(_mahs), mins, pfx))
    print("-" * 72)
    print("合计 %d 腿, 预计 %.1f 小时 (%.0f min); 不含设备通信开销, 实际只会更长。"
          % (len(rows), total_min / 60.0, total_min))

    if args.dry_run:
        print("\n--dry-run: 只列命令行, 不碰设备\n")
        for (k, _i, d, s_from, s_to, pfx) in rows:
            print("  " + " ".join(leg_cmd(load_py, args, d, s_from, s_to, pfx)))
        print("\n[i] 去掉 --dry-run 就真跑。")
        return

    if not args.yes:
        print("\n每一腿都会真实驱动设备(共 %d 腿)。回车 = 开始, 输 n = 取消。" % len(rows))
        print("(跑起来后 Ctrl+C 只中止当前那一腿, 已跑完的腿不受影响)")
        try:
            if input().strip().lower().startswith("n"):
                print("已取消")
                return
        except (KeyboardInterrupt, EOFError):
            print("\n已取消")
            return

    results, t0 = [], datetime.datetime.now()
    for (k, li, d, s_from, s_to, pfx) in rows:
        nm = "放电" if d == "discharge" else "充电"
        print("\n" + "=" * 72)
        print("[轮 %d/%d] 第 %d 腿  %s  %g%% -> %g%%   %s   (现在 %s)"
              % (k, args.cycles, li, nm, s_from, s_to, pfx,
                 datetime.datetime.now().strftime("%H:%M:%S")))
        print("已用 %.2f 小时 / 预计合计 %.2f 小时"
              % ((datetime.datetime.now() - t0).total_seconds() / 3600.0,
                 total_min / 60.0))
        print("=" * 72)
        cmd = leg_cmd(load_py, args, d, s_from, s_to, pfx)
        print("[i] " + " ".join(cmd))
        rc = run_leg(cmd)
        plan = read_leg_plan(pfx)
        if plan is None:
            tag = "没落盘计划 (退出码 %d)" % rc
        elif plan.get("aborted"):
            tag = "被保护中止 (%d/%d 档)" % (plan.get("segments_done", 0),
                                        plan.get("segments_planned", seg_n))
        elif rc != 0:
            tag = "退出码 %d (%d/%d 档)" % (rc, plan.get("segments_done", 0),
                                        plan.get("segments_planned", seg_n))
        else:
            tag = "完成 (%d/%d 档)" % (plan.get("segments_done", 0),
                                   plan.get("segments_planned", seg_n))
        results.append((k, nm, tag))
        print("[i] 轮 %d %s -> %s" % (k, nm, tag))
        # 一腿没跑满, 后面每腿的起始 SOC 就都对不上了(它是按"上一腿跑满"推出来的)。
        # 不硬着头皮往下跑: 先让用户核实实际 SOC, 再决定 --keep-going 还是重跑。
        if not tag.startswith("完成") and not args.keep_going:
            print("\n[!] 这一腿没按计划跑满, 已停下, 不会再动设备。")
            print("    后面每腿的起始 SOC 是按'上一腿跑满'推的, 接着跑会从一开始就对"
                  "不上 —— soc_load 的断点 OCV 校验也会先把它拦下来。")
            print("    确认电池实际 SOC 与该腿起点一致后, 加 --keep-going 跑剩下的腿。")
            break

    print("\n" + "=" * 72)
    print("循环汇总: 已执行 %d/%d 腿, 全程 %.2f 小时 (产物前缀 %s_*)"
          % (len(results), len(rows),
             (datetime.datetime.now() - t0).total_seconds() / 3600.0, args.prefix))
    print("%-3s %-4s  %s" % ("轮", "方向", "结果"))
    for (k, nm, tag) in results:
        print("%-3d %-4s  %s" % (k, nm, tag))
    print("\n[i] 每腿落两份: <前缀>.json 计划(档数/电流/mAh/是否被中止) 与 "
          "<前缀>.csv 档位时刻表(各档起止时刻)。")
    print("[i] 想让 stair 自动认出计划: 把该腿的采集 csv 命名成同样的前缀再")
    print("    python soc_bench.py stair --csv %s.csv" % rows[0][5])
    print("    循环测试一般是 soc_record 全程连续录, 跑完再按各腿 json 里的 "
          "t_start / t_rest_end 切段; 目前还没有自动切段工具。")


def cmd_wizard(_args=None):
    """无子命令时的交互向导: 先问由谁控制, 再分流。

    程控源载 -> 启动 soc_load.py 控载向导(它会继续问方向(充/放)/设备串口/
    电池容量/档位参数, 全部确认后才真实加载); 手动负载 -> 手动十档操作流程。
    采集(record) 与分析(stair/fit) 始终是独立步骤, 向导只负责统一入口。
    """
    print("=" * 60)
    print("SOC 十档标定向导")
    print("  [1] 程控源/载: 自动拉十档 (RS232/SCPI, 脚本发指令; 可选充/放)")
    print("  [2] 手动负载: 自己在负载面板设十档周期")
    while True:
        s = input("选择 1/2 (回车=1): ").strip()
        if s in ("", "1", "2"):
            break
    if s == "2":
        print("\n手动负载流程 (负载面板自设十档):")
        print("  1. 电池充满接入负载, 面板按你的档位计划设好周期")
        print("  2. 另开终端开始采集: python soc_record.py --port <MCU COM>")
        print("  3. 跑完 Ctrl+C 停采集, 再: python soc_bench.py stair --csv <file>")
        print("  注: 手动设档要能复现(电流/时长固定), 否则 stair 档位识别会错位")
        return
    print("\n程控源载: 将启动 soc_load.py 控载向导, 依次询问")
    print("  方向(1=放电/2=充电) -> 设备串口 -> 电池容量 -> 起末 SOC/档位分布")
    print("  -> 电流/静置/端压门限, 确认(回车)后才真实加载, Ctrl+C 可安全中止。")
    while True:
        s = input("现在启动? y/n (回车=y): ").strip().lower()
        if s in ("", "y", "n"):
            break
    if s == "n":
        print("未启动。稍后自己跑: python soc_load.py  (会自动识别负载串口)")
        return
    load = os.path.join(os.path.dirname(os.path.abspath(__file__)), "soc_load.py")
    sys.exit(subprocess.call([sys.executable, load]))


def main():
    ap = argparse.ArgumentParser(
        description="SOC 标定主工具: 无子命令=向导(程控/手动) | record 采集 | "
                    "fit 单脉冲RC | stair 多档RC+OCV-SOC | cycle 多轮充放循环")
    sub = ap.add_subparsers(dest="cmd")

    p_rec = sub.add_parser("record", help="采集: 恒流放电->断电静置")
    p_rec.add_argument("--port", default=None,
                       help="串口号 (如 COM3); 缺省则启动前自动检测/交互选择")
    p_rec.add_argument("--baud", type=int, default=None,
                       help="波特率 (如 115200); 缺省则启动前交互选择")
    p_rec.add_argument("--out", default="pulse.csv")
    p_rec.set_defaults(func=cmd_record)

    p_fit = sub.add_parser("fit", help="拟合: 自动找断电点并提取 RC 参数")
    p_fit.add_argument("--csv", default=None,
                       help="要分析的 CSV 文件; 缺省则扫描当前目录交互选择")
    p_fit.add_argument("--I", type=float, default=None, dest="I_fixed",
                       help="已知放电电流(A), 指定则用它替代均值")
    p_fit.set_defaults(func=cmd_fit)

    p_st = sub.add_parser("stair", help="N档周期: 逐档 R0/R1/C1/tau + OCV + SOC")
    p_st.add_argument("--csv", default=None,
                      help="要分析的 CSV; 缺省则扫描当前目录交互选择")
    p_st.add_argument("--capacity", type=float, default=None,
                      help=f"标称容量 mAh (缺省: 优先读档位计划文件, 否则 {STAIR_CAPACITY})")
    p_st.add_argument("--init-soc", type=float, default=None,
                      help="起始 SOC%% (缺省: 优先读档位计划文件, 否则用开录首段静置"
                           "电压反查 OCV 表)")
    p_st.add_argument("--i-on", type=float, default=STAIR_I_ON_MA,
                      help=f"带载进入阈值 mA (默认 {STAIR_I_ON_MA})")
    p_st.add_argument("--i-off", type=float, default=STAIR_I_OFF_MA,
                      help=f"带载退出阈值 mA (默认 {STAIR_I_OFF_MA})")
    p_st.add_argument("--idle-ma", type=float, default=IDLE_MA,
                      help=f"静置判定阈值 mA (默认 {IDLE_MA}, 与 MCU 一致)")
    p_st.add_argument("--min-pulse-s", type=float, default=STAIR_MIN_PULSE_S,
                      help=f"最短带载时长 s, 更短的当毛刺丢 (默认 {STAIR_MIN_PULSE_S})")
    p_st.add_argument("--ocv-tail", type=float, default=STAIR_TAIL_S,
                      help=f"OCV/V_inf 取静置段末尾多少秒均值 (默认 {STAIR_TAIL_S:.0f})")
    p_st.add_argument("--expect", type=int, default=None,
                      help=f"期望档位数, 不符只提示 (缺省: 优先读档位计划文件, "
                           f"否则 {STAIR_EXPECT})")
    p_st.add_argument("--plan", default=None, metavar="路径",
                      help="档位计划文件 (soc_load.py 输出的 *_plan.json / *_segments.csv); "
                           "缺省时自动找与 --csv 同前缀的那份; 填 none 关掉不认")
    p_st.add_argument("--ocv-table", default=None,
                      help="OCV 表 11 个 mV (0..100%%), 用于反查起始 SOC")
    p_st.add_argument("--out", default=None, help="输出文件前缀 (默认与 CSV 同名)")
    p_st.add_argument("--zero-drift", action="store_true",
                      help="扣除静置段电流零漂后再积分 (默认不扣, 保持与 MCU 侧一致)")
    p_st.add_argument("--emit-c", action="store_true",
                      help="额外输出 soc.c 用的 OCV 查表 C 数组 (SOC 0~100%%, step 10%%)")
    p_st.add_argument("--no-plot", action="store_true", help="只出表不出图")
    p_st.set_defaults(func=cmd_stair)

    p_cy = sub.add_parser(
        "cycle", help="多轮充放循环: 交替调 soc_load.py 跑放电腿/充电腿")
    p_cy.add_argument("--cycles", type=int, default=CYCLE_N,
                      help="循环轮数 (默认 %d; 每轮两条腿)" % CYCLE_N)
    p_cy.add_argument("--first", choices=("discharge", "charge"), default="discharge",
                      help="第一腿方向 (默认 discharge: 先放后充)")
    p_cy.add_argument("--soc-hi", type=float, default=None,
                      help="循环窗口上沿 %%%% (默认 %g)" % CYCLE_SOC_HI)
    p_cy.add_argument("--soc-lo", type=float, default=None,
                      help="循环窗口下沿 %%%% (默认 %g)" % CYCLE_SOC_LO)
    p_cy.add_argument("--start-soc", type=float, default=None,
                      help="第一腿的起始 SOC%%%% (放电首默认 %g / 充电首默认 --soc-lo); "
                           "电池刚充满就用缺省, 它会先把这一段放到窗口下沿"
                           % CYCLE_START_SOC)
    p_cy.add_argument("--segments", type=int, default=CYCLE_SEGMENTS,
                      help="每腿档位数 (默认 %d)" % CYCLE_SEGMENTS)
    p_cy.add_argument("--current-load", default=CYCLE_CUR_LOAD,
                      help="放电腿每档电流 A (默认 %s; 也可 0.5,1,.. 或 'a:b' 递变)"
                           % CYCLE_CUR_LOAD)
    p_cy.add_argument("--current-charge", default=CYCLE_CUR_CHG,
                      help="充电腿每档电流 A (默认 %s)" % CYCLE_CUR_CHG)
    p_cy.add_argument("--rest-s", type=float, default=CYCLE_REST_S,
                      help="档间静置 s (默认 %g)" % CYCLE_REST_S)
    p_cy.add_argument("--vlimit-load", type=float, default=CYCLE_VLIM_LOAD,
                      help="放电端压底线 V (默认 %g)" % CYCLE_VLIM_LOAD)
    p_cy.add_argument("--vlimit-charge", type=float, default=CYCLE_VLIM_CHG,
                      help="充电端压上限 V (默认 %g, 必须低于 --vset)" % CYCLE_VLIM_CHG)
    p_cy.add_argument("--vset", type=float, default=CYCLE_VSET,
                      help="电源恒压 CV 上限 V (默认 %g)" % CYCLE_VSET)
    p_cy.add_argument("--port-load", default=None,
                      help="电子负载串口 (强烈建议显式给; 缺省=该腿自己自动探测)")
    p_cy.add_argument("--port-psu", default=None,
                      help="可编程电源串口 (强烈建议显式给; 缺省=该腿自己自动探测)")
    p_cy.add_argument("--capacity", type=float, default=CYCLE_CAPACITY,
                      help="标称容量 mAh (默认 %g)" % CYCLE_CAPACITY)
    p_cy.add_argument("--prefix", default=CYCLE_PREFIX,
                      help="产物前缀 (默认 %s -> %s_c1_dis / %s_c1_chg ...)"
                           % (CYCLE_PREFIX, CYCLE_PREFIX, CYCLE_PREFIX))
    p_cy.add_argument("--keep-going", action="store_true",
                      help="某腿没跑满也接着跑后面的腿 (默认停下来, 因为后续腿的起始 "
                           "SOC 是按'上一腿跑满'算的)")
    p_cy.add_argument("--no-interlock", action="store_true",
                      help="不要求两台的口都给全, 也不做跨设备互锁 (默认: 两个口必须给全, "
                           "每腿开跑前先关另一台并复核电流)")
    p_cy.add_argument("--cmd-load", action="append", default=None, metavar="键=命令",
                      help="放电腿(电子负载)的命令方言覆盖, 可重复; 换品牌用, "
                           u"例: --cmd-load on=\"INP 1\"")
    p_cy.add_argument("--cmd-chg", action="append", default=None, metavar="键=命令",
                      help="充电腿(可编程电源)的命令方言覆盖, 可重复; 换品牌用, "
                           u"例: --cmd-chg vset=\"SOUR:VOLT {v}\"")
    p_cy.add_argument("--dry-run", action="store_true",
                      help="只打印腿序/预计时长/命令行, 不碰设备")
    p_cy.add_argument("--yes", action="store_true", help="不再确认, 直接开始")
    p_cy.set_defaults(func=cmd_cycle)

    args = ap.parse_args()
    if args.cmd is None:
        cmd_wizard()
    else:
        args.func(args)


if __name__ == "__main__":
    main()
