#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
soc_record.py - SOC 标定工具:串口采集记录器

从 MCU 串口接收一行一帧的文本数据, 由 PC 端打时间戳, 追加写入 CSV。
本脚本只负责"收数据 + 落盘", 不参与 SOC 计算 (SOC 由 soc_bench.py stair 离线算)。

MCU 端协议 (文本行, LF 或 CRLF 结尾, 建议 5 Hz = 200ms 一帧):
    D,V_mV,I_mA,T_0p1C,SOC_0p01,SOH_0p01,Q_mAh,R0_mohm
    首字段 'D' 为数据帧前缀 (Data), 用于与诊断打印区分, 无前缀行一律丢弃
示例:
    D,3845,1234,253,8654,9666,3238,62
      = 3.845 V, 放电 1.234 A, 25.3 C, SOC 86.54%, SOH 96.66%,
        在线学到的容量 3238 mAh, 10~90% 平均 R0 62 mΩ
    D,3845,-1234,253,2112,9666,3238,62  (充电: 电流为负)

单位与符号约定 (必须与 MCU 端一致):
    V_mV    : 电池电压(单节), 整数 mV
    I_mA    : 总线电流, 整数 mA;  放电为正, 充电为负
    T_0p1C  : 温度, 整数 0.1 C    (253 = 25.3 C)
    SOC_0p01: MCU 当前 SOC 估计, 整数 0.01%  (0=0%, 10000=100%)
    SOH_0p01: MCU 在线学习的 SOH, 整数 0.01% (容量 Q / 内阻 R0 取劣)
    Q_mAh   : MCU 在线学到的可用容量, 整数 mAh
    R0_mohm : MCU 学习后的 10~90% 平均欧姆内阻, 整数 mΩ

向后兼容: 字段较少的旧固件帧 ("D,V,I,T" / "D,V,I,T,SOC") 同样能解析,
缺失的列在 CSV 里留空, 历史 CSV 文件无需重录。

用法:
    python soc_record.py --out run1.csv
        不传 --port/--baud 时, 启动前自动检测可用串口, 交互选择 COM 口和波特率
    python soc_record.py --port COM3 --baud 115200 --out run1.csv
        命令行显式指定则跳过交互, 便于脚本化/批处理复用
    Ctrl+C 停止并保存。
"""

import argparse
import csv
import datetime
import sys

try:
    import serial
except ImportError:
    sys.exit("缺少 pyserial, 请先执行: pip install pyserial")


# 字段合理性检查范围 (宽范围, 只挡串口垃圾数据, 不做精度判断)
V_MIN_MV, V_MAX_MV = 1500, 5000      # 单节 1.5V ~ 5.0V
I_MIN_MA, I_MAX_MA = -30000, 30000   # 总线 -30A ~ +30A
T_MIN_DC, T_MAX_DC = -400, 1250      # -40.0C ~ +125.0C

CSV_HEADER = [
    "time_iso",      # PC 端 ISO 时间戳
    "elapsed_s",     # 距首帧的秒数 (float)
    "V_mV", "I_mA", "T_0p1C", "SOC_0p01",
    # v5 新增: 在线学习结果 (旧固件 / 历史文件留空)
    "SOH_0p01", "Q_mAh", "R0_mohm",
]


# ---------------- 启动前交互: 选 COM 口 + 波特率 ----------------
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


def parse_line(line: str):
    """解析一行文本, 返回 [v, i, t, soc, soh, q, r0] 或 None (垃圾行/半截行)。

    标准帧: 首字段为 'D' (Data 帧前缀), 其后 7 个整数字段。
    兼容旧固件: 5 字段帧 -> soh/q/r0 记 None; 4 字段帧 -> soc 也记 None
    (CSV 留空)。前缀不匹配的行 (诊断打印/半截行) 一律丢弃。
    """
    parts = line.strip().split(",")
    if parts[0] != "D" or len(parts) not in (4, 5, 8):
        return None
    try:
        v, i, t = (int(p) for p in parts[1:4])
        soc = int(parts[4]) if len(parts) >= 5 else None
        soh = int(parts[5]) if len(parts) >= 8 else None
        q   = int(parts[6]) if len(parts) >= 8 else None
        r0  = int(parts[7]) if len(parts) >= 8 else None
    except ValueError:
        return None
    if not (V_MIN_MV <= v <= V_MAX_MV):
        return None
    if not (I_MIN_MA <= i <= I_MAX_MA):
        return None
    if not (T_MIN_DC <= t <= T_MAX_DC):
        return None
    # SOC/SOH 越界(固件异常)不整行丢弃, 置 None 留空即可
    if soc is not None and not (0 <= soc <= 10000):
        soc = None
    if soh is not None and not (0 <= soh <= 10000):
        soh = None
    return [v, i, t, soc, soh, q, r0]


def main():
    ap = argparse.ArgumentParser(description="SOC 标定 - 串口采集记录器")
    ap.add_argument("--port", default=None,
                    help="串口号 (如 COM3); 缺省则启动前自动检测/交互选择")
    ap.add_argument("--baud", type=int, default=None,
                    help="波特率 (如 115200); 缺省则启动前交互选择")
    ap.add_argument("--out", default="soc_data.csv", help="输出 CSV 路径")
    args = ap.parse_args()

    # 串口参数: 命令行已给 -> 直接用; 缺省 -> 启动前交互选择
    port = args.port
    baud = args.baud
    if port is None:
        port = select_com_port()
    if baud is None:
        baud = select_baud(default_baud=115200)

    t0 = None          # 首帧时间, 用于 elapsed_s
    n_ok = 0           # 有效帧数
    n_bad = 0          # 丢弃帧数
    last = None        # 最近一帧, 用于终端刷新

    try:
        ser = serial.Serial(port, baud, timeout=1.0)
    except serial.SerialException as e:
        sys.exit(f"无法打开串口 {port}: {e}\n"
                 f"检查: 端口号是否正确 / 是否被其他程序占用 / 是否已插上 USB 转串口")

    print(f"[i] 打开 {port} @ {baud}, 输出 -> {args.out}")
    print("[i] 等待数据... (Ctrl+C 停止并保存)")

    with open(args.out, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(CSV_HEADER)
        try:
            while True:
                raw = ser.readline()          # 最多阻塞 timeout 秒
                if not raw:
                    continue                  # 超时无数据, 继续等
                try:
                    text = raw.decode("utf-8", errors="replace")
                except UnicodeDecodeError:
                    text = raw.decode("latin-1", errors="replace")
                vals = parse_line(text)
                now = datetime.datetime.now()
                if vals is None:
                    n_bad += 1
                    continue
                if t0 is None:
                    t0 = now
                elapsed = (now - t0).total_seconds()
                writer.writerow([now.isoformat(timespec="milliseconds"),
                                 f"{elapsed:.3f}"] + vals)
                f.flush()                     # 逐帧落盘, Ctrl+C 不丢数据
                n_ok += 1
                last = vals
                v, i, t, soc, soh, q, r0 = vals
                soc_s = "" if soc is None else f"SOC={soc/100:.2f}%"
                soh_s = "" if soh is None else f"SOH={soh/100:.2f}%"
                sys.stdout.write(
                    f"\r[{n_ok:>6} 帧 | 丢弃 {n_bad}] "
                    f"V={v}mV I={i:+d}mA T={t/10:.1f}C {soc_s:<12}{soh_s:<12}"
                    f"t={elapsed/60:7.1f}min   "
                )
                sys.stdout.flush()
        except KeyboardInterrupt:
            pass
        finally:
            print("\n[i] 停止采集")
            print(f"[i] 有效帧 {n_ok}, 丢弃 {n_bad}, 时长 {((datetime.datetime.now()-t0).total_seconds()/60) if t0 else 0:.1f} min")
            print(f"[i] 已保存: {args.out}")


if __name__ == "__main__":
    main()
