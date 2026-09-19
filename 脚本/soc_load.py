#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
soc_load.py - 程控源/载 十档标定控制器 (放电=电子负载 / 充电=可编程电源)

通过 RS232(9600, SCPI) 控制电子负载(放电) 或 可编程电源(充电), 自动跑
"恒流 <方向> -> 断开静置" 的 N 档周期 (默认 10 档), 供 MCU 侧 soc_record.py
同步采集做 R0/R1/OCV/SOC 标定。

方向用 --charge 切换。两个方向除下表三处外, 时序 / 保护 / 计划计算完全共用:

    项目        放电(默认)                充电(--charge)
    ---------   -----------------------   ---------------------------
    设备        电子负载                  可编程电源
    开关命令    INP 1|0   (输入/抽流)      OUTP 1|0  (输出)
    电压保护    端压 < --vlimit 即断开     端压 > --vlimit 即断开
    默认区间    100% -> 9%                9% -> 80%

充电侧会额外下发 VOLT <vset> 设恒压(CV)上限(默认 4.20V = 电芯绝对上限)。它是
最后一道防线, 正常应由 --vlimit(默认 4.15V)先收手 —— 两个都没碰到才是对的,
一旦端压顶到 CV, 电流自己衰减, 那一档的时长就不再受控(见计划预览的告警)。

命令集(默认方言 = 通用 SCPI 根命令写法, 艾德克斯 IT8500 同族负载、大多数国产
可编程电源都是这套):
    CURR <A>          设定 CC 电流
    INP 1|0           负载开关 (放电)
    OUTP 1|0          电源输出开关 (充电)
    VOLT <V>          设定恒压上限 (充电; SCPI 里 VOLT 是 SOURce 根节点)
    MEASure:VOLTage?  实测电压 (V)
    MEASure:CURRent?  实测电流 (A)
    CURR? / VOLT?     设定值读回(校验命令被认下, 不参与控制, 不支持时返回空)
    *IDN?             识别

CURR / INP / OUTP / VOLT 都是**根命令**: SCPI 允许省略可选的根节点, 所以
CURR <A> 与 :SOURce:CURRent <A> 等价, OUTP 1 与 :OUTPut:STATe 1 等价 ——
手册上写成哪个都不影响, 只要电压电流在 MEASure 节点(SCPI 里最统一的部分)。
换品牌若命令写法不同(例如输出开关走 :LOAD:STATe), 用
    --cmd on="LOAD:STATe 1" --cmd off="LOAD:STATe 0"
覆盖对应几条即可, 不必改代码; 键名与含义在 --help 里(充电模式多一个 vset)。

用法:
    python soc_load.py                      # 全交互: 自动识别串口, 缺省回车
    python soc_load.py --port COMx --capacity 3350 --current 2.0 \
        --to-soc 9 --rest-s 480 --cutoff 2.8
    python soc_load.py --charge --current 1.6 --to-soc 80    # 充电标定
    python soc_load.py --charge --peer-port COM9             # 另接一台负载: 带互锁跑
    python soc_load.py --demo --segments 3 --pulse-s 1 --rest-s 1   # 空载快速联调

串口不定时不用手填: 不传 --port 会自动探测(绝不写死某个 COM) —— 逐个串口先听
0.3s, 主动往上报数据的口(MCU 数据口)直接跳过, 再发 *IDN?, 谁应答谁就是设备;
都认不出来才退回串口菜单/手输。想限定品牌可加 --idn 关键字。

档位分布默认 ΔOCV 等分: OCV-SOC 曲线中段线性(每档放得多)、两端尤其是低 SOC
段斜率大(每档自动加密), 保证喂给 MCU 线性查表时每档的电压分辨率一致。
覆盖 100% -> 9% 时十档断点由 ../firmware/bms_config.h 的 OCV 表反解 (见 build_breaks)。
用 --spacing soc 回到传统每档等 SOC。

输出(两项都落盘, 后者专供 soc_bench.py stair 自动识别, 别删):
    <前缀>.csv    逐档时刻表(各档起止时刻/设定, 人看/与 record 对齐用)
    <前缀>.json   同一份计划的机器可读版: 档数 / 每档电流与 mAh 与时长 / 容量 /
                  起始 SOC / 用的哪张 OCV 表 / 是否被保护提前中止。
                  把它改名成 <你采集的那个 csv 的前缀>_plan.json, 跑
                  "soc_bench.py stair --csv <那个 csv>" 时就会被自动认到,
                  不用再手输 --capacity / --init-soc / --expect。

安全保护 (全程生效, 两个方向一致):
  1. 电压门限: 放电端压 < / 充电端压 > --vlimit 立即断开;
  2. 断点校验: 每档开始实测开路电压须在内置 OCV 表预期 ±0.5V 内 (差>0.5V 中止,
     常见于 电池没充满/没放空 / 容量或表与电芯不符 / 接错电池组);
  3. 开载确认: 开关打开 1.5s 后实测电流须达设定的 85%+ (取绝对值, 电源与负载
     的电流符号约定不同, 一概不看符号), 否则视为线断/接触不良/模式错, 断开中止;
  4. 通信看门狗: 循环中连续 3 次读不到设备 → 强制断开并终止 (写均带 0.5s 写超时);
  5. Ctrl+C / 异常 finally 必关输出, 关闭指令失败会明确警告你去面板手动关。
  6. 跨设备互锁 (--peer-port, 两台并接同一节电池时给): 开自己的串口**之前**先连上
     另一台、下发它的断开命令并复核实测电流≈0; 确认不了就中止(fail-closed), 绝不
     "先开自己再说"。断开命令重试 3 次、复核阈值 0.05A(50mA)。
"""

import argparse
import csv
import json
import sys
import time
import datetime

import serial

LOAD_BAUD = 9600

# ---------------- 方向表 ----------------
# 充放电的差异全集中在这里。想加第三种用法(比如"只静置不加载")在下面加一项即可,
# 控制/保护/计划计算一行都不用动。
#
#   guard="low"  端压跌破 v_limit 即中止(放电, 保护电池不过放)
#   guard="high" 端压涨过 v_limit 即中止(充电, 保护电池不过充)
MODES = {
    "discharge": {
        "key": "discharge",
        "name": "放电",
        "dev": "电子负载",
        "cmds": {"on": "INP 1", "off": "INP 0"},
        "guard": "low",
        "soc_sign": -1.0,          # SOC 变化方向: 放电 SOC 递减
        "init_soc": 100.0,
        "to_soc": 9.0,
        "v_limit": 2.8,
        "v_hint": "放电端压底线 V (端压=OCV-压降, 只兜底不设目标)",
        "current": 2.0,
        "out": "load_segments",
        "tail": "端压 = OCV - I*R",
    },
    "charge": {
        "key": "charge",
        "name": "充电",
        "dev": "可编程电源",
        "cmds": {"on": "OUTP 1", "off": "OUTP 0",
                 "vset": "VOLT {v}", "volt_q": "VOLT?"},
        "guard": "high",
        "soc_sign": +1.0,          # 充电 SOC 递增
        "init_soc": 9.0,
        # 到 80% 就停, 不是保守 —— 再往上端压(OCV+I*R)会顶到电源 CV 设定值,
        # 一进 CV 电流就自己衰减, 档位时长不再受控。想标高 SOC 段请减小电流。
        "to_soc": 80.0,
        "v_limit": 4.15,
        "v_hint": "充电端压上限 V (低于电源 CV 设定值, 由它先收手才正常)",
        "current": 1.6,
        "out": "charge_segments",
        "tail": "端压 = OCV + I*R",
    },
}
MODE = MODES["discharge"]

# 命令方言表 —— 默认是通用 SCPI 根命令写法。
# 换品牌只要改这几条, 控制逻辑一行都不用动; 命令行 --cmd 键=命令 也能覆盖。
#   curr   设 CC 电流, {i} 替换成数值(A)
#   curr_q 读回设定电流(只写日志校验用, 不支持时返回空即可, 不致命)
#   on/off 加载 / 卸载 (放电=INP, 充电=OUTP)
#   vset   设恒压上限, {v} 替换成电压值(V) —— 仅充电
#   volt_q 读回恒压设定(同上, 仅充电)
#   volt   读端压(V)      amp  读电流(A)
CMDS_BASE = {
    "curr": "CURR {i}",
    "curr_q": "CURR?",
    "volt": "MEASure:VOLTage?",
    "amp": "MEASure:CURRent?",
}
CMDS = {}

# 电源恒压(CV)默认设定值: 单节锂电 4.20V。它是最后防线而不是控制目标 ——
# 正常应该由 --vlimit(默认 4.15)先触发中止, 顶到这里就说明计划排高了。
V_SET_DEFAULT = 4.20


def apply_mode(mode_key, cmd_overrides=None):
    """按方向装配命令表: 先铺公共命令, 再叠方向专有命令, 最后套 --cmd 覆盖。

    放在 --cmd 覆盖**之前**, 是为了让用户能盖掉充放电任意一条(例如某品牌的
    输出开关叫 "LOAD:STATe"); 反过来就会把用户的自定义冲掉。
    """
    global MODE
    MODE = MODES[mode_key]
    CMDS.clear()
    CMDS.update(CMDS_BASE)
    CMDS.update(MODE["cmds"])
    apply_cmd_overrides(cmd_overrides)


def apply_cmd_overrides(pairs):
    """把 ["on=OUTP 1", "volt=:MEAS:VOLT?"] 套到 CMDS 上。

    键名写错直接报错退出, 不静默忽略 —— 免得"改了没生效"还要回头查半天。
    """
    for it in pairs or ():
        if "=" not in it:
            sys.exit("--cmd 要写成 键=命令, 收到 %r" % it)
        k, v = it.split("=", 1)
        k, v = k.strip(), v.strip()
        if k not in CMDS:
            sys.exit("--cmd 不认识的键 %r, 当前方向(%s)可用: %s"
                     % (k, MODE["name"], ", ".join(sorted(CMDS))))
        if not v:
            sys.exit("--cmd %s= 后面的命令不能空" % k)
        CMDS[k] = v
    if "{i}" not in CMDS["curr"]:
        sys.exit("--cmd curr= 里必须留一个 {i} 当电流数值的位置")
    if "vset" in CMDS and "{v}" not in CMDS["vset"]:
        sys.exit("--cmd vset= 里必须留一个 {v} 当电压数值的位置")


# 默认按放电装配一次, 让 import 之后命令表就是可用的(被当模块引、被测试脚本
# 直接调 meas_v 时才不会撞上空表); CLI 会在 cmd_run 开头按 --charge 重新装配。
apply_mode("discharge")

# 电芯 OCV 表 (mV, SOC 0%..100% 步长 10) —— 与固件 bms_config.h 的 OCV 表保持同步!
# 来源: 十档 HPPC 实测 (2 A 放电 + 静置回弹后取值, 0% 与 100% 为端点外推值)。
# 改过固件表后这里要一起改; --ocv-table 可临时覆盖。
DEFAULT_OCV_MV = [3050, 3188, 3375, 3524, 3629, 3748, 3845, 3938, 4027, 4096, 4161]


def soc_ocv(soc, tbl):
    """SOC% -> OCV(mV), 段内线性。"""
    soc = min(max(float(soc), 0.0), 100.0)
    for i in range(10):
        lo, hi = i * 10, (i + 1) * 10
        if lo <= soc <= hi:
            f = (soc - lo) / 10.0
            return tbl[i] + f * (tbl[i + 1] - tbl[i])
    return tbl[-1]


def soc_from_ocv(mv, tbl):
    """OCV(mV) -> SOC% (OCV 表的分段线性反解, 供 ΔOCV 等分档位用)。"""
    if mv >= tbl[-1]:
        return 100.0
    if mv <= tbl[0]:
        return 0.0
    for i in range(10):
        if tbl[i] <= mv <= tbl[i + 1]:
            f = (mv - tbl[i]) / (tbl[i + 1] - tbl[i])
            return i * 10 + f * 10.0
    return float("nan")


def build_breaks(tbl, soc_from, soc_to, n, spacing):
    """n 段循环的 SOC 断点 [s0, s1, ..., sn], 方向由 soc_from/soc_to 的相对大小定
    (放电 soc_from > soc_to, 充电反过来)。

    spacing='ocv': ΔOCV 等分 —— 曲线陡的段(低 SOC)自动加密, 缓的平台段放宽,
        每档的"电压分辨率"一致, 适合喂给线性查表;
    spacing='soc': ΔSOC 等分 (传统均匀阶梯)。

    两个方向共用同一段代码: 充放电只是把起点终点调个个儿, 写成分式后
    放电时的取值与旧版逐位相同(旧版写死 soc_from - dv*k, 这里 dv 带符号)。
    """
    soc_from = min(max(float(soc_from), 0.0), 100.0)
    soc_to = min(max(float(soc_to), 0.0), 100.0)
    if abs(soc_to - soc_from) < 1e-9:
        return [soc_from] * (n + 1)
    if spacing == "soc":
        return [soc_from + (soc_to - soc_from) * k / n for k in range(n + 1)]
    v0, v1 = soc_ocv(soc_from, tbl), soc_ocv(soc_to, tbl)
    dv = (v1 - v0) / n
    brk = [soc_from]
    for k in range(1, n + 1):
        brk.append(soc_from_ocv(v0 + dv * k, tbl))
    return brk


# ---------------- 方向相关的保护语义 ----------------
def guard_hit(v, limit):
    """端压是否触碰保护门限 (放电=跌破下限 / 充电=涨过上限)。"""
    return v < limit if MODE["guard"] == "low" else v > limit


def guard_margin(v, limit):
    """离保护门限还剩多少 V (正值=还安全)。放电算到下限的余量, 充电算到上限的余量。

    两种方向统一成一个"越大越安全"的数, 告警就不用写两套比较符。
    limit 由调用方传 args.v_limit 而**不是**读 MODE 里的默认值 —— 否则
    --vlimit 传了也不生效, 而保护门限失效是没人会发现的那种错。
    """
    d = v - limit
    return d if MODE["guard"] == "low" else -d


# ---------------- 串口底层 ----------------
def ask(ser, cmd, wait=0.15):
    """发一条命令并读响应, 返回去空白字符串。写命令多数无回显。

    写失败(线松/设备重启, write_timeout 到时)抛异常, 由上层计数处理 ——
    保护逻辑依赖"通信还活着", 不能把故障当无回显吞掉。
    """
    ser.reset_input_buffer()
    ser.write(cmd)
    time.sleep(wait)
    return ser.read(4096).decode("ascii", errors="replace").strip()


def _num(s):
    """响应字符串 -> 数值。空响应当 0(沿用原行为, 0V 会立刻触发保护)。

    设备把命令原样回显成错误串(如 "ERROR")时抛串口异常, 走上层的通信看门狗 ——
    比抛 ValueError 崩掉一路都不断输出要安全。
    """
    s = (s or "").strip()
    if not s:
        return 0.0
    try:
        return float(s)
    except ValueError:
        raise serial.SerialException("设备返回非数值: %r" % s)


def meas_v(ser):
    return _num(ask(ser, (CMDS["volt"] + "\r\n").encode()))


def meas_i(ser):
    return _num(ask(ser, (CMDS["amp"] + "\r\n").encode()))


def set_curr(ser, amp):
    ask(ser, (CMDS["curr"].format(i="%.4f" % amp) + "\r\n").encode())
    r = ask(ser, (CMDS["curr_q"] + "\r\n").encode())
    try:
        return float(r) if r else None
    except ValueError:
        return None


def set_volt(ser, volt):
    """[仅充电] 设电源恒压(CV)上限, 读回只做日志校验(电源不回就是 None, 不致命)。

    先设 CV 再开输出: 顺序反了的话第一档就有一次无上限的恒流输出。
    """
    if "vset" not in CMDS:
        return None
    ask(ser, (CMDS["vset"].format(v="%.3f" % volt) + "\r\n").encode())
    if "volt_q" not in CMDS:
        return None
    r = ask(ser, (CMDS["volt_q"] + "\r\n").encode())
    try:
        return float(r) if r else None
    except ValueError:
        return None


def inp(ser, on):
    """输出/负载开关, 写失败重试 3 次, 返回是否成功 (保护路径要确保能断开)。"""
    cmd = ((CMDS["on"] if on else CMDS["off"]) + "\r\n").encode()
    for _ in range(3):
        try:
            ask(ser, cmd)
            return True
        except serial.SerialException:
            time.sleep(0.15)
    return False


# ---------------- 跨设备互锁 (两台程控设备并接同一节电池时用) ----------------
# soc_load 一次只开一个串口、只管得着自己那台(finally 里断开), **伙伴的开关状态不归
# 它管** —— 上一腿异常退出(拔线/掉电/被强杀)就可能把伙伴留在导通态, 这一腿一开输出
# 两台就对打。--peer-port 就是为此: 开自己的口之前, 先连上伙伴把它关掉并复核。
PEER_OFF_MAX_A = 0.05      # 断开后实测电流仍 >= 这个值就认为没真断 (与试拉判据同量级)


def off_and_verify(ser, off_cmd, amp_cmd, who):
    """下发断开命令并复核"真的断了"。三档判据, 每档的含义都在下面写清:

      · 命令重试 3 次都发不出去           -> False
      · 断开后实测电流 |I| >= 阈值        -> False (没真断 / 还有并联支路在流)
      · 电流读数拿不到(设备不实现/无响应) -> True, 但只算"命令已下发", 以面板为准

    最后一档故意不判失败: 不少负载/电源不实现 MEASure:CURRent? 或断开态不回数, 拿它
    当失败就等于"不支持读回的设备不许用互锁", 那是把"读不到"当成"不安全"。代价是这
    一档只确认到命令被认下 —— 日志会把它写清楚, 别把它当"电流复核过了"。
    """
    for _ in range(3):
        try:
            ask(ser, (off_cmd + "\r\n").encode())
            break
        except serial.SerialException:
            time.sleep(0.15)
    else:
        print("[!!] %s: 下发 %r 重试 3 次都失败" % (who, off_cmd))
        return False
    try:
        r = ask(ser, (amp_cmd + "\r\n").encode()).strip()
        i = float(r) if r else None
    except (serial.SerialException, ValueError):
        i = None
    if i is None:
        print("[i] %s: 已下发 %r, 电流读数拿不到 -> 只确认到'命令已下发', 请目视面板"
              % (who, off_cmd))
        return True
    if abs(i) >= PEER_OFF_MAX_A:
        print("[!!] %s: 已下发 %r 但实测仍 %.0f mA (>= %.0f mA) -> 没真断开, 拒绝继续"
              % (who, off_cmd, i * 1000, PEER_OFF_MAX_A * 1000))
        return False
    print("[i] %s: 已断开 (%s, 实测 %.0f mA)" % (who, off_cmd, i * 1000))
    return True


def peer_off(port, off_cmd=None):
    """把"另一台设备"关掉: 连它 -> 下发它自己的断开命令 -> 复核 -> 关串口。

    伙伴是谁、该发什么命令, 由**本腿方向**反推: 本腿放电则伙伴是电源 -> OUTP 0,
    本腿充电则伙伴是负载 -> INP 0。所以只给口通常就够了, 不用再给 --peer-off。
    返回 False 表示没能确认伙伴断开 -> 上层 fail-closed, 不驱动本腿设备。
    """
    other = "charge" if MODE["key"] == "discharge" else "discharge"
    dev = MODES[other]["dev"]
    ocmd = off_cmd or MODES[other]["cmds"]["off"]
    who = "互锁 %s %s" % (dev, port)
    try:
        ser = serial.Serial(port, LOAD_BAUD, timeout=0.4, write_timeout=0.5)
    except serial.SerialException as e:
        print("[!!] %s: 打不开 (%s)" % (who, e))
        return False
    try:
        idn = ask(ser, b"*IDN?\r\n", wait=0.4)
        if not idn:
            print("[!!] %s: 无 *IDN? 响应, 无法确认它是%s" % (who, dev))
            return False
        print("[i] %s: %s" % (who, idn))
        return off_and_verify(ser, ocmd, CMDS_BASE["amp"], who)
    except serial.SerialException as e:
        print("[!!] %s: 通信异常 (%s)" % (who, e))
        return False
    finally:
        try:
            ser.close()
        except Exception:
            pass


def open_port(port):
    # write_timeout 必须有: 线松/设备无响应时写会阻塞, 不放行保护逻辑
    ser = serial.Serial(port, LOAD_BAUD, timeout=0.4, write_timeout=0.5)
    idn = ask(ser, b"*IDN?\r\n", wait=0.4)
    if not idn:
        ser.close()
        sys.exit("%s 无 *IDN? 响应 —— 确认接的是程控%s (SCPI, 波特率 %d)?"
                 % (port, MODE["dev"], LOAD_BAUD))
    print(f"[i] {MODE['dev']}: {idn}")
    return ser


# ---------------- 串口自动识别(不依赖品牌) ----------------
def _idn_probe(dev):
    """探测一个串口像不像程控设备: 像就返回 IDN 串, 不像返回 None。

    两步, 都不依赖具体品牌:
      1. 静默检查: 先只听 0.3s。**主动往上报数据的口不是设备** —— 负载/电源只在
         被问的时候才说话, 而 MCU 数据口 / 定位模块这类是持续吐的。少了这一步,
         MCU 口吐的 "D,..." 帧会被当成 *IDN? 的应答, 认成设备。
      2. 再发 *IDN?, 谁应答谁就是候选; 是不是真设备由后面的试拉(0.1A)兜底确认。

    只读探测, 不改设备状态; 打不开的口(被占用 / 蓝牙假口 / AMT)直接跳过。
    write_timeout 必须有: 那类假串口的 write 会永久阻塞, 不带写超时整个扫描挂死。
    """
    try:
        s = serial.Serial(dev, LOAD_BAUD, timeout=0.3, write_timeout=0.5)
    except serial.SerialException:
        return None
    try:
        s.reset_input_buffer()
        if s.read(64):                     # 没人问就自己说话 -> 不是设备
            return None
        s.write(b"*IDN?\r\n")
        return s.read(256).decode("ascii", errors="replace").strip() or None
    except Exception:
        return None
    finally:
        try:
            s.close()
        except Exception:
            pass


def scan_load_port(idn_filter=()):
    """遍历系统串口, 返回 [(port, idn), ...] —— **能应答 *IDN? 的都算候选**。

    idn_filter 给了关键字(如 "ITECH")才按品牌收窄。默认不按品牌筛: 命令集本身
    就是通用 SCPI 写法, 认品牌没有意义, 只会把没见过的牌子挡在外面。

    蓝牙 SPP / Intel AMT-SOL 这类"假串口"按描述先过滤掉不试(每口都要等超时,
    程控设备不可能接在上面)。
    """
    import serial.tools.list_ports as lp
    skip = ("bluetooth", "蓝牙", "amt", "intel(r) active")
    found = []
    for p in lp.comports():
        d = (p.description or "").lower()
        if any(k in d for k in skip):
            continue
        idn = _idn_probe(p.device)
        if not idn:
            continue
        if idn_filter and not any(k.lower() in idn.lower() for k in idn_filter):
            continue
        found.append((p.device, idn))
    return found


def select_load_port(auto=False, idn_filter=()):
    """没有 --port 时选口: 先自动探测, 认不出来再列全串口手输。

    auto=True (--yes 非交互): 只认探测结果, 认不到就报错退出, 绝不提问。
    """
    found = scan_load_port(idn_filter)
    if found:
        if len(found) == 1 or auto:
            print(f"[i] 自动识别到{MODE['dev']}: {found[0][0]}  ({found[0][1]})")
            return found[0][0]
        print("识别到多个会应答 *IDN? 的设备:")
        for i, (dev, idn) in enumerate(found, 1):
            print(f"  [{i}] {dev:<6} {idn}")
        s = _input(f"选择 1-{len(found)} (回车=1): ").strip()
        if s and s.isdigit() and 1 <= int(s) <= len(found):
            return found[int(s) - 1][0]
        return found[0][0]
    import serial.tools.list_ports as lp
    ports = [p.device for p in lp.comports()]
    print("[!] 没探到会应答 *IDN? 的设备 (确认 RS232 已插/没被占用/波特率 %d)" % LOAD_BAUD)
    if auto:
        sys.exit("[X] 非交互模式无法询问串口: 请用 --port COMx 指定%s口 "
                 "(当前串口: %s)" % (MODE["dev"], ", ".join(ports) or "无"))
    if ports:
        print("现有串口:")
        for i, p in enumerate(ports, 1):
            print(f"  [{i}] {p}")
        s = _input(f"输入编号或直接输 COM 号: ").strip()
        if s:
            if s.isdigit() and 1 <= int(s) <= len(ports):
                return ports[int(s) - 1]
            return s.upper()
    return _input("输入串口 (如 COM5): ").strip().upper()


# ---------------- 交互询问 ----------------
def _input(prompt=""):
    """input 包装: EOF(管道/被向导调用)时友好退出而非抛异常。"""
    try:
        return input(prompt)
    except EOFError:
        print("\n(输入流结束, 取消)")
        sys.exit(2)


def askf(prompt, default, cast=float):
    s = _input(prompt).strip()
    return cast(s) if s else default


def parse_currents(text, n=10):
    """电流参数: 单值 -> N 档都用它; 逗号列表 -> 逐档; 'a:b' -> 在 n 档上等分。"""
    if ":" in text:
        a, b = [float(x) for x in text.split(":", 1)]
        return [a + (b - a) * k / max(n - 1, 1) for k in range(n)]
    parts = [float(x) for x in text.split(",")]
    return parts


def currents_for(text, n):
    """把 --current 的写法折成恰好 n 档的电流列表。

    单值 -> 每档都用它; 逗号列表 -> 逐档(不够就沿用最后一个); 'a:b' -> 在 n 档等分。

    单独抽成函数是为了让 soc_bench.py 的 cycle 子命令也算同一份 —— 循环测试要跑
    几小时, 预览的"预计时长"必须与 soc_load 真跑出来的档位/时长逐个一致, 两边
    各写一遍迟早会对不上(而且对不上时已经浪费了半天)。
    """
    currs = parse_currents(str(text), n)
    if len(currs) == 1:
        currs = currs * n
    if len(currs) < n:
        currs = currs + [currs[-1]] * (n - len(currs))
    return currs[:n]


def parse_idn(text):
    """--idn "ITECH,MAYNUO" -> ("ITECH", "MAYNUO"); 空 -> () 表示不按品牌筛。"""
    return tuple(x.strip() for x in str(text or "").split(",") if x.strip())


def interactive(args):
    """交互询问缺省参数 (命令行已给的跳过)。

    --yes 时是纯非交互: 缺的参数一律取括号里的缺省值, 一次也不提问
    (GUI 用它; 真实加载/输出的"确认"这一步由界面上的确认框承担)。
    """
    auto = bool(getattr(args, "yes", False))
    nm = MODE["name"]
    print("-" * 62)
    if auto:
        print("非交互模式 (--yes): 未指定的参数全部取缺省值")
    print("%s标定参数 (%s, 回车=括号内缺省)" % (nm, MODE["dev"]))
    if args.port is None:
        args.port = select_load_port(auto=auto,
                                     idn_filter=parse_idn(getattr(args, "idn", None)))
    if args.capacity is None:
        args.capacity = 3350 if auto else askf("电池容量 mAh (3350): ", 3350)
    if args.init_soc is None:
        args.init_soc = (MODE["init_soc"] if auto else
                         askf("起始 SOC%% (%.0f): " % MODE["init_soc"],
                              MODE["init_soc"]))
    if args.to_soc is None:
        args.to_soc = (MODE["to_soc"] if auto else
                       askf("末点 SOC%% (%.0f): " % MODE["to_soc"], MODE["to_soc"]))
    if args.spacing is None:
        if auto:
            args.spacing = "ocv"
        else:
            s = _input("档位分布 ocv=每档压降相等(默认,曲率大处加密) / soc=每档SOC相等: ").strip()
            args.spacing = s if s in ("soc",) else "ocv"
    if args.segments is None:
        args.segments = 10 if auto else int(askf("档位数 (10): ", 10, int))
    if args.current is None:
        args.current = (MODE["current"] if auto else
                        askf("每档电流 A (%.1f; 或用 0.5,1,.. 逗号序列 / a:b 递变): "
                             % MODE["current"], MODE["current"]))
    if args.rest_s is None:
        args.rest_s = 480 if auto else askf(
            "档间静置 s (480 = 6*tau, OCV 恢复): ", 480)
    if args.v_limit is None:
        args.v_limit = (MODE["v_limit"] if auto else
                        askf("%s (%.2f): " % (MODE["v_hint"], MODE["v_limit"]),
                             MODE["v_limit"]))
    if MODE["key"] == "charge" and args.v_set is None:
        args.v_set = (V_SET_DEFAULT if auto else
                      askf("电源恒压 CV 上限 V (%.2f, 电芯绝对上限, 最后防线): "
                           % V_SET_DEFAULT, V_SET_DEFAULT))
    print("-" * 62)
    return args


# ---------------- 档位计划落盘 ----------------
def write_plan(out_prefix, args, brk, currs, mah_list, pulse_list,
               seg_rows, aborted, idn):
    """落盘两件事, 供两个不同的读者:

      <前缀>.csv   逐档时刻表 —— 人看的, 也是与 record 做的起点时刻对齐表;
      <前缀>.json  同一份计划的机器可读版 —— soc_bench.py stair 靠它自动认出
                   "这次标定了几档 / 每档计划多少 mAh 多久多大电流 / 起始 SOC
                   是多少 / 有没有被保护提前中止"。

    为什么非要有 json: **计划里没跑完的档也要记下来**。stair 只看 CSV 的话,
    被保护中止后它只能看到 5 档, 跟"本来就只打算跑 5 档"分不清; 有了
    segments_done / aborted 就能给准话。改名成 <record前缀>_plan.json 即可被认到。
    """
    done_by_seg = {r["seg"]: r for r in seg_rows}
    segs_json = []
    for k in range(len(currs)):
        row = done_by_seg.get(k + 1)
        item = {"seg": k + 1,
                "soc_from": round(brk[k], 3), "soc_to": round(brk[k + 1], 3),
                "mah": round(mah_list[k], 2), "i_set_a": currs[k],
                "pulse_s": round(pulse_list[k], 1),
                "done": row is not None}
        if row is not None:
            item.update({"t_start": row["t_start"], "t_load_on": row["t_load_on"],
                         "t_load_off": row["t_load_off"],
                         "t_rest_end": row["t_rest_end"]})
        segs_json.append(item)

    plan = {
        "format": "soc-plan/1",
        "tool": "soc_load.py",
        "created": datetime.datetime.now().isoformat(timespec="seconds"),
        "mode": MODE["key"],
        "device": MODE["dev"],
        "idn": idn,
        "port": args.port,
        "capacity_mah": float(args.capacity),
        "init_soc": float(args.init_soc),
        "to_soc": float(args.to_soc),
        "spacing": args.spacing,
        "segments_planned": len(currs),
        "segments_done": len(seg_rows),
        "aborted": bool(aborted),
        "rest_s": float(args.rest_s),
        "v_limit": float(args.v_limit),
        "v_set": (float(args.v_set) if MODE["key"] == "charge" else None),
        "current_a": [float(c) for c in currs],
        "breaks_pct": [round(float(b), 3) for b in brk],
        "mah_planned": [round(float(m), 2) for m in mah_list],
        "pulse_s_planned": [round(float(p), 1) for p in pulse_list],
        "ocv_table_mv": [float(x) for x in args.ocv_table_used],
        "segments": segs_json,
    }
    pj = out_prefix + ".json"
    with open(pj, "w", encoding="utf-8") as f:
        json.dump(plan, f, indent=2, ensure_ascii=False)
    if seg_rows:
        pc = out_prefix + ".csv"
        with open(pc, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=list(seg_rows[0].keys()))
            w.writeheader()
            w.writerows(seg_rows)
        print(f"[i] 档位时刻表: {pc}")
    print(f"[i] 档位计划:   {pj}")
    print("[i] 标定采集: 另开终端跑 soc_record.py (MCU 串口), 结束后 "
          "soc_bench.py stair --csv <record.csv> 出 R1/OCV/SOC")
    print(f"[i] 想让 stair 自动认出这次计划: 把上面那份 json 改名成 "
          f"<record.csv 的前缀>_plan.json 放在同一目录即可, "
          f"届时 --capacity/--init-soc/--expect 都不用再输")


# ---------------- 主流程 ----------------
def _stdin_is_tty():
    """stdin 是真实终端才允许提问。

    被 soc_bench.py 向导拉起时 subprocess 会继承 TTY, 所以向导那条路径照样能选
    方向; 被 GUI(--yes) 或测试脚本(重定向 stdin) 调用时返回 False, 既不提问也
    不会撞上 EOFError。
    """
    try:
        return bool(sys.stdin) and sys.stdin.isatty()
    except Exception:
        return False


def ask_direction(args):
    """没显式给 --charge 时, 在终端问一次方向 (回车=放电)。

    放在这里而不是 argparse 里, 是为了让"直接跑 soc_load.py"与"从向导零参数
    拉起"两条路径都能选到充电。
    三种情况不问: ① 已给 --charge; ② --yes(非交互契约, GUI 走这条);
    ③ --demo(联调模式本来就不问参数, 它自己把参数写死了)。
    """
    if (getattr(args, "charge", False) or getattr(args, "yes", False)
            or getattr(args, "demo", False)):
        return
    if not _stdin_is_tty():
        return
    s = _input("方向  1=放电(电子负载)  2=充电(可编程电源)  [1]: ").strip()
    if s == "2":
        args.charge = True


def cmd_run(args):
    ask_direction(args)
    mode_key = "charge" if getattr(args, "charge", False) else "discharge"
    apply_mode(mode_key, getattr(args, "cmd", None))
    nm = MODE["name"]

    if args.demo:
        # demo: 空载联调, 不询问参数, 小电流短脉冲, 只验证时序与指令
        if args.port is None:
            # 同样自动探测串口
            args.port = select_load_port(
                idn_filter=parse_idn(getattr(args, "idn", None)))
        args.segments = args.segments or 2
        args.pulse_s = args.pulse_s if args.pulse_s is not None else 2
        args.rest_s = args.rest_s if args.rest_s is not None else 2
        args.current = args.current or "0.1"
        args.capacity = 3350.0
        args.init_soc = MODE["init_soc"]
        args.to_soc = MODE["to_soc"]
        args.spacing = "ocv"
        args.v_limit = 0.0
        args.v_set = V_SET_DEFAULT if MODE["key"] == "charge" else None
        print("[demo] 空载联调模式: 不校验电流, 只跑指令时序")
    else:
        args = interactive(args)

    tbl = [float(x) for x in str(args.ocv_table).split(",")] if args.ocv_table \
        else DEFAULT_OCV_MV
    if len(tbl) != 11:
        sys.exit("--ocv-table 需要 11 个 mV 值 (SOC 0,10,...,100%)")
    args.ocv_table_used = tbl
    seg_n = args.segments
    currs = currents_for(args.current, seg_n)

    # 方向自洽检查: 放电必须 soc_from > soc_to, 充电反过来。传反了当场报错,
    # 而不是让 build_breaks 把整个计划倒着排一遍(那会得到一份"看起来正常但方向
    # 相反"的计划, 现场跑起来才发现)。
    if not args.demo:
        d_soc = args.to_soc - args.init_soc
        if MODE["guard"] == "low" and d_soc >= 0:
            sys.exit("[X] 放电要求 起始 SOC > 末点 SOC (现在是 %.1f%% -> %.1f%%)。"
                     "想充电请加 --charge" % (args.init_soc, args.to_soc))
        if MODE["guard"] == "high" and d_soc <= 0:
            sys.exit("[X] 充电要求 起始 SOC < 末点 SOC (现在是 %.1f%% -> %.1f%%)。"
                     "想去掉 --charge 按放电跑" % (args.init_soc, args.to_soc))

    # 档位断点 -> 每档放出/充入容量与时长 (demo 用固定脉冲)
    if args.demo:
        pulse_list = [args.pulse_s] * seg_n
        mah_list = [0.0] * seg_n
        brk = build_breaks(tbl, args.init_soc, args.to_soc, seg_n, "ocv")
    else:
        brk = build_breaks(tbl, args.init_soc, args.to_soc, seg_n, args.spacing)
        # 取绝对值: 充电时断点是递增的, 差值本身为负, 容量与时长只关心幅值
        mah_list = [abs(brk[k + 1] - brk[k]) / 100.0 * args.capacity
                    for k in range(seg_n)]
        pulse_list = [m / c * 3.6 for m, c in zip(mah_list, currs)]

    total_min = sum(pulse_list) / 60 + seg_n * args.rest_s / 60
    q_total = sum(mah_list)
    soc_end = args.init_soc + MODE["soc_sign"] * q_total / args.capacity * 100
    print(f"{seg_n} 档 | {args.init_soc:.0f}% -> {soc_end:.1f}% | "
          f"分布={args.spacing} (ΔOCV等分: 曲线陡处自动加密)")
    print(f"总{nm} {q_total:.0f}mAh | 预计 {total_min:.0f}min | "
          f"每档静置 {args.rest_s:.0f}s")
    for k in range(seg_n):
        print(f"  [{k + 1:>2}] SOC {brk[k]:5.1f}% -> {brk[k + 1]:5.1f}%   "
              f"{mah_list[k]:5.0f}mAh  {currs[k]:.2f}A / {pulse_list[k]:.0f}s")
    if MODE["key"] == "charge":
        print(f"电源恒压上限 {args.v_set:.2f}V | 脚本中止门限 {args.v_limit:.2f}V "
              f"(端压先到门限才正常; 顶到 CV 说明计划排高了)")
    if args.v_limit > 0:
        print(f"{'端压上限' if MODE['guard'] == 'high' else '端压底线'} "
              f"{args.v_limit}V ({MODE['tail'].split('=')[-1].strip()}, "
              f"非 OCV 目标, 只有异常/走过头时才会碰到)")
    # 末档端压估算: 按 ~150mΩ 保守表观压降(低 SOC 内阻升高+线阻), 提醒会不会被门限/CV 误截
    vt = soc_ocv(brk[-1], tbl) / 1000.0
    drop = max(currs) * 0.15
    exp_end_v = vt + drop if MODE["guard"] == "high" else vt - drop
    print(f"末档预期: OCV≈{vt:.2f}V {'+' if MODE['guard'] == 'high' else '-'} "
          f"{max(currs):.2f}A×~0.15Ω → 端压≈{exp_end_v:.2f}V "
          f"({MODE['tail']})")
    if args.v_limit > 0 and guard_margin(exp_end_v, args.v_limit) < 0.15:
        if MODE["guard"] == "high":
            hint = ("--vlimit %.2f 以上, 或降低 --to-soc / 减小电流"
                    % (exp_end_v + 0.15))
        else:
            hint = ("--vlimit %.2f 以下, 或提高 --to-soc / 减小电流"
                    % (exp_end_v - 0.15))
        print(f"[!] 末档端压离门限只有 "
              f"{guard_margin(exp_end_v, args.v_limit):+.2f}V, 会提前截断! 建议 {hint}")
    if MODE["key"] == "charge" and args.v_set and exp_end_v > args.v_set - 0.02:
        print(f"[!] 末档估算端压 {exp_end_v:.2f}V 已达/超过电源 CV {args.v_set:.2f}V "
              f"→ 会在进入 CV 后电流自然衰减, 那一档时长不再受控。"
              f"建议 减 --current 或 降 --to-soc")
    if not args.demo:
        print("以上计划确认后回车 = 开始真实%s; 输 n = 取消。" % nm)
        print("(开始后 Ctrl+C 可安全中止, 会自动断开%s)" % MODE["dev"])
        if getattr(args, "yes", False):
            print("[i] --yes: 跳过回车确认, 直接开始")
        else:
            try:
                if _input().strip().lower().startswith("n"):
                    print("已取消")
                    return
            except KeyboardInterrupt:
                return

    # ---- 跨设备互锁: 先关伙伴, 再关自己, 最后才开跑 (顺序不可颠倒) ----
    # 本脚本只管得着自己那台; 伙伴的开关状态是上一腿/人留下的, 异常退出就可能还开着。
    # 所以在 open_port(自己) **之前**先把伙伴关掉并复核 —— 顺序反了中间会有一瞬间两台
    # 同时导通, 互锁就白做了。check_consts.py 用行号把这条顺序守着。
    peer = (getattr(args, "peer_port", None) or "").strip()
    if peer:
        if peer.upper() == (args.port or "").strip().upper():
            sys.exit("[X] --peer-port 与 --port 是同一个口 (%s): 那会把自己的设备"
                     "当成伙伴关掉, 本腿就没设备可用" % peer)
        print("\n[互锁] 两台设备并接: 先断开另一台, 再驱动本腿设备")
        if not peer_off(peer, getattr(args, "peer_off_cmd", None)):
            sys.exit("[X] 互锁未通过: 伙伴设备没能确认断开, 本腿设备未被驱动。"
                     "检查伙伴是否上电/串口是否被占用; 确实没有伙伴设备可去掉 "
                     "--peer-port, 但两台并接时去掉就等于放弃互锁")
    ser = open_port(args.port)
    idn = ask(ser, b"*IDN?\r\n")
    # CHAN? 是多通道机型才有的查询; 单通道设备不回或报错都无所谓, 这里只是个提示
    ch = ask(ser, b"CHAN?\r\n")
    print(f"[i] 当前通道: {ch or '?'}  ({seg_n} 档将操作该通道)")
    # 自己也先确认断开: 本腿设备同样可能被上一腿的异常退出留在导通态
    if not off_and_verify(ser, CMDS["off"], CMDS["amp"],
                          "本腿%s %s" % (MODE["dev"], args.port)):
        ser.close()
        sys.exit("[X] 本腿设备(%s)的断开命令发不出去, 不开始加载 —— 通信/接线有问题"
                 % args.port)
    ask(ser, b"*CLS\r\n")
    if MODE["key"] == "charge":
        # 先落 CV 上限再开输出: 反了的话第一档就有一次无上限的恒流输出
        rv = set_volt(ser, args.v_set)
        print(f"[i] 电源恒压上限设为 {args.v_set:.2f}V"
              + (f"  (读回 {rv:.2f}V)" if rv is not None else "  (读数不支持, 以面板为准)"))

    aborted_all = False
    seg_rows = []
    try:
        if not args.demo:
            # 试拉验证: 0.1A 确认当前模式是 CC 且接线有电池
            print("\n[验证] 设 0.1A 试拉 0.5s, 确认 CC 模式与电池接线 ...")
            set_curr(ser, 0.1)
            inp(ser, True)
            time.sleep(0.5)
            vi, ii = meas_v(ser), meas_i(ser)
            inp(ser, False)
            # 电流只看绝对幅值: 电子负载报"抽流为正", 多数可编程电源报"源出为正",
            # 而 MCU/INA226 又是"放电为正、充电为负" —— 三套符号约定互不相同,
            # 我们从不拿设备电流去积分(积分数据全部来自 MCU), 所以符号一概不关心。
            if abs(ii) < 0.05 and vi > 0.5:
                sys.exit(f"试拉电流设定 0.1A 但实测 {ii*1000:.0f}mA / {vi:.2f}V -> "
                         f"当前模式可能不是 CC(检查面板 MODE) 或通道不对, 已断开")
            if vi < 1.0:
                print(f"[!] 输入端只有 {vi:.2f}V, 确认电池已接上")
            print(f"[i] 试拉通过: 端压 {vi:.2f}V 实测 {abs(ii)*1000:.0f}mA (CC 模式 OK)")

        t_abs0 = datetime.datetime.now()
        print("\n开始 %d 档循环 (Ctrl+C 安全中断, 自动断开输出):" % seg_n)
        for k in range(seg_n):
            print(f"\n===== 档 {k+1}/{seg_n}  SOC {brk[k]:.1f}% -> {brk[k+1]:.1f}%  "
                  f"{currs[k]:.2f}A / {pulse_list[k]:.0f}s =====")
            t1 = datetime.datetime.now()
            if not args.demo:
                # 档前保护: 设电流 + 读开路电压, 通信异常即中止
                try:
                    set_curr(ser, currs[k])
                    v0 = meas_v(ser)
                except serial.SerialException:
                    print("  [!!] 设电流/读电压通信失败, 中止整轮")
                    break
                # 断点 OCV 校验: 该档起始电压应≈内置表在此 SOC 的预期值,
                # 偏离过大说明 电池没充满/没放空 / 容量或 OCV 表与电芯不符 / 接错电池
                if v0 > 0.1:
                    exp_v = soc_ocv(brk[k], tbl) / 1000.0
                    dev = v0 - exp_v
                    print(f"  {'充电' if MODE['guard'] == 'high' else '放电'}前电压 "
                          f"{v0:.3f}V (预期~{exp_v:.3f}, 门限 {args.v_limit}V)")
                    if abs(dev) > 0.5:
                        print(f"  [!!] 与预期差 {dev:+.3f}V (>0.5V): 电池没充满/没放空/"
                              f"容量或 OCV 表不对/接错电池组, 中止")
                        inp(ser, False)
                        break
                    if abs(dev) > 0.25:
                        print(f"  [!] 与预期差 {dev:+.3f}V (>0.25V), 后面档位电压会整体偏移, "
                              f"注意核对容量/起始 SOC")
                else:
                    print(f"  档前电压 {v0:.3f}V (门限 {args.v_limit}V)")
                if guard_margin(v0, args.v_limit) < 0.1:
                    print("  [!] 电压已接近门限, 终止剩余档位")
                    inp(ser, False)
                    break
            inp(ser, True)
            t_on = datetime.datetime.now()
            if not args.demo:
                # 开载后电流确认: 1.5s 后应达到设定电流, 否则线断/接触不良/模式错
                time.sleep(1.5)
                try:
                    i_chk = meas_i(ser)
                except serial.SerialException:
                    i_chk = None
                if i_chk is None or abs(i_chk) < max(0.1, currs[k] * 0.85):
                    inp(ser, False)
                    print(f"  [!!] 加载 1.5s 实测电流 "
                          f"{('?A' if i_chk is None else f'{i_chk:+.2f}A')}, 远小于设定 "
                          f"{currs[k]:.2f}A -> 接线/接触不良或模式错, 已断开, 中止整轮")
                    break
            # 循环段: 周期轮询电压门限; 连续通信失败 3 次视为线松, 强制断开
            deadline = time.monotonic() + pulse_list[k]
            aborted = False
            comm_fail = 0
            while time.monotonic() < deadline:
                if not args.demo:
                    try:
                        v = meas_v(ser)
                    except serial.SerialException:
                        comm_fail += 1
                        if comm_fail >= 3:
                            print("\n  [!!] 连续 3 次读不到设备 (通信中断/线松), 强制断开并终止")
                            aborted = True
                            break
                        time.sleep(0.6)
                        continue
                    comm_fail = 0
                    if guard_hit(v, args.v_limit):
                        print(f"\n  [!!] 端压 {v:.3f}V "
                              f"{'>' if MODE['guard'] == 'high' else '<'} "
                              f"{args.v_limit}V, 立即断开 (本档中止)")
                        aborted = True
                        break
                time.sleep(min(3.0, deadline - time.monotonic()))
            inp(ser, False)
            t_off = datetime.datetime.now()
            # 静置段
            t_rest0 = datetime.datetime.now()
            rem = args.rest_s
            while rem > 0:
                time.sleep(min(10.0, rem))
                rem = args.rest_s - (datetime.datetime.now() - t_rest0).total_seconds()
            seg_rows.append({
                "seg": k + 1, "soc_from": brk[k], "soc_to": brk[k + 1],
                "mah": mah_list[k], "i_set_a": currs[k], "pulse_s": pulse_list[k],
                "t_start": t1.isoformat(timespec="seconds"),
                "t_load_on": t_on.isoformat(timespec="seconds"),
                "t_load_off": t_off.isoformat(timespec="seconds"),
                "t_rest_end": datetime.datetime.now().isoformat(timespec="seconds"),
            })
            v_off = meas_v(ser) if not args.demo else 0
            print(f"  断开时刻 {t_off.strftime('%H:%M:%S')}  静置 {args.rest_s}s 后 "
                  f"电压 {v_off:.3f}V")
            if aborted:
                done = (t_off - t_on).total_seconds()
                frac = done / pulse_list[k] if pulse_list[k] else 0
                if frac < 0.25:
                    print(f"[!] 门限来得太早 (本档只跑了 {frac*100:.0f}%): "
                          f"容量/电流/接线可能不对, 中止整轮")
                else:
                    print(f"[!] 电压门限触发 (已跑 {frac*100:.0f}%), 若已到"
                          f"{'高' if MODE['guard'] == 'high' else '低'} SOC 属正常, "
                          f"中止后续档位")
                aborted_all = True
                break
        # 任何"提前 break"都算中止。循环里有 5 处 break(通信失败 / OCV 偏离 /
        # 接近门限 / 开载未确认 / 电压门限触发), 只有最后一处自己赋值过
        # aborted_all; 靠这一行兜住其余四处, 否则计划 json 会谎报 aborted=false
        # —— 而 stair 正是靠这个字段区分"本来只打算跑 3 档"和"第 3 档被截停了"。
        if len(seg_rows) < seg_n:
            aborted_all = True
    except KeyboardInterrupt:
        print("\n[!] Ctrl+C 收到, 断开输出")
        aborted_all = True
    finally:
        if not inp(ser, False):
            print("[!!] 断开指令重试 3 次失败, 请到%s面板手动关闭输出!" % MODE["dev"])
        try:
            ser.close()
        except Exception:
            pass
        print(f"[i] {MODE['dev']}已断开, 串口已关")

    # 计划无条件落盘: 即使一档没跑成(比如试拉就失败), stair 也要能读到"打算跑什么"
    out = args.out or MODE["out"]
    write_plan(out, args, brk, currs, mah_list, pulse_list,
               seg_rows, aborted_all, idn_clean(idn))


def idn_clean(s):
    """IDN 去掉控制字符, 免得写进 json 变成不可见垃圾。"""
    return "".join(c for c in (s or "") if 32 <= ord(c) < 127)


def main():
    ap = argparse.ArgumentParser(
        description="程控源/载多档标定控制器 (SCPI, 配合 soc_record.py 采集); "
                    "默认放电(电子负载), --charge 切成充电(可编程电源)")
    ap.add_argument("--charge", action="store_true",
                    help="充电方向: 控可编程电源 OUTP+VOLT, 端压上限保护, 默认 9%%->80%%")
    ap.add_argument("--port", default=None, help="设备 RS232 串口 (缺省自动探测)")
    ap.add_argument("--capacity", type=float, default=None, help="电池容量 mAh (默认 3350)")
    ap.add_argument("--init-soc", type=float, default=None,
                    help="起始 SOC%% (放电默认 100 / 充电默认 9)")
    ap.add_argument("--to-soc", type=float, default=None,
                    help="末点 SOC%% (放电默认 9: 覆盖低端非线性段 / 充电默认 80: 避开 CV)")
    ap.add_argument("--spacing", choices=("ocv", "soc"), default=None,
                    help="档位分布: ocv=每档压降相等(默认, 陡段加密) / soc=每档SOC相等")
    ap.add_argument("--ocv-table", default=None,
                    help="OCV 表 11 个 mV (0..100%%), 覆盖内置表 (与 soc.c 同步)")
    ap.add_argument("--segments", type=int, default=None, help="档位数 (默认 10)")
    ap.add_argument("--current", default=None,
                    help="每档电流 A: 单值 / 逗号序列 / '0.5:3'递变 (放电默认 2.0 / 充电 1.6)")
    ap.add_argument("--rest-s", type=float, default=None, help="档间静置 s (默认 480)")
    ap.add_argument("--vlimit", "--cutoff", dest="v_limit", type=float, default=None,
                    help="脚本中止门限 V: 放电=端压底线(默认 2.8) / 充电=端压上限(默认 4.15)。"
                         "只兜底非目标, 见计划预览的末档估算")
    ap.add_argument("--vset", dest="v_set", type=float, default=None,
                    help="[仅充电] 电源恒压 CV 设定值 V, 默认 %.2f (电芯绝对上限, 最后防线)"
                         % V_SET_DEFAULT)
    ap.add_argument("--out", default=None,
                    help="输出文件名前缀 (默认 放电=load_segments / 充电=charge_segments)")
    ap.add_argument("--demo", action="store_true",
                    help="空载联调: 跳过试拉/门限检查, 用 --pulse-s 固定脉冲")
    ap.add_argument("--pulse-s", type=float, default=None, help="demo 模式每档时长 s")
    ap.add_argument("--idn", default=None,
                    help="自动探测时按 IDN 关键字收窄, 逗号分隔 (缺省=谁应答 *IDN? 就认谁, "
                         "不按品牌筛)")
    ap.add_argument("--cmd", action="append", default=None, metavar="键=命令",
                    help="覆盖命令方言表, 可重复; 键: curr/curr_q/on/off/volt/amp, "
                         "充电另有 vset/volt_q。"
                         "例: --cmd on=\"LOAD:STATe 1\" --cmd off=\"LOAD:STATe 0\" (换品牌用)")
    ap.add_argument("--peer-port", default=None,
                    help="另一台程控设备的串口 (充/放两台并接同一节电池时给): 开跑前先"
                         "连它、按反方向下发断开命令并复核电流, 再驱动本腿设备。连不上/"
                         "关不掉即中止, 不会'先开自己再说'")
    ap.add_argument("--peer-off", dest="peer_off_cmd", default=None, metavar="命令",
                    help="覆盖给伙伴设备下发的断开命令 (缺省按反方向自动取: 放电腿给电源 "
                         "OUTP 0 / 充电腿给负载 INP 0)")
    ap.add_argument("--yes", action="store_true",
                    help="非交互: 缺省参数全取默认值, 跳过「回车开始」确认 (上位机调用用)")
    args = ap.parse_args()
    cmd_run(args)


if __name__ == "__main__":
    main()
