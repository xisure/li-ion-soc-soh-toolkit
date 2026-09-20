#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""check_consts.py - 多副本常量对拍（固件 ↔ 四个 PC 副本 ↔ 上位机）

同一张表在工程里存了好几份，改一边忘另一边，标定结果就会与板子行为悄悄错开。
这个脚本直接从**源码文本**里抽表比对（不 import 任何脚本，所以无依赖、不受
serial / numpy / tkinter 是否装好影响），把[README 第 5 节]那张"固件 ↔ PC 常量
同步表"逐项变成可执行断言。

    python check_consts.py            # 在 脚本/ 或 脚本/test/ 下都能跑
    python check_consts.py --quiet    # 只打印结论

对拍项:
  [1] OCV 11 点   固件 SOC_OCV_25C_MV ↔ soc_load / soc_bench / soh_learn / kalman_tune
  [2] R0/R1/tau   固件 ↔ soh_learn ↔ kalman_tune
  [3] 标称容量    固件 ↔ soc_bench.STAIR_CAPACITY / soh_learn.Q_NOM_MAH / kalman_tune.CAP
  [4] 静置门限    固件 ↔ soc_bench.IDLE_MA / soh_learn.IDLE_MA / kalman_tune.IDLE_MA
  [5] stair 判据  soc_bench 的带载滞回/最短脉冲/取尾时长
  [6] 方向默认值  soc_load.MODES ↔ bms_gui.DIR_DEFAULTS（含"脚本门限 < 电源 CV"）
  [7] 计划格式串  soc_load 写的 format == soc_bench.PLAN_FORMAT
  [8] 形态检查    OCV 严格递增 / R0、R1、tau 非负且 11 点
  [9] 循环默认值  soc_bench.CYCLE_* ↔ soc_load 的 MODES 与 --yes 默认值
                  （窗口两端/两腿电流/两个门限/电源 CV/档数/静置；含"两腿首尾相接"
                   与"充电门限 < 电源 CV"）
  [10] 跨设备互锁  soc_load.--peer-port ↔ soc_bench.cycle ↔ bms_gui
  [11] 三模式      bms_gui.DIR_CHOICES / CYC_DEFAULTS ↔ soc_bench.CYCLE_* / LEG_RUNNER
                  （模式顺序 0/1/2 固定；循环默认值逐项；两个 --port-* 不同源；
                   腿执行器 LEG_RUNNER 可替换且默认仍是子进程）
                  （顺序不变量：先关伙伴再开自己；伙伴命令按反方向反推；cycle 两条腿
                   的 --peer-port 交叉配对不能反；同口校验两侧都在）
  [12] 协议元数据  bms_config.h / bms_tune.h 的 PROTO_VER（两处 #ifndef 兜底必须同值）
                  ↔ bms_tune_proto.PROTO_VER；BMS_NVM_VER ↔ DEFAULT_DIMS['nvm_ver']；
                  BMS_NVM_PAYLOAD_LEN ↔ SOH_BLOB_BYTES。
                  前两个源头文件不在时打 SKIP（产物里只带 bms_config.h）。

退出码 0 = 全部一致；1 = 有对不上的项。
"""
import io
import os
import re
import sys

# 控制台是 GBK 时, 报告里的 ↔ / ★ 这类字符会让 print 直接抛 UnicodeEncodeError —— 报错
# 发生在"打印"这一步, 整份对拍结果一个字都看不到。强制按 UTF-8 输出、编不出来的顶 ?:
# 宁可某个符号变问号, 也不能让结论整份丢掉。
try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)                     # 脚本/
REPO = os.path.dirname(ROOT)                     # 仓库根

FW = next((p for p in (os.path.join(REPO, "算法库", "bms_config.h"),
                       os.path.join(REPO, "firmware", "bms_config.h"),
                       # 打进产物后: 上位机/tools/ 根上就有一份(build_exe 拷的),
                       # 加这两个候选, 在产物里也能做固件侧对拍
                       os.path.join(ROOT, "bms_config.h"),
                       os.path.join(REPO, "bms_config.h"))
           if os.path.isfile(p)), None)
S_LOAD = os.path.join(ROOT, "soc_load.py")
S_BENCH = os.path.join(ROOT, "soc_bench.py")
S_SOH = os.path.join(ROOT, "soh_learn.py")
S_KAL = os.path.join(ROOT, "test", "kalman_tune.py")
S_GUI = os.path.join(ROOT, "bms_gui.py")

bad = []
lines = []


def rd(p):
    return io.open(p, encoding="utf-8", errors="replace", newline="").read()


def ok(name, msg):
    lines.append("  OK   %-34s %s" % (name, msg))


def no(name, msg):
    lines.append("  FAIL %-34s %s" % (name, msg))
    bad.append("%s: %s" % (name, msg))


def skip(name, msg):
    """对拍项在当前目录结构下**无从对起**（源头文件没随包发布）。

    与 no() 的分工: "文件不在" 是环境事实, "文件在却抽不到值" 才是缺陷。
    产物 `上位机/tools/` 只带 `bms_config.h`（build_exe 的 FW_CFG）, 拿它去
    对 `bms_tune.h` 里的同名兜底本来就无意义 —— 原来一律判 FAIL, 产物自检
    永远红两条, 真漂移时反被噪声盖住。

    **但仍要打一行**: 静默跳过等于这一项不存在, 那是本项目踩过的老坑。
    """
    lines.append("  SKIP %-34s %s" % (name, msg))


def macro_body(text, name):
    """取 #define 宏的完整正文（跟着行尾反斜杠续行）。找不到返回 None。"""
    ls = text.splitlines()
    for i, ln in enumerate(ls):
        if re.match(r"\s*#\s*define\s+%s\b" % re.escape(name), ln):
            body, j = [], i
            while j < len(ls):
                body.append(ls[j])
                if not ls[j].rstrip().endswith("\\"):
                    break
                j += 1
            return "\n".join(body)
    return None


def macro_ints(text, name):
    b = macro_body(text, name)
    if b is None:
        return None
    b = b.split(None, 2)
    b = b[2] if len(b) > 2 else ""
    b = re.sub(r"/\*.*?\*/", "", b, flags=re.S)      # 块注释（里面全是 0% / 13.8 之类数字）
    b = re.sub(r"//[^\n]*", "", b)
    return [int(x) for x in re.findall(r"-?\d+", b)]


def py_list(text, name):
    """取模块顶层的 NAME = [1, 2, 3]（支持跨行）。"""
    m = re.search(r"^%s\s*=\s*\[(.*?)\]" % re.escape(name), text, re.M | re.S)
    if not m:
        return None
    return [int(float(x)) for x in re.findall(r"-?[\d.]+", m.group(1))]


def py_num(text, name):
    m = re.search(r"^%s\s*=\s*([-\d.]+)" % re.escape(name), text, re.M)
    return float(m.group(1)) if m else None


def py_str(text, name):
    m = re.search(r"^%s\s*=\s*['\"]([^'\"]+)['\"]" % re.escape(name), text, re.M)
    return m.group(1) if m else None


def same(a, b):
    return a is not None and b is not None and list(a) == list(b)


def main():
    quiet = "--quiet" in sys.argv
    if not all(os.path.isfile(p) for p in (S_LOAD, S_BENCH, S_SOH, S_KAL, S_GUI)):
        print("[X] 找不到脚本文件，请在 脚本/ 或 脚本/test/ 下运行")
        return 2

    fw = rd(FW) if FW else ""
    load, bench, soh, kal, gui = rd(S_LOAD), rd(S_BENCH), rd(S_SOH), rd(S_KAL), rd(S_GUI)

    if FW:
        lines.append("固件: %s" % os.path.relpath(FW, REPO))
    else:
        lines.append("固件: 没找到 bms_config.h —— 只做 PC 侧互比")
    lines.append("")

    # ---------------- [1] OCV 表 ----------------
    lines.append("[1] OCV 表（11 点 mV）")
    fw_ocv = macro_ints(fw, "SOC_OCV_25C_MV") if FW else None
    pc = [("soc_load.DEFAULT_OCV_MV", py_list(load, "DEFAULT_OCV_MV")),
          ("soc_bench.STAIR_OCV_MV", py_list(bench, "STAIR_OCV_MV")),
          ("soh_learn.OCV_MV", py_list(soh, "OCV_MV")),
          ("kalman_tune.OCV", py_list(kal, "OCV"))]
    ref = pc[0][1]
    if ref is None:
        no("OCV 基准", "soc_load.DEFAULT_OCV_MV 都没抽到")
    for nm, v in pc:
        if same(v, ref):
            ok(nm, "= 基准")
        else:
            no(nm, "与 soc_load 不一致\n        基准 %s\n        实际 %s" % (ref, v))
    if FW:
        if same(fw_ocv, ref):
            ok("固件 SOC_OCV_25C_MV", "= PC 基准")
        else:
            no("固件 SOC_OCV_25C_MV", "与 PC 不一致\n        基准 %s\n        实际 %s"
               % (ref, fw_ocv))
        tb = macro_body(fw, "SOC_OCV_TABLE_MV") or ""
        n = tb.count("SOC_OCV_25C_MV")
        if n >= 3:
            ok("固件 OCV_TABLE_MV", "3 个温度点都指向 25C 行（n=%d，占位状态）" % n)
        else:
            no("固件 OCV_TABLE_MV", "温度点引用只有 %d 处，应 ≥3" % n)
    else:
        lines.append("  --   固件跳过（没找到 bms_config.h）")

    # ---------------- [2] R0 / R1 / tau ----------------
    lines.append("")
    lines.append("[2] R0 / R1 / tau 表（11 点）")
    for tag, mine, fwname in (("R0", "R0_CAL", "SOC_R0_25C_MOHM"),
                              ("R1", "R1_CAL", "SOC_R1_25C_MOHM"),
                              ("tau", "TAU_S", "SOC_TAU_25C_S")):
        a = py_list(soh, mine)
        b = py_list(kal, {"R0": "R0", "R1": "R1", "tau": "TAU"}[tag])
        if same(a, b):
            ok("%s  soh_learn ↔ kalman_tune" % tag, "一致")
        else:
            no("%s  soh_learn ↔ kalman_tune" % tag, "%s vs %s" % (a, b))
        if FW:
            f = macro_ints(fw, fwname)
            if same(f, a):
                ok("%s  ↔ 固件 %s" % (tag, fwname), "一致")
            else:
                no("%s  ↔ 固件 %s" % (tag, fwname), "%s vs %s" % (a, f))

    # ---------------- [3] 标称容量 ----------------
    lines.append("")
    lines.append("[3] 标称容量 mAh")
    cap = {"soc_bench.STAIR_CAPACITY": py_num(bench, "STAIR_CAPACITY"),
           "soh_learn.Q_NOM_MAH": py_num(soh, "Q_NOM_MAH"),
           "kalman_tune.CAP": py_num(kal, "CAP")}
    if FW:
        cap["固件 SOC_CAPACITY_NOMINAL_MAH"] = macro_ints(fw, "SOC_CAPACITY_NOMINAL_MAH")
        cap["固件 SOC_CAPACITY_NOMINAL_MAH"] = (
            cap["固件 SOC_CAPACITY_NOMINAL_MAH"] or [None])[0]
    vals = set(v for v in cap.values() if v is not None)
    if len(vals) == 1:
        ok("容量四方一致", "= %g" % vals.pop())
    else:
        no("容量不一致", "  ".join("%s=%s" % kv for kv in cap.items()))

    # ---------------- [4] 静置门限 ----------------
    lines.append("")
    lines.append("[4] 静置电流门限 mA")
    idle = {"soc_bench.IDLE_MA": py_num(bench, "IDLE_MA"),
            "soh_learn.IDLE_MA": py_num(soh, "IDLE_MA"),
            "kalman_tune.IDLE_MA": py_num(kal, "IDLE_MA")}
    if FW:
        m = re.search(r"#\s*define\s+SOC_IDLE_CUR_MA\s+(\d+)", fw)
        idle["固件 SOC_IDLE_CUR_MA"] = float(m.group(1)) if m else None
    vals = set(v for v in idle.values() if v is not None)
    if len(vals) == 1:
        ok("静置门限一致", "= %g mA" % vals.pop())
    else:
        no("静置门限不一致", "  ".join("%s=%s" % kv for kv in idle.items()))

    # ---------------- [5] stair 判据 ----------------
    lines.append("")
    lines.append("[5] stair 判据（soc_bench）")
    for nm in ("STAIR_I_ON_MA", "STAIR_I_OFF_MA", "STAIR_MIN_PULSE_S",
               "STAIR_TAIL_S", "STAIR_EXPECT"):
        v = py_num(bench, nm)
        if v is None:
            no(nm, "抽不到 —— 变量名改了吗？")
        else:
            ok(nm, "= %g" % v)
    if py_num(bench, "STAIR_I_ON_MA") is not None:
        if py_num(bench, "STAIR_I_ON_MA") > py_num(bench, "STAIR_I_OFF_MA"):
            ok("滞回方向", "进入阈值 > 退出阈值（防边缘抖动）")
        else:
            no("滞回方向", "进入阈值应大于退出阈值")

    # ---------------- [6] 方向默认值 ----------------
    lines.append("")
    lines.append("[6] 方向默认值  soc_load.MODES ↔ bms_gui.DIR_DEFAULTS")
    for key, gui_idx in (("discharge", 0), ("charge", 1)):
        blk = re.search(r'"%s":\s*\{(.*?)\n    \}' % key, load, re.S)
        if not blk:
            no("soc_load.MODES['%s']" % key, "抽不到")
            continue
        b = blk.group(1)

        def g(field, pat=r"%s\"?:\s*([-\d.]+)"):
            m = re.search(pat % field, b)
            return float(m.group(1)) if m else None

        vals = {"init_soc": g("init_soc"), "to_soc": g("to_soc"),
                "v_limit": g("v_limit"), "current": g("current")}
        gm = re.findall(r'DIR_CHOICES\[%d\]:\s*\{(.*?)\}' % gui_idx, gui, re.S)
        gb = gm[0] if gm else ""
        # 库里的键叫 v_limit，上位机表单里叫 vlimit —— 映射一下再比
        GKEY = {"v_limit": "vlimit"}
        gv = {}
        for k in vals:
            gk = GKEY.get(k, k)
            m2 = re.search(r'"%s":\s*"([-\d.]+)"' % gk, gb)
            gv[k] = float(m2.group(1)) if m2 else None
        for k in vals:
            if vals[k] is not None and vals[k] == gv[k]:
                ok("MODES['%s'].%s" % (key, k), "= %g" % vals[k])
            else:
                no("MODES['%s'].%s" % (key, k), "库 %s vs 界面 %s" % (vals[k], gv[k]))
    # 充电侧: 脚本中止门限必须低于电源 CV 设定
    m = re.search(r'V_SET_DEFAULT\s*=\s*([-\d.]+)', load)
    vset = float(m.group(1)) if m else None
    m = re.search(r'"charge":\s*\{.*?v_limit"?:\s*([-\d.]+)', load, re.S)
    vlim = float(m.group(1)) if m else None
    if vset and vlim and vlim < vset:
        ok("充电双保护顺序", "脚本门限 %.2f < 电源 CV %.2f（脚本先收手）" % (vlim, vset))
    else:
        no("充电双保护顺序", "vlimit=%s 应低于 vset=%s" % (vlim, vset))

    # ---------------- [7] 计划格式串 ----------------
    lines.append("")
    lines.append("[7] 档位计划格式串")
    w = re.search(r'"format"\s*:\s*[\'"]([\w./-]+)[\'"]', load)
    r = py_str(bench, "PLAN_FORMAT")
    if w and r and w.group(1) == r:
        ok("writer ↔ reader", "= %r" % w.group(1))
    else:
        no("计划格式串不一致", "soc_load 写 %r，soc_bench 认 %r"
           % (w.group(1) if w else None, r))

    # ---------------- [8] 形态检查 ----------------
    lines.append("")
    lines.append("[8] 表的形态")
    if ref:
        if len(ref) == 11:
            ok("OCV 点数", "11")
        else:
            no("OCV 点数", "%d，应为 11" % len(ref))
        if all(ref[i] < ref[i + 1] for i in range(len(ref) - 1)):
            ok("OCV 单调", "严格递增")
        else:
            no("OCV 单调", "有非递增处：%s" % ref)
    for tag, name, src in (("R0", "R0_CAL", soh), ("R1", "R1_CAL", soh),
                           ("tau", "TAU_S", soh)):
        v = py_list(src, name)
        if v and len(v) == 11 and all(x >= 0 for x in v):
            ok("%s 形态" % tag, "11 点且非负")
        else:
            no("%s 形态" % tag, "%r" % v)

    # ---------------- [9] 循环默认值 ----------------
    lines.append("")
    lines.append("[9] 循环默认值  soc_bench.CYCLE_* ↔ soc_load.MODES / --yes 默认")

    def _blk9(key):
        m = re.search(r'"%s":\s*\{(.*?)\n    \}' % key, load, re.S)
        return m.group(1) if m else ""

    def _gn9(blk, field):
        m = re.search(r"%s\"?:\s*([-\d.]+)" % field, blk)
        return float(m.group(1)) if m else None

    def _fs9(text, name):
        s = py_str(text, name)
        try:
            return float(s)
        except (TypeError, ValueError):
            return None

    dis9, chg9 = _blk9("discharge"), _blk9("charge")
    # cycle 的窗口两端刻意对齐 soc_load 的方向默认值 —— "两腿首尾相接"(充电起点 =
    # 放电终点) 全靠这一条。改任何一边都必须两边一起改, 否则第二条腿就对不上, 而且
    # 预览表看不出来(它照样打得出一个"看着挺合理"的区间)。
    pairs9 = [
        ("CYCLE_SOC_HI", py_num(bench, "CYCLE_SOC_HI"),
         _gn9(chg9, "to_soc"), "充电末点"),
        ("CYCLE_SOC_LO", py_num(bench, "CYCLE_SOC_LO"),
         _gn9(dis9, "to_soc"), "放电末点"),
        ("CYCLE_CUR_LOAD", _fs9(bench, "CYCLE_CUR_LOAD"),
         _gn9(dis9, "current"), "放电电流"),
        ("CYCLE_CUR_CHG", _fs9(bench, "CYCLE_CUR_CHG"),
         _gn9(chg9, "current"), "充电电流"),
        ("CYCLE_VLIM_LOAD", py_num(bench, "CYCLE_VLIM_LOAD"),
         _gn9(dis9, "v_limit"), "放电门限"),
        ("CYCLE_VLIM_CHG", py_num(bench, "CYCLE_VLIM_CHG"),
         _gn9(chg9, "v_limit"), "充电门限"),
        ("CYCLE_VSET", py_num(bench, "CYCLE_VSET"),
         py_num(load, "V_SET_DEFAULT"), "电源 CV"),
    ]
    for nm, a9, b9, t9 in pairs9:
        if a9 is not None and b9 is not None and a9 == b9:
            ok("%s (%s)" % (nm, t9), "= %g" % a9)
        else:
            no("%s (%s)" % (nm, t9), "soc_bench %s vs soc_load %s" % (a9, b9))

    # --yes 下的档数与档间静置: 不传参时 soc_load 就用这两个值。cycle 若与它们不同,
    # 预览里的"预计时长"会与真跑不一致(而循环测试一跑就是几小时, 这个账必须对得上)。
    for nm9, pat9, cyc9 in (("每腿档数", r"args\.segments\s*=\s*(\d+)\s*if auto",
                             "CYCLE_SEGMENTS"),
                            ("档间静置", r"args\.rest_s\s*=\s*(\d+)\s*if auto",
                             "CYCLE_REST_S")):
        m9 = re.search(pat9, load)
        a9 = float(m9.group(1)) if m9 else None
        b9 = py_num(bench, cyc9)
        if a9 is not None and b9 is not None and a9 == b9:
            ok("%s (%s ↔ soc_load --yes)" % (cyc9, nm9), "= %g" % b9)
        else:
            no("%s (%s ↔ soc_load --yes)" % (cyc9, nm9),
               "soc_bench %s vs soc_load %s" % (b9, a9))

    if re.search(r"^CYCLE_CAPACITY\s*=\s*STAIR_CAPACITY", bench, re.M):
        ok("CYCLE_CAPACITY 引用 STAIR_CAPACITY", "不另抄一份数值")
    else:
        no("CYCLE_CAPACITY", "应写成 = STAIR_CAPACITY, 别抄数值")

    hi9, lo9 = py_num(bench, "CYCLE_SOC_HI"), py_num(bench, "CYCLE_SOC_LO")
    if hi9 is not None and lo9 is not None and lo9 < hi9:
        ok("CYCLE_SOC_LO < CYCLE_SOC_HI", "%g < %g" % (lo9, hi9))
    else:
        no("CYCLE_SOC_LO < CYCLE_SOC_HI", "%s vs %s" % (lo9, hi9))
    if lo9 is not None and _gn9(chg9, "init_soc") == lo9 and _gn9(dis9, "to_soc") == lo9:
        ok("两腿首尾相接", "充电起点 = 放电终点 = %g%%" % lo9)
    else:
        no("两腿首尾相接", "charge.init_soc=%s discharge.to_soc=%s CYCLE_SOC_LO=%s"
           % (_gn9(chg9, "init_soc"), _gn9(dis9, "to_soc"), lo9))
    cvc9, cvs9 = py_num(bench, "CYCLE_VLIM_CHG"), py_num(bench, "CYCLE_VSET")
    if cvc9 is not None and cvs9 is not None and cvc9 < cvs9:
        ok("cycle 双保护顺序", "充电门限 %.2f < 电源 CV %.2f（脚本先收手）" % (cvc9, cvs9))
    else:
        no("cycle 双保护顺序", "CYCLE_VLIM_CHG=%s 应低于 CYCLE_VSET=%s" % (cvc9, cvs9))

    # ---------------- [10] 跨设备互锁 ----------------
    lines.append("")
    lines.append("[10] 跨设备互锁  soc_load.--peer-port ↔ soc_bench.cycle ↔ bms_gui")

    def _line10(text, pat):
        for i, l in enumerate(text.splitlines()):
            if re.search(pat, l):
                return i
        return None

    # 这一条是真正的**顺序**不变量: 先关伙伴, 再开自己。反过来中间会有一瞬间两台
    # 同时导通, 互锁就白做了 —— 而代码"看着"仍然是对的(两个调用都还在), 只有行号
    # 能把它抓住。
    i_peer10 = _line10(load, r"not peer_off\(peer,")
    i_open10 = _line10(load, r"ser = open_port\(args\.port\)")
    for nm10, got10 in (("soc_load 有 --peer-port 调用", i_peer10),
                        ("soc_load 有 open_port(自己)", i_open10),
                        ("soc_load 定义 peer_off", _line10(load, r"def peer_off\(")),
                        ("soc_load 定义 off_and_verify",
                         _line10(load, r"def off_and_verify\("))):
        if got10 is None:
            no(nm10, "在 soc_load.py 里找不到")
    if i_peer10 is not None and i_open10 is not None and i_peer10 < i_open10:
        ok("互锁顺序", "peer_off L%d 早于 open_port L%d" % (i_peer10 + 1, i_open10 + 1))
    else:
        no("互锁顺序", "peer_off L%s 必须先于 open_port L%s" % (i_peer10, i_open10))

    m10 = re.search(r"PEER_OFF_MAX_A\s*=\s*([\d.]+)", load)
    v10 = float(m10.group(1)) if m10 else None
    if v10 is not None and 0.0 < v10 <= 0.5:
        ok("PEER_OFF_MAX_A 量级", "%g A（断开后残留电流判据）" % v10)
    else:
        no("PEER_OFF_MAX_A 量级", "应在 (0, 0.5] 之间, 实得 %s" % v10)
    m10b = re.search(r"复核阈值\s*([\d.]+)A", load)
    d10 = float(m10b.group(1)) if m10b else None
    if d10 is not None and v10 is not None and d10 == v10:
        ok("docstring 阈值 == 常量", "%g A" % d10)
    else:
        no("docstring 阈值 == 常量", "docstring %s vs PEER_OFF_MAX_A %s" % (d10, v10))

    # 伙伴是谁、该发什么命令, 必须由**本腿反方向**反推出来, 不能写死 "OUTP 0" ——
    # 写死的话哪天换了品牌(--cmd off=...) 互锁就会下发一条设备不认的命令, 而日志
    # 里"看着"还在工作。
    if 'other = "charge" if MODE["key"] == "discharge" else "discharge"' in load and \
            'MODES[other]["cmds"]["off"]' in load:
        ok("伙伴命令按反方向反推", 'MODES[other]["cmds"]["off"]')
    else:
        no("伙伴命令按反方向反推", '应引用 MODES[other]["cmds"]["off"], 别写死命令')

    # 两台设备不可能接在同一个串口 —— 两侧各有一道, 少一道就少了半张安全网
    got10 = [n for n, t in (("soc_load", load), ("soc_bench", bench))
             if "是同一个口" in t]
    if len(got10) == 2:
        ok("同口校验两侧都在", "soc_load + soc_bench")
    else:
        no("同口校验两侧都在", "只找到 %r" % got10)

    # cycle 的交叉配对: 充电腿必须关**负载**, 放电腿必须关**电源**。配反了照样能跑,
    # 但互锁关的会是本腿自己那台 —— 比没有互锁更危险。
    # ⚠ 别写成"两串在文件里存不存在"就完事: 那样把两条腿写反了照样全 OK(证伪时真踩到
    #   过)。必须按 leg_cmd 的 else 分界切成两半, 要求**本腿自己的那一半里**只有本腿
    #   该关的那台。
    m10f = re.search(r"def leg_cmd\(.*?\n(?=\ndef )", bench, re.S)
    body10 = m10f.group(0) if m10f else ""
    i_else10 = body10.find('    else:\n        cmd += ["--current", args.current_load')
    if not body10 or i_else10 < 0:
        no("cycle 配对: 切得开 leg_cmd 的两条分支", "找不到 leg_cmd 或 else 分界")
    else:
        for nm10, seg10, want10, ban10 in (
                ("充电腿 → 关负载", body10[:i_else10], "args.port_load", "args.port_psu"),
                ("放电腿 → 关电源", body10[i_else10:], "args.port_psu", "args.port_load")):
            p_want = 'cmd += ["--peer-port", %s]' % want10
            p_ban = 'cmd += ["--peer-port", %s]' % ban10
            if p_want in seg10 and p_ban not in seg10:
                ok("cycle %s" % nm10, "在本腿分支内, 只关该关的那台")
            else:
                no("cycle %s" % nm10,
                   "本腿分支内应当只有 %s (该关的 %s / 不该出现的 %s)"
                   % (want10, p_want in seg10, p_ban in seg10))

    if "--no-interlock" in bench:
        ok("cycle --no-interlock", "缺串口时的唯一放行方式")
    else:
        no("cycle --no-interlock", "没有它就只能强制两个口都给全")

    gui10 = rd(S_GUI)
    if ('("peer_port", "另一台串口"' in gui10
            and '"--peer-port", self.port_of("peer_port")' in gui10
            and '"--port-psu", self.port_of("peer_port")' in gui10):
        ok("bms_gui ③ 页", "「另一台串口」-> 单腿 --peer-port / 循环 --port-psu")
    else:
        no("bms_gui ③ 页", "控载页缺「另一台串口」或没拼进命令行")

    # ---------------- [11] 三模式 + 循环执行器 ----------------
    lines.append("")
    lines.append("[11] 三模式  bms_gui.DIR_CHOICES / CYC_DEFAULTS ↔ soc_bench.CYCLE_* / LEG_RUNNER")

    # 模式的**顺序**是不变量: DIR_DEFAULTS 与 [6] 都按下标 0/1 取值, 顺序一动那边就
    # 静默错位(照样能跑, 只是把充电的默认值填给了拉载)。所以这里连顺序一起钉死。
    want11 = ["拉载（电子负载）", "充电（可编程电源）", "循环（多轮充放）"]
    # 三个名字是一次元组赋值绑定的, 所以整段抽; 单按 NAME = "..." 抽不到(踩过)。
    # 顺序另有一条字面断言钉住 —— DIR_DEFAULTS 与 [6] 都按下标 0/1 取值,
    # DIR_CHOICES 的顺序一动, 那边就会静默错位(照样能跑, 只是把充电的默认值填给了拉载)。
    m11o = re.search(r"MODE_LOAD,\s*MODE_CHARGE,\s*MODE_CYCLE\s*=\s*\((.*?)\)",
                     gui, re.S)
    got11 = re.findall(r'"([^"]+)"', m11o.group(1)) if m11o else []
    if same(got11, want11) and \
            "DIR_CHOICES = [MODE_LOAD, MODE_CHARGE, MODE_CYCLE]" in gui:
        ok("三模式", "拉载 / 充电 / 循环（下标 0/1/2 固定）")
    else:
        no("三模式", "应依次是 %s，实得 %s（或 DIR_CHOICES 顺序被改）" % (want11, got11))

    m11 = re.search(r"CYC_DEFAULTS\s*=\s*\{(.*?)\n\}", gui, re.S)
    cb11 = m11.group(1) if m11 else ""

    def _cd11(key):
        m = re.search(r'"%s":\s*"([^"]*)"' % key, cb11)
        return m.group(1) if m else None

    # 循环那套默认值是 soc_load 默认值的第 N 份副本, 而界面是它唯一的入口:
    # 对不上就会"界面报 3 轮、脚本跑 5 轮"这种谁都没改错的怪事。
    for gkey, cname in (("cycles", "CYCLE_N"), ("soc_lo", "CYCLE_SOC_LO"),
                        ("soc_hi", "CYCLE_SOC_HI"), ("start_soc", "CYCLE_START_SOC"),
                        ("vlim_load", "CYCLE_VLIM_LOAD"), ("vlim_chg", "CYCLE_VLIM_CHG"),
                        ("vset_cyc", "CYCLE_VSET")):
        gv11, cv11 = _cd11(gkey), py_num(bench, cname)
        try:
            gv11 = float(gv11)
        except (TypeError, ValueError):
            gv11 = None
        if gv11 is not None and cv11 is not None and gv11 == cv11:
            ok("CYC_DEFAULTS[%s]" % gkey, "= soc_bench.%s = %g" % (cname, cv11))
        else:
            no("CYC_DEFAULTS[%s]" % gkey, "界面 %s vs soc_bench.%s %s"
               % (_cd11(gkey), cname, cv11))
    for gkey, cname in (("cur_load", "CYCLE_CUR_LOAD"), ("cur_chg", "CYCLE_CUR_CHG"),
                        ("prefix", "CYCLE_PREFIX")):
        gv11, cv11 = _cd11(gkey), py_str(bench, cname)
        if gv11 is not None and gv11 == cv11:
            ok("CYC_DEFAULTS[%s]" % gkey, "= soc_bench.%s = %r" % (cname, cv11))
        else:
            no("CYC_DEFAULTS[%s]" % gkey, "界面 %r vs soc_bench.%s %r" % (gv11, cname, cv11))

    # 循环要两台设备各占一个口: 两个 --port-* 必须分别来自两个字段, 不能同源
    if ('"--port-load", self.port_of("port")' in gui
            and '"--port-psu", self.port_of("peer_port")' in gui):
        ok("循环两个口", "port -> --port-load / peer_port -> --port-psu")
    else:
        no("循环两个口", "两个 --port-* 必须分别由 port / peer_port 两个字段来")
    # 看的是**带引号的形式**: 说明文字里提醒一句 --no-interlock 是好事, 真正要防的是
    # 把它拼进 argv 变成界面上的一条后门。文字里出现过 ≠ 界面上能用。
    if '"--no-interlock"' not in gui:
        ok("界面不给 --no-interlock", "互锁在界面上是强制的（只在说明文字里提了一句）")
    else:
        no("界面不给 --no-interlock", "界面上不该把它拼进命令行")

    # "一腿怎么跑"必须可替换, 且默认仍是真子进程 —— 上位机是 exe, sys.executable 指向
    # 它自己, 另起进程只会再弹一个界面; 所以那边换成进程内调 soc_load。
    if ("LEG_RUNNER = None" in bench and "def run_leg(cmd):" in bench
            and "rc = run_leg(cmd)" in bench
            and "rc = subprocess.call(cmd)" not in bench):
        ok("腿执行器可替换", "cmd_cycle 走 run_leg()，默认 subprocess.call")
    else:
        no("腿执行器可替换", "cmd_cycle 必须改回 run_leg(cmd)，别再直接 subprocess.call")
    if ("sbm.LEG_RUNNER = self._leg_inproc" in gui
            and "def _leg_inproc(self, cmd):" in gui
            and 'ScriptRunner.call("soc_load", argv, self.log)' in gui):
        ok("上位机进程内跑腿", "LEG_RUNNER -> ScriptRunner.call(soc_load)")
    else:
        no("上位机进程内跑腿", "循环必须换成进程内调 soc_load（exe 里起不了 python）")

    # 源和载可能是两个牌子: 方言要按腿分传, 不能一锅端
    if ("--cmd-load" in bench and "--cmd-chg" in bench
            and 'getattr(args, "cmd_charge", None) if direction == "charge"' in bench):
        ok("每腿方言分开传", "--cmd-load / --cmd-chg，leg_cmd 按 direction 选")
    else:
        no("每腿方言分开传", "leg_cmd 应按 direction 在 --cmd-load / --cmd-chg 之间选")

    # ---------------- [12] 协议版本号 / NVM 载荷长度 ----------------
    lines.append("")
    lines.append("[12] 协议版本  bms_config.h ↔ bms_tune.h ↔ bms_tune_proto.py ↔ bms_nvm.h")

    # BMS_TUNE_PROTO_VER 在 bms_config.h 与 bms_tune.h 里**各有一处 #ifndef 兜底**。
    # 带 #ifndef 的两个定义, 谁的 #define 先被预处理到谁生效, 而生效顺序由 include
    # 顺序决定 —— bms_tune.h 第 71 行自己就 include 了 bms_config.h, 所以后者总是先
    # 定义、前者那个值被挡掉。两值不同时协议版本随编译单元静默漂移(板子上报 1、
    # 仿真里报 2), 而长度/语义类测试全都不看版本号, 照样全绿。所以把三处钉成同一个数。
    def _rd_opt(p):
        return rd(p) if os.path.isfile(p) else None

    def _first(text, name):
        v = macro_ints(text, name) if text else None
        return v[0] if v else None

    tune_h12 = _rd_opt(os.path.join(REPO, "算法库", "bms_tune.h"))
    nvm_h12 = _rd_opt(os.path.join(REPO, "算法库", "bms_nvm.h"))
    proto12 = _rd_opt(os.path.join(ROOT, "bms_tune_proto.py"))

    fw_pv = _first(fw, "BMS_TUNE_PROTO_VER")
    th_pv = _first(tune_h12, "BMS_TUNE_PROTO_VER")
    pc_pv = py_num(proto12, "PROTO_VER") if proto12 else None
    fw_nv = _first(fw, "BMS_NVM_VER")
    fw_pl = _first(nvm_h12, "BMS_NVM_PAYLOAD_LEN")

    pc_nv = None
    if proto12:
        m12 = re.search(r"DEFAULT_DIMS\s*=\s*\{(.*?)\}", proto12, re.S)
        if m12:
            mm12 = re.search(r"['\"]nvm_ver['\"]\s*:\s*(\d+)", m12.group(1))
            pc_nv = float(mm12.group(1)) if mm12 else None

    # 脚本侧 SOH_BLOB_BYTES = struct.calcsize(_SOH_FMT), 源码里是表达式。剖析一遍格式串
    # 自己算 —— 只为拿 struct.calcsize, 不执行脚本本体(本脚本刻意零依赖)。
    pc_blob = None
    if proto12:
        try:
            import struct as _st12
            # _SOH_FMT_V2 是单行; _SOH_FMT 是**跨行**的括号表达式, 所以这里要
            # re.S + 非贪婪到第一个 ")" —— 按单行抓会得到括号没闭合的片段,
            # eval 抛 SyntaxError 被吞掉, 这一整项对拍就静默消失了。
            m_v2 = re.search(r"^_SOH_FMT_V2\s*=\s*(.+)$", proto12, re.M)
            m_fm = re.search(r"^_SOH_FMT\s*=\s*\((.*?)\)", proto12, re.M | re.S)
            fmt_v2 = eval(m_v2.group(1), {"__builtins__": {}}, {}) if m_v2 else None
            if fmt_v2 is not None and m_fm:
                fmt = eval("(" + m_fm.group(1) + ")", {"__builtins__": {}},
                           {"_SOH_FMT_V2": fmt_v2})
                pc_blob = _st12.calcsize(fmt)
        except Exception as e12:                 # noqa: BLE001
            lines.append("    (SOH 块字节数: 格式串解析失败: %s)" % e12)
            pc_blob = None

    lines.append("    固件 proto_ver: bms_config.h=%s / bms_tune.h=%s; 脚本 PROTO_VER=%s"
                 % (fw_pv, th_pv, pc_pv))
    lines.append("    固件 nvm_ver=%s, 脚本 DEFAULT_DIMS['nvm_ver']=%s; "
                 "固件载荷=%s, 脚本 SOH_BLOB_BYTES=%s"
                 % (fw_nv, pc_nv, fw_pl, pc_blob))

    if fw_pv is None:
        no("proto_ver 双处兜底", "bms_config.h 里没抽到 BMS_TUNE_PROTO_VER")
    elif th_pv is None and tune_h12 is None:
        skip("proto_ver 双处兜底",
             "bms_tune.h 不在这里（产物只随包带 bms_config.h）; "
             "bms_config.h=%s 已单独与脚本对上" % fw_pv)
    elif th_pv is None:
        no("proto_ver 双处兜底", "bms_tune.h 在, 但没抽到 BMS_TUNE_PROTO_VER")
    elif fw_pv != th_pv:
        no("proto_ver 双处兜底",
           "bms_config.h=%s 但 bms_tune.h=%s —— 两个 #ifndef 兜底值不同, "
           "协议版本会随 include 顺序漂移" % (fw_pv, th_pv))
    else:
        ok("proto_ver 双处兜底", "bms_config.h == bms_tune.h == %s" % fw_pv)

    if fw_pv is not None and pc_pv is not None:
        if fw_pv == pc_pv:
            ok("proto_ver 固件↔脚本", "= %s" % fw_pv)
        else:
            no("proto_ver 固件↔脚本",
               "固件 %s vs bms_tune_proto.PROTO_VER %s（两端握不上手）" % (fw_pv, pc_pv))
    if fw_nv is not None and pc_nv is not None:
        if fw_nv == pc_nv:
            ok("nvm_ver 固件↔脚本", "= %s" % fw_nv)
        else:
            no("nvm_ver 固件↔脚本", "固件 %s vs 脚本 %s" % (fw_nv, pc_nv))
    if pc_blob is None:
        # 脚本侧的字节数算不出来是**真缺陷**（格式串被人改坏了）—— 必须 FAIL
        no("SOH 块字节数",
           "没算出脚本侧 SOH_BLOB_BYTES（bms_tune_proto.py %s）"
           % ("在" if proto12 else "不在"))
    elif fw_pl is None and nvm_h12 is None:
        skip("SOH 块字节数",
             "bms_nvm.h 不在这里（产物只随包带 bms_config.h）; 脚本侧 = %s B"
             % pc_blob)
    elif fw_pl is None:
        no("SOH 块字节数", "bms_nvm.h 在, 但没抽到 BMS_NVM_PAYLOAD_LEN")
    elif fw_pl == pc_blob:
        ok("SOH 块字节数", "bms_nvm.h ↔ bms_tune_proto = %s B" % fw_pl)
    else:
        no("SOH 块字节数", "固件 BMS_NVM_PAYLOAD_LEN=%s vs 脚本 SOH_BLOB_BYTES=%s"
           % (fw_pl, pc_blob))

    # ---------------- 汇总 ----------------
    lines.append("")
    lines.append("=" * 72)
    if bad:
        lines.append("问题汇总 (%d):" % len(bad))
        for b in bad:
            lines.append("  !! " + b)
    else:
        lines.append("问题汇总: 无 —— 固件与四个 PC 副本、上位机方向表全部一致")
    lines.append("=" * 72)
    txt = "\n".join(lines)
    if not quiet:
        print(txt)
    else:
        print("问题数 = %d" % len(bad))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
