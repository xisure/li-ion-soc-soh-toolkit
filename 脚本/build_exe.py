#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 bms_gui.py 打包成双击就能跑的 exe（PyInstaller）。

用法
----
    python build_exe.py            # 打包，产物在 <仓库根>/上位机/
    python build_exe.py --onefile  # 改打成单个 exe（见下面"为什么默认不是单文件"）

打包形态（默认 --onedir）
------------------------
    <仓库根>/上位机/
        锂电池上位机.exe        双击即用，不用敲命令
        _internal/             解释器 + numpy + matplotlib + tcl/tk（PyInstaller 生成）
        tools/                 脚本本体（exe 优先用这份，用户可随时改）
        读我.txt               怎么用 / 各页签干什么

**为什么默认不是单文件**：`--onefile` 每次启动都要把 ~130 MB 解压到 %TEMP% 再
被 Windows Defender 扫一遍，实测冷启动 18 s、热启动也要 12.5 s。`--onedir` 起一次
就完事，实测 2.5~3 s（刚构建完首次启动要读盘，会顶到 3 s 出头）。这个工具是要反复点开的，
所以默认给快的那个。

`--onefile` 下包里会额外塞一份 tools/ 作兜底（只拷走 exe 也能跑），
只要 exe 旁边有 tools/ 目录就优先用旁边那份。两种形态都不挡改脚本。

不管哪种形态，`tools/` 都是可编辑的：改了脚本下次启动就生效，不用重新打包。

注意：分析类脚本（soc_bench / soh_learn / ...）是在同一个进程里 import 进来
跑的，不是子进程，所以 matplotlib / numpy / pyserial 这些必须在打包时就
--hidden-import 进去 —— bms_gui 自己没 import 它们，PyInstaller 静态分析看不到。
"""

import os
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))          # = tools 源码目录
ROOT = os.path.dirname(HERE)
# 每次都用**全新的空**临时目录做中间产物：PyInstaller 遇到旧产物会先删再建，
# 在带"批量删除拦截"的环境里那一步会被挡下来。空目录起步就完全不需要删东西。
# 代价是这些中间目录不会自动清掉（占 ~200 MB），所以在最后把路径打出来。
RUN = tempfile.mkdtemp(prefix="bms_build_")
STAGE = os.path.join(RUN, "stage")
DIST = os.path.join(RUN, "dist")
WORK = os.path.join(RUN, "work")
OUT = os.path.join(ROOT, "上位机")
ONEFILE = "--onefile" in sys.argv[1:]     # 默认 --onedir（启动快得多，见文件头）

APP_ASCII = "bms_gui"                     # PyInstaller 的 --name 用纯 ASCII，稳
APP_NAME = "锂电池上位机"                   # 最后重命名成这个
ICON = os.path.join(HERE, "bms_icon.ico")  # 有就用，没有就用 PyInstaller 默认图标

# 要放进包里的脚本（bms_gui.py 是入口，不重复放）
TOOLS_PY = ["bms_tune_proto.py", "soc_bench.py", "soc_load.py", "soc_record.py",
            "soh_learn.py", "soh_mcu_sim.py"]
TOOLS_OTHER = ["README.md", "requirements.txt"]
# test/check_consts.py 是纯标准库、无依赖的"多副本常量对拍"，tools/README §5 让用户跑它，
# 所以必须一起打进包（否则文档指着一个不存在的文件）。
TOOLS_SUB = ["test/kalman_tune.py", "test/check_consts.py"]

# 固件头文件：上位机 ⑥EKF 页的「填固件现值」按钮要读它（EKF 三参数的真源）。
# 放在 tools/ 根 —— kalman_tune.py 的 _find_cfg() 从 tools/test/ 上溯一层就命中，
# 于是 exe 里填的也是固件当前值，而不是脚本里另抄的一份。
FW_CFG = next((p for p in (os.path.join(ROOT, "算法库", "bms_config.h"),
                           os.path.join(ROOT, "firmware", "bms_config.h"))
               if os.path.isfile(p)), None)

HIDDEN = [
    "serial", "serial.tools", "serial.tools.list_ports",
    "numpy", "numpy.lib.stride_tricks",
    "matplotlib", "matplotlib.pyplot",
    # 图表面板（ImagePane）：把脚本落盘的 PNG 缩放后嵌进窗口
    "PIL", "PIL.Image", "PIL.ImageTk",
]

EXCLUDE = ["PyInstaller", "pydoc_data", "unittest", "test", "tkinter.test",
           "numpy.f2py", "numpy.distutils", "setuptools", "pkg_resources"]

README = """锂电池 SOC/SOH 上位机
============================================================

怎么用
------
双击「锂电池上位机.exe」，六个页签从左到右是标定的实际流程：

  ① 采集监视     串口收 MCU 的数据帧 -> 大字号实时值 + 落 CSV
                 跑任何实验之前，都先在这里点「开始采集」
  ② 在线调参     bms_tune 协议：标定表 / R0 / 容量 / SOC 运行态 / EKF 参数 /
                 SOH / 掉电保持
                 每个参数都有输入框，CRC 自动算，不用手动对齐字节
  ③ 控载         通过 RS232 控制程控电子负载（SCPI），自动跑十档「放电 -> 静置」
  ④ 分析         soc_bench 的 fit（单脉冲 RC 拟合）/ stair（多档 RC + OCV-SOC）
  ⑤ SOH 验证     soh_learn（离线定算法）/ soh_mcu_sim（逐帧复刻，两者应逐项吻合）
  ⑥ EKF 调参     在 PC 上回放采集数据、评估一组 EKF 参数 (q_soc, q_vrc, R)
                 「只回放这一组」填 3 个数 = 单组仿真；留空 = 网格搜索
                 「填固件现值」直接读固件的 bms_config.h，不用手抄
                 「从板子读现值」/「写回板子」借 ② 页那条串口读写板上的 8 个参数

④⑤⑥ 三页跑完，脚本出的图会**直接显示在页面里**（上半是图、下半是日志）：
  · ◀ ▶ 换图，「适应窗口」整张缩进可见，「1:1」+ Ctrl 滚轮看细节
  · 双击图弹独立大窗，Esc 关；「另存为」拷一份走

典型顺序
--------
  ② 在线调参 里「① 握手 / 探活」->「② 读板子配置」，确认协议版本和固件对得上
  ① 采集监视 开采集  ->  ③ 控载 跑十档放电
  ④ 分析 stair 出 OCV 表  ->  ② 在线调参 把标定表写回板子
  ⑤ SOH 验证 确认学习算法在 PC 上和 MCU 上一致
  ⑥ EKF 调参 定完参再进固件

目录说明
--------
  锂电池上位机.exe   双击它就行（其余文件都是它要用的，别单独挪走）
  _internal/        解释器 + numpy + matplotlib + tcl/tk，程序自己找得到
  tools/            命令行脚本本体。exe 优先用这一份，改了立刻生效（不用重新打包）。
                    同样可以脱离界面直接用，用法见 tools/README.md
                    tools/bms_config.h 是固件参数头文件的副本，⑥EKF 页读它
  读我.txt           本文件

整个文件夹可以整体拷到别的机器上（不用装 Python）。要挪位置就整体挪。

几条要紧的
----------
· ② 页的「单条命令」是按用途分组、能用中文搜的：左边点一条，右边就出它要填
  什么、填在哪个框里。命令码和协议名（0x03 / WR_CAL_POINT）收在右侧小字里，
  真是要去对协议手册时才用得上。
· 真实充/放电前，界面上会弹确认框列接线确认 —— 确认框里那几条每条都别跳过。
· 放电中途要停，用③页的「紧急停止」（等价命令行按 Ctrl+C，脚本会先断开负载）。
· 改标定表先「读整包」再改再「写回整包」。别从零拼一套新表：现在只有 25 °C
  的实测数据，把 5/45 °C 写成 0 会把低温插值彻底搞崩。
· 分析产物（png / csv）落在你选的数据 CSV 旁边，不在这个目录里。
  图当场就能在④⑤⑥页看到，不用去文件夹里找；要发给别人用「另存为」拷一份。
"""


def log(s):
    sys.stdout.write(s + "\n")
    sys.stdout.flush()


def sync_tree(src, dst, keep_extra=(), skip=(), prune=False):
    """把 src 同步到 dst：同名文件覆盖。

    默认 **只覆盖不删除** —— 在带"批量删除拦截"的环境里，一次删掉超过几十个
    文件会被挡下来，打包工具不该因此挂掉。`prune=True` 才把 dst 里多出来的
    文件逐个删掉（只在暂存目录这种"本来就是空的"地方用得上）。
    `keep_extra` 是 dst 里要保留的顶层名字（工具自己放的 tools/、读我.txt）；
    `skip` 是 src 里不要拷过去的相对路径。
    """
    os.makedirs(dst, exist_ok=True)
    skip = set(os.path.normpath(x) for x in skip)
    keep = set()
    for dirpath, _dn, fns in os.walk(src):
        for fn in fns:
            rel = os.path.normpath(os.path.relpath(os.path.join(dirpath, fn), src))
            if rel in skip:
                continue
            keep.add(rel)
            out = os.path.join(dst, rel)
            os.makedirs(os.path.dirname(out), exist_ok=True)
            shutil.copy2(os.path.join(dirpath, fn), out)
    if not prune:
        return dst
    extra = set(os.path.normpath(x) for x in keep_extra)
    for dirpath, _dn, fns in os.walk(dst):
        for fn in fns:
            full = os.path.join(dirpath, fn)
            rel = os.path.normpath(os.path.relpath(full, dst))
            if rel in keep or rel.split(os.sep)[0] in extra:
                continue
            os.remove(full)
    return dst


def stage_tools():
    """只把要用的文件挑进暂存目录，别把 png / csv / __pycache__ 也塞进包里。

    返回的是**暂存根目录**（里面有个 `tools/`）。
    """
    dst = os.path.join(STAGE, "tools")
    names = TOOLS_PY + TOOLS_OTHER + TOOLS_SUB + ["bms_gui.py"]
    missing = [n for n in names
               if not os.path.isfile(os.path.join(HERE, n.replace("/", os.sep)))]
    if missing:
        raise SystemExit("缺文件: %s" % ", ".join(missing))
    os.makedirs(dst, exist_ok=True)
    keep = set()
    for name in names:
        rel = name.replace("/", os.sep)
        out = os.path.join(dst, rel)
        os.makedirs(os.path.dirname(out), exist_ok=True)
        shutil.copy2(os.path.join(HERE, rel), out)
        keep.add(os.path.normpath(rel))
    if FW_CFG:
        shutil.copy2(FW_CFG, os.path.join(dst, "bms_config.h"))
        keep.add("bms_config.h")
    else:
        print("[!] 没找到算法库/bms_config.h："
              "exe 里「填固件现值」只能给内置兜底值 0.001 / 0.5 / 10")
    return STAGE


def build(stage):
    tools = os.path.join(stage, "tools")
    cmd = [sys.executable, "-m", "PyInstaller",
           "--noconfirm",
           "--onefile" if ONEFILE else "--onedir",
           "--noconsole",
           "--name", APP_ASCII,
           "--distpath", DIST, "--workpath", WORK,
           "--specpath", WORK,
           "--paths", tools,
           "--add-data", tools + os.pathsep + "tools"]
    for h in HIDDEN:
        cmd += ["--hidden-import", h]
    for e in EXCLUDE:
        cmd += ["--exclude-module", e]
    if os.path.isfile(ICON):
        cmd += ["--icon", ICON]
    cmd.append(os.path.join(HERE, "bms_gui.py"))
    log("[i] " + " ".join(cmd))
    rc = subprocess.call(cmd)
    if rc != 0:
        raise SystemExit("[X] PyInstaller 失败，返回码 %d" % rc)
    if ONEFILE:
        exe = os.path.join(DIST, APP_ASCII + ".exe")
    else:
        exe = os.path.join(DIST, APP_ASCII, APP_ASCII + ".exe")
    if not os.path.isfile(exe):
        raise SystemExit("[X] 没找到产物 %s" % exe)
    return exe


def assemble(exe, stage):
    """把产物摊到 上位机/ 下：exe 更名为中文名，tools/ 放旁边。

    全程**只覆盖、不删除**：`bms_gui.exe` 直接跳过不拷，中文名那份直接拷过去
    覆盖旧的，`keep_extra` 保住 tools/ 和读我.txt。这样任何一步都不需要删文件。
    """
    os.makedirs(OUT, exist_ok=True)
    dst_exe = os.path.join(OUT, APP_NAME + ".exe")
    if ONEFILE:
        shutil.copy2(exe, dst_exe)
    else:
        sync_tree(os.path.dirname(exe), OUT,
                  keep_extra=(APP_NAME + ".exe", "tools", "读我.txt"),
                  skip=(APP_ASCII + ".exe",))
        shutil.copy2(exe, dst_exe)
    sync_tree(os.path.join(stage, "tools"), os.path.join(OUT, "tools"))
    with open(os.path.join(OUT, "读我.txt"), "w", encoding="utf-8") as f:
        f.write(README)
    n = sum(len(fns) for _d, _s, fns in os.walk(os.path.join(OUT, "tools")))
    total = sum(len(fns) for _d, _s, fns in os.walk(OUT))
    return dst_exe, n, total


def main():
    log("[i] 形态：%s%s" % ("单文件 exe（启动慢）" if ONEFILE else "目录（启动快）",
                            "" if os.path.isfile(ICON) else "，没找到图标"))
    log("[i] 暂存 tools/ ...")
    stage = stage_tools()
    log("[i] 打包中（matplotlib + numpy 打进去，要等 2~4 分钟）...")
    exe = build(stage)
    dst_exe, n_tools, n_all = assemble(exe, stage)
    size = os.path.getsize(dst_exe) / 1024.0 / 1024.0
    tot = sum(os.path.getsize(os.path.join(d, f))
              for d, _s, fns in os.walk(OUT) for f in fns)
    log("")
    log("=" * 62)
    log("完成：%s  (%.1f MB)" % (dst_exe, size))
    log("       %s  (%d 个文件)" % (os.path.join(OUT, "tools"), n_tools))
    log("       整个 上位机/ 共 %d 个文件、%.1f MB" % (n_all, tot / 1048576.0))
    log("")
    log("中间目录（可以手动删掉，占几百 MB）：")
    log("  " + RUN)
    log("=" * 62)


if __name__ == "__main__":
    main()
