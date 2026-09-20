#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""bms_gui.py — 锂电池 SOC/SOH 标定上位机（图形界面）

把 `tools/` 下那套命令行脚本包成一个点点点的窗口程序：

    采集监视   串口收 MCU 数据帧 -> 实时显示 + 落 CSV
    在线调参   bms_tune 协议：标定表 / R0 / 容量 / SOH / 掉电保持 逐项下发，
               界面从协议命令表自动生成，不用手算字节和 CRC
    控载       程控源/载十档充/放（SCPI，换品牌只改「命令方言覆盖」那几个字）
    分析       soc_bench fit / stair
    SOH 验证   soh_learn / soh_mcu_sim
    EKF 调参   kalman_tune

设计取舍
--------
分析类脚本**不重写**：直接 `import` 原脚本、把参数拼成 argv 调它的 `main()`，
stdout 重定向到界面日志。命令行版和界面版永远同一套逻辑，不会两边漂移。
只有「采集」和「在线调参」是新写的 —— 它们本来就是交互式的，套命令行反而难用。

协议部分全部在 `bms_tune_proto.py`（含 CRC 自检与真实固件对拍过的帧实现）。
"""

import importlib
import math
import os
import queue
import shutil
import sys
import threading
import time
import traceback
import datetime

import tkinter as tk
from tkinter import ttk, filedialog, messagebox, scrolledtext

# --------------------------------------------------------------------------
# 路径：源码运行 = 本文件所在目录；打包成 exe = exe 旁边的 tools/（优先）
#       或 PyInstaller 解包目录里的 tools/
# --------------------------------------------------------------------------
FROZEN = getattr(sys, "frozen", False)


def tools_dir():
    if FROZEN:
        for cand in (os.path.join(os.path.dirname(sys.executable), "tools"),
                     os.path.join(getattr(sys, "_MEIPASS", ""), "tools")):
            if os.path.isdir(cand):
                return cand
        return os.path.dirname(sys.executable)
    return os.path.dirname(os.path.abspath(__file__))


TOOLS = tools_dir()
if TOOLS not in sys.path:
    sys.path.insert(0, TOOLS)

import bms_tune_proto as P                                  # noqa: E402

APP_TITLE = "锂电池 SOC/SOH 上位机"
FONT_UI = ("Microsoft YaHei UI", 9)
FONT_BOLD = ("Microsoft YaHei UI", 9, "bold")
FONT_H1 = ("Microsoft YaHei UI", 11, "bold")
FONT_MONO = ("Consolas", 9)
FONT_BIG = ("Consolas", 20, "bold")


# ==========================================================================
# 小工具
# ==========================================================================

# ==========================================================================
# 说明文字：一行摘要 + 「?」弹窗看全文
# ==========================================================================
#
# 为什么这么做：说明段落只要一常驻就要占掉好几行（③ 页原来那段约 215 字，
# 折成 3 行），而看一眼的人一次只要一句话；细节留到真要用的时候点开看。
# 所以长段一律收进 detail，界面上只留一行摘要。
NAV_FG = "#1a6ea8"                    # 「本页做什么 / 下一步」那行的颜色

NAV_TEXT = {
    "record": "本页：接板子，把 MCU 每 5 Hz 上报的遥测行录成 CSV。"
              "下一步：③ 控载（开跑前先在这里按「开始采集」）",
    "tune": "本页：连板子直接读 / 写固件参数（握手 → 单条命令 → 标定表 / R0 / SOH / 容量）。"
            "下一步：④ 分析出表后，用「标定表 → 写回整包」落进板子",
    "load": "本页：控源 / 载自动跑充放档位，或按「循环」跑多轮。下一步：④ 分析（stair）",
    "analysis": "本页：把 ③ 跑出来的 CSV 拟合成 OCV / R0 / R1 / τ 表。"
                "下一步：② 在线调参 →「标定表 → 写回整包」",
    "soh": "本页：拿实测数据在 PC 上复刻 SOH 学习，确认判据与参数。"
           "下一步：满意后把结果写回板子（走 ② 页）",
    "ekf": "本页：PC 回放采集数据，扫 EKF 参数找最优。"
           "下一步：「写回板子」（走 ② 页那条串口）",
}

HELP = {
    "cmd": u"""左边按用途分组，点一条命令，右边就出现它要填的东西。
参数直接填工程值（电压填 mV、温度填 °C），换算不用管 —— 字节和 CRC 由界面算。

橙色的命令需要固件里先打开对应编译开关，没打开的话板子会回错误。

勾上「显示收发的原始字节」后，下面会多打一行 hex —— 对着协议手册核帧的时候用。""",

    "cal": u"""用法：① 读整包 → ② 改要改的点 → ③ 写回整包。

· 上面那张「出厂基准表（可直接改数值）」是能编辑的；
  下面那张「活跃表」是板子按「当前温度」插值出来的只读结果。
· 别从零拼一套新表：只有 25 °C 数据时把 5 / 45 °C 写成 0，会把低温插值彻底搞崩。
· 改了 5 °C 的表而当前温度是 25 °C 时「活跃表」不变 —— 这是温度解耦，不是没生效。
  想看改动效果，用旁边的「温度」子页先把当前温度切过去。

「恢复出厂」会把整块表打回固件里的默认值，改乱了可以用它兜底。""",

    "r0": u"""这里填的是绝对值（不是增量），库内部会换算成「相对当前温度基准的增量」。

所以先把温度切到目标温度（或用「容量 / 温度」子页的「读当前温度」确认）再写，
否则增量会算在错误的基准上 —— 看起来写成功了，实际偏了。

「读第 N 点 / 写第 N 点」只动一个 SOC 点，N 取 0~10（对应 0%~100%，每 10% 一格）。""",

    "stair": u"""容量 / 起始 SOC / 期望档数留空时，按三级自动定：
    命令行参数  >  档位计划文件  >  标称容量或电压反查

档位计划文件是 ③ 控载页跑完落的那份 *_plan.json。
把它改名成 <数据CSV前缀>_plan.json 放在同目录，就会被自动认到
（比如数据叫 stair1.csv，计划就命名 stair1_plan.json）。

「期望档数」填了会跟 CSV 里实际数出来的档数对一下，不符直接报错 ——
这是防止拿错文件最省事的一招。""",

    "ekf": u"""在 PC 上回放采集数据，看一组 EKF 参数跑出来是什么样 —— 不用烧板就能试参数。

· 「只回放这一组」填 3 个数（q_soc  q_vrc  R）就是单组仿真，
  配「跑完出轨迹图」看图；留空则按网格逐组跑，以 stair 静置末参考点选最优。
· 网格格式 qs档|qv档|r档（逗号分隔）；留空 = 脚本默认 4x3x3 共 36 组。
· 「只取前 N 帧」先填 30000 试 —— 全量跑一次比较慢。

与板子交互走 ② 页那条串口（协议 0x18 / 0x19）：
· 「读」是整组 8 个参数。
· 「写」只动这里的 q_soc / q_vrc / R 三个，其余 5 个
  （p0_* / 野值门限 / 两个下限）原样带回板子。
· q_soc / q_vrc / R 写完立刻生效；p0_* 要再发一次「复位 EKF」才生效。""",

    "load": u"""通过 RS232 控程控电子负载（拉载）或可编程电源（充电），自动跑 N 档
「恒流充 / 放 -> 断开静置」，供 MCU 侧同步采集。
「循环」交给 soc_bench.py cycle 逐腿调度，放 / 充交替多轮。

跑之前先在【采集监视】页按「开始采集」，否则数据对不上。

安全保护 6 项 ——
  1. 端压门限      到门限就停，不靠 SOC 猜
  2. 断点校验      每档收尾核对电流 / 电压，不对就停
  3. 带载确认      开跑前确认负载真的带上了
  4. 通信看门狗    设备不应答就断开输出
  5. 异常兜底断开  Ctrl+C / 拔线也走 finally 断开
  6. 跨设备互锁    填了「另一台串口」就在开跑前先关它并复核电流

「循环」模式下两个串口都必须填：放电口接负载、充电口接电源。
缺一个的话那一腿只能自动探测，而两台设备同时插着时自动探测会挑错，
互锁就无从建立。""",
}


def nav_label(master, text, **kw):
    """页首那行「本页做什么 · 下一步去哪」。调用方自己 pack / grid。"""
    return ttk.Label(master, text=text, font=FONT_UI, foreground=NAV_FG,
                     justify="left", wraplength=1000, **kw)


def _help_window(parent, detail, title="说明"):
    """说明弹窗。用 Text + 滚动条而不是 messagebox —— messagebox 不能滚，
    说明一长就把按钮顶出屏幕。非模态：看着说明还能继续操作。"""
    win = tk.Toplevel(parent)
    win.title(title)
    win.transient(parent.winfo_toplevel())
    body = ttk.Frame(win, padding=8)
    body.pack(fill="both", expand=True)
    txt = tk.Text(body, wrap="word", width=80, height=18, font=FONT_UI,
                  relief="flat", background="#fbfbfb")
    sb = ttk.Scrollbar(body, command=txt.yview)
    txt.configure(yscrollcommand=sb.set)
    txt.insert("1.0", detail)
    txt.configure(state="disabled")
    txt.pack(side="left", fill="both", expand=True)
    sb.pack(side="right", fill="y")
    ttk.Button(win, text="关闭", command=win.destroy).pack(pady=(0, 8))
    win.update_idletasks()
    try:
        top = parent.winfo_toplevel()
        win.geometry("+%d+%d" % (top.winfo_rootx() + 90, top.winfo_rooty() + 90))
    except Exception:
        pass


class HelpLine(ttk.Frame):
    """一行摘要 + 右侧「?」；点「?」弹窗看完整说明。"""

    def __init__(self, master, summary, detail, **kw):
        super().__init__(master, **kw)
        self._detail = detail
        ttk.Label(self, text=summary, font=FONT_UI, foreground="#555",
                  justify="left", wraplength=850).pack(side="left", anchor="w")
        ttk.Button(self, text="?", width=3, command=self._show).pack(
            side="left", anchor="w", padx=(6, 0))

    def _show(self):
        _help_window(self, self._detail)


class LogPane(ttk.Frame):
    """带线程安全队列的日志面板。`write()` 可以从任何线程调。"""

    def __init__(self, master, height=10, mono=True, **kw):
        super().__init__(master, **kw)
        self.txt = scrolledtext.ScrolledText(
            self, height=height, wrap="word",
            font=FONT_MONO if mono else FONT_UI,
            background="#fbfbfb", relief="solid", borderwidth=1)
        self.txt.pack(fill="both", expand=True)
        self.txt.configure(state="disabled")
        self._q = queue.Queue()
        self._drain()

    def write(self, s):
        self._q.put(("t", s))

    def writeline(self, s=""):
        self._q.put(("t", s + "\n"))

    def clear(self):
        self._q.put(("c", None))

    # 给重定向 stdout 用
    def flush(self):
        pass

    def _drain(self):
        try:
            while True:
                kind, s = self._q.get_nowait()
                if kind == "c":
                    self.txt.configure(state="normal")
                    self.txt.delete("1.0", "end")
                else:
                    self.txt.configure(state="normal")
                    self.txt.insert("end", s)
                    self.txt.see("end")
                self.txt.configure(state="disabled")
        except queue.Empty:
            pass
        self.after(80, self._drain)


def run_bg(fn, on_error=None):
    """后台线程跑 fn（界面不卡）；异常打到 on_error（默认打印到 stderr）。

    --noconsole 打包后 sys.stderr 是 None，所以这里不能直接往 stderr 写。
    """
    def wrapper():
        try:
            fn()
        except Exception:
            tb = traceback.format_exc()
            if on_error:
                try:
                    on_error(tb)
                except Exception:
                    pass
            elif sys.stderr is not None:
                sys.stderr.write(tb)
    t = threading.Thread(target=wrapper, daemon=True)
    t.start()
    return t


class ScriptRunner(object):
    """把命令行脚本当函数调：argv 注入 + stdout 捕获（不改原脚本一行）。"""

    _lock = threading.Lock()

    @staticmethod
    def load(name):
        """按名字加载同目录脚本（懒加载 + 缓存）。name 可含子目录，如 test/kalman_tune。"""
        key = "_bms_script_" + name.replace("/", "_").replace("\\", "_")
        mod = sys.modules.get(key)
        if mod is not None:
            return mod
        path = os.path.join(TOOLS, name + ".py")
        if not os.path.isfile(path):
            raise FileNotFoundError("找不到脚本 %s" % path)
        sub = os.path.dirname(path)
        if sub and sub not in sys.path:       # 让脚本能 import 同目录的伙伴
            sys.path.insert(0, sub)
        import importlib.util
        spec = importlib.util.spec_from_file_location(key, path)
        mod = importlib.util.module_from_spec(spec)
        sys.modules[key] = mod
        spec.loader.exec_module(mod)
        return mod

    @classmethod
    def call(cls, name, argv, log):
        """同步执行（调用方自己开线程）。返回退出码。"""
        with cls._lock:
            mod = cls.load(name)
            old_argv, old_out = sys.argv, sys.stdout
            sys.argv = [name + ".py"] + list(argv)
            sys.stdout = log
            try:
                mod.main()
                return 0
            except SystemExit as e:
                if e.code not in (0, None):
                    log.writeline("[i] 脚本退出：%s" % (e.code,))
                return 1 if e.code else 0
            except Exception:
                log.writeline("[X] 脚本异常：\n" + traceback.format_exc())
                return 1
            finally:
                sys.argv, sys.stdout = old_argv, old_out


def open_path(path):
    """用系统默认程序打开文件或文件夹。"""
    try:
        if os.name == "nt":
            os.startfile(path)                       # noqa: S606
        else:
            import subprocess
            subprocess.Popen(["xdg-open", path])
    except Exception as e:
        messagebox.showerror(APP_TITLE, "打不开 %s：\n%s" % (path, e))


def fmt_hex(b):
    return " ".join("%02X" % x for x in b)


def raise_async_exc(thread, exc=KeyboardInterrupt):
    """往后台线程里注入一个异常 —— 等价于在那个线程内部按 Ctrl+C。

    soc_load.py 的加载循环有 KeyboardInterrupt 兜底（finally 里断开），
    所以这是上位机唯一能安全中止一次 2.8 h 放电的入口。
    """
    try:
        import ctypes
        tid = thread.ident
        if tid is None:
            return False
        r = ctypes.pythonapi.PyThreadState_SetAsyncExc(
            ctypes.c_ulong(tid), ctypes.py_object(exc))
        if r > 1:                       # 理论上不会发生，保险起见回滚
            ctypes.pythonapi.PyThreadState_SetAsyncExc(ctypes.c_ulong(tid), None)
            return False
        return r == 1
    except Exception:
        return False


# ==========================================================================
# 图表面板：分析脚本落盘的 PNG 直接在窗口里看
# ==========================================================================

try:
    from PIL import Image as _PILImage
    from PIL import ImageTk as _PILImageTk
    _PIL_LANCZOS = getattr(getattr(_PILImage, "Resampling", _PILImage),
                           "LANCZOS", 1)
except Exception:                       # 没装 Pillow 也能用：退化到 tk 原生图
    _PILImage = None
    _PILImageTk = None
    _PIL_LANCZOS = None


class ImagePane(ttk.Frame):
    """内嵌的图表面板 —— 分析跑完的图直接显示，不用切到看图软件。

    用法（任何线程都能调，内部自己排队回主线程）::

        pane.show(paths)          # 显示一组图，默认停在最后一张
        pane.append(path)         # 追加一张并切过去（跑完一张接一张时用）
        pane.clear()

    操作：◀ ▶ 换图｜「适应窗口」整张缩进可见｜「1:1」+ 滚轮/滚动条看细节｜
    双击弹独立大窗｜「另存为」拷到别处｜「外部打开」交给系统看图器。
    """

    EMPTY = "跑一次分析 —— 图会显示在这里"

    def __init__(self, master, app=None, height=260, allow_popup=True):
        super().__init__(master)
        self.app = app
        self.allow_popup = allow_popup
        self._paths = []
        self._idx = -1
        self._img = None                # Pillow 图（原图，未缩放）
        self._tkimg = None              # 无 Pillow 时的 tk.PhotoImage
        self._photo = None              # 当前显示用，必须持引用否则被 GC
        self._key = None                # (路径, mtime, 缩放比) 缓存键
        self._fit = True
        self._zoom = 1.0

        bar = ttk.Frame(self)
        bar.pack(fill="x")
        self.var_title = tk.StringVar(value=self.EMPTY)
        ttk.Label(bar, textvariable=self.var_title, font=FONT_UI,
                  anchor="w").pack(side="left", padx=2, fill="x", expand=True)
        for txt, cmd, w in (("▶", lambda: self.step(1), 3),
                            ("◀", lambda: self.step(-1), 3),
                            ("另存为…", self.save_as, None),
                            ("外部打开", self.open_ext, None),
                            ("1:1", self.zoom_reset, 4),
                            ("适应窗口", self.fit_win, None),
                            ("+", lambda: self.zoom_by(1.25), 3),
                            ("−", lambda: self.zoom_by(1 / 1.25), 3)):
            ttk.Button(bar, text=txt, command=cmd,
                       width=w).pack(side="right", padx=1)

        mid = ttk.Frame(self)
        mid.pack(fill="both", expand=True, pady=(2, 0))
        self.canvas = tk.Canvas(mid, height=height, bg="#f5f6f7",
                                highlightthickness=1,
                                highlightbackground="#c8c8c8")
        vs = ttk.Scrollbar(mid, orient="vertical", command=self.canvas.yview)
        hs = ttk.Scrollbar(mid, orient="horizontal", command=self.canvas.xview)
        self.canvas.configure(yscrollcommand=vs.set, xscrollcommand=hs.set)
        self.canvas.grid(row=0, column=0, sticky="nsew")
        vs.grid(row=0, column=1, sticky="ns")
        hs.grid(row=1, column=0, sticky="ew")
        mid.rowconfigure(0, weight=1)
        mid.columnconfigure(0, weight=1)
        self.canvas.bind("<Configure>", lambda e: self._render())
        self.canvas.bind("<Enter>", lambda e: self.canvas.focus_set())
        self.canvas.bind("<MouseWheel>", self._wheel)
        if allow_popup:
            self.canvas.bind("<Double-Button-1>", lambda e: self.popup())
            self.canvas.bind("<Return>", lambda e: self.popup())

    # ---------------- 线程安全入口 ----------------
    def _call(self, fn):
        if (self.app is not None
                and threading.current_thread() is not threading.main_thread()):
            self.app.ui(fn)
        else:
            fn()

    def show(self, paths, newest=True):
        ps = [p for p in (paths or []) if p and os.path.isfile(p)]
        self._call(lambda: self._show(ps, newest))

    def _show(self, ps, newest):
        self._paths = list(ps)
        if not self._paths:
            self._clear()
            return
        self._idx = len(self._paths) - 1 if newest else 0
        self._load()

    def append(self, path):
        if path and os.path.isfile(path):
            self._call(lambda: self._append(path))

    def _append(self, path):
        if path not in self._paths:
            self._paths.append(path)
        self._idx = self._paths.index(path)
        self._load()

    def clear(self):
        self._call(self._clear)

    def _clear(self):
        self._paths, self._idx = [], -1
        self._img = self._tkimg = self._photo = self._key = None
        self.var_title.set(self.EMPTY)
        self.canvas.delete("all")
        self._draw_empty()

    def step(self, d):
        if not self._paths:
            return
        self._idx = (self._idx + d) % len(self._paths)
        self._load()

    # ---------------- 载入与绘制 ----------------
    def _load(self):
        if not self._paths:
            self._clear()
            return
        path = self._paths[self._idx]
        self._key = None
        self._img = self._tkimg = self._photo = None
        try:
            if _PILImage is not None:
                im = _PILImage.open(path)
                im.load()
                self._img = im if im.mode in ("RGB", "RGBA") else im.convert("RGB")
            else:
                self._tkimg = tk.PhotoImage(file=path)
        except Exception as e:
            self.var_title.set("打不开 %s：%s" % (os.path.basename(path), e))
            self.canvas.delete("all")
            self._draw_empty("这个图打不开：%s" % e)
            return
        self._render()
        self._update_title()

    def _render(self):
        if self._img is None and self._tkimg is None:
            self._draw_empty()
            return
        cw, ch = self.canvas.winfo_width(), self.canvas.winfo_height()
        if cw <= 1 or ch <= 1:                  # 还没完成布局，等 <Configure>
            return
        if self._img is not None:
            iw, ih = self._img.size
        else:
            iw, ih = self._tkimg.width(), self._tkimg.height()
        scale = self._fit_scale(cw, ch, iw, ih) if self._fit else self._zoom
        path = self._paths[self._idx]
        try:
            mt = os.path.getmtime(path)         # 同名文件被重跑覆盖时要重绘
        except OSError:
            mt = 0
        key = (path, mt, round(scale, 5), cw, ch)
        if key == self._key:
            return
        self._key = key

        self.canvas.delete("all")
        if self._img is not None:
            w = max(1, int(round(iw * scale)))
            h = max(1, int(round(ih * scale)))
            im = self._img if (w, h) == (iw, ih) else \
                self._img.resize((w, h), _PIL_LANCZOS)
            self._photo = _PILImageTk.PhotoImage(im)
        else:                                    # tk 原生：只能整数倍缩小
            k = max(1, int(math.ceil(1.0 / max(scale, 1e-6))))
            self._photo = self._tkimg if k <= 1 else self._tkimg.subsample(k)
            w, h = self._photo.width(), self._photo.height()
        x, y = max(0, (cw - w) // 2), max(0, (ch - h) // 2)
        self.canvas.create_image(x, y, anchor="nw", image=self._photo)
        self.canvas.configure(scrollregion=(0, 0, max(cw, w), max(ch, h)))

    @staticmethod
    def _fit_scale(cw, ch, iw, ih):
        """整数倍放大会糊，所以只缩不放（>1 时按 1:1 居中）。

        下限 2%：画布还没布局完时 cw/ch 是 1，不兜底会算出负数。
        """
        return max(0.02, min((cw - 6.0) / iw, (ch - 6.0) / ih, 1.0))

    def _draw_empty(self, text=None):
        self.canvas.delete("all")
        cw = max(self.canvas.winfo_width(), 160)
        ch = max(self.canvas.winfo_height(), 80)
        self.canvas.create_text(cw // 2, ch // 2, text=text or self.EMPTY,
                                fill="#9aa0a6", font=FONT_UI, width=cw - 20)
        self.canvas.configure(scrollregion=(0, 0, cw, ch))

    def _update_title(self):
        if not self._paths:
            self.var_title.set(self.EMPTY)
            return
        path = self._paths[self._idx]
        if self._img is not None:
            iw, ih = self._img.size
        else:
            iw, ih = self._tkimg.width(), self._tkimg.height()
        cw, ch = self.canvas.winfo_width(), self.canvas.winfo_height()
        if not self._fit:
            pct = "%d%%" % int(round(self._zoom * 100))
        elif cw > 1 and ch > 1:
            pct = "%d%%" % int(round(self._fit_scale(cw, ch, iw, ih) * 100))
        else:
            pct = "待布局"
        self.var_title.set("%s   %d×%d   显示 %s   [%d/%d]%s"
                           % (os.path.basename(path), iw, ih, pct,
                              self._idx + 1, len(self._paths),
                              "" if _PILImage is not None else "   (没装 Pillow，缩放较粗)"))

    # ---------------- 视图模式 ----------------
    def fit_win(self):
        self._fit = True
        self._key = None
        self._render()
        self._update_title()

    def zoom_reset(self):
        self._fit, self._zoom = False, 1.0
        self._key = None
        self._render()
        self._update_title()

    def zoom_by(self, factor):
        self._fit = False
        self._zoom = max(0.05, min(8.0, self._zoom * factor))
        self._key = None
        self._render()
        self._update_title()

    def _wheel(self, e):
        if e.state & 0x0004:                     # Ctrl + 滚轮 = 缩放
            self.zoom_by(1.15 if e.delta > 0 else 1 / 1.15)
        else:
            self.canvas.yview_scroll(-1 if e.delta > 0 else 1, "units")

    # ---------------- 打开 / 保存 ----------------
    def open_ext(self):
        if self._paths:
            open_path(self._paths[self._idx])

    def save_as(self):
        if not self._paths:
            return
        src = self._paths[self._idx]
        dst = filedialog.asksaveasfilename(
            title="另存为", defaultextension=".png",
            initialfile=os.path.basename(src), filetypes=[("PNG", "*.png")])
        if not dst:
            return
        try:
            shutil.copy2(src, dst)
        except Exception as e:
            messagebox.showerror(APP_TITLE, "另存失败：\n%s" % e)

    def popup(self):
        """双击弹一个独立大窗，1:1 慢慢看（可滚动）。"""
        if not self._paths or not self.allow_popup:
            return
        p = self._paths[self._idx]
        win = tk.Toplevel(self)
        win.title("%s  —  %s" % (os.path.basename(p), APP_TITLE))
        win.geometry("1120x780")
        try:
            win.transient(self.winfo_toplevel())
        except Exception:
            pass
        pane = ImagePane(win, self.app, allow_popup=False)
        pane.pack(fill="both", expand=True)
        pane.show([p])
        pane.zoom_reset()
        win.bind("<Escape>", lambda e: win.destroy())


# ==========================================================================
# 采集监视
# ==========================================================================

class RecordTab(ttk.Frame):
    tab_name = "采集监视"

    def __init__(self, master, app):
        super().__init__(master, padding=8)
        self.app = app
        self.ser = None
        self.thread = None
        self.stop_flag = threading.Event()
        self.q = queue.Queue()
        self.f = None
        self.t0 = None
        self.n_ok = self.n_bad = 0
        # 默认名带时间戳：同名时采集是**直接覆盖**写的，连采两次会把前一次冲掉
        self.csv_path = tk.StringVar(value=os.path.join(
            os.path.expanduser("~"), "Desktop",
            "soc_data_" + time.strftime("%m%d-%H%M") + ".csv"))
        self._build()
        self._drain()

    def _build(self):
        nav_label(self, NAV_TEXT["record"]).pack(anchor="w", pady=(0, 6))
        top = ttk.LabelFrame(self, text="串口", padding=6)
        top.pack(fill="x")
        ttk.Label(top, text="串口").grid(row=0, column=0, sticky="w")
        self.cb_port = ttk.Combobox(top, width=28, state="readonly")
        self.cb_port.grid(row=0, column=1, padx=4)
        ttk.Button(top, text="刷新", command=self.refresh_ports).grid(row=0, column=2)
        ttk.Label(top, text="波特率").grid(row=0, column=3, padx=(12, 0))
        self.cb_baud = ttk.Combobox(top, width=10, values=[
            9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600])
        self.cb_baud.set("115200")
        self.cb_baud.grid(row=0, column=4, padx=4)

        ttk.Label(top, text="输出 CSV").grid(row=1, column=0, sticky="w", pady=(6, 0))
        ttk.Entry(top, textvariable=self.csv_path, width=60).grid(
            row=1, column=1, columnspan=3, sticky="we", padx=4, pady=(6, 0))
        ttk.Button(top, text="浏览…", command=self.pick_csv).grid(
            row=1, column=4, pady=(6, 0))

        btns = ttk.Frame(self)
        btns.pack(fill="x", pady=6)
        self.btn_start = ttk.Button(btns, text="▶ 开始采集", command=self.start)
        self.btn_start.pack(side="left")
        self.btn_stop = ttk.Button(btns, text="■ 停止并保存",
                                   command=self.stop, state="disabled")
        self.btn_stop.pack(side="left", padx=6)
        self.lbl_stat = ttk.Label(btns, text="未开始", font=FONT_UI)
        self.lbl_stat.pack(side="left", padx=12)

        g = ttk.LabelFrame(self, text="实时值", padding=8)
        g.pack(fill="x")
        self.vars = {}
        # 第三行原来写 "0.1°C" / "0.01%" —— 那是**精度**不是单位，容易看岔。
        # 现在写成 "°C (±0.1)" 这种「单位（精度）」的形式。
        items = [("V", "mV"), ("I", "mA"), ("T", "°C (±0.1)"), ("SOC", "% (0.01)"),
                 ("SOH", "% (0.01)"), ("Q", "mAh"), ("R0", "mΩ")]
        for i, (k, u) in enumerate(items):
            ttk.Label(g, text=k, font=FONT_UI).grid(row=0, column=i * 2, sticky="e")
            v = tk.StringVar(value="--")
            ttk.Label(g, textvariable=v, font=FONT_BIG, width=7,
                      anchor="e").grid(row=1, column=i * 2, sticky="e")
            ttk.Label(g, text=u, font=FONT_UI).grid(row=2, column=i * 2, sticky="e")
            ttk.Label(g, text=" ", width=2).grid(row=1, column=i * 2 + 1)
            self.vars[k] = v

        ttk.Label(self, text="原始帧", font=FONT_UI).pack(anchor="w", pady=(8, 0))
        self.log = LogPane(self, height=12)
        self.log.pack(fill="both", expand=True)

    def refresh_ports(self):
        try:
            ports = P.SerialTransport.list_ports()
        except Exception as e:
            messagebox.showerror(APP_TITLE, "枚举串口失败：%s" % e)
            return
        self.cb_port["values"] = ["%s  %s" % (d, s) for d, s in ports]
        if ports:
            self.cb_port.current(0)
        self.log.writeline("[i] 发现 %d 个串口：%s"
                           % (len(ports), ", ".join(d for d, _ in ports) or "无"))

    def pick_csv(self):
        p = filedialog.asksaveasfilename(
            title="输出 CSV", defaultextension=".csv",
            initialfile=os.path.basename(self.csv_path.get()),
            filetypes=[("CSV", "*.csv")])
        if p:
            self.csv_path.set(p)

    def _port(self):
        s = self.cb_port.get().strip()
        if not s:
            return None
        return s.split()[0]

    def start(self):
        port = self._port()
        if not port:
            messagebox.showwarning(APP_TITLE, "先选串口（点「刷新」）")
            return
        if self.app.port_in_use(port, self):
            return
        try:
            import serial
            self.ser = serial.Serial(port, int(self.cb_baud.get()), timeout=0.5)
        except Exception as e:
            messagebox.showerror(APP_TITLE, "打开串口失败：%s" % e)
            return
        self.t0 = None
        self.n_ok = self.n_bad = 0
        self.stop_flag.clear()
        path = self.csv_path.get()
        self.f = open_csv(path)
        self.log.clear()
        self.log.writeline("[i] 打开 %s @ %s，输出 -> %s" % (port, self.cb_baud.get(), path))
        self.log.writeline("[i] 等数据…（Ctrl+C 不管用，用「停止」按钮）")
        self.btn_start.configure(state="disabled")
        self.btn_stop.configure(state="normal")
        self.app.set_port_owner(port, self)
        self.thread = run_bg(self._loop, on_error=lambda tb: self.log.writeline(tb))

    def _loop(self):
        import soc_record
        while not self.stop_flag.is_set():
            try:
                raw = self.ser.readline()
            except Exception as e:
                self.q.put(("err", str(e)))
                break
            if not raw:
                continue
            text = raw.decode("utf-8", errors="replace")
            vals = soc_record.parse_line(text)
            now = datetime.datetime.now()
            if vals is None:
                self.n_bad += 1
                self.q.put(("raw", text.strip()))
                continue
            if self.t0 is None:
                self.t0 = now
            elapsed = (now - self.t0).total_seconds()
            if self.f:
                self.f.write("%s,%.3f,%s\n" % (
                    now.isoformat(timespec="milliseconds"), elapsed,
                    ",".join("" if v is None else str(v) for v in vals)))
                self.f.flush()
            self.n_ok += 1
            self.q.put(("vals", (vals, elapsed)))
        self.q.put(("done", None))

    def stop(self):
        self.stop_flag.set()
        self.btn_stop.configure(state="disabled")
        self.app.clear_port_owner()
        if self.f:
            try:
                self.f.close()
            except Exception:
                pass
            self.f = None
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
        self.log.writeline("[i] 停止：有效 %d 帧 / 丢弃 %d" % (self.n_ok, self.n_bad))
        self.btn_start.configure(state="normal")
        self.lbl_stat.configure(text="已停止（有效 %d，丢弃 %d）"
                                     % (self.n_ok, self.n_bad))

    def _drain(self):
        try:
            while True:
                kind, payload = self.q.get_nowait()
                if kind == "vals":
                    vals, elapsed = payload
                    v, i, t, soc, soh, q, r0 = vals
                    self.vars["V"].set(str(v))
                    self.vars["I"].set("%+d" % i)
                    self.vars["T"].set("%.1f" % (t / 10.0))
                    self.vars["SOC"].set("--" if soc is None else "%.2f" % (soc / 100.0))
                    self.vars["SOH"].set("--" if soh is None else "%.2f" % (soh / 100.0))
                    self.vars["Q"].set("--" if q is None else str(q))
                    self.vars["R0"].set("--" if r0 is None else str(r0))
                    self.lbl_stat.configure(
                        text="采集中：%d 帧 / 丢 %d / %.1f min"
                             % (self.n_ok, self.n_bad, elapsed / 60.0))
                elif kind == "raw":
                    self.log.writeline("  [丢弃] " + payload)
                elif kind == "err":
                    self.log.writeline("[X] " + payload)
                elif kind == "done":
                    pass
        except queue.Empty:
            pass
        self.after(80, self._drain)


def open_csv(path):
    try:
        f = open(path, "w", encoding="utf-8", newline="")
        f.write("time_iso,elapsed_s,V_mV,I_mA,T_0p1C,SOC_0p01,SOH_0p01,Q_mAh,R0_mohm\n")
        return f
    except Exception as e:
        messagebox.showerror(APP_TITLE, "建不了 CSV：%s" % e)
        return None


# ==========================================================================
# 在线调参
# ==========================================================================

class TuneTab(ttk.Frame):
    """bms_tune 协议上位机：命令表自动生成界面 + 三个批量编辑器 + SOC/EKF 读写。"""

    tab_name = "在线调参"

    def __init__(self, master, app):
        super().__init__(master, padding=8)
        self.app = app
        self.cli = None
        self.ser = None
        self.busy = False
        self.auto_busy = False
        self.auto_live = tk.BooleanVar(value=False)
        self.cal_tbls = {}          # {(ti,tbl): [11 个 int]}
        self.soc_state = None       # 最近一次 RD_SOC 的 soc01（0.01%）
        self.ekf_vals = None        # 最近一次 RD_EKF 的 8 个参数值
        self.dims = dict(P.DEFAULT_DIMS)
        self._uiq = queue.Queue()   # 工作线程 -> 主线程 的界面更新队列
        self._build()
        self._sync_cmd_list()
        self._drain_ui()

    def _drain_ui(self):
        """主线程统一处理界面更新（tkinter 不能从别的线程碰控件）。"""
        try:
            while True:
                fn = self._uiq.get_nowait()
                try:
                    fn()
                except Exception:
                    self.log.writeline(traceback.format_exc())
        except queue.Empty:
            pass
        self.after(60, self._drain_ui)

    def _ui(self, fn):
        self._uiq.put(fn)

    # ------------------------------------------------------------ 界面
    def _build(self):
        nav_label(self, NAV_TEXT["tune"]).pack(anchor="w", pady=(0, 6))
        top = ttk.LabelFrame(self, text="连接", padding=6)
        top.pack(fill="x")
        ttk.Label(top, text="串口").grid(row=0, column=0)
        self.cb_port = ttk.Combobox(top, width=26, state="readonly")
        self.cb_port.grid(row=0, column=1, padx=4)
        ttk.Button(top, text="刷新", command=self.refresh_ports).grid(row=0, column=2)
        ttk.Label(top, text="波特率").grid(row=0, column=3, padx=(10, 0))
        self.cb_baud = ttk.Combobox(top, width=9, values=[
            9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600])
        self.cb_baud.set("115200")
        self.cb_baud.grid(row=0, column=4, padx=4)
        self.btn_conn = ttk.Button(top, text="连接", command=self.toggle_conn)
        self.btn_conn.grid(row=0, column=5, padx=6)
        self.lbl_state = ttk.Label(top, text="未连接", font=FONT_UI, foreground="#a00")
        self.lbl_state.grid(row=0, column=6, padx=8)
        ttk.Label(top, text="超时(ms)").grid(row=0, column=7, padx=(10, 0))
        self.var_to = tk.StringVar(value="300")
        ttk.Entry(top, width=6, textvariable=self.var_to).grid(row=0, column=8)

        bar = ttk.Frame(self)
        bar.pack(fill="x", pady=6)
        for text, cmd in (("① 握手 / 探活", lambda: self.quick("PING")),
                          ("② 读板子配置", lambda: self.quick("INFO")),
                          ("③ 读实时量", lambda: self.quick("RD_LIVE")),
                          ("CRC 自检（不连板子也能跑）", self.show_selftest)):
            ttk.Button(bar, text=text, command=cmd).pack(side="left", padx=(0, 6))
        ttk.Checkbutton(bar, text="每秒自动读一次实时量",
                        variable=self.auto_live,
                        command=self._auto_tick).pack(side="left", padx=10)

        self.nb = ttk.Notebook(self)
        self.nb.pack(fill="both", expand=True)

        self._build_cmd_tab()
        self._build_cal_tab()
        self._build_r0_tab()
        self._build_soh_tab()
        self._build_misc_tab()

        self.log = LogPane(self, height=6)
        self.log.pack(fill="both", expand=True, pady=(6, 0))

    # ---- 命令面板（从命令表自动生成，但呈现成使用者能看懂的样子）
    #
    # 这里刻意不显示 hex 和英文命令名：那些是写协议的人关心的。用户看到的
    # 是「读一个标定点」「写回全部内阻」这种干活的说法。协议名和命令码放在
    # 右边的副标题里 —— 真要去对协议手册，一眼也能找到。
    def _build_cmd_tab(self):
        f = ttk.Frame(self.nb, padding=6)
        self.nb.add(f, text="单条命令")
        # 用 grid 让整页撑满 notebook：pack 的话左栏高度只看自己的请求高度，
        # 窗口拉高了列表也不会长，命令得滚着看。
        f.columnconfigure(0, weight=1)
        f.rowconfigure(1, weight=1)

        HelpLine(f, "左边点一条命令，右边就出现它要填的参数 —— 直接填工程值"
                    "（电压 mV、温度 °C），换算不用管", HELP["cmd"]).grid(
            row=0, column=0, sticky="ew", pady=(0, 6))

        body = ttk.Frame(f)
        body.grid(row=1, column=0, sticky="nsew")
        body.columnconfigure(1, weight=1)
        body.rowconfigure(0, weight=1)

        # ---------------------------------------------------------- 左：命令列表
        left = ttk.Frame(body)
        left.grid(row=0, column=0, sticky="nsw")
        left.columnconfigure(0, weight=1)
        left.rowconfigure(1, weight=1)
        sbar = ttk.Frame(left)
        sbar.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 4))
        ttk.Label(sbar, text="找命令", font=FONT_UI).pack(side="left")
        self.var_find = tk.StringVar()
        ent = ttk.Entry(sbar, textvariable=self.var_find, width=16)
        ent.pack(side="left", padx=4)
        ent.bind("<KeyRelease>", lambda e: self._sync_cmd_list())
        ttk.Button(sbar, text="显示全部", width=9,
                   command=self._clear_find).pack(side="left")

        self.tree = ttk.Treeview(left, columns=("code", "need"),
                                 show="tree headings", height=18,
                                 selectmode="browse")
        self.tree.heading("#0", text="命令", anchor="w")
        self.tree.heading("code", text="编码", anchor="center")
        self.tree.heading("need", text="参数", anchor="center")
        self.tree.column("#0", width=190, stretch=False)
        self.tree.column("code", width=50, stretch=False, anchor="center")
        self.tree.column("need", width=56, stretch=False, anchor="center")
        self.tree.tag_configure("cat", font=FONT_BOLD)
        self.tree.tag_configure("dep", foreground="#b35c00")
        self.tree.grid(row=1, column=0, sticky="nsew")
        self.tree.bind("<<TreeviewSelect>>", lambda e: self.on_pick())
        sb = ttk.Scrollbar(left, orient="vertical", command=self.tree.yview)
        sb.grid(row=1, column=1, sticky="ns")
        self.tree.configure(yscrollcommand=sb.set)

        # ---------------------------------------------------------- 右：这一条
        right = ttk.Frame(body, padding=(10, 0, 0, 0))
        right.grid(row=0, column=1, sticky="nsew")

        self.lbl_title = ttk.Label(right, text="← 左边点一条命令", font=FONT_H1)
        self.lbl_title.pack(anchor="w")
        self.lbl_sub = ttk.Label(right, text="", font=FONT_UI, foreground="#777")
        self.lbl_sub.pack(anchor="w", pady=(2, 0))
        self.lbl_desc = ttk.Label(right, text="", wraplength=620, font=FONT_UI,
                                  justify="left")
        self.lbl_desc.pack(anchor="w", pady=(6, 0))
        self.lbl_needs = ttk.Label(right, text="", wraplength=620, font=FONT_UI,
                                   justify="left", foreground="#b35c00")
        self.lbl_needs.pack(anchor="w", pady=(4, 0))

        self.frm_down = ttk.LabelFrame(right, text="要填的参数", padding=6)
        self.frm_down.pack(fill="x", pady=6)
        self.down_vars = {}

        self.btn_send = ttk.Button(right, text="▶ 发送", command=self.send_current)
        self.btn_send.pack(anchor="w")

        ttk.Label(right, text="板子回什么", font=FONT_BOLD).pack(anchor="w",
                                                              pady=(10, 2))
        cols = ("field", "value", "unit", "note")
        self.res = ttk.Treeview(right, columns=cols, show="headings", height=7)
        for c, t, w in (("field", "项目", 130), ("value", "数值", 150),
                        ("unit", "单位", 60), ("note", "说明", 230)):
            self.res.heading(c, text=t)
            self.res.column(c, width=w, anchor="w")
        self.res.pack(fill="both", expand=True)

        self.var_hex = tk.BooleanVar(value=False)
        ttk.Checkbutton(right, text="显示收发的原始字节（排错用）",
                        variable=self.var_hex,
                        command=self._toggle_hex).pack(anchor="w", pady=(4, 0))
        self.lbl_hex = ttk.Label(right, text="", font=FONT_MONO, justify="left",
                                 wraplength=620, foreground="#555")
        self.lbl_hex_raw = ""

    def _toggle_hex(self):
        if self.var_hex.get():
            self.lbl_hex.configure(text=self.lbl_hex_raw)
        else:
            self.lbl_hex.configure(text="")

    def _clear_find(self):
        self.var_find.set("")
        self._sync_cmd_list()

    def _match_cmd(self, c, kw):
        hay = " ".join((c.title, c.name, c.desc, c.cat, "0x%02x" % c.code,
                        c.needs)).lower()
        return kw in hay

    def _sync_cmd_list(self):
        kw = self.var_find.get().strip().lower()
        keep = self.cur_cmd.code if getattr(self, "cur_cmd", None) else None
        self.tree.delete(*self.tree.get_children())
        cats = {}
        for code in sorted(P.COMMANDS):
            c = P.COMMANDS[code]
            if kw and not self._match_cmd(c, kw):
                continue
            cats.setdefault(c.cat or "其它", []).append(c)
        self._items = {}
        pick = None
        for cat in P.DEFAULT_CMD_CATS:
            if cat not in cats:
                continue
            node = self.tree.insert("", "end", text=cat, open=True, tags=("cat",))
            for c in cats[cat]:
                iid = self.tree.insert(node, "end", text=c.title,
                                       values=("0x%02X" % c.code, c.n_down_text),
                                       tags=(("dep",) if c.dep else ()))
                self._items[iid] = c
                if keep is not None and c.code == keep:
                    pick = iid
        if pick is None:                       # 搜索后原选中项不在了 → 取第一条
            first = self.tree.get_children()
            if first:
                kids = self.tree.get_children(first[0])
                pick = kids[0] if kids else None
        if pick:
            self.tree.selection_set(pick)
            self.tree.focus(pick)
            self.tree.see(pick)
        else:
            self.lbl_title.configure(text="没有匹配的命令")
            self.lbl_sub.configure(text="")
            self.lbl_desc.configure(text="换个词试试，或者点「显示全部」。")
            self.lbl_needs.configure(text="")

    def _hint_for(self, f):
        """输入框右边的灰字提示：说清单位、范围和该填什么。"""
        d = self.dims
        if f.name == "ti":
            return "第几个温度点（0 ~ %d）" % max(d.get("temp_n", 1) - 1, 0)
        if f.name == "idx":
            return "第几个 SOC 点（0 ~ %d）" % max(d.get("soc_n", 1) - 1, 0)
        if f.name == "tbl":
            return "0 = OCV，1 = R0，2 = R1，3 = τ"
        parts = []
        if f.unit:
            parts.append("单位 %s" % f.unit)
        if f.note:
            parts.append(f.note)
        if f.div != 1:
            parts.append("直接填真实值")
        return "，".join(parts)

    def _default_for(self, f):
        if f.name in ("ti", "tbl", "idx"):
            return "0"
        if f.name == "t":
            return "25.0"
        if f.name == "mah":
            return str(self.dims.get("cap_nom_mah", 3350))
        if f.name == "soc01" and f.kind != "f32":
            # 先「读 SOC 运行态」再改，就能在现值上微调（不用自己按表算）
            if self.soc_state is not None:
                return "%.2f" % (self.soc_state / 100.0)
            return ""
        if f.kind == "f32" and f.name in P.EKF_NAMES:
            # 读过就填板子现值，没读过填固件出厂值 —— 两者都比空框好用：
            # WR_EKF 是整组写，留空的话 8 个框都得手填。
            i = P.EKF_NAMES.index(f.name)
            v = self.ekf_vals[i] if self.ekf_vals else P.EKF_DEFAULTS[i]
            return "%g" % v
        return ""

    def on_pick(self):
        sel = self.tree.selection()
        if not sel or sel[0] not in self._items:
            return
        c = self._items[sel[0]]
        self.cur_cmd = c
        self.lbl_title.configure(text=c.title)
        bits = ["命令码 0x%02X" % c.code, "协议名 %s" % c.name]
        if c.dep:
            bits.append("需固件打开 %s" % c.dep)
        self.lbl_sub.configure(text="    ·    ".join(bits))
        self.lbl_desc.configure(text=c.desc)
        self.lbl_needs.configure(text=("注意：" + c.needs) if c.needs else "")
        self.btn_send.configure(text="▶ 发送：%s" % c.title)

        for w in self.frm_down.winfo_children():
            w.destroy()
        self.down_vars = {}
        if not c.down:
            ttk.Label(self.frm_down, text="这条命令不用填参数，直接点下面的发送就行",
                      font=FONT_UI, foreground="#777").grid(row=0, column=0,
                                                            sticky="w")
            return
        for i, f in enumerate(c.down):
            ttk.Label(self.frm_down, text=f.title, font=FONT_UI).grid(
                row=i, column=0, sticky="e", padx=(0, 6), pady=3)
            v = tk.StringVar()
            if f.kind in ("blob", "cal", "r0all", "active"):
                e = ttk.Entry(self.frm_down, textvariable=v, width=30,
                              state="readonly")
                v.set("（内容由「标定表 / 内阻 R0 / SOH」页签维护）")
            elif f.name == "tbl":
                e = ttk.Combobox(self.frm_down, textvariable=v, width=6,
                                 state="readonly", values=["0", "1", "2", "3"])
                v.set("0")
            else:
                e = ttk.Entry(self.frm_down, textvariable=v, width=16)
                v.set(self._default_for(f))
            e.grid(row=i, column=1, sticky="w", pady=3)
            ttk.Label(self.frm_down, text=self._hint_for(f), font=FONT_UI,
                      foreground="#777", wraplength=330, justify="left").grid(
                row=i, column=2, sticky="w", padx=8)
            self.down_vars[f.name] = (f, v)

    # ---- 标定表
    def _build_cal_tab(self):
        f = ttk.Frame(self.nb, padding=6)
        self.nb.add(f, text="标定表")
        bar = ttk.Frame(f)
        bar.pack(fill="x")
        ttk.Label(bar, text="温度点", font=FONT_UI).pack(side="left")
        self.cb_ti = ttk.Combobox(bar, width=16, state="readonly")
        self.cb_ti.pack(side="left", padx=6)
        self.cb_ti.bind("<<ComboboxSelected>>", lambda e: self.load_cal_grid())
        for t, c in (("读整包(全部温度点)", self.read_cal_all),
                     ("写回整包", self.write_cal_all),
                     ("恢复出厂", self.cal_restore),
                     ("读活跃表(当前温度)", self.read_active),
                     ("导出到文件", self.export_cal),
                     ("从文件导入", self.import_cal)):
            ttk.Button(bar, text=t, command=c).pack(side="left", padx=(0, 4))

        self.cal_frm = ttk.LabelFrame(f, text="出厂基准表（可直接改数值）", padding=6)
        self.cal_frm.pack(fill="x", pady=6)
        self.cal_entries = {}

        self.act_frm = ttk.LabelFrame(f, text="活跃表（当前温度插值结果，只读）", padding=6)
        self.act_frm.pack(fill="x", pady=6)
        self.act_labels = {}

        HelpLine(f, "用法：① 读整包 → ② 改要改的点 → ③ 写回整包；"
                    "上面那张能改、下面那张只读", HELP["cal"]).pack(anchor="w")
        self._build_cal_grid()
        self._build_active_grid()

    def _grid_header(self, parent):
        ttk.Label(parent, text="表\\SOC", font=FONT_UI).grid(row=0, column=0, padx=4)
        for j in range(11):
            ttk.Label(parent, text="%d%%" % (j * 10), font=FONT_UI).grid(
                row=0, column=j + 1, padx=2)

    def _build_cal_grid(self):
        f = self.cal_frm
        self._grid_header(f)
        for i, tbl in enumerate(range(4)):
            ttk.Label(f, text=P.TBL_NAME[tbl], font=FONT_UI).grid(
                row=i + 1, column=0, sticky="e", padx=4, pady=1)
            for j in range(11):
                v = tk.StringVar(value="--")
                e = ttk.Entry(f, textvariable=v, width=6, justify="right")
                e.grid(row=i + 1, column=j + 1, padx=1, pady=1)
                self.cal_entries[(tbl, j)] = (v, e)

    def _build_active_grid(self):
        f = self.act_frm
        self._grid_header(f)
        for i, tbl in enumerate(range(4)):
            ttk.Label(f, text=P.TBL_NAME[tbl], font=FONT_UI).grid(
                row=i + 1, column=0, sticky="e", padx=4, pady=1)
            for j in range(11):
                v = tk.StringVar(value="--")
                ttk.Label(f, textvariable=v, font=FONT_MONO, width=6,
                          anchor="e", relief="solid", borderwidth=1).grid(
                    row=i + 1, column=j + 1, padx=1, pady=1)
                self.act_labels[(tbl, j)] = v

    def _ti_list(self):
        n = self.dims.get("temp_n", 3)
        return ["%d  %.1f °C" % (i, self.dims.get("temps", [50, 250, 450])[i] / 10.0)
                if i < len(self.dims.get("temps", [])) else "%d" % i
                for i in range(n)]

    def load_cal_grid(self):
        ti = self.cb_ti.current()
        for tbl in range(4):
            for j in range(11):
                var, _e = self.cal_entries[(tbl, j)]
                v = self.cal_tbls.get((ti, tbl))
                var.set("--" if v is None else str(v[j]))

    def _collect_cal_grid(self, ti):
        out = {}
        for tbl in range(4):
            row = []
            for j in range(11):
                var, _e = self.cal_entries[(tbl, j)]
                s = var.get().strip()
                if s in ("", "--"):
                    row.append(None)
                else:
                    try:
                        row.append(int(float(s)))
                    except ValueError:
                        raise ValueError("标定表 %s SOC=%d%% 的值 %r 不是数字"
                                         % (P.TBL_NAME[tbl], j * 10, s))
            out[(ti, tbl)] = row
        return out

    # ---- R0
    def _build_r0_tab(self):
        f = ttk.Frame(self.nb, padding=6)
        self.nb.add(f, text="R0 活跃值")
        bar = ttk.Frame(f)
        bar.pack(fill="x")
        for t, c in (("读全部(11 点)", self.read_r0_all),
                     ("写回全部", self.write_r0_all),
                     ("读第 N 点", self.read_r0_one),
                     ("写第 N 点", self.write_r0_one)):
            ttk.Button(bar, text=t, command=c).pack(side="left", padx=(0, 4))
        ttk.Label(bar, text="  第 N 点(0~10)", font=FONT_UI).pack(side="left")
        self.var_r0_idx = tk.StringVar(value="0")
        ttk.Spinbox(bar, from_=0, to=10, width=4,
                    textvariable=self.var_r0_idx).pack(side="left", padx=4)

        g = ttk.LabelFrame(f, text="R0 活跃值（出厂基准 + 老化增量），单位 mΩ", padding=8)
        g.pack(fill="x", pady=8)
        self.r0_vars = {}
        for j in range(11):
            ttk.Label(g, text="%d%%" % (j * 10), font=FONT_UI).grid(row=0, column=j)
            v = tk.StringVar(value="--")
            ttk.Entry(g, textvariable=v, width=4, justify="right").grid(
                row=1, column=j, padx=1)
            self.r0_vars[j] = v
        HelpLine(f, "填的是绝对值，库内部换算成「相对当前温度基准的增量」——"
                    "先切到目标温度再写", HELP["r0"]).pack(anchor="w")

    # ---- SOH
    def _build_soh_tab(self):
        f = ttk.Frame(self.nb, padding=6)
        self.nb.add(f, text="SOH")
        bar = ttk.Frame(f)
        bar.pack(fill="x")
        for t, c in (("读摘要", self.read_soh_sum),
                     ("读 120 B 参数块", self.read_soh_blob),
                     ("写回 120 B", self.write_soh_blob),
                     ("读累计与循环", self.read_soh_cnt),
                     ("清累计与循环", self.soh_cnt_reset),
                     ("导出块到文件", self.export_soh),
                     ("从文件读入块", self.import_soh),
                     ("清学习值(慎用)", self.soh_reset)):
            ttk.Button(bar, text=t, command=c).pack(side="left", padx=(0, 4))
        self.lbl_soh = ttk.Label(f, text="（还没读）", font=FONT_UI, justify="left",
                                 wraplength=900)
        self.lbl_soh.pack(anchor="w", pady=6)
        ttk.Label(f, text="120 B 参数块逐字段（与掉电保持落盘载荷同布局）"
                          " —— 前 68 B 与老版本一致，末尾 52 B 是充电方向 R0 "
                          "与累计充放电",
                  font=FONT_UI).pack(anchor="w")
        cols = ("name", "value", "note")
        self.soh_tree = ttk.Treeview(f, columns=cols, show="headings", height=17)
        for c, t, w in (("name", "字段", 170), ("value", "值", 300),
                        ("note", "说明", 320)):
            self.soh_tree.heading(c, text=t)
            self.soh_tree.column(c, width=w, anchor="w")
        self.soh_tree.pack(fill="both", expand=True)
        self.soh_blob = None

    # ---- 其它（容量 / 温度 / 掉电保持 / 原始帧）
    def _build_misc_tab(self):
        f = ttk.Frame(self.nb, padding=6)
        self.nb.add(f, text="容量 / 温度 / 落盘 / 原始帧")

        g1 = ttk.LabelFrame(f, text="容量 Q", padding=6)
        g1.pack(fill="x")
        self.var_cap = tk.StringVar(value="3350")
        ttk.Entry(g1, textvariable=self.var_cap, width=10).grid(row=0, column=0)
        ttk.Label(g1, text="mAh").grid(row=0, column=1, padx=4)
        ttk.Button(g1, text="读", command=self.read_cap).grid(row=0, column=2, padx=4)
        ttk.Button(g1, text="写（写后自动读回确认）",
                   command=self.write_cap).grid(row=0, column=3)
        ttk.Label(g1, text="库只做钳位：[50%, 110%] × 标称容量", font=FONT_UI,
                  foreground="#555").grid(row=0, column=4, padx=10)

        g2 = ttk.LabelFrame(f, text="温度", padding=6)
        g2.pack(fill="x", pady=6)
        self.var_temp = tk.StringVar(value="25.0")
        ttk.Entry(g2, textvariable=self.var_temp, width=10).grid(row=0, column=0)
        ttk.Label(g2, text="°C").grid(row=0, column=1, padx=4)
        ttk.Button(g2, text="读当前温度", command=self.read_temp).grid(row=0, column=2, padx=4)
        ttk.Button(g2, text="强制设置", command=self.write_temp).grid(row=0, column=3)
        ttk.Label(g2, text="强制值只在主循环没跑时有效（否则被传感器覆盖）",
                  font=FONT_UI, foreground="#555").grid(row=0, column=4, padx=10)
        ttk.Button(g2, text="读工况折算", command=self.read_corr).grid(
            row=0, column=5, padx=4)
        self.lbl_corr = ttk.Label(g2, text="折算系数：还没读", font=FONT_UI)
        self.lbl_corr.grid(row=1, column=0, columnspan=6, sticky="w", pady=(6, 0))

        g3 = ttk.LabelFrame(f, text="掉电保持", padding=6)
        g3.pack(fill="x", pady=6)
        ttk.Button(g3, text="立即落盘", command=self.nvm_save).grid(row=0, column=0)
        ttk.Button(g3, text="读落盘信息", command=self.nvm_info).grid(row=0, column=1, padx=6)
        self.lbl_nvm = ttk.Label(g3, text="（还没读）", font=FONT_UI)
        self.lbl_nvm.grid(row=0, column=2, padx=10)

        g4 = ttk.LabelFrame(f, text="直接发一帧（排错 / 试新固件用）", padding=6)
        g4.pack(fill="x", pady=6)
        ttk.Label(g4, text="命令码（十六进制）").grid(row=0, column=0, sticky="e")
        self.var_cmd = tk.StringVar(value="00")
        ttk.Entry(g4, textvariable=self.var_cmd, width=8).grid(row=0, column=1, padx=4)
        ttk.Label(g4, text="数据（十六进制）").grid(row=0, column=2, sticky="e")
        self.var_pay = tk.StringVar()
        ttk.Entry(g4, textvariable=self.var_pay, width=60).grid(row=0, column=3, padx=4)
        ttk.Button(g4, text="发送这一帧",
                   command=self.send_raw).grid(row=0, column=4, padx=6)
        ttk.Label(g4, text="十六进制，空格可省，CRC 自动算好。", font=FONT_UI,
                  foreground="#555").grid(row=1, column=0, columnspan=5, sticky="w")

    # ------------------------------------------------------------ 连接管理
    def refresh_ports(self):
        try:
            ports = P.SerialTransport.list_ports()
        except Exception as e:
            messagebox.showerror(APP_TITLE, "枚举串口失败：%s" % e)
            return
        self.cb_port["values"] = ["%s  %s" % (d, s) for d, s in ports]
        if ports:
            self.cb_port.current(0)
        self.log.writeline("[i] 发现 %d 个串口" % len(ports))

    def _port(self):
        s = self.cb_port.get().strip()
        return s.split()[0] if s else None

    def connected(self):
        return self.cli is not None

    def toggle_conn(self):
        if self.connected():
            self.disconnect()
        else:
            self.connect()

    def connect(self):
        port = self._port()
        if not port:
            messagebox.showwarning(APP_TITLE, "先选串口（点「刷新」）")
            return
        if self.app.port_in_use(port, self):
            return
        try:
            self.ser = P.SerialTransport(port, int(self.cb_baud.get()))
        except Exception as e:
            messagebox.showerror(APP_TITLE, "打开串口失败：%s" % e)
            return
        self.cli = P.Client(self.ser, timeout=int(self.var_to.get()) / 1000.0,
                            retries=2, dims=self.dims, log=self._hex_log)
        self.app.set_port_owner(port, self)
        self.btn_conn.configure(text="断开")
        self.lbl_state.configure(text="已连接 %s" % port, foreground="#080")
        self.log.writeline("[i] 打开 %s @ %s" % (port, self.cb_baud.get()))
        run_bg(self._handshake, on_error=lambda tb: self.log.writeline(tb))

    def disconnect(self):
        self.auto_live.set(False)
        if self.ser:
            self.ser.close()
        self.ser = None
        self.cli = None
        self.app.clear_port_owner()
        self.btn_conn.configure(text="连接")
        self.lbl_state.configure(text="未连接", foreground="#a00")
        self.log.writeline("[i] 已断开")

    def _hex_log(self, tag, data):
        if tag == "TX":
            self.log.writeline("TX > " + fmt_hex(data))
        elif tag == "RX":
            self.log.writeline("RX < " + fmt_hex(data))

    # ------------------------------------------------------------ 执行封装
    def _run(self, fn, desc="", then=None):
        if not self.connected():
            messagebox.showwarning(APP_TITLE, "先点「连接」")
            return
        if self.busy:
            self.log.writeline("[!] 上一件事还没做完，等一下")
            return
        self.busy = True
        self.log.writeline("[>] %s" % (desc or "执行"))

        def work():
            try:
                r = fn()
                if r is not None:
                    self._ui(lambda r=r: (self.fill_reply(r), self._stamp()))
            except P.TuneTimeout as e:
                self.log.writeline("[X] 超时：%s" % e)
            except Exception:
                self.log.writeline("[X] 出错：\n" + traceback.format_exc())
            finally:
                self.busy = False
                if then:
                    self._ui(then)
        run_bg(work)

    def _stamp(self):
        st = self.cli.stats if self.cli else {}
        self.log.writeline("[=] 收 %d / 发 %d / 超时 %d / CRC错 %d / 无关帧 %d"
                           % (st.get("rx", 0), st.get("tx", 0), st.get("timeout", 0),
                              st.get("crc_err", 0), st.get("odd", 0)))

    def borrow(self, desc):
        """别的页签要用串口时走这里，返回 `(client, release)`。

        串口句柄和 busy 标志都归本页独占 —— ⑥ EKF 页只是借一次链路来读/写
        参数，不能自己开第二个串口，也不能和本页的后台任务撞在一起。
        拿不到（没连接 / 正忙）就返回 `(None, None)`，调用方自己提示。
        """
        if not self.connected() or self.busy:
            return None, None
        self.busy = True
        self.log.writeline("[>] %s（⑥ EKF 页发起）" % (desc or "执行"))
        return self.cli, self._release

    def _release(self):
        self.busy = False
        self._stamp()

    def fill_reply(self, r):
        for i in self.res.get_children():
            self.res.delete(i)
        cmd = P.COMMANDS.get(r.cmd)
        ups = list(cmd.up) if cmd else []
        self.res.insert("", "end",
                        values=("执行结果", P.RC_NAME.get(r.rc, "rc=%d" % r.rc),
                                "", ""))
        for i, (name, val, text, unit, note) in enumerate(r.fields):
            title = ups[i].title if i < len(ups) else name
            self.res.insert("", "end", values=(title, text, unit, note))
        self.lbl_hex_raw = ("下行 %s\n上行 %s\n%s"
                            % (fmt_hex(r.raw_tx), fmt_hex(r.raw_rx),
                               P.RC_NAME.get(r.rc, "")))
        self._toggle_hex()
        self.log.writeline("[=] %s -> rc=%d %s（%d 次尝试）"
                           % (cmd.title if cmd else "0x%02X" % r.cmd, r.rc,
                              P.RC_NAME.get(r.rc, ""), r.tries))
        if r.ok and r.cmd == P.C_RD_LIVE:
            self.app.show_live(r)          # 「读实时量」也刷底部状态条
        if r.ok:
            self._cache_read(r)

    def _cache_read(self, r):
        """把读到的 SOC / EKF 现值记下来，当作「写」命令的输入框默认值。

        没有这层缓存，WR_EKF 的 8 个框全是空的 —— 而它是整组写，留空根本发
        不出去。顺带解决了「读一次再改一个」这个最常见的操作。
        """
        if r.cmd == P.C_RD_SOC:
            self.soc_state = r.get("soc01")
        elif r.cmd == P.C_RD_EKF:
            self.ekf_vals = [r.get(n) for n in P.EKF_NAMES]
        else:
            return
        # 当前正停在对应的「写」命令上就重画一遍，让默认值立刻变成刚读到的值
        cur = getattr(self, "cur_cmd", None)
        want = {P.C_RD_SOC: P.C_WR_SOC, P.C_RD_EKF: P.C_WR_EKF}.get(r.cmd)
        if cur is not None and cur.code == want:
            self.on_pick()
            self.log.writeline("[i] 输入框默认值已换成刚读到的现值")

    def show_selftest(self):
        bad = P.selftest()
        if bad:
            self.log.writeline("[X] CRC 自检失败：" + "; ".join(bad))
        else:
            self.log.writeline("[i] CRC 自检通过：crc16(\"123456789\")=0x29B1，"
                               "PING 帧 = AA 55 00 00 00 9C CC")

    # ---- 通用命令发送
    def send_current(self):
        c = getattr(self, "cur_cmd", None)
        if c is None:
            return
        try:
            pay = self._payload_for(c)
        except Exception as e:
            messagebox.showerror(APP_TITLE, str(e))
            return
        self._run(lambda: self.cli.request(c.code, pay),
                  desc="%s  下行 %s" % (c.title, fmt_hex(pay)))

    def _payload_for(self, c):
        if c.code == P.C_WR_CAL_ALL:
            if not self.cal_tbls:
                raise ValueError("先「读整包」，改完再写回")
            return self._cal_blob()
        if c.code == P.C_WR_R0_ALL:
            return self._r0_blob()
        if c.code == P.C_WR_SOH_BLOB:
            if self.soh_blob is None:
                raise ValueError("先「读 120 B 参数块」或从文件读入")
            return self.soh_blob
        values = {}
        for name, (f, var) in self.down_vars.items():
            values[name] = var.get()
        return P.encode_down(c, values, self.dims)

    # ---- 握手 / 快捷
    def _handshake(self):
        try:
            r = self.cli.ping()
            self.log.writeline("[i] PING -> rc=%d proto_ver=%s"
                               % (r.rc, r.get("proto_ver")))
            if r.ok and r.get("proto_ver") != P.PROTO_VER:
                self.log.writeline("[!] 固件协议版本 %s ≠ 上位机 %d，别硬着头皮继续"
                                   % (r.get("proto_ver"), P.PROTO_VER))
            r = self.cli.info()
            self.dims = self.cli.dims
            self.dims["temps"] = []
            self.log.writeline(
                "[i] INFO: 温度点 %d / 表 %d / 每表 %d 点 / 整包 %d B / 标称 %s mAh / 掉电保持 %s"
                % (r.get("temp_n"), r.get("tbl_n"), r.get("soc_n"),
                   r.get("cal_bytes"), r.get("cap_nom_mah"),
                   "开" if (r.get("flags") or 0) & 1 else "关"))
            rt = self.cli.rd_cal_temps()
            if rt.ok:
                self.dims["temps"] = rt.get("temps") or []
            self._ui(self._refresh_ti)
            self._ui(self._stamp)
        except P.TuneTimeout as e:
            self.log.writeline("[X] 握手超时：%s" % e)

    def _refresh_ti(self):
        vals = self._ti_list()
        self.cb_ti["values"] = vals
        if vals:
            self.cb_ti.current(0)
        self.load_cal_grid()

    def quick(self, name):
        c = P.COMMANDS_BY_NAME.get(name)
        self._run(lambda: self.cli.request(name),
                  desc=(c.title if c else name))

    # ---- 标定表
    def read_cal_all(self):
        def work():
            r = self.cli.rd_cal_all()
            if not r.ok:
                self.log.writeline("[X] RD_CAL_ALL rc=%d" % r.rc)
                return r
            blob = r.payload[1:]
            n = self.dims["soc_n"]
            self.cal_tbls = {}
            for ti in range(self.dims["temp_n"]):
                for tbl in range(self.dims["tbl_n"]):
                    off = ((ti * self.dims["tbl_n"] + tbl) * n) * 2
                    self.cal_tbls[(ti, tbl)] = list(
                        int(v) for v in _unpack_u16(blob[off:off + n * 2]))
            self.log.writeline("[i] 读到 %d 个温度点 × %d 张表 × %d 点"
                               % (self.dims["temp_n"], self.dims["tbl_n"], n))
            self._ui(self.load_cal_grid)
            return r
        self._run(work, desc="读整张标定表")

    def write_cal_all(self):
        ti = self.cb_ti.current()
        if ti < 0:
            ti = 0
        try:
            got = self._collect_cal_grid(ti)          # 先把界面上的改动收进内存
        except ValueError as e:
            messagebox.showerror(APP_TITLE, str(e))
            return
        for k, row in got.items():
            if any(v is None for v in row):
                messagebox.showerror(APP_TITLE, "标定表有空值，先点「读整包」再改")
                return
            self.cal_tbls[k] = row

        def work():
            blob = self._cal_blob()
            r = self.cli.wr_cal_all(blob)
            back = self.cli.rd_cal_all()
            same = bytes(back.payload[1:]) == bytes(blob)
            self.log.writeline("[i] 写回 %d B，读回校验：%s"
                               % (len(blob), "一致" if same else "不一致！"))
            self._ui(self.load_cal_grid)
            return r
        self._run(work, desc="写回整张标定表（写完读回校验）")

    def _cal_blob(self):
        n = self.dims["soc_n"]
        out = bytearray()
        for ti in range(self.dims["temp_n"]):
            for tbl in range(self.dims["tbl_n"]):
                row = self.cal_tbls.get((ti, tbl))
                if row is None:
                    raise ValueError("温度点 %d 表 %d 还没读到，先「读整包」" % (ti, tbl))
                out += _pack_u16(row)
        return bytes(out)

    def cal_restore(self):
        if not messagebox.askyesno(APP_TITLE, "恢复出厂标定表？当前改动会丢。"):
            return
        self._run(lambda: self.cli.cal_restore(), desc="标定表恢复出厂")

    def read_active(self):
        def work():
            r = self.cli.rd_cal_active()
            if not r.ok:
                return r
            blob = r.payload[1:]
            n = self.dims["soc_n"]

            def show():
                for tbl in range(self.dims["tbl_n"]):
                    off = tbl * n * 2
                    vals = _unpack_u16(blob[off:off + n * 2])
                    for j in range(n):
                        self.act_labels[(tbl, j)].set(str(vals[j]))
            self._ui(show)
            return r
        self._run(work, desc="读当前生效的表")

    def export_cal(self):
        if not self.cal_tbls:
            messagebox.showwarning(APP_TITLE, "先读整包")
            return
        p = filedialog.asksaveasfilename(title="导出标定表", defaultextension=".json",
                                         filetypes=[("JSON", "*.json")])
        if not p:
            return
        import json
        data = {"dims": {k: self.dims.get(k) for k in
                         ("temp_n", "tbl_n", "soc_n", "cal_bytes")},
                "temps_dc": self.dims.get("temps"),
                "cal": {"%d,%d" % k: v for k, v in self.cal_tbls.items()}}
        with open(p, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=1)
        self.log.writeline("[i] 已导出 %s" % p)

    def import_cal(self):
        p = filedialog.askopenfilename(title="导入标定表", filetypes=[("JSON", "*.json")])
        if not p:
            return
        import json
        with open(p, encoding="utf-8") as f:
            data = json.load(f)
        for k, v in data["cal"].items():
            ti, tbl = (int(x) for x in k.split(","))
            self.cal_tbls[(ti, tbl)] = [int(x) for x in v]
        self.load_cal_grid()
        self.log.writeline("[i] 已导入 %s（记得点「写回整包」）" % p)

    # ---- R0
    def read_r0_all(self):
        def work():
            r = self.cli.rd_r0_all()
            if r.ok:
                vals = list(_unpack_u16(r.payload[1:]))

                def show():
                    for j, v in enumerate(vals[:11]):
                        self.r0_vars[j].set(str(v))
                self._ui(show)
            return r
        self._run(work, desc="读全部内阻")

    def write_r0_all(self):
        try:
            vals = [int(float(self.r0_vars[j].get())) for j in range(11)]
        except ValueError:
            messagebox.showerror(APP_TITLE, "R0 每格都要填整数 mΩ")
            return

        def work():
            r = self.cli.wr_r0_all(_pack_u16(vals))
            back = self.cli.rd_r0_all()
            self.log.writeline("[i] 写回后读回：%s" % (list(_unpack_u16(back.payload[1:])),))
            return r
        self._run(work, desc="写回全部内阻")

    def read_r0_one(self):
        idx = int(self.var_r0_idx.get())
        self._run(lambda: self.cli.rd_r0(idx), desc="读内阻（第 %d 点）" % idx)

    def write_r0_one(self):
        idx = int(self.var_r0_idx.get())
        val = int(float(self.r0_vars[idx].get()))
        self._run(lambda: self.cli.wr_r0(idx, val),
                  desc="写内阻（第 %d 点 = %d mΩ）" % (idx, val))

    # ---- SOH
    def read_soh_sum(self):
        def work():
            r = self.cli.rd_soh_sum()
            if r.ok:
                txt = ("SOH = %.2f %%   容量保持 %s %%   内阻指标 %s %%\n"
                       "学习容量 = %s mAh   R0 均值 = %s mΩ   温度 = %.1f °C\n"
                       "有效标志 = 0x%02X（bit0 容量有效 / bit1 内阻有效）"
                       % ((r.get("soh01") or 0) / 100.0, r.get("cap_pct"),
                          r.get("r_pct"), r.get("q_mah"), r.get("r0_mohm"),
                          (r.get("t") or 0) / 10.0, r.get("valid") or 0))
                self._ui(lambda: self.lbl_soh.configure(text=txt))
            return r
        self._run(work, desc="读 SOH 摘要")

    def read_soh_blob(self):
        def work():
            r = self.cli.rd_soh_blob()
            if r.ok:
                blob = bytes(r.payload[1:])
                self.soh_blob = blob
                self._ui(lambda: self._show_blob(blob))
            return r
        self._run(work, desc="读 SOH 参数块")

    def _show_blob(self, blob):
        d = P.soh_blob_decode(blob)
        for i in self.soh_tree.get_children():
            self.soh_tree.delete(i)
        rows = [("cap_mah 学到的容量", "%d mAh" % d["cap_mah"], ""),
                ("r0[0..10] 学到的内阻", " ".join(str(x) for x in d["r0"]), "mΩ"),
                ("base[0..10] 基线快照", " ".join(str(x) for x in d["base"]),
                 "SOH_R 的分母"),
                ("cnt[0..10] 样本数", " ".join(str(x) for x in d["cnt"]),
                 "续接自适应 α"),
                ("q_n 容量学习次数", str(d["q_n"]), ""),
                ("r0_any", str(d["r0_any"]), "R0 至少 1 个有效样本"),
                ("q_any", str(d["q_any"]), "容量至少学到 1 次"),
                ("temp_dc 落盘温度", "%.1f °C" % (d["temp_dc"] / 10.0), ""),
                ("kf_p 卡尔曼协方差", "%.6g" % d["kf_p"], "续接滤波状态"),
                ("—— ver 3 追加 ——", "", "老版本（ver 2）的块这里全是 0"),
                ("r0_chg[0..10] 充电方向 R0",
                 " ".join(str(x) for x in d["r0_chg"]),
                 "mΩ；与放电方向各自学习，互不污染"),
                ("cnt_chg[0..10] 充电样本数",
                 " ".join(str(x) for x in d["cnt_chg"]), "各自收敛自适应 α"),
                ("r0_chg_any / cnt_any",
                 "%d / %d" % (d["r0_chg_any"], d["cnt_any"]),
                 "充电方向 R0 有效 / 计数字段有效"),
                ("cum_chg_mah 累计充入", "%d mAh" % d["cum_chg_mah"], ""),
                ("cum_dis_mah 累计放出", "%d mAh" % d["cum_dis_mah"], ""),
                ("等效满循环",
                 "%.3f 个" % (d["cum_dis_mah"] / max(1, d["cap_mah"])),
                 "= 累计放出 / 学到的容量，与放电深度无关"),
                ("half_cycle 半循环数", str(d["half_cycle"]),
                 "摆幅法；上下各走一趟 ≈ 1 个满循环")]
        for r in rows:
            self.soh_tree.insert("", "end", values=r)

    def write_soh_blob(self):
        if self.soh_blob is None:
            messagebox.showwarning(APP_TITLE, "先读参数块或从文件读入")
            return
        if not messagebox.askyesno(APP_TITLE, "把当前 120 B 参数块写进板子？"):
            return
        self._run(lambda: self.cli.wr_soh_blob(self.soh_blob),
                  desc="写 SOH 参数块")

    def export_soh(self):
        if self.soh_blob is None:
            messagebox.showwarning(APP_TITLE, "先读参数块")
            return
        p = filedialog.asksaveasfilename(title="导出 SOH 块", defaultextension=".bin",
                                         filetypes=[("BIN", "*.bin")])
        if p:
            with open(p, "wb") as f:
                f.write(self.soh_blob)
            self.log.writeline("[i] 已导出 %s（%d B）" % (p, len(self.soh_blob)))

    def import_soh(self):
        p = filedialog.askopenfilename(title="读入 SOH 块",
                                       filetypes=[("BIN", "*.bin"), ("所有文件", "*.*")])
        if not p:
            return
        with open(p, "rb") as f:
            blob = f.read()
        if len(blob) < P.SOH_BLOB_BYTES:
            messagebox.showerror(APP_TITLE, "文件只有 %d B，要 %d B"
                                 % (len(blob), P.SOH_BLOB_BYTES))
            return
        self.soh_blob = blob[:P.SOH_BLOB_BYTES]
        self._show_blob(self.soh_blob)
        self.log.writeline("[i] 读入 %s（点「写回 120 B」下发）" % p)

    def read_soh_cnt(self):
        def work():
            r = self.cli.rd_soh_cnt()
            if r.ok:
                cyc = (r.get("cycle_milli") or 0) / 1000.0
                txt = ("累计充入 %s mAh    累计放出 %s mAh\n"
                       "半循环 %s 个    等效满循环 %.3f 个    计数有效 = %s"
                       % (r.get("cum_chg_mah"), r.get("cum_dis_mah"),
                          r.get("half_cycle"), cyc,
                          "是" if r.get("cnt_any") else "否（老数据没有这几个字段）"))
                self._ui(lambda: self.lbl_soh.configure(text=txt))
            return r
        self._run(work, desc="读累计充放电与循环次数")

    def read_corr(self):
        def work():
            r = self.cli.rd_corr()
            if r.ok:
                k = (r.get("corr_ppm") or 0) / 10000.0
                tail = ("   ← 正好 100%，说明温度 / 倍率修正都还没标定"
                        if abs(k - 1.0) < 1e-6 else "")
                txt = ("折算系数 K = %.4f（%.2f%%）    折算后可用容量 = %s mAh    "
                       "参考工况 %.1f °C / %s mA%s"
                       % (k, k * 100.0, r.get("eff_cap_mah"),
                          (r.get("ref_temp_dc") or 0) / 10.0,
                          r.get("ref_cur_ma"), tail))
                self._ui(lambda: self.lbl_corr.configure(text=txt))
            return r
        self._run(work, desc="读工况折算系数")

    def soh_cnt_reset(self):
        if not messagebox.askyesno(
                APP_TITLE, "清掉累计充放电与循环次数？\n"
                           "学到的容量和内阻不动，只把寿命计数归零。"):
            return
        self._run(lambda: self.cli.soh_cnt_reset(), desc="清空累计与循环")

    def soh_reset(self):
        if not messagebox.askyesno(
                APP_TITLE, "清掉学到的容量和内阻？之后要重新学几个小时。"):
            return
        self._run(lambda: self.cli.soh_reset(), desc="清空学习值")

    # ---- 容量 / 温度 / 落盘
    def read_cap(self):
        def work():
            r = self.cli.rd_cap()
            if r.ok:
                self._ui(lambda: self.var_cap.set(str(r.get("mah"))))
            return r
        self._run(work, desc="读学到的容量")

    def write_cap(self):
        try:
            v = int(float(self.var_cap.get()))
        except ValueError:
            messagebox.showerror(APP_TITLE, "容量要填整数 mAh")
            return

        def work():
            r = self.cli.wr_cap(v)
            back = self.cli.rd_cap()
            self._ui(lambda: self.var_cap.set(str(back.get("mah"))))
            if back.get("mah") != v:
                self.log.writeline("[!] 写 %d，读回 %s —— 被库钳位了（正常行为）"
                                   % (v, back.get("mah")))
            return r
        self._run(work, desc="写容量 %d mAh（写完读回）" % v)

    def read_temp(self):
        def work():
            r = self.cli.rd_temp()
            if r.ok:
                t = r.get("t")
                self._ui(lambda: self.var_temp.set("%.1f" % (t / 10.0)))
            return r
        self._run(work, desc="读当前温度")

    def write_temp(self):
        try:
            t = int(round(float(self.var_temp.get()) * 10))
        except ValueError:
            messagebox.showerror(APP_TITLE, "温度要填数字（°C）")
            return
        self._run(lambda: self.cli.wr_temp(t), desc="强制设置温度 %.1f °C" % (t / 10.0))

    def nvm_save(self):
        self._run(lambda: self.cli.nvm_save(), desc="立即存 Flash")

    def nvm_info(self):
        def work():
            r = self.cli.nvm_info()
            if r.ok:
                txt = ("落盘次数 %s / 当前槽 %s / 序号 %s"
                       % (r.get("count"), r.get("slot"), r.get("seq")))
                self._ui(lambda: self.lbl_nvm.configure(text=txt))
            return r
        self._run(work, desc="读保存状态")

    def send_raw(self):
        try:
            cmd = int(self.var_cmd.get().strip(), 16)
            hx = self.var_pay.get().replace(" ", "").replace(",", "")
            pay = bytes.fromhex(hx) if hx else b""
        except Exception as e:
            messagebox.showerror(APP_TITLE, "命令码/hex 填错了：%s" % e)
            return
        self._run(lambda: self.cli.request(cmd, pay),
                  desc="原始帧 0x%02X %s" % (cmd, fmt_hex(pay)))

    # ---- 自动刷新实时量（每秒一次，不重入）
    def _auto_tick(self):
        if not self.auto_live.get():
            return
        if not self.connected():
            self.auto_live.set(False)
            return
        if self.auto_busy or self.busy:
            self.after(300, self._auto_tick)      # 让位给手动命令，稍后再试
            return
        self.auto_busy = True

        def work():
            try:
                r = self.cli.rd_live()
                self._ui(lambda: self.app.show_live(r))
            except P.TuneTimeout:
                pass                              # 丢一帧不要紧，下一拍再来
            except Exception:
                self.log.writeline(traceback.format_exc())
            finally:
                self._ui(self._auto_done)
        run_bg(work)

    def _auto_done(self):
        self.auto_busy = False
        if self.auto_live.get() and self.connected():
            self.after(1000, self._auto_tick)


def _unpack_u16(b):
    import struct
    n = len(b) // 2
    return struct.unpack_from("<%dH" % n, b, 0)


def _pack_u16(vals):
    import struct
    return struct.pack("<%dH" % len(vals), *[int(v) for v in vals])


# ==========================================================================
# 控载 / 分析 / SOH 验证 / EKF
# ==========================================================================

# 串口下拉的第一项 = "交给脚本自己探"（留空等价，脚本侧绝不写死某个 COM）
PORT_AUTO = "（自动探测）"


class Form(ttk.Frame):
    """一个简易参数表单：fields = [(key, 标签, 控件类型, 默认, 提示)]。

    控件类型：entry 单行 | combo 下拉（只读）| port 串口下拉（带"刷新"按钮，也能直接
    手输 COMx）| check 勾选框（用 hint 当勾选框的文字）。port 型要传 on_port_refresh。
    """

    def __init__(self, master, fields, cols=4, on_port_refresh=None, **kw):
        super().__init__(master, **kw)
        self.vars = {}
        self.labels = {}
        self.cells = {}
        self.combos = {}
        for i, (key, label, kind, default, hint) in enumerate(fields):
            r, c = divmod(i, cols)
            cell = ttk.Frame(self)
            cell.grid(row=r, column=c, sticky="we", padx=6, pady=3)
            lb = ttk.Label(cell, text=label, font=FONT_UI)
            lb.pack(anchor="w")
            v = tk.StringVar(value=str(default))
            if kind == "combo":
                w = ttk.Combobox(cell, textvariable=v, width=14, values=hint,
                                 state="readonly")
                w.pack(fill="x")
                self.combos[key] = w
            elif kind == "port":
                row = ttk.Frame(cell)
                row.pack(fill="x")
                w = ttk.Combobox(row, textvariable=v, width=13, values=[PORT_AUTO])
                w.pack(side="left", fill="x", expand=True)
                if on_port_refresh is not None:
                    ttk.Button(row, text="刷新", width=5,
                               command=lambda k=key: on_port_refresh(k)).pack(
                                   side="left", padx=(3, 0))
                self.combos[key] = w
            elif kind == "check":
                w = ttk.Checkbutton(cell, text=hint or "", variable=v,
                                    onvalue="yes", offvalue="no")
                w.pack(anchor="w")
                hint = ""              # 勾选框自带文字，不再单列提示行
            else:
                w = ttk.Entry(cell, textvariable=v, width=16)
                w.pack(fill="x")
            if hint and kind not in ("combo", "check"):
                ttk.Label(cell, text=hint, font=("Microsoft YaHei UI", 8),
                          foreground="#666").pack(anchor="w")
            self.vars[key] = v
            self.labels[key] = lb
            self.cells[key] = cell

    def get(self, key):
        return self.vars[key].get().strip()

    def set(self, key, val):
        self.vars[key].set(str(val))

    def set_label(self, key, text):
        """改标签文字（同一字段在不同模式下含义不同时用，例如"本机串口"）。"""
        self.labels[key].configure(text=text)

    def label_text(self, key):
        return self.labels[key].cget("text")



# ---- 模式表: 与 soc_load.py 的 MODES / soc_bench.py 的 CYCLE_* 逐项对应（改那边要一起改）----
# 前两项是"一次只跑一个方向"，第三项交给 soc_bench.py cycle 逐腿调度。三个模式共用同一套
# soc_load 保护；check_consts.py [6] 对拍前两项的默认值, [11] 对拍循环那几项。
MODE_LOAD, MODE_CHARGE, MODE_CYCLE = (
    "拉载（电子负载）", "充电（可编程电源）", "循环（多轮充放）")
DIR_CHOICES = [MODE_LOAD, MODE_CHARGE, MODE_CYCLE]
DIR_DEFAULTS = {
    DIR_CHOICES[0]: {"init_soc": "100", "to_soc": "9",
                     "current": "2.0", "vlimit": "2.8", "vset": "4.20"},
    DIR_CHOICES[1]: {"init_soc": "9", "to_soc": "80",
                     "current": "1.6", "vlimit": "4.15", "vset": "4.20"},
}
DIR_NAMES = {MODE_LOAD: "拉载", MODE_CHARGE: "充电", MODE_CYCLE: "多轮循环"}
DIR_DEVS = {MODE_LOAD: "电子负载", MODE_CHARGE: "可编程电源", MODE_CYCLE: "当前那台设备"}
MODE_TIPS = {
    MODE_LOAD: "一次跑一个方向：控电子负载，按档位「恒流放 -> 断开静置」走到终止 SOC。",
    MODE_CHARGE: "一次跑一个方向：控可编程电源，按档位「恒流充 -> 断开静置」走到终止 SOC。",
    MODE_CYCLE: "多轮循环：交给 soc_bench.py cycle 逐腿调度，放 / 充交替多轮。两个串口都要"
                "填（放电口 / 充电口），每腿开跑前先关另一台并复核电流（跨设备互锁）。",
}

# 循环默认值 —— 数值必须与 soc_bench.CYCLE_* 一致, check_consts.py [11] 逐项对拍
CYC_DEFAULTS = {
    "cycles": "3", "soc_lo": "9", "soc_hi": "80", "start_soc": "100",
    "cur_load": "2.0", "cur_chg": "1.6",
    "vlim_load": "2.8", "vlim_chg": "4.15", "vset_cyc": "4.20",
    "prefix": "cyc",
}


class LoadTab(ttk.Frame):
    """③ 控载页：拉载 / 充电 / 循环 三个模式共用一套参数区与保护。

    单腿（拉载 / 充电）走 soc_load.py；循环走 soc_bench.py cycle，每一腿仍是一次独立的
    soc_load 运行（见 soc_bench.cmd_cycle 的 docstring），所以保护与产物格式各处一致。
    """

    tab_name = "控载"

    def __init__(self, master, app):
        super().__init__(master, padding=8)
        self.app = app
        self.worker = None
        self._probing = False
        self._build()

    # ---------------- 界面 ----------------
    def _build(self):
        nav_label(self, NAV_TEXT["load"]).pack(anchor="w", pady=(0, 4))
        HelpLine(self, "控负载 / 电源自动跑 N 档「恒流充放 → 断开静置」，"
                       "或交给 cycle 跑多轮；安全保护 6 项",
                 HELP["load"]).pack(anchor="w", pady=(0, 4))

        self.tip = tk.StringVar(value=MODE_TIPS[MODE_LOAD])
        ttk.Label(self, textvariable=self.tip, font=FONT_UI, wraplength=980,
                  justify="left", foreground="#2a7").pack(anchor="w", pady=(0, 6))

        self.form = Form(self, [
            ("mode", "模式", "combo", MODE_LOAD, DIR_CHOICES),
            ("port", "本机串口", "port", PORT_AUTO, "留空=自动探测；也可下拉选或手输 COMx"),
            ("peer_port", "另一台串口", "port", PORT_AUTO, "互锁：跑前先关它（可留空）"),
            ("capacity", "电池容量 mAh", "entry", "3350", ""),
            ("segments", "档位数", "entry", "10", ""),
            ("rest_s", "档间静置 s", "entry", "480", ""),
        ], cols=4, on_port_refresh=self.refresh_ports)
        self.form.pack(fill="x")
        self.form.vars["mode"].trace_add("write", self._on_mode_change)

        # 单腿参数与循环参数只显示一份，切换见 _apply_mode()
        self.lf_single = ttk.LabelFrame(self, text="单腿参数（一次一个方向）", padding=6)
        self.lf_single.pack(fill="x", pady=(6, 0))
        self.form_single = Form(self.lf_single, [
            ("init_soc", "起始 SOC %", "entry", "100", ""),
            ("to_soc", "终止 SOC %", "entry", "9", ""),
            ("current", "每档电流 A", "entry", "2.0", "可写 0.5:3 递变"),
            ("spacing", "档位分布", "combo", "ocv", ["ocv", "soc"]),
            ("vlimit", "端压门限 V", "entry", "2.8", "放电=底线/充电=上限"),
            ("vset", "电源 CV V", "entry", "4.20", "仅充电：门限要低于它"),
            ("demo", "空载联调", "combo", "no", ["no", "yes"]),
            ("pulse_s", "联调脉冲 s", "entry", "1", "仅联调用"),
            ("cmd", "命令方言覆盖", "entry", "", "留空=通用 SCPI；分号隔开多条"),
        ], cols=4)
        self.form_single.pack(fill="x")

        self.lf_cycle = ttk.LabelFrame(
            self, text="循环参数（模式选「循环」时生效）", padding=6)
        self.form_cycle = Form(self.lf_cycle, [
            ("cycles", "轮数", "entry", CYC_DEFAULTS["cycles"], "每轮 = 一放一充"),
            ("soc_lo", "窗口下沿 %", "entry", CYC_DEFAULTS["soc_lo"], ""),
            ("soc_hi", "窗口上沿 %", "entry", CYC_DEFAULTS["soc_hi"], ""),
            ("start_soc", "首腿起始 %", "entry", CYC_DEFAULTS["start_soc"], ""),
            ("cur_load", "放电腿电流 A", "entry", CYC_DEFAULTS["cur_load"], ""),
            ("cur_chg", "充电腿电流 A", "entry", CYC_DEFAULTS["cur_chg"], ""),
            ("vlim_load", "放电端压底线 V", "entry", CYC_DEFAULTS["vlim_load"], ""),
            ("vlim_chg", "充电端压上限 V", "entry", CYC_DEFAULTS["vlim_chg"], "要低于 CV"),
            ("vset_cyc", "电源 CV V", "entry", CYC_DEFAULTS["vset_cyc"], ""),
            ("prefix", "产物前缀", "entry", CYC_DEFAULTS["prefix"], "-> cyc_c1_dis/c1_chg"),
            ("cmd_load", "放电腿方言", "entry", "", "留空=通用 SCPI"),
            ("cmd_chg", "充电腿方言", "entry", "", "留空=通用 SCPI"),
            ("keep_going", "跑不满时", "check", "no", "接着跑（默认：停下）"),
            ("dry_run", "先预览", "check", "no", "只列腿序，不碰设备"),
        ], cols=4)
        self.form_cycle.pack(fill="x")

        self.bar = ttk.Frame(self)
        self.bar.pack(fill="x", pady=6)
        self.btn = ttk.Button(self.bar, text="▸ 开始拉载", command=self.start)
        self.btn.pack(side="left")
        self.btn_stop = ttk.Button(self.bar, text="■ 紧急停止（断开电子负载）",
                                   command=self.stop, state="disabled")
        self.btn_stop.pack(side="left", padx=6)
        ttk.Button(self.bar, text="识别设备（*IDN?）",
                   command=self.probe_ports).pack(side="left", padx=(2, 8))
        ttk.Button(self.bar, text="命令方言",
                   command=self._cmd_help).pack(side="left", padx=2)

        ttk.Label(self, text="运行日志", font=FONT_UI).pack(anchor="w")
        self.log = LogPane(self, height=18)
        self.log.pack(fill="both", expand=True)
        self._apply_mode()

    # ---------------- 模式 ----------------
    def mode(self):
        return self.form.get("mode")

    def is_charge(self):
        return self.mode() == MODE_CHARGE

    def is_cycle(self):
        return self.mode() == MODE_CYCLE

    def _on_mode_change(self, *_a):
        """切模式：把"还停在另一个模式默认值上"的框换成新模式的默认值，再刷界面。

        只覆盖默认值、不动手工填的数 —— 否则用户填了一半再切模式，输入就白填了。
        """
        cur = self.mode()
        if cur in DIR_DEFAULTS:
            other = DIR_DEFAULTS[DIR_CHOICES[0] if cur == DIR_CHOICES[1]
                                 else DIR_CHOICES[1]]
            for k, v in DIR_DEFAULTS[cur].items():
                if self.form_single.get(k) in ("", other[k]):
                    self.form_single.set(k, v)
        self._apply_mode()

    def _apply_mode(self):
        """按模式换文案、按钮和参数区（单腿参数 / 循环参数 只留一个）。"""
        cur, cyc = self.mode(), self.is_cycle()
        self.tip.set(MODE_TIPS.get(cur, ""))
        self.form.set_label("port", "放电口（负载）" if cyc else "本机串口")
        self.form.set_label("peer_port", "充电口（电源）" if cyc else "另一台串口")
        self.btn.configure(text="▸ 开始%s" % DIR_NAMES.get(cur, "控载"))
        self.btn_stop.configure(
            text="■ 紧急停止（断开%s）" % DIR_DEVS.get(cur, "当前设备"))
        # pack 默认追加到末尾，所以重新显示时要 before=按钮条，否则会掉到日志下面
        if cyc:
            self.lf_single.pack_forget()
            self.lf_cycle.pack(fill="x", pady=(6, 0), before=self.bar)
        else:
            self.lf_cycle.pack_forget()
            self.lf_single.pack(fill="x", pady=(6, 0), before=self.bar)

    # ---------------- 串口 ----------------
    def port_of(self, key):
        """表单里的串口 -> 干净的 COM 号；"自动探测"/留空 -> ""（不传 --port，交给脚本探）。

        下拉项形如 "COM9  ★ ITECH,..."，取第一段就是口名；脚本侧绝不写死某个 COM。
        """
        s = self.form.get(key) or ""
        if not s or s == PORT_AUTO:
            return ""
        return s.split()[0].upper()

    def _fill_ports(self, idns=None):
        """把串口名单填进两个下拉（idns 里的口标 ★ 并显示设备身份）。返回枚举结果。"""
        try:
            ports = P.SerialTransport.list_ports()
        except Exception as e:
            self.log.writeline("[X] 枚举串口失败：%s" % e)
            return None
        idns = idns or {}
        vals = [PORT_AUTO]
        for d, s in ports:
            vals.append("%s  ★ %s" % (d, idns[d]) if d in idns
                        else "%s  %s" % (d, s))
        for k in ("port", "peer_port"):
            cb = self.form.combos.get(k)
            if cb is not None:
                cb["values"] = vals
        return ports

    def refresh_ports(self, key=None):
        """刷新串口下拉（只枚举，很快）。挑不到就留着「自动探测」让脚本自己探。"""
        ports = self._fill_ports()
        if ports is None:
            return
        self.log.writeline("[i] 刷新串口%s：%d 个 %s"
                           % ("（%s）" % self.form.label_text(key) if key else "",
                              len(ports), ", ".join(d for d, _ in ports) or "无"))

    def probe_ports(self):
        """逐个串口发 *IDN?，认认哪台设备接在哪个口上（只读，不改设备状态）。

        每口最多等 0.3~0.8 s，放后台线程；认出来就把身份写进下拉项 —— 选口时一眼能分辨
        哪台是负载、哪台是电源，不用去设备管理器猜。
        """
        if self._probing:
            self.log.writeline("[i] 正在识别，稍等…")
            return
        self._probing = True
        self.log.writeline("[i] 识别串口上的程控设备（逐个发 *IDN?，只读不改状态）…")

        def work():
            try:
                found = ScriptRunner.load("soc_load").scan_load_port()
            except Exception as e:
                self.log.writeline("[X] 识别失败：%s" % e)
                return
            finally:
                self._probing = False
            if not found:
                self.log.writeline("[!] 没有口应答 *IDN? —— 确认源/载已上电、RS232 已插好、"
                                   "且没被别的程序占用")
                return
            for dev, idn in found:
                self.log.writeline("    ★ %-6s %s" % (dev, idn))
            self._fill_ports(dict(found))
            self.log.writeline("[i] 已把身份写进两个串口下拉（★ = 会应答 *IDN?）。"
                               "认出两台就按负载 / 电源分别选；只认出一台也能先选它。")

        run_bg(work, on_error=lambda tb: self.log.writeline(tb))

    def _cmd_help(self):
        """命令方言说明：写清各键和它们的默认值，换品牌时照着改。"""
        try:
            _sl = ScriptRunner.load("soc_load")      # 顺带把 tools/ 加进 sys.path
        except Exception as e:
            messagebox.showerror(APP_TITLE, "读不到 soc_load.py：\n%s" % e)
            return
        # 表里补齐充电专有两条（import 后 CMDS 装的是当前方向, 可能没有 vset/volt_q）
        base = dict(_sl.CMDS)
        base.update(_sl.MODES["charge"]["cmds"])
        keys = ["curr", "curr_q", "on", "off", "volt", "amp"]
        if self.is_charge() or self.is_cycle():
            keys += ["vset", "volt_q"]
        rows = [(k, base.get(k, "?")) for k in keys]
        where = ("下方「放电腿方言」「充电腿方言」" if self.is_cycle()
                 else "上方「命令方言覆盖」")
        messagebox.showinfo(APP_TITLE, (
            "本页默认用通用 SCPI 根命令写法（多数国产源/载都是这套）：\n\n"
            + "\n".join("  %-8s %s" % r for r in rows) + "\n\n"
            "换别的品牌，如果命令写法不同，填在" + where + "，分号隔开多条，例如：\n\n"
            "  on=LOAD:STATe 1;off=LOAD:STATe 0     （换开关命令）\n"
            "  vset=SOUR:VOLT {v}                   （充电：换恒压设定）\n\n"
            "只填要改的那几条，没填的用默认；curr / vset 里必须留 {i} / {v} "
            "当数值的位置。\n填错会直接报错退出，不会静默忽略。\n\n"
            "循环模式下两条腿的设备可能是两个牌子，所以分成「放电腿 / 充电腿」两栏；"
            "每腿的方言只作用于那一腿自己的设备（互锁那条断开命令自动跟着走）。"))

    # ---------------- 确认 ----------------
    def _confirm(self):
        if self.is_cycle():
            return self._confirm_cycle()
        f, fs = self.form, self.form_single
        ch = self.is_charge()
        nm = DIR_NAMES[f.get("mode")]
        d = DIR_DEFAULTS[f.get("mode")]
        vm = "端压上限" if ch else "端压底线"
        # 先拼好整条模板再一次性 %, 不要写成 "A%s" + cond + "B%s" % (...) ——
        # % 比 + 先算, 那样只有最后一段吃得到参数(踩过一次)。
        warn = ("· 电池已放到起始 SOC 附近并静置 >=30 min\n"
                "· 电源 CV 必须高于端压门限（默认 4.20 > 4.15）\n" if ch else
                "· 电池已充满 4.20 V 并静置 >=30 min\n")
        # 只在填了「另一台串口」时提示: 单台设备跑的时候不需要这一行, 免得成噪音
        _peer = self.port_of("peer_port")
        pline = ("· 另一台设备（%s）已上电：脚本会先关它并复核电流\n" % _peer
                 if _peer else "")
        tpl = ("确认开始真实%s？\n\n"
               "方向：%s（%s）\n"
               "起始 %s%% -> 末点 %s%%\n"
               "档位 %s 档    每档 %s A    档间静置 %s s\n"
               "%s %s V    标称容量 %s mAh\n\n"
               "接线确认：\n"
               "· 充/放电流必须流经 INA226 的分流电阻\n"
               "· 两条 USB 都插上（MCU 数据口 + 源/载 RS232）\n"
               + warn + pline +
               "· 【采集监视】页已点「开始采集」\n\n"
               "点「是」立刻开始%s。中止请用本页的「紧急停止」。")
        return messagebox.askyesno(APP_TITLE, tpl % (
            nm, nm, DIR_DEVS[f.get("mode")],
            fs.get("init_soc") or d["init_soc"], fs.get("to_soc") or d["to_soc"],
            f.get("segments") or "10", fs.get("current") or d["current"],
            f.get("rest_s") or "480", vm, fs.get("vlimit") or d["vlimit"],
            f.get("capacity") or "3350", nm))

    def _confirm_cycle(self):
        """循环的确认框：把腿序、两台设备、两个口和安全前提一次讲全。"""
        f, fc = self.form, self.form_cycle
        pl, pp = self.port_of("port"), self.port_of("peer_port")
        try:
            legs_n = int(float(fc.get("cycles") or CYC_DEFAULTS["cycles"])) * 2
        except ValueError:
            legs_n = "?"
        warn = ("· 电池已充满 4.20 V 并静置 >=30 min（首腿从窗口上沿往下放）\n"
                if fc.get("start_soc") == "100" else
                "· 电池实际 SOC 与「首腿起始」一致，且已静置 >=30 min\n")
        tpl = ("确认开始多轮充放循环？\n\n"
               "轮数：%s 轮（每轮 一放一充，共 %s 腿）\n"
               "窗口：%s%% <-> %s%%    首腿起始 %s%%\n"
               "每腿 %s 档    档间静置 %s s    标称容量 %s mAh\n"
               "放电腿 %s A / 门限 %s V  ->  %s\n"
               "充电腿 %s A / 门限 %s V  ->  %s（电源 CV %s V）\n"
               "具体腿序与预计时长会在日志开头再列一遍\n\n"
               "接线确认：\n"
               "· 电子负载与可编程电源并接在同一节电池上，两台各占一个串口\n"
               "· 每腿开跑前脚本会先关另一台并复核电流；确认不了就停下，不会先开自己\n"
               "· 充/放电流必须流经 INA226 的分流电阻\n"
               "· 两条 USB 都插上（MCU 数据口 + 源/载 RS232）\n"
               + warn +
               "· 【采集监视】页已点「开始采集」（循环要全程连续录）\n\n"
               "点「是」开始。中止请用本页的「紧急停止」（只中止当前那一腿）。")
        return messagebox.askyesno(APP_TITLE, tpl % (
            fc.get("cycles") or CYC_DEFAULTS["cycles"], legs_n,
            fc.get("soc_lo") or CYC_DEFAULTS["soc_lo"],
            fc.get("soc_hi") or CYC_DEFAULTS["soc_hi"],
            fc.get("start_soc") or CYC_DEFAULTS["start_soc"],
            f.get("segments") or "10", f.get("rest_s") or "480",
            f.get("capacity") or "3350",
            fc.get("cur_load") or CYC_DEFAULTS["cur_load"],
            fc.get("vlim_load") or CYC_DEFAULTS["vlim_load"], pl or "自动探测",
            fc.get("cur_chg") or CYC_DEFAULTS["cur_chg"],
            fc.get("vlim_chg") or CYC_DEFAULTS["vlim_chg"], pp or "自动探测",
            fc.get("vset_cyc") or CYC_DEFAULTS["vset_cyc"]))

    # ---------------- 命令行 ----------------
    def _args(self):
        """表单 -> 命令行。单腿走 soc_load.py，循环走 soc_bench.py cycle。"""
        return self._args_cycle() if self.is_cycle() else self._args_single()

    def _args_single(self):
        args = []
        f, fs = self.form, self.form_single
        ch = self.is_charge()
        if ch:
            args.append("--charge")
        if self.port_of("port"):
            args += ["--port", self.port_of("port")]
        if self.port_of("peer_port"):
            # 互锁: 让 soc_load 开跑前先把另一台设备关掉并复核电流
            args += ["--peer-port", self.port_of("peer_port")]
        for k, a in (("capacity", "--capacity"), ("init_soc", "--init-soc"),
                     ("to_soc", "--to-soc"), ("segments", "--segments"),
                     ("current", "--current"), ("rest_s", "--rest-s"),
                     ("vlimit", "--vlimit")):
            v = f.get(k) if k in f.vars else fs.get(k)
            if v:
                args += [a, v]
        # --vset 只对充电有意义; 放电方向传了会让 soc_load 报"不是充电方向"
        if ch and fs.get("vset"):
            args += ["--vset", fs.get("vset")]
        if fs.get("spacing") and fs.get("spacing") != "ocv":
            args += ["--spacing", fs.get("spacing")]
        if fs.get("demo") == "yes":
            args += ["--demo"]
            if fs.get("pulse_s"):
                args += ["--pulse-s", fs.get("pulse_s")]
        for one in (fs.get("cmd") or "").replace("\n", ";").split(";"):
            if one.strip():
                args += ["--cmd", one.strip()]
        args.append("--yes")
        return args

    def _args_cycle(self):
        """循环 -> soc_bench.py cycle 的命令行（腿序 / 窗口 / 互锁由它自己算）。"""
        f, fc = self.form, self.form_cycle

        def g(key):
            return fc.get(key) or CYC_DEFAULTS[key]

        args = ["cycle",
                "--cycles", g("cycles"),
                "--soc-lo", g("soc_lo"), "--soc-hi", g("soc_hi"),
                "--start-soc", g("start_soc"),
                "--segments", f.get("segments") or "10",
                "--rest-s", f.get("rest_s") or "480",
                "--capacity", f.get("capacity") or "3350",
                "--current-load", g("cur_load"),
                "--current-charge", g("cur_chg"),
                "--vlimit-load", g("vlim_load"),
                "--vlimit-charge", g("vlim_chg"),
                "--vset", g("vset_cyc"),
                "--prefix", g("prefix"),
                "--port-load", self.port_of("port"),
                "--port-psu", self.port_of("peer_port")]
        if fc.get("keep_going") == "yes":
            args.append("--keep-going")
        if fc.get("dry_run") == "yes":
            args.append("--dry-run")
        # 两台设备可能是两个牌子，方言分开传（soc_bench 按腿各带一份 --cmd）
        for key, opt in (("cmd_load", "--cmd-load"), ("cmd_chg", "--cmd-chg")):
            for one in (fc.get(key) or "").replace("\n", ";").split(";"):
                if one.strip():
                    args += [opt, one.strip()]
        args.append("--yes")
        return args

    # ---------------- 跑 ----------------
    def start(self):
        if self.worker is not None and self.worker.is_alive():
            self.log.writeline("[!] 上一次控载任务还没结束")
            return
        cyc = self.is_cycle()
        dry = cyc and self.form_cycle.get("dry_run") == "yes"
        demo = (not cyc) and self.form_single.get("demo") == "yes"
        # 循环的两台设备必须各占一个口：缺一个时那一腿只能自动探测，而两台同插时自动
        # 探测会挑错，跨设备互锁（每腿先关另一台）就无从建立 —— 直接拦住。
        if cyc and not dry and not (self.port_of("port") and self.port_of("peer_port")):
            messagebox.showwarning(APP_TITLE, (
                "循环模式要把两个串口都填上：\n\n"
                "· 放电口（负载）—— 放电腿控的电子负载\n"
                "· 充电口（电源）—— 充电腿控的可编程电源\n\n"
                "缺一个的话，那一腿只能自动探测，而两台设备同时插着时自动探测会挑错，"
                "跨设备互锁就无从建立（每腿开跑前先关另一台并复核电流）。\n"
                "确实不用互锁，请改用命令行并显式加 --no-interlock。"))
            return
        if not (demo or dry) and not self._confirm():
            return
        args = self._args()
        self.log.clear()
        self.log.writeline("[i] 启动：%s %s"
                           % ("soc_bench.py cycle" if cyc else "soc_load.py",
                              " ".join(args)))
        self.btn.configure(state="disabled")
        self.btn_stop.configure(state="normal")

        def work():
            if cyc:
                # 上位机是 exe：sys.executable 指向它自己，给每腿另起进程只会又弹一个
                # 界面出来。把"跑一腿"换成进程内调 soc_load，调度逻辑仍是 soc_bench 那份。
                sbm = ScriptRunner.load("soc_bench")
                sbm.LEG_RUNNER = self._leg_inproc
                try:
                    rc = ScriptRunner.call("soc_bench", args, self.log)
                finally:
                    sbm.LEG_RUNNER = None
            else:
                rc = ScriptRunner.call("soc_load", args, self.log)
            self.log.writeline("[i] 退出码 %d" % rc)

        self.worker = run_bg(work, on_error=lambda tb: self.log.writeline(tb))
        self._wait_done()

    def _leg_inproc(self, cmd):
        """在**本进程**里跑一腿（循环模式走这条）。

        soc_bench 命令行默认给每腿另起一个 python 进程；上位机打包成 exe 之后没有独立
        的解释器可起，所以换成这里。编排本身（腿序 / 窗口 / 互锁 / 停跑判据）一行没变。
        """
        argv = list(cmd)[2:]              # [解释器, soc_load.py, ...] -> [...]
        self.log.writeline("[i] 进程内执行：soc_load.py " + " ".join(argv))
        return ScriptRunner.call("soc_load", argv, self.log)

    def _wait_done(self):
        """轮询工作线程，结束后恢复按钮（不在后台线程里碰控件）。"""
        t = self.worker
        if t is None or not t.is_alive():
            self.worker = None
            self.btn.configure(state="normal")
            self.btn_stop.configure(state="disabled")
            return
        self.after(300, self._wait_done)

    def stop(self):
        t = self.worker
        if t is None or not t.is_alive():
            self.log.writeline("[i] 当前没有正在跑的控载任务")
            return
        if not messagebox.askyesno(
                APP_TITLE, "紧急停止：立即断开输出并结束本次任务？\n\n"
                           "（等价于在命令行里按 Ctrl+C，脚本会走兜底断开流程"
                           + ("；循环模式下只中止当前那一腿" if self.is_cycle() else "")
                           + "）"):
            return
        if raise_async_exc(t, KeyboardInterrupt):
            self.log.writeline("[!] 已发中断信号，等脚本断开设备…")
        else:
            self.log.writeline("[X] 中断信号没送进去，请到源/载面板手动断开输出")


class AnalysisTab(ttk.Frame):
    def __init__(self, master, app):
        super().__init__(master, padding=8)
        self.app = app
        self.csv = tk.StringVar()
        nav_label(self, NAV_TEXT["analysis"]).pack(anchor="w", pady=(0, 6))
        top = ttk.Frame(self)
        top.pack(fill="x")
        ttk.Label(top, text="数据 CSV", font=FONT_UI).pack(side="left")
        ttk.Entry(top, textvariable=self.csv, width=70).pack(side="left", padx=4)
        ttk.Button(top, text="选文件…", command=self.pick).pack(side="left")
        ttk.Button(top, text="打开所在目录",
                   command=lambda: open_path(os.path.dirname(self.csv.get()) or ".")).pack(
            side="left", padx=4)

        f1 = ttk.LabelFrame(self, text="stair：多档周期分析（主力）", padding=6)
        f1.pack(fill="x", pady=6)
        self.form_stair = Form(f1, [
            ("capacity", "标称容量 mAh", "entry", "", "留空=读档位计划"),
            ("init_soc", "起始 SOC %", "entry", "", "留空=计划/电压反查"),
            ("plan", "档位计划 json", "entry", "", "留空=按 CSV 前缀自动找"),
            ("expect", "期望档位数", "entry", "", "留空=读计划"),
            ("i_on", "带载进入 mA", "entry", "200", ""),
            ("i_off", "带载退出 mA", "entry", "50", ""),
            ("idle_ma", "静置阈值 mA", "entry", "20", ""),
            ("min_pulse", "最短带载 s", "entry", "2.0", ""),
            ("ocv_tail", "OCV 取尾 s", "entry", "60", ""),
        ], cols=5)
        self.form_stair.pack(fill="x")
        HelpLine(f1, "容量 / 起始 SOC / 期望档数留空时，按「命令行 > 计划文件 > "
                     "标称值或电压反查」三级自动定", HELP["stair"]).pack(anchor="w")
        b1 = ttk.Frame(f1)
        b1.pack(fill="x", pady=4)
        ttk.Button(b1, text="▶ 跑 stair（出 OCV/R 表 + 5 联图）",
                   command=self.run_stair).pack(side="left")
        self.var_emit = tk.BooleanVar(value=False)
        ttk.Checkbutton(b1, text="同时打印固件用的 11 点 C 数组",
                        variable=self.var_emit).pack(side="left", padx=10)

        # fit 只有一个字段，压成一行 —— 省下的高度全给图
        f2 = ttk.LabelFrame(self, text="fit：单脉冲 RC 拟合", padding=6)
        f2.pack(fill="x", pady=(6, 0))
        row = ttk.Frame(f2)
        row.pack(fill="x")
        ttk.Label(row, text="已知放电电流 A", font=FONT_UI).pack(side="left")
        self.v_fit_i = tk.StringVar()
        ttk.Entry(row, textvariable=self.v_fit_i, width=10,
                  font=FONT_UI).pack(side="left", padx=4)
        ttk.Label(row, text="留空 = 自动取均值", font=FONT_UI,
                  foreground="#777").pack(side="left")
        ttk.Button(row, text="▶ 跑 fit", command=self.run_fit).pack(
            side="left", padx=18)

        # 用经典 tk.PanedWindow（ttk 版没有 minsize，图区会被压得太扁）
        pw = tk.PanedWindow(self, orient="vertical", sashwidth=5,
                            bg="#d0d0d0", bd=0)
        pw.pack(fill="both", expand=True, pady=(6, 0))

        box = ttk.Frame(pw)
        ttk.Label(box, text="图（脚本跑完自动显示，双击放大）",
                  font=FONT_UI).pack(anchor="w")
        self.graph = ImagePane(box, app)
        self.graph.pack(fill="both", expand=True)
        pw.add(box, minsize=300, stretch="always")

        # 产物列表和日志并排 —— 竖着摞会各占一份高度，把图挤扁
        bottom = ttk.Frame(pw)
        left = ttk.Frame(bottom)
        left.pack(side="left", fill="both")
        ttk.Label(left, text="产物（双击打开）", font=FONT_UI).pack(anchor="w")
        self.files = tk.Listbox(left, height=6, width=34, font=FONT_MONO)
        self.files.pack(fill="both", expand=True)
        self.files.bind("<Double-Button-1>", self.open_sel)
        right = ttk.Frame(bottom)
        right.pack(side="left", fill="both", expand=True, padx=(8, 0))
        ttk.Label(right, text="日志", font=FONT_UI).pack(anchor="w")
        self.log = LogPane(right, height=6)
        self.log.pack(fill="both", expand=True)
        pw.add(bottom, minsize=130)

        self._saw = set()
        self._seen_png = set()

    def pick(self):
        p = filedialog.askopenfilename(title="选数据 CSV",
                                       filetypes=[("CSV", "*.csv")])
        if p:
            self.csv.set(p)

    # 表单键 -> 命令行开关（stair 用；fit 只认 --I，自己拼）
    STAIR_KEYS = (("--capacity", "capacity"), ("--init-soc", "init_soc"),
                  ("--plan", "plan"), ("--i-on", "i_on"), ("--i-off", "i_off"),
                  ("--idle-ma", "idle_ma"), ("--min-pulse-s", "min_pulse"),
                  ("--ocv-tail", "ocv_tail"), ("--expect", "expect"))

    def _stair_args(self):
        """只把**填了的**框变成开关 —— 留空就是让脚本自己去认。

        容量 / 起始 SOC / 期望档数留空, soc_load.py 落的档位计划文件就会被采用;
        再不行才退到标称值与"用开录首段静置电压反查 OCV 表"。
        留空比填错强: 填了 100 % 而实际是从 40 % 开始录, 整条 OCV-SOC 会平移。
        """
        args = []
        for a, k in self.STAIR_KEYS:
            v = self.form_stair.get(k) if hasattr(self, "form_stair") else ""
            if v:
                args += [a, v]
        return args

    def _args(self, emitter):
        return ["--csv", self.csv.get()] + self._stair_args()

    def _scan_products(self, base_csv):
        d = os.path.dirname(base_csv) or "."
        stem = os.path.splitext(os.path.basename(base_csv))[0]
        out = []
        for fn in sorted(os.listdir(d)):
            if fn.startswith(stem) and fn != os.path.basename(base_csv):
                out.append(os.path.join(d, fn))
        return out

    def run_stair(self):
        if not self.csv.get():
            messagebox.showwarning(APP_TITLE, "先选数据 CSV")
            return
        args = ["stair", "--csv", self.csv.get()] + self._stair_args()
        if self.var_emit.get():
            args.append("--emit-c")
        self._run_script("soc_bench", args, "stair")

    def run_fit(self):
        if not self.csv.get():
            messagebox.showwarning(APP_TITLE, "先选数据 CSV")
            return
        args = ["fit", "--csv", self.csv.get()]
        if self.v_fit_i.get().strip():
            args += ["--I", self.v_fit_i.get().strip()]
        self._run_script("soc_bench", args, "fit")

    def open_sel(self, _e=None):
        sel = self.files.curselection()
        if sel:
            open_path(self.files.get(sel[0]))

    def _run_script(self, name, args, tag):
        csv = self.csv.get()                 # 主线程取好，别在后台线程读控件
        self.log.clear()
        self.files.delete(0, "end")
        self.log.writeline("[i] %s.py %s" % (name, " ".join(args)))

        def work():
            rc = ScriptRunner.call(name, args, self.log)
            self.log.writeline("[i] 退出码 %d" % rc)
            prods = self._scan_products(csv)
            pngs = []
            for p in prods:
                if p not in self._saw:
                    self._saw.add(p)
                    self.app.ui(lambda p=p: self.files.insert("end", p))
                if p.lower().endswith(".png") and p not in self._seen_png:
                    self._seen_png.add(p)
                    pngs.append(p)
            if pngs:
                # 主图（R0/OCV 汇总那张）文件名最短，放前面作为默认显示
                pngs.sort(key=lambda p: (len(os.path.basename(p)),
                                         os.path.basename(p)))
                self.graph.show(pngs, newest=False)
        run_bg(work, on_error=lambda tb: self.log.writeline(tb))


class SohTab(ttk.Frame):
    def __init__(self, master, app):
        super().__init__(master, padding=8)
        self.app = app
        self.csv = tk.StringVar()
        self.refs = tk.StringVar()
        nav_label(self, NAV_TEXT["soh"]).pack(anchor="w", pady=(0, 6))
        for label, var in (("数据 CSV", self.csv), ("参考 stair CSV", self.refs)):
            r = ttk.Frame(self)
            r.pack(fill="x")
            ttk.Label(r, text=label, font=FONT_UI, width=14).pack(side="left")
            ttk.Entry(r, textvariable=var, width=70).pack(side="left", padx=4)
            ttk.Button(r, text="选文件…",
                       command=lambda v=var: self.pick(v)).pack(side="left")

        self.form = Form(self, [
            ("scale", "合成老化 scale", "entry", "", "如 0.8 = 容量缩到 80%"),
            ("i_on", "带载进入 mA", "entry", "500", ""),
            ("i_off", "带载退出 mA", "entry", "200", ""),
            ("min_pulse", "最短带载 s", "entry", "5.0", ""),
            ("min_dsoc", "最小 ΔSOC %", "entry", "15.0", ""),
        ], cols=5)
        self.form.pack(fill="x", pady=6)

        bar = ttk.Frame(self)
        bar.pack(fill="x")
        ttk.Button(bar, text="▶ soh_learn（离线定算法）",
                   command=self.run_learn).pack(side="left")
        ttk.Button(bar, text="▶ soh_mcu_sim（逐帧复刻，和上面应逐项吻合）",
                   command=self.run_sim).pack(side="left", padx=6)
        self.var_plot = tk.BooleanVar(value=True)
        ttk.Checkbutton(bar, text="出图", variable=self.var_plot).pack(side="left", padx=6)

        pw = tk.PanedWindow(self, orient="vertical", sashwidth=5,
                            bg="#d0d0d0", bd=0)
        pw.pack(fill="both", expand=True, pady=(6, 0))
        box = ttk.Frame(pw)
        ttk.Label(box, text="图（R0 学习结果，跑完自动显示）",
                  font=FONT_UI).pack(anchor="w")
        self.graph = ImagePane(box, app)
        self.graph.pack(fill="both", expand=True)
        pw.add(box, minsize=260, stretch="always")
        self.log = LogPane(pw, height=7)
        pw.add(self.log, minsize=120)

    def pick(self, var):
        p = filedialog.askopenfilename(title="选 CSV", filetypes=[("CSV", "*.csv")])
        if p:
            var.set(p)

    def _base(self, name):
        args = ["--csv", self.csv.get()]
        if self.refs.get():
            args += ["--refs", self.refs.get()]
        if self.form.get("scale"):
            args += ["--scale", self.form.get("scale")]
        if name == "soh_learn":
            for a, k in (("--i-on", "i_on"), ("--i-off", "i_off"),
                         ("--min-pulse", "min_pulse"), ("--min-dsoc", "min_dsoc")):
                if self.form.get(k):
                    args += [a, self.form.get(k)]
        return args

    def run_learn(self):
        self._go("soh_learn")

    def run_sim(self):
        self._go("soh_mcu_sim")

    def _go(self, name):
        csv = self.csv.get()
        if not csv:
            messagebox.showwarning(APP_TITLE, "先选数据 CSV")
            return
        args = self._base(name)
        do_plot = name == "soh_learn" and self.var_plot.get()
        if do_plot:
            args.append("--plot")
        png = csv.rsplit(".", 1)[0] + "_soh.png"
        self.log.clear()
        self.log.writeline("[i] %s.py %s" % (name, " ".join(args)))

        def work():
            rc = ScriptRunner.call(name, args, self.log)
            self.log.writeline("[i] 退出码 %d" % rc)
            if do_plot and os.path.isfile(png):
                self.graph.show([png])
            elif name == "soh_mcu_sim":
                self.log.writeline("[i] soh_mcu_sim 出的是逐项对账表，"
                                   "图看上面 soh_learn 那张")
        run_bg(work, on_error=lambda tb: self.log.writeline(tb))


class EkfTab(ttk.Frame):
    def __init__(self, master, app):
        super().__init__(master, padding=8)
        self.app = app
        self.csv = tk.StringVar()
        self.refs = tk.StringVar()
        nav_label(self, NAV_TEXT["ekf"]).pack(anchor="w", pady=(0, 6))
        HelpLine(self, "回放数据试 EKF 参数；「读 / 写板子」走 ② 页那条串口"
                       "（协议 0x18 / 0x19）", HELP["ekf"]).pack(anchor="w",
                                                                pady=(0, 6))
        for label, var in (("数据 CSV", self.csv), ("参考 stair CSV", self.refs)):
            r = ttk.Frame(self)
            r.pack(fill="x")
            ttk.Label(r, text=label, font=FONT_UI, width=14).pack(side="left")
            ttk.Entry(r, textvariable=var, width=70).pack(side="left", padx=4)
            ttk.Button(r, text="选文件…",
                       command=lambda v=var: self.pick(v)).pack(side="left")
        self.form = Form(self, [
            ("param", "只回放这一组", "entry", "", "q_soc  q_vrc  R"),
            ("grid", "网格（可选）", "entry", "", "qs档|qv档|r档，逗号分隔"),
            ("limit", "只取前 N 帧", "entry", "", "先填 30000 试"),
        ], cols=3)
        self.form.pack(fill="x", pady=6)
        bar = ttk.Frame(self)
        bar.pack(fill="x")
        ttk.Button(bar, text="▶ 跑 kalman_tune", command=self.run).pack(side="left")
        ttk.Button(bar, text="填固件现值",
                   command=self.fill_fw).pack(side="left", padx=6)
        ttk.Button(bar, text="从板子读现值",
                   command=self.read_board).pack(side="left")
        ttk.Button(bar, text="写回板子",
                   command=self.write_board).pack(side="left", padx=6)
        self.plot_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(bar, text="跑完出轨迹图",
                        variable=self.plot_var).pack(side="left", padx=6)
        ttk.Label(bar, text="网格留空 = 脚本默认 4x3x3 共 36 组",
                  font=FONT_UI, foreground="#555").pack(side="left", padx=10)

        pw = tk.PanedWindow(self, orient="vertical", sashwidth=5,
                            bg="#d0d0d0", bd=0)
        pw.pack(fill="both", expand=True, pady=(6, 0))
        box = ttk.Frame(pw)
        ttk.Label(box, text="SOC 轨迹（跑完自动显示，双击放大）",
                  font=FONT_UI).pack(anchor="w")
        self.graph = ImagePane(box, app)
        self.graph.pack(fill="both", expand=True)
        pw.add(box, minsize=260, stretch="always")
        self.log = LogPane(pw, height=7)
        pw.add(self.log, minsize=120)

    def pick(self, var):
        p = filedialog.askopenfilename(title="选 CSV", filetypes=[("CSV", "*.csv")])
        if p:
            var.set(p)

    def fill_fw(self):
        """把固件 bms_config.h 里正在跑的 EKF 三参数填进来。

        值是从固件头文件现读的，不是界面里另抄一份 —— 改固件后这里跟着变。
        """
        qs, qv, r = 0.001, 0.5, 10.0
        src = "(读固件参数失败，用内置兜底值)"
        try:
            vals, path = ScriptRunner.load("test/kalman_tune").fw_defaults_info()
            qs, qv, r = vals
            src = path or "(没找到 bms_config.h，用内置兜底值)"
        except Exception:
            pass
        self.form.set("param", "%g %g %g" % (qs, qv, r))
        self.log.writeline("[i] 固件现值  q_soc=%g  q_vrc=%g  r=%g\n    来源 %s"
                           % (qs, qv, r, src))

    # ---- 板子侧读写（走 ② 页那条串口：协议 0x18 RD_EKF / 0x19 WR_EKF）
    def _three(self):
        """把「只回放这一组」的三个数解析出来，不对就提示并返回 None。"""
        parts = (self.form.get("param") or "").split()
        if len(parts) != 3:
            messagebox.showwarning(
                APP_TITLE, "「只回放这一组」要正好 3 个数：q_soc q_vrc R")
            return None
        try:
            qs, qv, r = [float(x) for x in parts]
        except ValueError:
            messagebox.showwarning(APP_TITLE, "这三项都得是数字：q_soc q_vrc R")
            return None
        return qs, qv, r

    def read_board(self):
        """从板子读 EKF 整组参数：3 个能填进本页的就填，8 个全打日志。"""
        cli, release = self.app.tab_tune.borrow("读板子上的 EKF 参数（RD_EKF）")
        if cli is None:
            messagebox.showwarning(
                APP_TITLE, "先在「② 在线调参」里连接串口（串口归那一页管）")
            return
        self.log.clear()
        self.log.writeline("[i] RD_EKF ...")

        def work():
            try:
                r = cli.rd_ekf()
                self.app.ui(lambda: self._show_ekf(r))
            except Exception:
                self.app.ui(lambda: self.log.writeline(traceback.format_exc()))
            finally:
                self.app.ui(release)
        run_bg(work)

    def _show_ekf(self, r):
        if not r.ok:
            self.log.writeline("[X] RD_EKF rc=%d %s"
                               % (r.rc, P.RC_NAME.get(r.rc, "")))
            return
        vals = {n: r.get(n) for n in P.EKF_NAMES}
        for i, n in enumerate(P.EKF_NAMES):
            e = P.EKF_SPEC[i]
            self.log.writeline("    %-11s = %-10g %-8s  (%s)"
                               % (n, vals[n], e["unit"], e["label"]))
        self.form.set("param", "%g %g %g"
                      % (vals["q_soc"], vals["q_vrc"], vals["r_v"]))
        self.log.writeline("[i] 已把 q_soc / q_vrc / R 填进「只回放这一组」，"
                           "可以改完直接回放或写回板子")

    def write_board(self):
        """把本页这 3 个参数写回板子：读整组 → 只换这 3 个 → 写回整组。"""
        t = self._three()
        if t is None:
            return
        qs, qv, r_v = t
        cli, release = self.app.tab_tune.borrow("把 EKF 三参数写回板子（WR_EKF）")
        if cli is None:
            messagebox.showwarning(
                APP_TITLE, "先在「② 在线调参」里连接串口（串口归那一页管）")
            return
        self.log.clear()
        self.log.writeline("[i] WR_EKF  q_soc=%g  q_vrc=%g  r_v=%g "
                           "（其余 5 个读整组后原样带回）" % (qs, qv, r_v))

        def work():
            try:
                r = cli.ekf_write_subset({"q_soc": qs, "q_vrc": qv, "r_v": r_v})
                self.app.ui(lambda: self._show_write(r, qs, qv, r_v))
            except Exception:
                self.app.ui(lambda: self.log.writeline(traceback.format_exc()))
            finally:
                self.app.ui(release)
        run_bg(work)

    def _show_write(self, r, qs, qv, r_v):
        if not r.ok:
            self.log.writeline("[X] 写回失败 rc=%d %s —— 参数没变"
                               % (r.rc, P.RC_NAME.get(r.rc, "")))
            if r.rc == P.RC_EBADARG:
                self.log.writeline("[!] rc=1 = 有值超出固件接受的量级区间，"
                                   "合法范围见「② 在线调参 → 写 EKF 参数」每个"
                                   "输入框右边的灰字")
            return
        self.log.writeline("[i] 已写回板子并生效：q_soc=%g q_vrc=%g r_v=%g"
                           % (qs, qv, r_v))
        self.log.writeline("[i] 其余 5 个参数保持板子原值；"
                           "想让 p0_* 生效再发一次「复位 EKF」（② 页单条命令 → 复位 EKF）")

    def run(self):
        if not self.csv.get():
            messagebox.showwarning(APP_TITLE, "先选数据 CSV")
            return
        args = ["--csv", self.csv.get()]
        if self.refs.get():
            args += ["--refs", self.refs.get()]
        param = self.form.get("param")
        if param:
            parts = param.split()
            if len(parts) != 3:
                messagebox.showwarning(
                    APP_TITLE, "「只回放这一组」要正好 3 个数：q_soc q_vrc R")
                return
            try:
                for x in parts:
                    float(x)
            except ValueError:
                messagebox.showwarning(
                    APP_TITLE, "这三项都得是数字：q_soc q_vrc R")
                return
            args += ["--param"] + parts
        elif self.form.get("grid"):
            # 单组模式下不再塞网格，免得看命令行以为两个都生效
            args += ["--grid", self.form.get("grid")]
        if self.form.get("limit"):
            args += ["--limit", self.form.get("limit")]
        plot = None
        if self.plot_var.get():
            plot = self.csv.get().rsplit(".", 1)[0] + "_ekf.png"
            args += ["--plot", plot]
        self.log.clear()
        self.log.writeline("[i] test/kalman_tune.py " + " ".join(args))
        run_bg(lambda: self._after(
            ScriptRunner.call("test/kalman_tune", args, self.log), plot),
            on_error=lambda tb: self.log.writeline(tb))

    def _after(self, rc, plot):
        """跑完在后台线程里回调 —— 图直接进窗口，不用切到看图软件。"""
        if plot and os.path.isfile(plot):
            self.graph.show([plot])


# ==========================================================================
# 主窗口
# ==========================================================================

class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title(APP_TITLE)
        self.geometry("1180x820")
        self.minsize(980, 700)
        try:
            ttk.Style().theme_use("vista")
        except Exception:
            pass
        self._port_owner = (None, None)

        nb = ttk.Notebook(self)
        # ⚠ 这里**不能** pack —— packer 按打包顺序发空间，notebook 带 expand=True
        # 会把整窗高度吃满，下面那条底栏只剩 1 px、根本不映射（实测 h=1/mapped=0，
        # 1180x820 与放大到 1400x1000 一个样）。底栏的子件 pack 完之后再收尾，见下方。
        self.tab_record = RecordTab(nb, self)
        self.tab_tune = TuneTab(nb, self)
        self.tab_load = LoadTab(nb, self)
        self.tab_analysis = AnalysisTab(nb, self)
        self.tab_soh = SohTab(nb, self)
        self.tab_ekf = EkfTab(nb, self)
        nb.add(self.tab_record, text="① 采集监视")
        nb.add(self.tab_tune, text="② 在线调参")
        nb.add(self.tab_load, text="③ 控载（拉载/充电/循环）")
        nb.add(self.tab_analysis, text="④ 分析（fit / stair）")
        nb.add(self.tab_soh, text="⑤ SOH 验证")
        nb.add(self.tab_ekf, text="⑥ EKF 调参")

        bar = ttk.Frame(self)
        bar.pack(fill="x", side="bottom")
        self.lbl_live = ttk.Label(
            bar, font=FONT_MONO,
            text="实时：SOC --  SOH --  Q --  R0 --  T --")
        self.lbl_live.pack(side="left", padx=8, pady=4)
        # 工具目录原来是常驻的完整绝对路径 —— 平时没信息量还占半行，收进「?」。
        ttk.Label(bar, font=FONT_UI, foreground="#666",
                  text="协议 v%d  ·  可用命令 %d 条"
                       % (P.PROTO_VER, len(P.COMMANDS))).pack(side="right", padx=8)
        ttk.Button(bar, text="?", width=3, command=lambda: _help_window(
            bar,
            "工具目录\n%s\n\n"
            "上位机是 PyInstaller 打包产物，不是源码。真正的源码是\n"
            "  tools/bms_gui.py  +  tools/bms_tune_proto.py\n\n"
            "exe 优先加载「同级 tools/ 里那份脚本」，所以只改了 tools/*.py\n"
            "（bms_gui.py 除外）不用重新打包，把新脚本覆盖过去就生效；\n"
            "改了 bms_gui.py 本身才要 python tools/build_exe.py 重打。"
            % TOOLS)).pack(side="right", padx=(0, 4))

        # 底栏自己那一行已经占住了，notebook 现在才吃剩下的空间。
        nb.pack(fill="both", expand=True, padx=6, pady=6)

        self.protocol("WM_DELETE_WINDOW", self.on_close)
        self._uiq = queue.Queue()
        self._drain_ui()

    # ---- 主线程 UI 队列：任何后台线程都能安全地改界面
    def ui(self, fn):
        self._uiq.put(fn)

    def _drain_ui(self):
        try:
            while True:
                fn = self._uiq.get_nowait()
                try:
                    fn()
                except Exception:
                    # 窗口程序没有控制台，出错得写进界面日志才看得见
                    try:
                        self.tab_tune.log.writeline(
                            "[X] 界面更新出错：\n" + traceback.format_exc())
                    except Exception:
                        pass
        except queue.Empty:
            pass
        self.after(60, self._drain_ui)

    # ---- 串口占用互斥（同一个 COM 不能被两个页同时打开）
    def set_port_owner(self, port, who):
        self._port_owner = (port, who)

    def clear_port_owner(self):
        self._port_owner = (None, None)

    def port_in_use(self, port, who):
        p, owner = self._port_owner
        if p == port and owner is not None and owner is not who:
            messagebox.showwarning(APP_TITLE,
                                   "%s 已被「%s」页占用，先在那里断开。"
                                   % (port, getattr(owner, "tab_name", "另一")))
            return True
        return False

    def show_live(self, r):
        self.lbl_live.configure(
            text="实时：SOC %.2f %%  SOH %.2f %%  Q %s mAh  R0 %s mΩ  T %.1f °C"
                 % ((r.get("soc01") or 0) / 100.0, (r.get("soh01") or 0) / 100.0,
                    r.get("q_mah"), r.get("r0_mohm"), (r.get("t") or 0) / 10.0))

    def on_close(self):
        lt = self.tab_load
        if lt.worker is not None and lt.worker.is_alive():
            if not messagebox.askyesno(
                    APP_TITLE, "控载任务还在跑。关掉窗口会中止它"
                               "（脚本会先断开负载）。确定关闭？"):
                return
            raise_async_exc(lt.worker, KeyboardInterrupt)
        try:
            self.tab_record.stop()
        except Exception:
            pass
        try:
            self.tab_tune.disconnect()
        except Exception:
            pass
        self.destroy()


def main():
    if os.name == "nt":
        try:
            import ctypes
            ctypes.windll.shcore.SetProcessDpiAwareness(1)
        except Exception:
            pass
    app = App()
    app.tab_record.refresh_ports()
    app.tab_tune.refresh_ports()
    app.tab_load.refresh_ports()
    app.mainloop()


if __name__ == "__main__":
    main()
