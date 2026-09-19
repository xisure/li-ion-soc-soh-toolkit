#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""bms_tune_proto.py - bms_tune 在线调参协议的客户端实现

协议规范见 `../算法库/在线调参协议.md`（权威文档），本模块是它的上位机侧实现。

职责边界
--------
只做协议，不碰界面、不做业务判断：

* `crc16` / `build_frame` / `Unframer` —— 帧层（含垃圾数据自恢复）
* `COMMANDS` —— 34 条命令的字段表（下行参数 + 上行应答逐字段描述）
* `EKF_SPEC` —— EKF 8 个可调参数的权威表（名字 / 固件宏 / 默认值 / 合法区间）
* `Client` —— 请求/应答、超时重传、应答解码
* 传输层可替换：`SerialTransport` 走 pyserial，测试时传任何带
  `write(bytes)` / `read(n, timeout)` 的对象即可（回归脚本就是这么接
  指令级模拟器的）。

为什么把字段表做成数据而不是一堆 if
-----------------------------------
GUI 的下行参数输入框、上行结果表格全是从 `COMMANDS` 自动生成的 ——
加一条新命令只要在表里加一行，界面自动多出对应的输入区和结果区，
不会出现"协议改了界面忘了改"的两份实现。

字段类型
--------
`u8/i8/u16/i16/u32/i32/f32` 定长整数/浮点；`blob` 定长字节块；
`temps` 变长温度点数组（个数由 INFO 的 `temp_n` 决定）；
`cal` / `r0all` / `active` 三种长度随维度变的字节块。
"""

import struct
import time

__all__ = [
    "PROTO_VER", "RC_OK", "RC_EBADARG", "RC_EBADLEN", "RC_ENOSUP", "RC_NAME",
    "crc16", "build_frame", "Unframer", "Field", "Cmd", "COMMANDS",
    "COMMANDS_BY_NAME", "DEFAULT_DIMS", "dims_with", "Reply", "Client",
    "SerialTransport", "TuneTimeout", "decode_payload", "encode_down",
    "scale_in", "scale_out", "field_size", "soh_blob_decode", "soh_blob_encode",
    "SOH_BLOB_BYTES", "TBL_NAME", "TBL_UNIT", "TBL_OCV", "TBL_R0", "TBL_R1",
    "TBL_TAU", "C_RD_CAL_ALL", "C_WR_CAL_ALL", "EKF_SPEC", "EKF_PARAM_BYTES",
    "EKF_NAMES", "C_RD_SOC", "C_WR_SOC", "C_RD_EKF", "C_WR_EKF", "C_EKF_RESET",
    "C_RD_SOH_CNT", "C_SOH_CNT_RESET", "C_RD_R0_CHG_ALL", "C_WR_R0_CHG_ALL",
    "C_RD_CORR", "SOH_BLOB_BYTES_V2",
    "selftest",
]

# ---------------------------------------------------------------------------
# 协议常量
# ---------------------------------------------------------------------------

PROTO_VER = 2          # BMS_TUNE_PROTO_VER
ACK = 0x80             # 应答 CMD = 请求 CMD | 0x80
SOF = b"\xaa\x55"

RC_OK, RC_EBADARG, RC_EBADLEN, RC_ENOSUP = 0, 1, 2, 3
RC_NAME = {0: "OK 成功", 1: "EBADARG 参数越界", 2: "EBADLEN 长度不符",
           3: "ENOSUP 未编译进库"}

# 命令码（与 bms_tune.h 一一对应）
C_PING, C_INFO = 0x00, 0x01
C_RD_CAL_POINT, C_WR_CAL_POINT = 0x02, 0x03
C_RD_CAL_ALL, C_WR_CAL_ALL = 0x04, 0x05
C_CAL_RESTORE, C_RD_CAL_TEMPS = 0x06, 0x07
C_RD_TEMP, C_WR_TEMP, C_RD_CAL_ACTIVE = 0x08, 0x09, 0x0A
C_RD_R0, C_WR_R0, C_RD_R0_ALL, C_WR_R0_ALL = 0x10, 0x11, 0x12, 0x13
C_RD_CAP, C_WR_CAP = 0x14, 0x15
C_RD_SOC, C_WR_SOC = 0x16, 0x17
C_RD_EKF, C_WR_EKF, C_EKF_RESET = 0x18, 0x19, 0x1A
C_RD_SOH_BLOB, C_WR_SOH_BLOB, C_RD_SOH_SUM, C_SOH_RESET = 0x20, 0x21, 0x22, 0x23
C_RD_SOH_CNT, C_SOH_CNT_RESET = 0x24, 0x25
C_RD_R0_CHG_ALL, C_WR_R0_CHG_ALL = 0x26, 0x27
C_RD_CORR = 0x28
C_NVM_SAVE, C_NVM_INFO = 0x30, 0x31
C_RD_LIVE = 0x40

# ---------------------------------------------------------------------------
# EKF 8 个可调参数的权威表
# ---------------------------------------------------------------------------
# 顺序 = 固件 soc.h 的 soc_ekf_param_t，也是 WR_EKF / RD_EKF 里 8 个 f32 的
# 字节顺序。名字是协议里的字段名（与固件 accessor 一一对应），macro 是它在
# bms_config.h 里的宏后缀，default 是出厂宏的值。
#
# default / lo / hi 三项**必须**和固件一致 —— 固件侧是 bms_config.h 的
# BMS_EKF_* 宏（默认值）与 soc.c 里 ekf_param_ok() 的两个数组（合法区间，
# 开区间：v 必须严格大于 lo 且严格小于 hi）。改了一边忘另一边不会报错，
# 只会在写参数时莫名收到 rc=1；所以 check_proto.py 有一节专门逐项对拍。
EKF_SPEC = [
    dict(name="q_soc", macro="Q_SOC", default=0.001, lo=1e-9, hi=1.0,
         label="过程噪声 SOC", unit="%^2/帧",
         note="每帧 SOC 变化的不确定度。调小=更信模型（稳但跟不上），"
              "调大=更信端压（跟得紧但抖动大）"),
    dict(name="q_vrc", macro="Q_VRC", default=0.5, lo=1e-9, hi=1e4,
         label="过程噪声 极化", unit="mV^2/帧",
         note="一阶 RC 极化电压每帧的不确定度，一般不用动"),
    dict(name="r_v", macro="R_V", default=10.0, lo=1e-9, hi=1e6,
         label="量测噪声 端压", unit="mV^2",
         note="端压读数的不确定度。显示抖就调大，跟不紧就调小"),
    dict(name="p0_soc", macro="P0_SOC", default=10.0, lo=1e-9, hi=1e4,
         label="初值协方差 SOC", unit="%^2",
         note="只在 EKF 建立/复位那一次用（默认 10 ≈ ±3%），"
              "改完要发一次「复位 EKF」才生效"),
    dict(name="p0_vrc", macro="P0_VRC", default=400.0, lo=1e-9, hi=1e6,
         label="初值协方差 极化", unit="mV^2",
         note="默认 400 ≈ ±20 mV，同样只在复位时生效"),
    dict(name="res_max_mv", macro="RES_MAX_MV", default=250.0, lo=1e-6, hi=1e5,
         label="新息野值门限", unit="mV",
         note="新息绝对值超过它就丢掉这一帧量测 —— 挡的是接触弹跳 / 电流计"
              "校准尖峰，不是正常极化"),
    dict(name="s_min", macro="S_MIN", default=1e-6, lo=1e-12, hi=1.0,
         label="新息方差下限", unit="mV^2", note="防除零，一般不用动"),
    dict(name="p_min", macro="P_MIN", default=1e-4, lo=1e-12, hi=1e4,
         label="协方差下限", unit="",
         note="防协方差被压到 0 后卡尔曼增益永久为 0（滤波器卡死）"),
]
EKF_PARAM_BYTES = 4 * len(EKF_SPEC)          # 32
EKF_NAMES = tuple(e["name"] for e in EKF_SPEC)
EKF_DEFAULTS = tuple(e["default"] for e in EKF_SPEC)


def _ekf_fields(unit_split=False):
    """从 EKF_SPEC 生成 8 个 f32 字段（下行用 / 上行用同一套）。"""
    return [Field(e["name"], "f32", unit=e["unit"], label=e["label"],
                  note=(e["note"] if not unit_split
                        else "%s；合法区间 %g ~ %g" % (e["note"], e["lo"], e["hi"])))
            for e in EKF_SPEC]

# 表号 / SOC 点（INFO 拿到维度，这里只做默认值）
TBL_OCV, TBL_R0, TBL_R1, TBL_TAU = 0, 1, 2, 3
TBL_NAME = {0: "OCV (mV)", 1: "R0 (mΩ)", 2: "R1 (mΩ)", 3: "τ (s)"}
TBL_UNIT = {0: "mV", 1: "mΩ", 2: "mΩ", 3: "s"}
CAL_UNIT_BY_TBL = {0: "mV", 1: "mΩ", 2: "mΩ", 3: "s"}

# 维度默认值：与 bms_config.h 出厂配置一致。接上板子后应由 0x01 INFO 覆盖，
# **不要硬编码 cal_bytes**（温度点个数一改就变）。
DEFAULT_DIMS = {
    "proto_ver": PROTO_VER,
    "nvm_ver": 3,
    "temp_n": 3,
    "tbl_n": 4,
    "soc_n": 11,
    "cal_bytes": 3 * 4 * 11 * 2,
    "flags": 1,           # bit0 = 掉电保持已启用
    "cap_nom_mah": 3350,
}


def dims_with(dims, **kw):
    d = dict(DEFAULT_DIMS)
    if dims:
        d.update(dims)
    d.update(kw)
    return d


# ---------------------------------------------------------------------------
# 帧层
# ---------------------------------------------------------------------------

def crc16(data):
    """CRC-CCITT-FALSE：poly 0x1021 / init 0xFFFF / 不反射 / 不取反。

    覆盖范围是 `LEN + CMD + PAYLOAD`，**不含 `AA 55` 前导**。
    自检：crc16(b"123456789") == 0x29B1
    """
    crc = 0xFFFF
    for b in data:
        crc ^= (b << 8)
        crc &= 0xFFFF
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def build_frame(cmd, payload=b""):
    """拼一整帧：AA 55 LEN(u16) CMD PAYLOAD CRC(u16)。LEN 是 payload 字节数。

    注意：这是**请求**帧拼装器 —— CMD 会被 `& 0x7F`（应答标志位由固件加）。
    所以**不能**用它生成应答帧来对拍（`build_frame(0x80, ...)` 会得到 CMD=0x00）。
    要拼应答帧（测试/回放用）自己来：`SOF + struct.pack("<HB", len(pay), 0x80|cmd)
    + pay + struct.pack("<H", crc16(body))`。
    """
    pay = bytes(payload)
    if len(pay) > 0xFFFF:
        raise ValueError("payload 太长: %d B" % len(pay))
    body = struct.pack("<HB", len(pay), cmd & 0x7F) + pay
    return SOF + body + struct.pack("<H", crc16(body))


class Unframer(object):
    """字节流 → 帧列表。带帧同步自恢复：垃圾前缀、AA AA 55、半截帧都不怕。

    `feed()` 返回 `[(cmd, payload, crc_ok), ...]`；`crc_ok=False` 的帧也返回，
    好让上层能显示"收到了 CRC 错的帧"（固件侧是静默丢弃的）。
    """

    def __init__(self, rx_max=8192):
        self.buf = bytearray()
        self.rx_max = rx_max
        self.dropped = 0        # 因长度不合理丢弃的字节数

    def feed(self, data):
        self.buf += data
        out = []
        i = 0
        n = len(self.buf)
        while i + 7 <= n:
            if self.buf[i] != 0xAA or self.buf[i + 1] != 0x55:
                i += 1
                continue
            ln = self.buf[i + 2] | (self.buf[i + 3] << 8)
            if ln > self.rx_max:
                # 长度不可能：直接把 SOF 之后丢掉，继续找下一个 SOF
                self.dropped += 2
                i += 2
                continue
            if i + 7 + ln > n:
                break                      # 半截帧，等更多字节
            body = bytes(self.buf[i + 2:i + 5 + ln])
            crc = self.buf[i + 5 + ln] | (self.buf[i + 6 + ln] << 8)
            out.append((self.buf[i + 4], bytes(self.buf[i + 5:i + 5 + ln]),
                        crc == crc16(body)))
            i += 7 + ln
        del self.buf[:i]
        return out

    def reset(self):
        self.buf = bytearray()


# ---------------------------------------------------------------------------
# 字段表
# ---------------------------------------------------------------------------

class Field(object):
    """一个字段：名字 / 类型 / 单位 / 显示换算。

    `name` 是协议里的字段名（与固件 `bms_tune.h` 的字段一一对应，排错时
    要拿它去对协议手册），`label` 是给界面看的中文名。两者都留着：
    界面显示 label，日志和文档里对不上号时还能顺着 name 查回去。
    """

    def __init__(self, name, kind, unit="", div=1, note="", size=None,
                 fmt=None, arr=0, label=""):
        self.name = name
        self.kind = kind          # u8 i8 u16 i16 u32 i32 f32 blob temps cal r0all active
        self.unit = unit
        self.div = div            # 显示时除以它（250 → 25.0）
        self.note = note
        self.size = size
        self.fmt = fmt            # blob 的解包格式串（可选）
        self.arr = arr            # blob 内元素个数（可选）
        self.label = label        # 中文名（空则回落到 name）

    @property
    def title(self):
        return self.label or self.name


class Cmd(object):
    def __init__(self, code, name, desc, down=(), up=(), dep="", cat="",
                 var_down=None, label="", needs=""):
        self.code = code
        self.name = name
        self.desc = desc
        self.down = tuple(down)
        self.up = tuple(up)
        self.dep = dep            # 非空表示依赖某个编译开关
        self.cat = cat
        self.var_down = var_down  # 变长下行的长度函数名
        self.label = label        # 中文名（空则回落到 name）
        self.needs = needs        # 前置条件的大白话说明（空 = 没前提）

    @property
    def title(self):
        return self.label or self.name

    @property
    def n_down_text(self):
        """界面上「参数」那一列：这条命令要填几项。"""
        if not self.down:
            return "无"
        for f in self.down:
            if f.kind in ("blob", "cal", "r0all", "active"):
                return "专用页"
        return "%d 项" % len(self.down)

    @property
    def ack_code(self):
        return self.code | ACK


def _cal_len(d):
    return d["cal_bytes"]


def _r0all_len(d):
    return d["soc_n"] * 2


def _active_len(d):
    return d["tbl_n"] * d["soc_n"] * 2


DEFAULT_CMD_CATS = ("链路", "标定表", "内阻与容量", "SOC 与 EKF", "SOH",
                    "掉电保持", "实时量", "其它")

COMMANDS = {c.code: c for c in [
    # ---------------------------------------------------------------- 链路
    Cmd(C_PING, "PING", "看板子在不在、固件协议版本对不对。接上串口先发它",
        label="握手（板子在不在）",
        up=[Field("proto_ver", "u16", label="协议版本", note="应等于 1")],
        cat="链路"),
    Cmd(C_INFO, "INFO", "读固件的配置维度：几个温度点、几张表、每张几点、标称容量",
        label="读板子配置",
        up=[Field("proto_ver", "u16", label="协议版本"),
            Field("nvm_ver", "u16", label="掉电保持版本"),
            Field("temp_n", "u8", label="温度点个数"),
            Field("tbl_n", "u8", label="标定表张数"),
            Field("soc_n", "u8", label="每张表点数"),
            Field("cal_bytes", "u16", label="整包字节数"),
            Field("flags", "u8", label="功能开关", note="bit0 = 掉电保持已启用"),
            Field("rsv", "u8", label="（保留）"),
            Field("cap_nom_mah", "u32", unit="mAh", label="标称容量")],
        cat="链路"),
    # ---------------------------------------------------------------- 标定表
    Cmd(C_RD_CAL_POINT, "RD_CAL_POINT", "读某个温度点、某张表、某个 SOC 点上的出厂基准值",
        label="读一个标定点",
        down=[Field("ti", "u8", label="温度点序号", note="0 ~ 温度点个数-1"),
              Field("tbl", "u8", label="表号", note="0 OCV / 1 R0 / 2 R1 / 3 τ"),
              Field("idx", "u8", label="SOC 点序号", note="0 ~ 每表点数-1")],
        up=[Field("v", "u16", label="数值", note="单位随表号变化")],
        cat="标定表"),
    Cmd(C_WR_CAL_POINT, "WR_CAL_POINT", "写某个点的出厂基准值。改的是基准，不含老化量",
        label="改一个标定点",
        needs="改完不落盘，断电就回旧值；要保下来得再发「立即存 Flash」",
        down=[Field("ti", "u8", label="温度点序号", note="0 ~ 温度点个数-1"),
              Field("tbl", "u8", label="表号", note="0 OCV / 1 R0 / 2 R1 / 3 τ"),
              Field("idx", "u8", label="SOC 点序号", note="0 ~ 每表点数-1"),
              Field("v", "u16", label="数值", note="单位随表号变化")],
        cat="标定表"),
    Cmd(C_RD_CAL_ALL, "RD_CAL_ALL", "把整张标定表读上来（改表的固定第一步）",
        label="读整张标定表",
        up=[Field("cal", "blob", label="标定表整包", note="温度点 × 表 × 点数 个 u16")],
        cat="标定表"),
    Cmd(C_WR_CAL_ALL, "WR_CAL_ALL", "把整张标定表写回去。必须由「读整张标定表」改出来",
        label="写回整张标定表",
        needs="必须先读整张表再改 —— 别从零拼一张新表",
        down=[Field("cal", "blob", label="标定表整包", note="长度由板子配置决定")],
        cat="标定表"),
    Cmd(C_CAL_RESTORE, "CAL_RESTORE", "标定表恢复出厂（回到固件编译时的默认值）",
        label="标定表恢复出厂",
        needs="不可撤销，会覆盖当前整张表",
        cat="标定表"),
    Cmd(C_RD_CAL_TEMPS, "RD_CAL_TEMPS", "读板子上三个温度点各是多少度",
        label="读温度点列表",
        up=[Field("temps", "temps", unit="0.1°C", div=10, label="温度点",
                  note="个数 = 温度点个数")],
        cat="标定表"),
    Cmd(C_RD_TEMP, "RD_TEMP", "读当前温度 —— 它决定你改哪个温度点的表会立刻生效",
        label="读当前温度",
        up=[Field("t", "i16", unit="°C", div=10, label="当前温度")],
        cat="标定表"),
    Cmd(C_WR_TEMP, "WR_TEMP", "强制把温度设成某个值（调试用）",
        label="强制设置温度",
        needs="只在主循环没跑时有效，跑起来会被传感器值覆盖",
        down=[Field("t", "i16", unit="°C", div=10, label="温度",
                    note="填 25.0 就是 25.0 °C")],
        cat="标定表"),
    Cmd(C_RD_CAL_ACTIVE, "RD_CAL_ACTIVE", "读当前温度插值出来的那张表 —— 算法真正在用的",
        label="读当前生效的表",
        up=[Field("active", "blob", label="生效中的表", note="表数 × 点数 个 u16")],
        cat="标定表"),
    # ---------------------------------------------------------------- R0 / 容量
    Cmd(C_RD_R0, "RD_R0", "读某个 SOC 点上的内阻 R0（= 当前温度基准 + 老化增量）",
        label="读一个点的内阻",
        down=[Field("idx", "u8", label="SOC 点序号", note="0 ~ 每表点数-1")],
        up=[Field("v", "u16", unit="mΩ", label="内阻 R0")],
        cat="内阻与容量"),
    Cmd(C_WR_R0, "WR_R0", "写某个 SOC 点的内阻 R0（填绝对值，库内部自己换算成增量）",
        label="改一个点的内阻",
        needs="填的是绝对值，不是增量",
        down=[Field("idx", "u8", label="SOC 点序号", note="0 ~ 每表点数-1"),
              Field("v", "u16", unit="mΩ", label="内阻 R0", note="绝对值")],
        cat="内阻与容量"),
    Cmd(C_RD_R0_ALL, "RD_R0_ALL", "一次读回全部 SOC 点的内阻 R0",
        label="读全部内阻",
        up=[Field("r0all", "blob", label="内阻整包", note="每点一个 u16")],
        cat="内阻与容量"),
    Cmd(C_WR_R0_ALL, "WR_R0_ALL", "整包写回全部内阻。比逐点写快，也不会写一半掉电",
        label="写回全部内阻",
        needs="必须先「读全部内阻」再改 —— 别从零拼",
        down=[Field("r0all", "blob", label="内阻整包", note="每点一个 u16")],
        cat="内阻与容量"),
    Cmd(C_RD_R0_CHG_ALL, "RD_R0_CHG_ALL",
        "读**充电方向**那张 R0 表。板子按充/放两个方向各学一张表，两张互不干扰",
        label="读全部内阻（充电方向）",
        up=[Field("r0all", "blob", label="充电方向内阻整包", note="每点一个 u16")],
        cat="内阻与容量"),
    Cmd(C_WR_R0_CHG_ALL, "WR_R0_CHG_ALL",
        "整包写回充电方向的内阻。必须先读再改，和放电方向那对命令对称",
        label="写回全部内阻（充电方向）",
        needs="填的是绝对值；改完不落盘，断电回旧值",
        down=[Field("r0all", "blob", label="充电方向内阻整包", note="每点一个 u16")],
        cat="内阻与容量"),
    Cmd(C_RD_CAP, "RD_CAP", "读在线学习到的容量 Q",
        label="读学到的容量",
        up=[Field("mah", "u32", unit="mAh", label="容量 Q")],
        cat="内阻与容量"),
    Cmd(C_WR_CAP, "WR_CAP", "写容量 Q。库只做钳位，写完会自动读回确认",
        label="写容量",
        needs="只会被钳到标称容量的 50% ~ 110% 之间",
        down=[Field("mah", "u32", unit="mAh", label="容量 Q")],
        cat="内阻与容量"),
    # ---------------------------------------------------------------- SOC / EKF
    Cmd(C_RD_SOC, "RD_SOC",
        "读 SOC 运行态：估计值、积分基准、累计电荷、EKF 的极化和协方差",
        label="读 SOC 运行态",
        up=[Field("soc01", "u16", unit="%", div=100, label="SOC（估计值）",
                  note="EKF 建立后就是 EKF 那个 SOC"),
            Field("base01", "u16", unit="%", div=100, label="积分基准",
                  note="SOC = 基准 − 累计电荷 / 容量"),
            Field("coul_mah", "i32", unit="mAh", label="累计电荷",
                  note="自基准起，放电为正"),
            Field("flags", "u8", label="状态标志", note="bit0 = EKF 已建立"),
            Field("kf_vrc", "f32", unit="mV", label="EKF 极化电压",
                  note="未建立时为 0"),
            Field("p11", "f32", label="SOC 方差 P11", note="不收敛时看它"),
            Field("p22", "f32", label="极化方差 P22")],
        cat="SOC 与 EKF"),
    Cmd(C_WR_SOC, "WR_SOC",
        "把一个已知 SOC 强制对齐进去（现场标定 / 拿表对过之后纠偏）",
        label="强制设置 SOC",
        needs="只重建积分基准 + 直接置 EKF 的 SOC（不按端压查表，"
              "带载时端压查表值系统性偏低）；SOH 学习不受影响",
        down=[Field("soc01", "u16", unit="%", div=100, label="要设成的 SOC",
                    note="0.00 ~ 100.00，填 43.21 就是 43.21 %")],
        cat="SOC 与 EKF"),
    Cmd(C_RD_EKF, "RD_EKF",
        "读 EKF 的 8 个参数现值 —— 改之前先读，改完再读回确认",
        label="读 EKF 参数（整组）",
        up=_ekf_fields(),
        cat="SOC 与 EKF"),
    Cmd(C_WR_EKF, "WR_EKF",
        "整组写 EKF 的 8 个参数。有一个越界整组不写，不留半写状态",
        label="写 EKF 参数（整组）",
        needs="必须先读整组再改 —— 只想改一个也要把其余 7 个原样带上；"
              "参数不落盘，掉电回 bms_config.h 的默认值",
        down=_ekf_fields(unit_split=True),
        cat="SOC 与 EKF"),
    Cmd(C_EKF_RESET, "EKF_RESET",
        "把滤波器打回刚建立的状态：极化清零、协方差回 P0 初值，SOC 不动",
        label="复位 EKF",
        needs="改完 p0_soc / p0_vrc 要发一次这个才生效；"
              "带载时用它重新收敛，不用等静置",
        cat="SOC 与 EKF"),
    # ---------------------------------------------------------------- SOH
    Cmd(C_RD_SOH_BLOB, "RD_SOH_BLOB", "读整块 SOH 参数（和掉电保持存在 Flash 里的是同一份）",
        label="读 SOH 参数块",
        up=[Field("blob", "blob", label="SOH 参数块", note="120 B（ver 2 的前 68 B 布局不变）",
                  size=120)],
        dep="BMS_USE_NVM=1", cat="SOH"),
    Cmd(C_WR_SOH_BLOB, "WR_SOH_BLOB", "整块写回 SOH 参数 —— 换板子时把学到的结果搬过去",
        label="写 SOH 参数块",
        needs="必须先读出来或从文件载入，不能手拼",
        down=[Field("blob", "blob", label="SOH 参数块", note="120 B", size=120)],
        dep="BMS_USE_NVM=1", cat="SOH"),
    Cmd(C_RD_CORR, "RD_CORR",
        "读当前工况的容量折算系数 K 与折算后的可用容量（温度 / 倍率修正）",
        label="读工况折算（温度/倍率）",
        up=[Field("corr_ppm", "i32", unit="%", div=10000, label="折算系数 K",
                  note="100.00% = 正好在参考工况；恒为 100% 说明还没标定"),
            Field("eff_cap_mah", "u32", unit="mAh", label="折算后可用容量",
                  note="= 学到的容量 × K。低温 / 大电流下会比额定小"),
            Field("ref_temp_dc", "i16", unit="°C", div=10, label="参考温度"),
            Field("ref_cur_ma", "i32", unit="mA", label="参考电流")],
        cat="SOC 与 EKF"),
    Cmd(C_RD_SOH_SUM, "RD_SOH_SUM", "读 SOH 摘要：健康度、容量保持率、内阻指标",
        label="读 SOH 摘要",
        up=[Field("soh01", "u16", unit="%", div=100, label="SOH（精细）",
                  note="两位小数"),
            Field("soh_pct", "u8", unit="%", label="SOH（整数）"),
            Field("cap_pct", "u8", unit="%", label="容量保持率"),
            Field("r_pct", "u8", unit="%", label="内阻指标"),
            Field("q_mah", "u32", unit="mAh", label="容量 Q"),
            Field("r0_mohm", "u16", unit="mΩ", label="内阻 R0"),
            Field("t", "i16", unit="°C", div=10, label="温度"),
            Field("valid", "u8", label="有效标志",
                  note="bit0 容量有效 / bit1 内阻有效")],
        cat="SOH"),
    Cmd(C_SOH_RESET, "SOH_RESET", "清掉学习值，回到「没学过」的状态",
        label="清空学习值",
        needs="不可撤销；清完要重新学几个小时才有结果",
        cat="SOH"),
    Cmd(C_RD_SOH_CNT, "RD_SOH_CNT",
        "读累计充放电与循环次数 —— 五个数都是从「学到现在」的累计量",
        label="读累计充放电与循环",
        up=[Field("cum_chg_mah", "u32", unit="mAh", label="累计充入电量",
                  note="只统计真的在充放的时段，静置的电流噪声不算进去"),
            Field("cum_dis_mah", "u32", unit="mAh", label="累计放出电量"),
            Field("half_cycle", "u16", unit="个", label="半循环计数（摆幅法）",
                  note="SOC 摆动超过阈值记半个；上下各走一趟 = 1 个满循环"),
            Field("cycle_milli", "u32", unit="/1000", label="等效满循环 ×1000",
                  note="累计放出 / 标称容量，与放电深度无关；1000000 = 1000 个"),
            Field("cnt_any", "u8", label="计数有效",
                  note="1 = 至少累计过一次充放")],
        cat="SOH"),
    Cmd(C_SOH_CNT_RESET, "SOH_CNT_RESET",
        "只把累计充放电和循环次数清零，学到的容量与 R0 一点不动",
        label="清空累计与循环",
        needs="换电芯后归零用；要连学习值一起清请用「清空学习值」",
        cat="SOH"),
    # ---------------------------------------------------------------- 掉电保持
    Cmd(C_NVM_SAVE, "NVM_SAVE", "立刻把当前参数写进 Flash，不等那 60 秒的去抖",
        label="立即存 Flash",
        needs="擦写期间会关中断十几毫秒，别在放电关键期点",
        dep="BMS_USE_NVM=1", cat="掉电保持"),
    Cmd(C_NVM_INFO, "NVM_INFO", "读存了多少次、现在用哪个槽、序号是多少",
        label="读保存状态",
        up=[Field("count", "u32", label="保存次数"),
            Field("slot", "u8", label="当前槽号", note="255 = 没有有效槽"),
            Field("seq", "u16", label="序号")],
        dep="BMS_USE_NVM=1", cat="掉电保持"),
    # ---------------------------------------------------------------- 实时量
    Cmd(C_RD_LIVE, "RD_LIVE", "一条命令拿全调试最关心的五个数",
        label="读实时量",
        up=[Field("soc01", "u16", unit="%", div=100, label="SOC"),
            Field("soh01", "u16", unit="%", div=100, label="SOH"),
            Field("q_mah", "u32", unit="mAh", label="容量 Q"),
            Field("r0_mohm", "u16", unit="mΩ", label="内阻 R0"),
            Field("t", "i16", unit="°C", div=10, label="温度")],
        cat="实时量"),
]}

COMMANDS_BY_NAME = {c.name: c for c in COMMANDS.values()}

# 变长字段的长度（字节）
VAR_LEN = {"cal": _cal_len, "r0all": _r0all_len, "active": _active_len}


def field_size(f, dims):
    """一个字段在 payload 里占多少字节（不含 rc）。`temps` 返回的是"个数"。"""
    k = f.kind
    if k in ("u8", "i8"):
        return 1
    if k in ("u16", "i16"):
        return 2
    if k in ("u32", "i32", "f32"):
        return 4
    if k == "temps":
        return dims["temp_n"]
    if k == "blob":
        if f.name in VAR_LEN:
            return VAR_LEN[f.name](dims)
        if f.size is not None:
            return f.size
        raise ValueError("blob 字段 %s 没给长度" % f.name)
    raise ValueError("未知字段类型 %r" % k)


_PACK = {"u8": "<B", "i8": "<b", "u16": "<H", "i16": "<h",
         "u32": "<I", "i32": "<i", "f32": "<f"}


def decode_payload(cmd, payload, dims=None):
    """把应答 payload 解成 {rc, fields:[(名字, 数值, 显示文本, 单位, 备注)]}。

    payload[0] 恒为 rc；`rc != 0` 时后面不跟数据（规范 §4）。
    """
    d = dims_with(dims)
    rc = payload[0] if payload else 0xFF
    out = {"rc": rc, "rc_text": RC_NAME.get(rc, "未知 rc %d" % rc), "fields": []}
    if rc != 0 or len(payload) < 1:
        return out
    off = 1
    for f in cmd.up:
        if f.kind == "temps":
            cnt = d["temp_n"]
            if off + cnt * 2 > len(payload):
                out["fields"].append((f.name, None, "(数据不足)", f.unit, f.note))
                break
            vals = list(struct.unpack_from("<%dh" % cnt, payload, off))
            off += cnt * 2
            txt = " / ".join("%.1f" % (v / 10.0) for v in vals)
            out["fields"].append((f.name, vals, txt, f.unit, f.note))
            continue
        n = field_size(f, d)
        if off + n > len(payload):
            out["fields"].append((f.name, None, "(数据不足)", f.unit, f.note))
            break
        raw = payload[off:off + n]
        off += n
        if f.kind in _PACK:
            v, = struct.unpack(f.fmt or _PACK[f.kind], raw)
            txt = _fmt_value(v, f)
            out["fields"].append((f.name, v, txt, f.unit, f.note))
        else:                                    # 各种 blob
            out["fields"].append((f.name, raw, _blob_text(f, raw, d),
                                  f.unit, f.note))
    return out


def _fmt_value(v, f):
    if f.div != 1:
        return ("%.*f" % (_decimals(f.div), v / float(f.div)))
    if isinstance(v, float):
        # EKF 参数跨 6 个数量级 (p_min 1e-4 ~ r_v 1e6), %g 才不会被截成 0 或
        # 拖一长串尾零。
        return "%g" % v
    return str(v)


def _decimals(div):
    d, n = int(div), 0
    while d > 1 and d % 10 == 0:
        d //= 10
        n += 1
    return n


def _blob_text(f, raw, d):
    if len(raw) == 0:
        return "(空)"
    show = " ".join("%02X" % b for b in raw[:24])
    if len(raw) > 24:
        show += " …"
    return "%d B: %s" % (len(raw), show)


def scale_in(f, text):
    """把界面上的文本转成协议里的整数（如 "25.0" °C → 250）。"""
    s = str(text).strip()
    if s == "":
        raise ValueError("空值")
    if f.kind in ("f32",):
        return float(s)
    if f.div != 1:
        return int(round(float(s) * f.div))
    return int(s, 0) if s.lower().startswith(("0x", "-0x")) else int(float(s))


def scale_out(f, value):
    """协议整数 → 界面文本。"""
    if f.div != 1:
        return "%.*f" % (_decimals(f.div), value / float(f.div))
    return str(value)


def encode_down(cmd, values, dims=None):
    """按字段表把界面输入拼成 payload 字节串。

    `values` 是 {字段名: 文本}；blob 类字段的值直接给 bytes。
    """
    d = dims_with(dims)
    buf = bytearray()
    for f in cmd.down:
        v = values.get(f.name)
        if f.kind == "blob":
            if not isinstance(v, (bytes, bytearray)):
                raise ValueError("字段 %s 需要 bytes（这里是 %s）"
                                 % (f.name, type(v).__name__))
            buf += bytes(v)
            continue
        buf += struct.pack(f.fmt or _PACK[f.kind], scale_in(f, v))
    return bytes(buf)


# ---------------------------------------------------------------------------
# SOH 参数块（与 bms_nvm 落盘载荷逐字节同布局）
# ---------------------------------------------------------------------------
# ver 2 -> ver 3 时载荷从 68 B 加到 120 B，但**前 68 字节一个都没动**
# （放电方向 R0 / 基线 / 样本数 / 容量 / 温度 / 卡尔曼协方差 / 三个标志），
# 新增的充电方向 R0 与累计充放电全部追加在 68 之后。所以：
#   * 用 _SOH_FMT_V2 只读前 68 B 的老代码，拿到的数据仍然完全正确；
#   * 新代码一律按 120 B 读写，多出来的字段才有值。
_SOH_FMT_V2 = "<I" + "H" * 11 + "H" * 11 + "B" * 11 + "BBB" + "h" + "f"
SOH_BLOB_BYTES_V2 = struct.calcsize(_SOH_FMT_V2)    # 68

_SOH_FMT = (_SOH_FMT_V2 + "H" * 11 + "B" * 11 + "BBB"
            + "I" + "I" + "H" + "H" + "I")
SOH_BLOB_BYTES = struct.calcsize(_SOH_FMT)          # 120


def soh_blob_decode(blob):
    """解 SOH 参数块 → dict。字段名取自 soh.h 的 `soh_param_t`。

    ver 2 的 68 B 块也能解：后 52 B 按 0 填（旧板子 / 早先存下的文件）。
    """
    if len(blob) < SOH_BLOB_BYTES_V2:
        raise ValueError("SOH 块长度 %d < %d"
                         % (len(blob), SOH_BLOB_BYTES_V2))
    if len(blob) < SOH_BLOB_BYTES:
        blob = bytes(blob) + b"\x00" * (SOH_BLOB_BYTES - len(blob))
    v = struct.unpack_from(_SOH_FMT, blob, 0)
    n = 11
    # ver 3 字段的起始下标: 3n 个数组元素 + 5 个标量 (q_n/r0_any/q_any/
    # temp_dc/kf_p) + cap_mah 这一项 = 6 + 3n = 39
    b = 6 + 3 * n
    return {
        "cap_mah": v[0],
        "r0":       list(v[1:1 + n]),
        "base":     list(v[1 + n:1 + 2 * n]),
        "cnt":      list(v[1 + 2 * n:1 + 3 * n]),
        "q_n":      v[1 + 3 * n],
        "r0_any":   v[2 + 3 * n],
        "q_any":    v[3 + 3 * n],
        "temp_dc":  v[4 + 3 * n],
        "kf_p":     v[5 + 3 * n],
        # ---- ver 3 ----
        "r0_chg":     list(v[b:b + n]),
        "cnt_chg":    list(v[b + n:b + 2 * n]),
        "r0_chg_any": v[b + 2 * n],
        "cnt_any":    v[b + 2 * n + 1],
        "pad0":       v[b + 2 * n + 2],
        "cum_chg_mah": v[b + 2 * n + 3],
        "cum_dis_mah": v[b + 2 * n + 4],
        "half_cycle":  v[b + 2 * n + 5],
        "pad1":        v[b + 2 * n + 6],
        "pad2":        v[b + 2 * n + 7],
    }


def soh_blob_encode(p):
    """dict → 120 B（与 bms_nvm 的 `BMS_NVM_ParamToBytes` 同布局）。

    ver 3 新增字段缺失时按 0 填 —— 这样"把老版本读出来的 68 B dict 稍改
    一下再写回"仍然可行，不会因为少一个 key 直接报错。
    """
    return struct.pack(
        _SOH_FMT,
        p["cap_mah"], *p["r0"], *p["base"], *p["cnt"],
        p["q_n"], p["r0_any"], p["q_any"], p["temp_dc"], p["kf_p"],
        *(p.get("r0_chg") or [0] * 11),
        *(p.get("cnt_chg") or [0] * 11),
        p.get("r0_chg_any", 0), p.get("cnt_any", 0), p.get("pad0", 0),
        p.get("cum_chg_mah", 0), p.get("cum_dis_mah", 0),
        p.get("half_cycle", 0), p.get("pad1", 0), p.get("pad2", 0))


# ---------------------------------------------------------------------------
# 传输层
# ---------------------------------------------------------------------------

class SerialTransport(object):
    """pyserial 传输层。`read(n, timeout)` 返回"截至 deadline 收到的字节"。"""

    def __init__(self, port, baud, write_timeout=1.0):
        import serial                       # 延迟导入，纯协议测试不需要它
        import serial.tools.list_ports     # noqa: F401
        self.ser = serial.Serial(port, baud, timeout=0.02,
                                 write_timeout=write_timeout)
        self.port = port
        self.baud = baud

    def write(self, data):
        self.ser.write(data)

    def read(self, n, timeout=0.05):
        end = time.time() + timeout
        buf = bytearray()
        while True:
            chunk = self.ser.read(max(1, n - len(buf)))
            if chunk:
                buf += chunk
                if len(buf) >= n:
                    break
            if time.time() >= end:
                break
        return bytes(buf)

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass

    @staticmethod
    def list_ports():
        from serial.tools import list_ports
        return [(p.device, p.description) for p in list_ports.comports()]


class TuneTimeout(TimeoutError):
    """请求超时。也覆盖"CRC 错被固件静默丢弃"这种情况 —— 表象都是没应答。"""


class Reply(object):
    __slots__ = ("cmd", "rc", "payload", "fields", "raw_tx", "raw_rx", "tries")

    def __init__(self, cmd, rc, payload, decoded, raw_tx=b"", raw_rx=b"", tries=1):
        self.cmd = cmd
        self.rc = rc
        self.payload = payload
        self.fields = decoded
        self.raw_tx = raw_tx
        self.raw_rx = raw_rx
        self.tries = tries

    @property
    def ok(self):
        return self.rc == RC_OK

    def get(self, name, default=None):
        for n, v, _t, _u, _nt in self.fields:
            if n == name:
                return v
        return default

    def text(self, name):
        for n, _v, t, u, _nt in self.fields:
            if n == name:
                return ("%s %s" % (t, u)).strip()
        return ""

    def __repr__(self):
        return "Reply(cmd=0x%02X, rc=%d, fields=%s)" % (
            self.cmd, self.rc, [f[0] for f in self.fields])


# ---------------------------------------------------------------------------
# 客户端
# ---------------------------------------------------------------------------

class Client(object):
    """请求 / 应答。超时重传（规范 §8.2：CRC 错固件不应答，只能靠超时）。

    `log` 是可选回调 `log(direction, data)`，direction 为 "TX"/"RX"/"ERR"，
    界面用它显示原始字节流。
    """

    def __init__(self, transport, timeout=0.30, retries=2, dims=None, log=None):
        self.t = transport
        self.timeout = timeout
        self.retries = retries
        self.dims = dims_with(dims)
        self.log = log
        self.unfr = Unframer()
        self.stats = {"tx": 0, "rx": 0, "timeout": 0, "crc_err": 0, "odd": 0}
        self.last_tx = b""
        self.last_rx = b""

    # ---------------------------------------------------------------- 底层
    def _log(self, tag, data):
        if self.log:
            try:
                self.log(tag, data)
            except Exception:
                pass

    def request(self, cmd_or_name, payload=b"", timeout=None, retries=None):
        """发一条命令并等应答。返回 `Reply`；超时抛 `TuneTimeout`。

        `cmd_or_name` 可以是命令名、命令码，或表里没有的命令码（用于测
        "固件不认这条命令" 的路径，应答 rc=3）。
        """
        if isinstance(cmd_or_name, str):
            cmd = COMMANDS_BY_NAME[cmd_or_name]
        else:
            cmd = COMMANDS.get(cmd_or_name)
            if cmd is None:
                cmd = Cmd(cmd_or_name, "CMD_0x%02X" % cmd_or_name, "表里没有的命令")
        pay = bytes(payload)
        expect = cmd.ack_code
        timeout = self.timeout if timeout is None else timeout
        retries = self.retries if retries is None else retries

        tries = 0
        while True:
            tries += 1
            raw = build_frame(cmd.code, pay)
            self.unfr.reset()
            self._log("TX", raw)
            self.stats["tx"] += 1
            self.last_tx = raw
            try:
                self.t.write(raw)
            except Exception as e:
                self._log("ERR", str(e).encode("utf-8", "replace"))
                raise

            end = time.time() + timeout
            rx_all = bytearray()
            while time.time() < end:
                chunk = self.t.read(256, min(0.05, max(0.005, end - time.time())))
                if not chunk:
                    continue
                rx_all += chunk
                self._log("RX", chunk)
                for c, p, okc in self.unfr.feed(chunk):
                    if c != expect:
                        self.stats["odd"] += 1
                        continue
                    if not okc:
                        self.stats["crc_err"] += 1
                        continue
                    self.stats["rx"] += 1
                    self.last_rx = bytes(rx_all)
                    dec = decode_payload(cmd, p, self.dims)
                    return Reply(cmd.code, dec["rc"], p, dec["fields"],
                                 raw_tx=raw, raw_rx=bytes(rx_all), tries=tries)
            self.stats["timeout"] += 1
            if tries > retries:
                raise TuneTimeout(
                    "%s (0x%02X) 无应答：%d 次尝试，每次 %.0f ms。"
                    "排查：BMS_USE_TUNE 是否为 1 / BMS_TUNE_SetTx 是否注册 / "
                    "是否在中断里调了 Feed" % (cmd.name, cmd.code, tries,
                                              timeout * 1000))
        # 不会到这里

    # ---------------------------------------------------------------- 高层
    def ping(self, **kw):
        return self.request("PING", **kw)

    def info(self, **kw):
        r = self.request("INFO", **kw)
        if r.ok:
            self.apply_info(r)
        return r

    def apply_info(self, reply):
        """用 INFO 的应答刷新维度（GUI 之后所有长度都按它算）。"""
        for n, v, _t, _u, _nt in reply.fields:
            if n in ("proto_ver", "nvm_ver", "temp_n", "tbl_n", "soc_n",
                     "cal_bytes", "flags"):
                self.dims[n] = v
        self.dims["cap_nom_mah"] = reply.get("cap_nom_mah")
        return self.dims

    def rd_cal_point(self, ti, tbl, idx, **kw):
        return self.request(C_RD_CAL_POINT,
                            struct.pack("<BBB", ti, tbl, idx), **kw)

    def wr_cal_point(self, ti, tbl, idx, v, **kw):
        return self.request(C_WR_CAL_POINT,
                            struct.pack("<BBBH", ti, tbl, idx, v), **kw)

    def rd_cal_all(self, **kw):
        return self.request(C_RD_CAL_ALL, **kw)

    def wr_cal_all(self, blob, **kw):
        return self.request(C_WR_CAL_ALL, bytes(blob), **kw)

    def cal_restore(self, **kw):
        return self.request(C_CAL_RESTORE, **kw)

    def rd_cal_temps(self, **kw):
        return self.request(C_RD_CAL_TEMPS, **kw)

    def rd_temp(self, **kw):
        return self.request(C_RD_TEMP, **kw)

    def wr_temp(self, t_dc, **kw):
        return self.request(C_WR_TEMP, struct.pack("<h", t_dc), **kw)

    def rd_cal_active(self, **kw):
        return self.request(C_RD_CAL_ACTIVE, **kw)

    def rd_r0(self, idx, **kw):
        return self.request(C_RD_R0, struct.pack("<B", idx), **kw)

    def wr_r0(self, idx, v, **kw):
        return self.request(C_WR_R0, struct.pack("<BH", idx, v), **kw)

    def rd_r0_all(self, **kw):
        return self.request(C_RD_R0_ALL, **kw)

    def wr_r0_all(self, blob, **kw):
        return self.request(C_WR_R0_ALL, bytes(blob), **kw)

    def rd_cap(self, **kw):
        return self.request(C_RD_CAP, **kw)

    def wr_cap(self, mah, **kw):
        return self.request(C_WR_CAP, struct.pack("<I", mah), **kw)

    # ---------------------------------------------------------------- SOC / EKF
    def rd_soc(self, **kw):
        return self.request(C_RD_SOC, **kw)

    def wr_soc(self, soc01, **kw):
        """把 SOC 强制设成 soc01（0.01% 单位，0~10000）。"""
        s = int(soc01)
        if not 0 <= s <= 10000:
            raise ValueError("SOC 要在 0 ~ 10000（0.01%% 单位），给了 %r" % soc01)
        return self.request(C_WR_SOC, struct.pack("<H", s), **kw)

    def rd_ekf(self, **kw):
        return self.request(C_RD_EKF, **kw)

    @staticmethod
    def ekf_blob(vals):
        """8 个 EKF 参数 -> 32 B 下行 payload。

        `vals` 可以是 `{名字: 值}` 也可以是有序的 8 个值；顺序恒为 `EKF_SPEC`
        （= 固件的 `soc_ekf_param_t`），所以 dict 少一项就会报错而不是静默错位。
        """
        if isinstance(vals, dict):
            missing = [n for n in EKF_NAMES if n not in vals]
            if missing:
                raise ValueError("EKF 参数少了 %s（整组写必须 8 个都给）"
                                 % ", ".join(missing))
            seq = [vals[n] for n in EKF_NAMES]
        else:
            seq = list(vals)
        if len(seq) != len(EKF_SPEC):
            raise ValueError("EKF 参数要 %d 个，给了 %d 个"
                             % (len(EKF_SPEC), len(seq)))
        return b"".join(struct.pack("<f", float(x)) for x in seq)

    def wr_ekf(self, vals, **kw):
        return self.request(C_WR_EKF, self.ekf_blob(vals), **kw)

    def ekf_write_subset(self, changes, **kw):
        """读整组 -> 只改 `changes` 里的项 -> 整组写回。

        协议里只有"整组"这一个形态（一个值越界就整组不写），所以想单独改
        q_soc 也必须先把板子上的现值读回来，其余 7 个原样带上。
        返回最后那次写回的 `Reply`；读失败时返回读的 `Reply`。
        """
        r = self.rd_ekf(**kw)
        if not r.ok:
            return r
        cur = {n: r.get(n) for n in EKF_NAMES}
        cur.update(changes)
        return self.wr_ekf(cur, **kw)

    def ekf_reset(self, **kw):
        return self.request(C_EKF_RESET, **kw)

    def rd_soh_blob(self, **kw):
        return self.request(C_RD_SOH_BLOB, **kw)

    def wr_soh_blob(self, blob, **kw):
        return self.request(C_WR_SOH_BLOB, bytes(blob), **kw)

    def rd_soh_sum(self, **kw):
        return self.request(C_RD_SOH_SUM, **kw)

    def soh_reset(self, **kw):
        return self.request(C_SOH_RESET, **kw)

    def rd_soh_cnt(self, **kw):
        return self.request(C_RD_SOH_CNT, **kw)

    def soh_cnt_reset(self, **kw):
        return self.request(C_SOH_CNT_RESET, **kw)

    def rd_r0_chg_all(self, **kw):
        return self.request(C_RD_R0_CHG_ALL, **kw)

    def wr_r0_chg_all(self, blob, **kw):
        return self.request(C_WR_R0_CHG_ALL, bytes(blob), **kw)

    def rd_corr(self, **kw):
        return self.request(C_RD_CORR, **kw)

    def nvm_save(self, **kw):
        return self.request(C_NVM_SAVE, **kw)

    def nvm_info(self, **kw):
        return self.request(C_NVM_INFO, **kw)

    def rd_live(self, **kw):
        return self.request(C_RD_LIVE, **kw)


def selftest():
    """CRC / 拼帧自检向量（规范 §3.2、§7）—— 接板子之前先跑它。"""
    bad = []
    if crc16(b"123456789") != 0x29B1:
        bad.append('crc16("123456789") = 0x%04X, 期望 0x29B1'
                   % crc16(b"123456789"))
    if crc16(b"\x00\x00\x00") != 0xCC9C:
        bad.append("crc16(00 00 00) = 0x%04X, 期望 0xCC9C"
                   % crc16(b"\x00\x00\x00"))
    if crc16(bytes.fromhex("050003010105D204")) != 0xB3B4:
        bad.append("crc16(05 00 03 01 01 05 D2 04) = 0x%04X, 期望 0xB3B4"
                   % crc16(bytes.fromhex("050003010105D204")))
    if build_frame(0x00).hex(" ").upper() != "AA 55 00 00 00 9C CC":
        bad.append("PING 帧 = %s, 期望 AA 55 00 00 00 9C CC"
                   % build_frame(0x00).hex(" ").upper())
    # 命令表完整性: 5 条 SOC/EKF 命令都在，且命令码不重复
    codes = sorted(COMMANDS)
    if len(codes) != len(set(codes)):
        bad.append("命令表有重复命令码")
    for c in (C_RD_SOC, C_WR_SOC, C_RD_EKF, C_WR_EKF, C_EKF_RESET):
        if c not in COMMANDS:
            bad.append("命令表缺 0x%02X" % c)
    # 五项新功能对应的命令都在，且**上行长度与固件逐条一致**
    for c in (C_RD_SOH_CNT, C_SOH_CNT_RESET, C_RD_R0_CHG_ALL,
              C_WR_R0_CHG_ALL, C_RD_CORR):
        if c not in COMMANDS:
            bad.append("命令表缺 0x%02X" % c)
    for c, want in ((C_RD_SOH_CNT, 16), (C_SOH_CNT_RESET, 1),
                    (C_RD_R0_CHG_ALL, 23), (C_WR_R0_CHG_ALL, 1),
                    (C_RD_CORR, 15)):
        if c in COMMANDS:
            got = 1 + sum(field_size(f, DEFAULT_DIMS) for f in COMMANDS[c].up)
            if got != want:
                bad.append("0x%02X 上行长度 %d，固件是 %d" % (c, got, want))
    # SOH 参数块: 120 B, 前 68 B 与 ver 2 视图一致, 往返不丢字段
    if SOH_BLOB_BYTES != 120:
        bad.append("SOH_BLOB_BYTES = %d，期望 120" % SOH_BLOB_BYTES)
    if SOH_BLOB_BYTES_V2 != 68:
        bad.append("SOH_BLOB_BYTES_V2 = %d，期望 68" % SOH_BLOB_BYTES_V2)
    _p = dict(cap_mah=3300, r0=[55] * 11, base=[50] * 11, cnt=[3] * 11,
              q_n=2, r0_any=1, q_any=1, temp_dc=250, kf_p=0.25,
              r0_chg=[61] * 11, cnt_chg=[4] * 11, r0_chg_any=1,
              cnt_any=1, pad0=0, cum_chg_mah=12345, cum_dis_mah=23456,
              half_cycle=7, pad1=0, pad2=0)
    _b = soh_blob_encode(_p)
    if len(_b) != SOH_BLOB_BYTES:
        bad.append("soh_blob_encode 产出 %d B，期望 %d"
                   % (len(_b), SOH_BLOB_BYTES))
    else:
        _q = soh_blob_decode(_b)
        for k in ("cap_mah", "q_n", "r0_any", "q_any", "temp_dc",
                  "r0_chg_any", "cnt_any", "cum_chg_mah",
                  "cum_dis_mah", "half_cycle"):
            if _q[k] != _p[k]:
                bad.append("soh_blob 往返 %s: %r != %r" % (k, _q[k], _p[k]))
        for k in ("r0", "base", "cnt", "r0_chg", "cnt_chg"):
            if _q[k] != _p[k]:
                bad.append("soh_blob 往返数组 %s 不一致" % k)
        if struct.unpack_from(_SOH_FMT_V2, _b, 0)[0] != _p["cap_mah"]:
            bad.append("soh_blob 前 68 B 的 ver 2 视图读不到 cap_mah")
        if soh_blob_decode(_b[:68])["cum_dis_mah"] != 0:
            bad.append("68 B 老块解码时未按 0 补尾部")
    # EKF 整组读写: 长度 + 字段表解码 + dict/序列两种入参等价
    blob = Client.ekf_blob(EKF_DEFAULTS)
    if len(blob) != EKF_PARAM_BYTES:
        bad.append("EKF blob = %d B, 期望 %d" % (len(blob), EKF_PARAM_BYTES))
    if Client.ekf_blob({e["name"]: e["default"] for e in EKF_SPEC}) != blob:
        bad.append("EKF blob: dict 入参与序列入参结果不一致")
    try:
        Client.ekf_blob([1.0] * 7)
        bad.append("EKF blob 少一个参数没报错")
    except ValueError:
        pass
    vals = [0.002, 0.8, 20.0, 20.0, 500.0, 300.0, 1e-5, 1e-3]
    pay = b"\x00" + b"".join(struct.pack("<f", x) for x in vals)
    dec = decode_payload(COMMANDS[C_RD_EKF], pay)
    got = [v for _n, v, _t, _u, _nt in dec["fields"]]
    want = [struct.unpack("<f", struct.pack("<f", x))[0] for x in vals]
    if got != want:
        bad.append("RD_EKF 字段表解码结果 %s, 期望 %s" % (got, want))
    # RD_SOC 22 B 也得能解出 7 个字段
    pay = (b"\x00" + struct.pack("<HHiBfff", 4321, 4000, -123, 1,
                                 12.5, 3.25, 400.0))
    if len(pay) != 22:
        bad.append("RD_SOC 测试 payload = %d B, 期望 22" % len(pay))
    dec = decode_payload(COMMANDS[C_RD_SOC], pay)
    if [n for n, _v, _t, _u, _nt in dec["fields"]] != \
            ["soc01", "base01", "coul_mah", "flags", "kf_vrc", "p11", "p22"]:
        bad.append("RD_SOC 字段表解码出的字段名不对: %s"
                   % [n for n, _v, _t, _u, _nt in dec["fields"]])
    return bad


if __name__ == "__main__":
    p = selftest()
    if p:
        print("自检失败:")
        for x in p:
            print("  " + x)
        raise SystemExit(1)
    print("bms_tune_proto 自检通过（CRC 向量 + PING 帧逐字节一致）")
    print("命令表 %d 条:" % len(COMMANDS))
    for code in sorted(COMMANDS):
        c = COMMANDS[code]
        print("  0x%02X %-16s %s%s" % (code, c.name, c.desc,
                                      ("  [" + c.dep + "]") if c.dep else ""))
