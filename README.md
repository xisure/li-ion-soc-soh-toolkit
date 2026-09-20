# 锂电池 SOC / SOH 估计算法库 + PC 标定工具链

[English](README_EN.md) | 简体中文

面向单片机的单节锂电池 **SOC（荷电状态）+ SOH（健康度）** 在线估计算法库（纯 C99），
配套一套用于标定、参数辨识与离线验证的 PC 工具链（Python）。

参考硬件：单节 18650（NCR18650GA / 18650-3350D，标称 3350 mAh）
+ CH32X035 采集板（INA226 + NTC + SSD1306）+ 一台程控电子负载（RS232 / SCPI）。
算法库本身**不绑定这套硬件**；工具链的控载脚本也只用通用 SCPI 根命令写法，
不绑定负载品牌（换品牌用 `--cmd` 覆盖几条命令即可，见 `脚本/README.md` §3.2）。

---

## 仓库结构

```
.
├── 算法库/              MCU 算法库（纯 C99，平台无关）
│   ├── README.md            算法库文档：快速开始 / 配置 / 标定表 / 掉电保持 / 调参协议
│   ├── 在线调参协议.md      ★ 写上位机看这份（帧格式 / CRC 向量 / 逐字节命令布局）
│   ├── 移植与对齐指南.md    ★ 接到自己板子上看这份（要写哪些代码 + 怎么验收）
│   ├── bms_algo.h           总入口，应用代码只 include 这一个
│   ├── bms_config.h         ★ 唯一配置入口：电芯参数 / 判据 / 周期 / 功能开关
│   ├── bms_port.h/.c        平台时基封装
│   ├── bms_cal.h/.c         多温度标定表（5 / 25 / 45 °C）+ 按当前温度插值
│   │                        + 自带级数近似（exp / ln / pow，不依赖 libm）
│   ├── bms_nvm.h/.c         掉电保持（可选，默认关闭）
│   ├── bms_tune.h/.c        在线调参协议层（可选，默认关闭）
│   ├── soc.h/.c             SOC 估计：安时积分 + OCV 重同步 + EKF
│   │                        + 温度 / 倍率修正（可用容量折算）、充放双向 R0 表
│   ├── soh.h/.c             SOH 在线学习：R0（充电 / 放电各一张表）+ 容量 Q
│   │                        + 循环次数（摆幅 / 等效满循环）+ 累计充放电
│   └── 移植示例/            各平台的时基 / 掉电保持介质适配，挑一个拷走
│
├── 脚本/                PC 标定工具链（Python 3）
│   ├── README.md            操作手册：脚本详解 + 完整测试流程 + 图形界面
│   ├── bms_gui.py           ★ 图形界面：六个页签，点点点完成标定
│   ├── bms_tune_proto.py    在线调参协议（帧层 + 34 条命令表 + 串口客户端）
│   ├── build_exe.py         用 PyInstaller 把 bms_gui.py 打成 exe
│   ├── soc_record.py        串口采集 MCU 数据帧 → CSV
│   ├── soc_load.py          程控电子负载（SCPI）自动跑十档“放电 → 静置”
│   ├── soc_bench.py         主工具：向导 + record / fit / stair 三类分析
│   ├── soh_learn.py         SOH 学习器离线验证（事后全局视角）
│   ├── soh_mcu_sim.py       SOH 状态机逐帧复刻验证（在线因果视角）
│   ├── test/kalman_tune.py  一阶 RC EKF：单组参数回放仿真 + 出轨迹图 + 网格调参
│   ├── requirements.txt     依赖清单：pyserial / numpy / matplotlib / Pillow（打包加 pyinstaller）
│   └── stair1*.csv          示例实测数据（不可再生）
│
└── 上位机/              PC 工具链的打包成品（免安装，Windows x64，已在仓库内）
    ├── 锂电池上位机.exe     ★ 不想敲命令就双击这个
    ├── _internal/          解释器 + numpy + matplotlib + tcl/tk
    └── tools/              脚本副本，exe 优先用这一份（改了立刻生效）
```

> 三块目录名就按上面这样放，无需重命名 —— 文档里的相对链接与它们一一对应。

---

## 先读哪个

| 你想做什么 | 看这里 |
|---|---|
| **把算法库移植到自己的 MCU 上** | [`算法库/AI移植指令.md`](算法库/AI移植指令.md)（给 AI 用的标准流程） |
| 同上，人工版 | [`算法库/README.md`](算法库/README.md) §3 快速开始 |
| 改电芯参数 / 判据 / 周期 | [`算法库/README.md`](算法库/README.md) §4 配置（全部在 `bms_config.h`） |
| 给标定表换自己的实测数据 | [`算法库/README.md`](算法库/README.md) §5 多温度标定表 |
| 写上位机做在线调参 | [`算法库/在线调参协议.md`](算法库/在线调参协议.md) |
| 把在线调参接到自己的板子上 | [`算法库/移植与对齐指南.md`](算法库/移植与对齐指南.md) |
| **不想敲命令，用界面** | [`脚本/README.md`](脚本/README.md) §7 图形界面（exe） |
| 跑一遍完整标定 | [`脚本/README.md`](脚本/README.md) §4 完整测试流程 |
| 采集 / 拟合 / 调参某个脚本怎么用 | [`脚本/README.md`](脚本/README.md) §3 脚本详解 |

---

## 三十秒上手

**固件侧**（只写 1 个函数就能跑）：

```c
#include "bms_algo.h"

BMS_CAL_Init();                 /* 刷出当前温度的活跃标定表 */
SOH_Init();                     /* 快照 R0 基线 + 清学习状态 */
/* 开了掉电保持的话: BMS_NVM_Init(); 必须放在 SOH_Init() 之后 */

/* 主循环里，每 200 ms： */
if (sample(&v_mv, &i_ma, &t_dc))            /* 你的采样函数 */
{
    BMS_CAL_Update(t_dc);                   /* 温度变了就刷新表 */
    SOC_AhUpdate(v_mv, i_ma);               /* 先 SOC */
    SOH_Update(v_mv, i_ma, t_dc);           /* 后 SOH */
}
```

**PC 侧**（图形界面，不想敲命令用这个）：

```bash
python 脚本/build_exe.py        # 打一次包，产物在仓库根的 上位机/
上位机/锂电池上位机.exe           # 之后双击它就行
```

命令行方式：

```bash
pip install -r 脚本/requirements.txt

python soc_record.py --out stair1.csv        # 终端 B：采数据
python soc_bench.py                          # 终端 A：向导 → 程控负载 → 十档放电
python soc_bench.py stair --csv stair1.csv --capacity 3350
```

---

## 编码约定

| 内容 | 编码 | 换行 |
|---|---|---|
| `算法库/` 下 `.c` / `.h` | UTF-8 | LF，无 BOM |
| `脚本/` 下脚本与文档 | UTF-8 | LF |

仓库根目录放了 `.gitattributes` 强制全仓库 LF，Windows 上 `git clone`
不会把源码变成 CRLF。

> 老版本工程（Keil / MounRiver 默认 GBK）接入时，把 `算法库/` 的源码
> `iconv -f UTF-8 -t GBK` 转一遍再入工程即可，改动只在注释里；
> 注意 GBK 表示不了的符号（emoji、`⚠` 等）会被静默替换成 `?`。

---

## 许可证

**MIT** —— 见 [`LICENSE`](LICENSE)。

可以自由用于商业闭源项目、修改、再发布，只需保留版权声明与许可全文；
软件按"现状"提供，不附带任何担保。
