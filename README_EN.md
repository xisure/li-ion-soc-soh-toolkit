# Li-ion SOC / SOH Estimation Library + PC Calibration Toolchain

[English](README_EN.md) | [简体中文](README.md)

A battery **SOC (State of Charge) + SOH (State of Health)** online estimation
library for microcontrollers (pure C99), plus a Python PC toolchain for cell
characterization, parameter identification, and offline verification.

> **Language note**: the in-depth documentation is written in Chinese (the
> project's primary audience). Code comments are in Chinese as well. This
> file covers the essentials in English; see the linked documents for details.

Reference hardware: a single 18650 cell (NCR18650GA / 18650-3350D, 3350 mAh)
+ a CH32X035 acquisition board (INA226 + NTC + SSD1306) + a programmable
electronic load or power supply (RS-232 / SCPI). **The library itself is not
bound to this hardware** — it only needs one platform function. The load
control scripts use generic SCPI root commands; a different instrument brand
can be adopted by overriding a few commands with `--cmd`.

---

## Features

- **Hybrid SOC estimation** — coulomb counting under load; OCV look-up-table
  resynchronization after a settled period; a first-order RC Extended Kalman
  Filter (EKF) runs in parallel for terminal-voltage smoothing, polarization
  observation, and fine SOC correction
- **Online SOH learning** — R0 learned from current-step ΔV/ΔI (separate
  charge/discharge tables); capacity Q learned from settled OCV anchors with
  scalar-Kalman fusion; results persist across power cycles (optional NVM)
- **Multi-temperature calibration tables** — three temperature points
  (5 / 25 / 45 °C), linear interpolation at run time; aging deltas and
  temperature baselines are strictly decoupled so a temperature switch never
  wipes learned data
- **Operating-condition correction** — effective capacity adjusted for
  temperature and discharge rate (Peukert); bit-identical to "off" until
  calibrated
- **Cycle counting** — half cycles (swing method) + equivalent full cycles +
  cumulative charge/discharge, sharing the same persisted payload
- **Optional NVM persistence** — 132-byte endian-safe record, CRC-protected,
  multi-slot wear leveling; compiles to zero bytes when disabled
- **Optional online tuning** — channel-agnostic binary protocol (UART / USB
  CDC / CAN / BLE), 34 commands to read/write calibration tables and
  parameters at run time
- **Tiny porting cost** — default requirement is a single function:
  `BMS_Port_GetTickMs()`

## Resource footprint (measured, RISC-V GCC `-Os`)

| Item | Core | + NVM | + Tuning |
|---|---|---|---|
| Flash | 12.6 KB | +2.3 KB | +4.7 KB |
| RAM | 375 B | +266 B | +291 B |

C99, `<stdint.h>` only; `expf()` can be compiled out (built-in Taylor
approximation). No dynamic memory, no recursion, max stack frame < 64 B.

---

## Repository layout

```
├── 算法库/    (library)   MCU algorithm library — 14 C99 source files,
│                         porting guides, protocol spec, 8 port examples
├── 脚本/      (scripts)   PC toolchain — acquisition, load control, fitting,
│                         SOH verification, EKF tuning, GUI + sample data
└── 上位机/    (host app)  PyInstaller-packaged GUI exe (git-ignored build
                          artifact, not source)
```

## Thirty-second start

**Firmware** (implement one function, then):

```c
#include "bms_algo.h"

BMS_CAL_Init();                 /* build the active table for current temp */
SOH_Init();                     /* snapshot R0 baselines, clear learning  */

/* in the main loop, every BMS_UPDATE_PERIOD_MS (default 200 ms): */
if (sample(&v_mv, &i_ma, &t_dc))            /* your sampling function */
{
    BMS_CAL_Update(t_dc);                   /* refresh table if T changed */
    SOC_AhUpdate(v_mv, i_ma);               /* SOC first */
    SOH_Update(v_mv, i_ma, t_dc);           /* SOH after */
}
```

**PC**:

```bash
pip install -r 脚本/requirements.txt
python 脚本/soc_bench.py                          # wizard → load control → 10-step discharge
python 脚本/soc_bench.py stair --csv stair1.csv --capacity 3350   # fit R0/R1/τ + OCV table
```

A GUI (tkinter) is also included: `python 脚本/bms_gui.py`, or build a
standalone exe with `python 脚本/build_exe.py`.

Sample real-cell measurement data (`脚本/stair1.csv`, 10-step
discharge-rest profile at 5 Hz) ships with the repo, so every analysis step
can be exercised **without any hardware**.

---

## Documentation map

| What you want to do | Read |
|---|---|
| Port the library to your MCU (recommended flow for AI-assisted porting) | [`算法库/AI移植指令.md`](算法库/AI移植指令.md) |
| Same, manual walkthrough | [`算法库/README.md`](算法库/README.md) §3 |
| Change cell parameters / thresholds / periods | [`算法库/README.md`](算法库/README.md) §4 (all in `bms_config.h`) |
| Re-calibrate the tables with your own data | [`算法库/README.md`](算法库/README.md) §5, [`脚本/README.md`](脚本/README.md) §4 |
| Write a host tool for online tuning | [`算法库/在线调参协议.md`](算法库/在线调参协议.md) |
| Wire tuning into your board | [`算法库/移植与对齐指南.md`](算法库/移植与对齐指南.md) |
| Use the GUI | [`脚本/README.md`](脚本/README.md) §7 |

Key English-language entry points inside the docs: algorithm summary,
resource table, and known limitations are in
[`算法库/README.md`](算法库/README.md) §1, §9, §10 (Chinese).

## Known limitations

- Shipped calibration tables are measured at 25 °C only (5/45 °C are
  placeholders — identical copies of 25 °C); **re-calibration is mandatory
  for a different cell chemistry or capacity**
- Capacity learning requires ≥ 600 s rests and ≥ 15 % SOC swing / 200 mAh
  per event by design; shallow cycles cannot update it
- R0 sampling is restricted to 15–35 °C
- Online tuning performs **no authentication** — keep `BMS_USE_TUNE = 0`
  in production firmware
- Single-cell (1S) only

## CI

GitHub Actions (`.github/workflows/ci.yml`): compiles the library with
`gcc -std=c99 -Wall -Wextra -Werror` across all feature-switch combinations
(NVM × tuning × libm-free × tick-free), verifies every text file decodes as
UTF-8, and runs the firmware↔PC constant cross-check (`脚本/test/check_consts.py`).

## License

**MIT** — see [`LICENSE`](LICENSE). Free for commercial closed-source use;
keep the copyright notice and license text.
