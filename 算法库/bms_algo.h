/*********************************************************************************
 * File Name          : bms_algo.h
 * Description        : 电池 SOC / SOH 算法库 —— 总入口
 *
 * 应用代码只需要 #include "bms_algo.h" 这一个头文件。
 *
 * 初始化顺序与主循环里的调用顺序有依赖关系, 弄反不会报错、只会静默出错
 * (典型症状是 SOH 恒等于 100%) —— 见 README.md §3 的"三条硬性约定"。
 *
 * 源码编码 UTF-8, 换行 LF。
 *
 * 许可: MIT (见仓库根 LICENSE)。Copyright (c) 2026 锂电池 SOC/SOH 算法库 贡献者。
 *******************************************************************************/

#ifndef __BMS_ALGO_H
#define __BMS_ALGO_H

#include "bms_config.h"   /* 全部可调参数 (电芯表 / 判据 / 周期 / 移植开关) */
#include "bms_port.h"     /* 平台接口声明 */
#include "bms_cal.h"      /* 标定表管理: 多温度点 + 按当前温度插值 */
#include "soc.h"          /* SOC 估计 */
#include "soh.h"          /* SOH 在线学习 */

#if BMS_USE_NVM
#include "bms_nvm.h"      /* 掉电保持 (bms_config.h [8] 里 BMS_USE_NVM = 1 才有) */
#endif

#if BMS_USE_TUNE
#include "bms_tune.h"     /* 在线调参协议层 (bms_config.h [9] 里 BMS_USE_TUNE = 1 才有) */
#endif

#endif /* __BMS_ALGO_H */
