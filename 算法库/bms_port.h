/*********************************************************************************
 * File Name          : bms_port.h
 * Description        : 电池 SOC / SOH 算法库 —— 平台移植接口
 *
 * 本库对目标平台的全部要求只有 1 个函数: BMS_Port_GetTickMs()。
 * 不需要 RTOS、不需要动态内存、不需要文件系统, 只用 <stdint.h> 和 (可选)
 * <math.h> 的 expf()。
 *
 * 接入方式 (三步, 见 README.md §2):
 *   1. 把本目录的 .c/.h 加入工程
 *   2. 实现 BMS_Port_GetTickMs() (可从 移植示例/ 里拷一个)
 *   3. 主循环里按周期调 SOC_AhUpdate() / SOH_Update()
 *
 * 源码编码 UTF-8, 换行 LF。
 *
 * 许可: MIT (见仓库根 LICENSE)。Copyright (c) 2026 锂电池 SOC/SOH 算法库 贡献者。
 *******************************************************************************/

#ifndef __BMS_PORT_H
#define __BMS_PORT_H

#include <stdint.h>

/*********************************************************************
 * @fn      BMS_Port_GetTickMs
 *
 * @brief   平台毫秒时基 —— 本库唯一需要目标平台实现的函数
 *          (BMS_USE_TICK = 0 时不需要, 见 bms_config.h)
 *
 * @return  单调递增的毫秒计数。允许 32 位回绕 —— 库内部一律用无符号
 *          差值 (now - last) 比较, 回绕安全, 只要回绕周期 (49.7 天)
 *          远大于任何一次积分间隔即可。
 *
 * @note    分辨率要求不高, 1ms 足够; 若平台只有 1us / 100us 时基,
 *          在实现里除一下换算成 ms 即可 (注意别丢高位)。
 *          不需要纳秒级精度 —— 本函数用于安时积分的 Δt 与静置计时,
 *          1ms 相对 200ms 周期就是 0.5% 分辨率, 足够了。
 *
 *          典型实现:
 *            Cortex-M / STM32 HAL : return HAL_GetTick();
 *            CH32X035             : return SWT_GetTickMs();   // 软件定时器
 *            Linux / PC 测试      : clock_gettime -> ms
 *            裸机 AVR / 无 OS     : 1ms 定时器中断里 g_ms++, 这里返回它
 */
uint32_t BMS_Port_GetTickMs(void);

/*********************************************************************
 * @fn      BMS_NowMs
 *
 * @brief   库内统一时间戳出口 —— soc.c / soh.c 只调用这一个
 *
 * @note    由 bms_port.c 实现, 按 bms_config.h 的 BMS_USE_TICK 自动切换:
 *            1 -> BMS_Port_GetTickMs()
 *            0 -> 内部按 BMS_UPDATE_PERIOD_MS 自累加
 *          so.c / soh.c 不关心用的是哪一种, 移植细节全部收在这里。
 *          应用代码不需要调用它。
 */
uint32_t BMS_NowMs(void);

#endif /* __BMS_PORT_H */
