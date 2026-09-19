/*********************************************************************************
 * File Name          : port_ch32x035.c
 * Description        : 移植示例 —— CH32X035 (sw_timer.c 软件定时器)
 *
 * 用法: 把本文件拷到工程里即可。前提是工程里已经有 sw_timer.c 且
 *       SWT_Init() 被调用过 (它负责起 SysTick 1ms 节拍并维护 s_tick_ms)。
 *
 * 说明: sw_timer.c 的 SWT_GetTickMs() 返回的就是 1ms 软件节拍, 32 位回绕,
 *       正是算法库需要的形态, 所以这里只是一层转发, 不需要额外开定时器。
 *
 *       注意 sw_timer.c 的 SysTick_Handler 在入口才重装 CMP, 节拍会
 *       系统性偏慢约 0.06~0.1% (24h 累计 43~86s)。固件内部自洽 (调度与
 *       积分用同一个钟), 但如果对绝对时间精度有要求, 需要另行校准。
 *
 * 源码编码 UTF-8, 换行 LF。
 *
 * 许可: MIT (见仓库根 LICENSE)。Copyright (c) 2026 锂电池 SOC/SOH 算法库 贡献者。
 *******************************************************************************/

#include "bms_port.h"

/* sw_timer.c 提供的 1ms 软件节拍 (uint32_t, 回绕安全) */
extern uint32_t SWT_GetTickMs(void);

uint32_t BMS_Port_GetTickMs(void)
{
    return SWT_GetTickMs();
}
