/*********************************************************************************
 * File Name          : port_bare_1ms_isr.c
 * Description        : 移植示例 —— 裸机 / 无 OS: 自己开 1ms 定时器中断累加
 *
 * 用法 (三步):
 *   1. 把本文件拷进工程, 在定时器初始化里调用 BMS_TickSource_Init()
 *      (或直接把 TIM_*_IRQHandler 的内容抄到工程已有的 1ms 中断里)
 *   2. 在 1ms 定时器中断服务函数里调用 BMS_TickSource_ISR()
 *   3. 中断优先级任意 (本函数只做一次自增, 不在中断里调算法)
 *
 * 说明: 32 位 MCU 上对 uint32_t 的读/写是单指令, 天然原子, 所以这里不加
 *       临界区。若目标平台是 8/16 位机 (读 uint32 会拆成多条指令), 请把
 *       BMS_TickSource_ISR 里的自增和 BMS_Port_GetTickMs 里的读取都放进
 *       临界区 (关中断 / 恢复)。
 *
 * 源码编码 UTF-8, 换行 LF。
 *
 * 许可: MIT (见仓库根 LICENSE)。Copyright (c) 2026 锂电池 SOC/SOH 算法库 贡献者。
 *******************************************************************************/

#include "bms_port.h"

/* 1ms 节拍计数。volatile: 中断里写, 主循环读 */
static volatile uint32_t s_tick_ms = 0;

/*********************************************************************
 * @fn      BMS_TickSource_ISR
 *
 * @brief   放到 1ms 定时器中断服务函数里 (只做自增, 保持中断短小)
 *
 * @note    s_tick_ms 允许 32 位回绕 (49.7 天), 算法库内部用无符号差值
 *          比较, 回绕安全。若希望绝对不回绕, 可改成 uint64_t (代价是
 *          读取不再原子, 需要临界区)。
 */
void BMS_TickSource_ISR(void)
{
    s_tick_ms++;
}

/*********************************************************************
 * @fn      BMS_TickSource_Init
 *
 * @brief   上电清零 (可选; 不做也不影响, 只是起始值不为 0)
 */
void BMS_TickSource_Init(void)
{
    s_tick_ms = 0;
}

/*********************************************************************
 * @fn      BMS_Port_GetTickMs
 *
 * @brief   算法库要求的平台时基
 */
uint32_t BMS_Port_GetTickMs(void)
{
    return s_tick_ms;
}
