/*********************************************************************************
 * File Name          : port_stm32_hal.c
 * Description        : 移植示例 —— STM32 + HAL 库 (1ms SysTick)
 *
 * 用法: 把本文件拷到工程里, 若工程里已有一份 BMS_Port_GetTickMs 实现则
 *       不要重复添加 (会链接冲突)。本文件不需要修改, 直接可用。
 *
 * 说明: HAL_GetTick() 返回的就是 1ms SysTick 计数 (uwTick), 32 位回绕,
 *       正是算法库需要的形态。HAL 需要在中断里调用 HAL_IncTick(),
 *       这是 CubeMX 生成的默认行为。
 *
 * 源码编码 UTF-8, 换行 LF。
 *
 * 许可: MIT (见仓库根 LICENSE)。Copyright (c) 2026 锂电池 SOC/SOH 算法库 贡献者。
 *******************************************************************************/

#include "bms_port.h"

/* 若不用 HAL, 改成工程自己的 1ms 计数变量或函数即可 */
extern uint32_t HAL_GetTick(void);

uint32_t BMS_Port_GetTickMs(void)
{
    return HAL_GetTick();
}
