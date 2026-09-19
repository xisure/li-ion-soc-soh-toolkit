/*********************************************************************************
 * File Name          : bms_port.c
 * Description        : 电池 SOC / SOH 算法库 —— 平台时间戳封装
 *
 * 把"有没有平台时基"这件事收在这一个文件里, soc.c / soh.c 只调 BMS_NowMs()。
 * 本文件不含任何平台专有代码, 可原样用于任何工具链。
 *
 * 源码编码 UTF-8, 换行 LF。
 *
 * 许可: MIT (见仓库根 LICENSE)。Copyright (c) 2026 锂电池 SOC/SOH 算法库 贡献者。
 *******************************************************************************/

#include "bms_port.h"
#include "bms_config.h"

#if BMS_USE_TICK
/*********************************************************************
 * @fn      BMS_NowMs
 *
 * @brief   有平台时基: 直接转发 BMS_Port_GetTickMs()
 */
uint32_t BMS_NowMs(void)
{
    return BMS_Port_GetTickMs();
}

#else /* BMS_USE_TICK = 0: 不使用时间戳, 按标称周期自累加 */
/*********************************************************************
 * @fn      BMS_NowMs
 *
 * @brief   无平台时基: 每次调用自增 BMS_UPDATE_PERIOD_MS
 *
 * @note    这种方式假定主循环周期严格等于 BMS_UPDATE_PERIOD_MS。
 *          若实际周期偏大 (比如刷屏、大量打印这类长阻塞抢占), 安时积分
 *          会系统性偏小。能用平台时基就优先用平台时基 (BMS_USE_TICK = 1)。
 *          首次调用返回 BMS_UPDATE_PERIOD_MS (不是 0), 这样第一帧的
 *          Δt 恰好等于一个周期, 不会出现 0 间隔的退化帧。
 */
uint32_t BMS_NowMs(void)
{
    static uint32_t s_ms = 0;

    s_ms += BMS_UPDATE_PERIOD_MS;
    return s_ms;
}
#endif /* BMS_USE_TICK */
