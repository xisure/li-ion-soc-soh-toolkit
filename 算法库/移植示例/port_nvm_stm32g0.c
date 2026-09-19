/*********************************************************************************
 * File Name          : port_nvm_stm32g0.c
 * Description        : bms_nvm 移植示例 —— STM32G0 / L4 / L5 / WB 系列 (HAL)
 *
 * 平台事实:
 *   页大小      2 KB (G0 / L4 / L5), 4 KB (WB)  —— U5 是 8KB 页 + 16 字节写
 *   写入粒度    双字 (8 字节), 必须 8 字节对齐
 *   擦除粒度    整页
 *   擦除后值    0xFF
 *
 *   页小 (2KB) + 双字写, 在 STM32 里算"比较适合存参数"的一档:
 *   2KB 页给 2~4 个槽只吃掉 4~8KB, 比 F4 的 16KB 起步厚道得多。
 *
 * ---------------------------------------------------------------------
 * 配置区 (按你的型号核对页数与页号)
 *
 *   型号            总 Flash   页大小   页数   末尾两页号
 *   STM32G030x8     64 KB      2 KB     32     30 / 31
 *   STM32G031x8     64 KB      2 KB     32     30 / 31
 *   STM32G071xB    128 KB      2 KB     64     62 / 63
 *   STM32L431xB    128 KB      2 KB     64     62 / 63
 *   STM32L476xG    1 MB        2 KB     512    510 / 511
 *
 *   页号统一用"从 0 开始"的绝对编号 (双 Bank 器件也是连续编号)。
 *
 * !!! 链接脚本必须让出这些页 !!!
 *   FLASH LENGTH 减去 NVM_SLOT_COUNT * NVM_PAGE_SIZE。
 *   例如 G071 (128KB) 用末尾 2 页: LENGTH = 124K;
 * ---------------------------------------------------------------------
 *
 * 源码编码 UTF-8, 换行 LF。
 *
 * 许可: MIT (见仓库根 LICENSE)。Copyright (c) 2026 锂电池 SOC/SOH 算法库 贡献者。
 *******************************************************************************/

#include "bms_nvm.h"
#include "stm32g0xx_hal.h"      /* L4 改成 stm32l4xx_hal.h */

/* =====================================================================
 * 配置区
 * ===================================================================== */
#define NVM_PAGE_SIZE       2048u           /* WB 改 4096u; U5 改 8192u */
#define NVM_FIRST_PAGE      62u             /* 起始页号 (见上表, 按型号改!) */
#define NVM_SLOT_COUNT      2u              /* 槽数 = 占用的页数 */

#define NVM_SLOT_ADDR(slot) \
    (FLASH_BASE + ((uint32_t)NVM_FIRST_PAGE + (uint32_t)(slot)) * NVM_PAGE_SIZE)

/* =====================================================================
 * 移植层实现
 * ===================================================================== */

uint8_t BMS_NVM_SlotCount(void)
{
    return (uint8_t)NVM_SLOT_COUNT;
}

uint8_t BMS_NVM_SlotRead(uint8_t slot, void *buf, uint32_t len)
{
    const uint8_t *src;
    uint8_t       *dst = (uint8_t *)buf;
    uint32_t       i;

    if(slot >= NVM_SLOT_COUNT) return 0;
    if(len >  NVM_PAGE_SIZE)   return 0;

    src = (const uint8_t *)NVM_SLOT_ADDR(slot);
    for(i = 0; i < len; i++) dst[i] = src[i];

    return 1;
}

uint8_t BMS_NVM_SlotWrite(uint8_t slot, const void *buf, uint32_t len)
{
    const uint8_t         *src = (const uint8_t *)buf;
    FLASH_EraseInitTypeDef erase;
    uint32_t               page_err = 0u;
    uint32_t               addr, i;
    uint8_t                ok = 1u;

    if(slot >= NVM_SLOT_COUNT) return 0;
    if(len >  NVM_PAGE_SIZE)   return 0;
    if(len & 7u)               return 0;     /* 双字写: 长度须是 8 的倍数 */

    addr = NVM_SLOT_ADDR(slot);

    HAL_FLASH_Unlock();

    /* ---- 1. 擦一整页 ---- */
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks     = FLASH_BANK_1;          /* L4/L5 没有 Banks 字段 -> 删掉这行 */
    erase.Page      = (uint32_t)NVM_FIRST_PAGE + (uint32_t)slot;
    erase.NbPages   = 1u;
    if(HAL_FLASHEx_Erase(&erase, &page_err) != HAL_OK) ok = 0u;

    /* ---- 2. 只写数据那 80 字节 (擦除后其余本就是 0xFF, 不用写) ---- */
    if(ok)
    {
        for(i = 0; i < len; i += 8u)
        {
            uint64_t dw = (uint64_t)src[i]
                        | ((uint64_t)src[i + 1u] << 8)
                        | ((uint64_t)src[i + 2u] << 16)
                        | ((uint64_t)src[i + 3u] << 24)
                        | ((uint64_t)src[i + 4u] << 32)
                        | ((uint64_t)src[i + 5u] << 40)
                        | ((uint64_t)src[i + 6u] << 48)
                        | ((uint64_t)src[i + 7u] << 56);

            if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                                 addr + i, dw) != HAL_OK)
            {
                ok = 0u;
                break;
            }
        }
    }

    HAL_FLASH_Lock();
    return ok;
}

/* =====================================================================
 * 寿命说明
 *   页擦写典型 10000 次。
 *   2 槽 + 60s 去抖 + 每天 10 次学习 -> 20000 / 10 = 2000 天 (5.5 年)。
 *   提到 4 槽 (多占 4KB) -> 11 年。页越小, 加大槽数越划算。
 * ===================================================================== */
