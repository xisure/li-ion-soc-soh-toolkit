/*********************************************************************************
 * File Name          : port_nvm_stm32f1.c
 * Description        : bms_nvm 移植示例 —— STM32F1 系列 (HAL) 内部 Flash 末尾页
 *
 * 平台事实 (务必按你手里那颗的具体型号核对):
 *   页大小      1 KB (中容量 64/128KB) 或 2 KB (大容量 >=256KB)
 *   写入粒度    半字 (2 字节), 必须 2 字节对齐
 *   擦除粒度    整页
 *   擦除后值    0xFF
 *   擦写期间    程序从 Flash 取指会被 stall; 本函数放主循环上下文调用,
 *               不要在中断里调 (bms_nvm 已经在主循环里调 BMS_NVM_Task)
 *
 * ---------------------------------------------------------------------
 * 配置区 (按你的芯片改这 3 个宏 + 链接脚本)
 *
 * 槽 = 独立的一页。给 2 片相邻的末尾页 = 双槽轮转, 寿命 x2 且抗
 * "擦除中途断电"(另一槽仍是上次的数据)。
 *
 *   型号            总 Flash   页大小   末尾两页地址
 *   STM32F103C8     64 KB      1 KB    0x0800F800 / 0x0800FC00
 *   STM32F103RB    128 KB      1 KB    0x0801F800 / 0x0801FC00
 *   STM32F103RC    256 KB      2 KB    0x0803F000 / 0x0803F800
 *   STM32F103ZE    512 KB      2 KB    0x0807F000 / 0x0807F800
 *
 * !!! 链接脚本必须让出这两页 !!!
 *   STM32 的 .ld 里 FLASH LENGTH 要减去 NVM_SLOT_COUNT * NVM_PAGE_SIZE。
 *   例如 C8: LENGTH = 64K - 2K;  否则链接器会把代码放进末尾页,
 *   一擦除就把程序擦没了。
 * ---------------------------------------------------------------------
 *
 * 源码编码 UTF-8, 换行 LF。
 *
 * 许可: MIT (见仓库根 LICENSE)。Copyright (c) 2026 锂电池 SOC/SOH 算法库 贡献者。
 *******************************************************************************/

#include "bms_nvm.h"
#include "stm32f1xx_hal.h"      /* 按你的系列改: stm32f1xx_hal.h */

/* =====================================================================
 * 配置区
 * ===================================================================== */
#define NVM_PAGE_SIZE       1024u           /* 1KB: 中容量; 大容量改 2048u */
#define NVM_SLOT_BASE       0x0800F800u     /* 末尾两页的第一页 (见上表) */
#define NVM_SLOT_COUNT      2u              /* 槽数 = 占用的页数 */

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

    if(slot >= NVM_SLOT_COUNT)   return 0;
    if(len >  NVM_PAGE_SIZE)     return 0;

    src = (const uint8_t *)(NVM_SLOT_BASE + (uint32_t)slot * NVM_PAGE_SIZE);
    for(i = 0; i < len; i++) dst[i] = src[i];

    return 1;   /* Flash 是内存映射的, 直接读即可, 不会失败 */
}

uint8_t BMS_NVM_SlotWrite(uint8_t slot, const void *buf, uint32_t len)
{
    const uint8_t         *src = (const uint8_t *)buf;
    FLASH_EraseInitTypeDef erase;
    uint32_t               page_err = 0u;
    uint32_t               addr, i;
    uint8_t                ok = 1u;

    if(slot >= NVM_SLOT_COUNT)   return 0;
    if(len >  NVM_PAGE_SIZE)     return 0;
    if(len & 1u)                 return 0;   /* 半字写: 长度必须是偶数 */

    addr = NVM_SLOT_BASE + (uint32_t)slot * NVM_PAGE_SIZE;

    HAL_FLASH_Unlock();

    /* ---- 1. 擦一整页 (Flash 只能 1->0, 必须先擦) ---- */
    erase.TypeErase   = FLASH_TYPEERASE_PAGES;
    erase.PageAddress = addr;
    erase.NbPages     = 1u;
    if(HAL_FLASHEx_Erase(&erase, &page_err) != HAL_OK) ok = 0u;

    /* ---- 2. 只写数据那 80 字节 ----
     * 擦除后整页都是 0xFF, 后面没数据的地方本来就是 0xFF, 不用写!
     * 所以这里**不需要整页 RAM 缓冲** (省 1~2KB), 直接逐半字写。
     * 这是 STM32 相对 CH32X035 的一处便利 —— 后者的 ROM 写接口要求
     * 长度是整页, 必须准备 256B 缓冲, 见 port_nvm_ch32x035.c。 */
    if(ok)
    {
        for(i = 0; i < len; i += 2u)
        {
            uint16_t hw = (uint16_t)((uint16_t)src[i] |
                                     ((uint16_t)src[i + 1u] << 8));

            /* 旧版 HAL 的形参是 uint32_t Data, 新版是 uint64_t —— 都能过 */
            if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
                                 addr + i, (uint64_t)hw) != HAL_OK)
            {
                ok = 0u;
                break;
            }
        }
    }

    HAL_FLASH_Lock();

    /* 失败必须返回 0: bms_nvm 靠返回值决定要不要清脏标记 */
    return ok;
}

/* =====================================================================
 * 寿命说明
 *   F1 页擦写典型 10000 次。
 *   2 槽轮转 -> 20000 次; 60s 去抖 + 每天 10 次学习 -> 2000 天 (5.5 年)。
 *   想更久: 把 NVM_SLOT_COUNT 提到 3~4 (多占 1~2 页),
 *           或把 bms_config.h 的 BMS_NVM_DEBOUNCE_MS 改大。
 * ===================================================================== */
