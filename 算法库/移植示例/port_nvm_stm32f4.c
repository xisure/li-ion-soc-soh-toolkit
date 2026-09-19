/*********************************************************************************
 * File Name          : port_nvm_stm32f4.c
 * Description        : bms_nvm 移植示例 —— STM32F4 系列 (HAL) 内部 Flash 末尾扇区
 *
 * 平台事实 (以 STM32F405/407/415/417, 1MB Flash 为例):
 *   扇区大小    16KB (S0~S3) / 64KB (S4) / 128KB (S5~S11)  —— 不均匀!
 *   写入粒度    字节 / 半字 / 字 / 双字 (最灵活)
 *   擦除粒度    整个扇区
 *   擦除后值    0xFF
 *
 * !!! 先说结论: F4 上"用内部 Flash 存 80 字节"代价极高 !!!
 *   最小扇区 16KB, 也就是说存 80 字节要报废 16384 字节, 利用率 0.5%。
 *   而且擦除只能整扇区, 所以轮转只能靠"多占几个扇区", 代价翻倍。
 *
 *   建议: 板上有 I2C EEPROM / SPI FRAM 就直接用它
 *         (见 port_nvm_i2c_eeprom.c) —— 字节写、无需擦除、不占程序空间。
 *   本文件保留给"板子上真的什么都没有, 又必须持久化"的情况。
 *
 * ---------------------------------------------------------------------
 * 配置区 (按你的型号核对 s_sector_addr 表!)
 *
 *   F407 (1MB) 推荐: NVM_FIRST_SECTOR = 11, NVM_SLOT_COUNT = 1
 *                    -> 只吃最后那个 128KB 扇区 (若代码 < 896KB 就安全)
 *   想双槽轮转 (寿命 x2): NVM_SLOT_COUNT = 2, 再吃掉 S10 (共 256KB)
 *   小容量 F401 (256KB): 扇区表不同, 必须按参考手册重写!
 *
 * !!! 链接脚本必须让出这些扇区 !!!
 *   FLASH LENGTH 要减到"起始扇区地址"对应的大小, 例如 F407 用 S11:
 *     LENGTH = 896K;      /* 0x080E0000 - 0x08000000 = 896KB */
 *   否则链接器会把程序放进末尾扇区, 一擦除就把代码擦了。
 * ---------------------------------------------------------------------
 *
 * 源码编码 UTF-8, 换行 LF。
 *
 * 许可: MIT (见仓库根 LICENSE)。Copyright (c) 2026 锂电池 SOC/SOH 算法库 贡献者。
 *******************************************************************************/

#include "bms_nvm.h"
#include "stm32f4xx_hal.h"

/* =====================================================================
 * 配置区
 * ===================================================================== */
#define NVM_FIRST_SECTOR    11u             /* 起始扇区号 (按型号改!) */
#define NVM_SLOT_COUNT      1u              /* 1 = 单槽; 2 = 双槽轮转(寿命x2) */

/* STM32F405/407/415/417 (1MB) 扇区起始地址表 —— 换型号必须重写这张表 */
static const uint32_t s_sector_addr[12] =
{
    0x08000000u, 0x08004000u, 0x08008000u, 0x0800C000u,   /* S0~S3  16KB */
    0x08010000u,                                          /* S4     64KB */
    0x08020000u, 0x08040000u, 0x08060000u, 0x08080000u,   /* S5~S8 128KB */
    0x080A0000u, 0x080C0000u, 0x080E0000u                 /* S9~S11 128KB */
};

static uint32_t nvm_sector_size(uint8_t sec)
{
    if(sec <= 3u)  return  16u * 1024u;
    if(sec == 4u)  return  64u * 1024u;
    return 128u * 1024u;
}

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
    if(len > nvm_sector_size((uint8_t)(NVM_FIRST_SECTOR + slot))) return 0;

    src = (const uint8_t *)s_sector_addr[NVM_FIRST_SECTOR + slot];
    for(i = 0; i < len; i++) dst[i] = src[i];

    return 1;
}

uint8_t BMS_NVM_SlotWrite(uint8_t slot, const void *buf, uint32_t len)
{
    const uint8_t         *src = (const uint8_t *)buf;
    FLASH_EraseInitTypeDef erase;
    uint8_t                sec;
    uint32_t               sec_err = 0u, addr, i;
    uint8_t                ok = 1u;

    if(slot >= NVM_SLOT_COUNT) return 0;
    if(len & 3u)               return 0;    /* 这里按"字"写: 长度须是 4 的倍数 */

    sec  = (uint8_t)(NVM_FIRST_SECTOR + slot);
    addr = s_sector_addr[sec];
    if(len > nvm_sector_size(sec)) return 0;

    HAL_FLASH_Unlock();

    /* ---- 1. 擦整个扇区 (16~128KB!) ---- */
    erase.TypeErase    = FLASH_TYPEERASE_SECTORS;
    erase.Sector       = sec;
    erase.NbSectors    = 1u;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;   /* 2.7~3.6V */
    if(HAL_FLASHEx_Erase(&erase, &sec_err) != HAL_OK) ok = 0u;

    /* ---- 2. 只写字的那 80 字节 ----
     * 擦除后扇区全是 0xFF, 后面不用写, 所以不需要 RAM 缓冲。
     * (F4 也支持 FLASH_TYPEPROGRAM_BYTE, 那样连长度限制都不用管) */
    if(ok)
    {
        for(i = 0; i < len; i += 4u)
        {
            uint32_t w = (uint32_t)src[i]
                       | ((uint32_t)src[i + 1u] << 8)
                       | ((uint32_t)src[i + 2u] << 16)
                       | ((uint32_t)src[i + 3u] << 24);

            if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                                 addr + i, (uint64_t)w) != HAL_OK)
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
 *   扇区擦写典型 10000 次。
 *   单槽 + 60s 去抖 + 每天 10 次学习 -> 1000 天 (2.7 年)。
 *   双槽轮转 -> 5.5 年 (代价: 再吃一个扇区)。
 *
 *   如果在 F4 上要长期可靠地跑, 正确做法是加一颗 I2C EEPROM
 *   (几毛钱, 两个引脚, 单字节写无需擦除, 10 万次/字节寿命),
 *   换用 port_nvm_i2c_eeprom.c, 而不是在这里加扇区。
 * ===================================================================== */
