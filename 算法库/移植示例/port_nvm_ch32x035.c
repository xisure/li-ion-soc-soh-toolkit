/*********************************************************************************
 * File Name          : port_nvm_ch32x035.c
 * Description        : bms_nvm 移植示例 —— CH32X035 (WCH), 内部 Flash 末尾两页
 *
 * 平台事实 (工程里核实过):
 *   Flash 容量   62 KB  (0x0000_0000 ~ 0x0000_F800)
 *   页大小       256 B
 *   擦除粒度     整页 (256B)
 *   擦除后值     0xFF
 *   写接口       FLASH_ROM_ERASE / FLASH_ROM_WRITE (ROM API, 内部自带
 *                unlock/lock, 调用者不要再手动 FLASH_Unlock)
 *   长度要求     必须是 256 的整数倍, 否则返回 FLASH_ALIGN_ERROR
 *                -> 所以本 port 必须准备一份 256B 整页缓冲并补 0xFF
 *   擦写期间     CPU 读不到 Flash (取指 stall) -> 必须关中断
 *
 * ---------------------------------------------------------------------
 * !!! 地址有两个视图, 这是这个平台最容易翻车的点 !!!
 *   链接/取指视图   0x0000_F600   (C 代码里的普通指针读)
 *   ROM API 视图    0x0800_F600   (FLASH_ROM_ERASE / WRITE 只认这个)
 *   传 0x0000F600 给 ROM API 会直接返回 FLASH_ADR_RANGE_ERROR, 什么都写不进去。
 *   本文件: 读用 0x0000_xxxx, 写用 0x0800_xxxx, 两个宏分开定义。
 *
 * !!! 链接脚本必须让出这两页 !!!
 *   Ld/Link.ld:  FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 0xF600
 *   (原 62K = 0xF800, 减去 2 x 256B = 512B)
 *   改完务必 Clean -> Rebuild, 否则旧 .elf 还在。
 * ---------------------------------------------------------------------
 *
 * 源码编码 UTF-8, 换行 LF。
 *
 * 许可: MIT (见仓库根 LICENSE)。Copyright (c) 2026 锂电池 SOC/SOH 算法库 贡献者。
 *******************************************************************************/

#include "bms_nvm.h"
#include "ch32x035.h"

/* =====================================================================
 * 配置区
 * ===================================================================== */
#define NVM_PAGE_SIZE       256u
#define NVM_SLOT_COUNT      2u              /* 两页 -> 双槽轮转 */

/* 两个视图, 别混用 */
#define NVM_SLOT_READ_BASE  0x0000F600u     /* 取指/读取视图 */
#define NVM_SLOT_ROM_BASE   0x0800F600u     /* ROM API 擦写视图 */

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

    src = (const uint8_t *)(NVM_SLOT_READ_BASE + (uint32_t)slot * NVM_PAGE_SIZE);
    for(i = 0; i < len; i++) dst[i] = src[i];

    return 1;
}

uint8_t BMS_NVM_SlotWrite(uint8_t slot, const void *buf, uint32_t len)
{
    /* 256B 整页缓冲。放 static 不占栈; 4 字节对齐 (uint32_t 数组天然对齐) */
    static uint32_t page[NVM_PAGE_SIZE / 4u];

    uint8_t       *p = (uint8_t *)page;
    const uint8_t *s = (const uint8_t *)buf;
    FLASH_Status   st;
    uint32_t       addr, i;

    if(slot >= NVM_SLOT_COUNT) return 0;
    if(len >  NVM_PAGE_SIZE)   return 0;

    /* ROM 写接口要求整页: 数据前面照抄, 后面补 0xFF (擦除后的值) */
    for(i = 0; i < NVM_PAGE_SIZE; i++) p[i] = 0xFFu;
    for(i = 0; i < len; i++)            p[i] = s[i];

    addr = NVM_SLOT_ROM_BASE + (uint32_t)slot * NVM_PAGE_SIZE;

    /* 擦写期间读不到 Flash: 关中断, 且只允许在主循环上下文调用。
     * 若你的 core_riscv.h 没提供这两个宏, 换成:
     *     uint32_t mie = __get_MIE();
     *     __clear_MIE();
     *     ... 擦写 ...
     *     if(mie) __set_MIE();
     * 注意 __enable_irq() 是无条件开中断, 只在主循环里用才是安全的。 */
    __disable_irq();
    st = FLASH_ROM_ERASE(addr, NVM_PAGE_SIZE);
    if(st == FLASH_COMPLETE)
    {
        st = FLASH_ROM_WRITE(addr, page, NVM_PAGE_SIZE);
    }
    __enable_irq();

    return (uint8_t)(st == FLASH_COMPLETE);
}

/* =====================================================================
 * 寿命说明
 *   页擦写典型 10000 次。
 *   2 槽 + 60s 去抖 + 每天 10 次学习 -> 20000 / 10 = 2000 天 (5.5 年)。
 *   CH32X035 的 62KB 里还剩 36KB 空闲, 想更久可以把 NVM_SLOT_COUNT
 *   提到 4 (占 1KB), 寿命变 11 年。
 *
 *   PC 端回读解析 (按 bms_nvm.h 的块格式):
 *     import struct
 *     d = struct.unpack('<IHHHH' + 'I 11H 11H 11B B B B 2x f', blob[:80])
 *     # d[0]=magic  d[1]=ver  d[2]=seq  d[3]=crc  d[4]=len  d[5]=cap_mah
 *     # d[6:17]=r0[11]  d[17:28]=base[11]  d[28:39]=cnt[11]
 * ===================================================================== */
