/*********************************************************************************
 * File Name          : port_nvm_i2c_eeprom.c
 * Description        : bms_nvm 移植示例 —— 外部 I2C EEPROM / FRAM (首选介质)
 *
 * 为什么这是首选 (算一下就知道):
 *                    写入粒度   擦除      寿命(次/字节)   占用程序 Flash
 *   外部 EEPROM      1 字节     不需要    100,000        0
 *   外部 FRAM        1 字节     不需要    几乎无限       0
 *   内部 Flash       2~8 字节   整页擦    10,000         一页~一扇区
 *
 *   外部 EEPROM 一颗几毛钱 (AT24C32 = 4KB), 只占两个引脚, 而且"字节可覆写、
 *   不需要擦除"意味着: 本 port 的 SlotWrite 就是一次普通的 I2C 写, 没有
 *   擦除动作、没有 >0xFF 回绕问题、速度也快得多 (80B @400kHz ≈ 2ms)。
 *   按 10 万次/字节 + 4 槽轮转算, 每天学 10 次也能顶 100 年以上。
 *   => 板子上能加的话, 一律走这个。
 *
 * 支持的器件 (改 3 个宏即可, 见配置区):
 *   AT24C02   256 B    8 位地址    写周期页 8 B
 *   AT24C32   4 KB    16 位地址    写周期页 32 B
 *   AT24C256  32 KB   16 位地址    写周期页 64 B
 *   FM24CL64  8 KB    16 位地址    FRAM, 无写周期 (PAGE 设 255 即可一次写完)
 *
 * 关键实现点: EEPROM 的"写周期页"不是内存页, 而是器件内部的写缓冲边界 ——
 * 一次 I2C 写如果跨过页边界, 地址会**回卷**覆盖本页开头, 数据悄悄错乱。
 * 所以本 port 按页边界把一次写拆成多段。
 * ---------------------------------------------------------------------
 *
 * 源码编码 UTF-8, 换行 LF。
 *
 * 许可: MIT (见仓库根 LICENSE)。Copyright (c) 2026 锂电池 SOC/SOH 算法库 贡献者。
 *******************************************************************************/

#include "bms_nvm.h"
#include "stm32f1xx_hal.h"      /* 按你的平台改; 非 STM32 就把下面两个函数换成自己的 I2C 读写 */

/* =====================================================================
 * 配置区
 * ===================================================================== */
#define NVM_EEPROM_ADDR     0xA0u       /* 7 位地址左移 1 位 (A0/A1/A2 全接地) */
#define NVM_EEPROM_A16      1u          /* 16 位内存地址 (AT24C32+); AT24C02 填 0 */
/* 写周期页: AT24C02=8, AT24C32/64=32, AT24C256=64, FRAM=255 */
#define NVM_EEPROM_PAGE     32u

#define NVM_SLOT_SIZE       128u        /* 每槽预留字节数 (>= 80 即可, 留余量便于以后扩展) */
#define NVM_SLOT_BASE       0x0000u     /* EEPROM 内的起始偏移 (给别的数据留出前段就改这里) */
#define NVM_SLOT_COUNT      4u          /* 槽数 = 磨损轮转份数 (4 槽 x 128B = 512B) */

/* 你的 I2C 句柄 (在别的文件里定义) */
extern I2C_HandleTypeDef nvm_i2c;

/* =====================================================================
 * I2C 原语 —— 换成你自己的平台代码
 * ===================================================================== */

static uint8_t ee_write_raw(uint16_t mem, const uint8_t *buf, uint16_t len)
{
#if NVM_EEPROM_A16
    return (uint8_t)(HAL_I2C_Mem_Write(&nvm_i2c, NVM_EEPROM_ADDR, mem,
                I2C_MEMADD_SIZE_16BIT, (uint8_t *)buf, len, 100u) == HAL_OK);
#else
    return (uint8_t)(HAL_I2C_Mem_Write(&nvm_i2c, NVM_EEPROM_ADDR, (uint16_t)(mem & 0xFFu),
                I2C_MEMADD_SIZE_8BIT, (uint8_t *)buf, len, 100u) == HAL_OK);
#endif
}

static uint8_t ee_read_raw(uint16_t mem, uint8_t *buf, uint16_t len)
{
#if NVM_EEPROM_A16
    return (uint8_t)(HAL_I2C_Mem_Read(&nvm_i2c, NVM_EEPROM_ADDR, mem,
                I2C_MEMADD_SIZE_16BIT, buf, len, 100u) == HAL_OK);
#else
    return (uint8_t)(HAL_I2C_Mem_Read(&nvm_i2c, NVM_EEPROM_ADDR, (uint16_t)(mem & 0xFFu),
                I2C_MEMADD_SIZE_8BIT, buf, len, 100u) == HAL_OK);
#endif
}

/* 按写周期页边界拆段写 —— 跨页回卷是 EEPROM 最隐蔽的坑 */
static uint8_t ee_write_paged(uint16_t mem, const uint8_t *buf, uint16_t len)
{
    uint16_t done = 0u;

    while(done < len)
    {
        uint16_t room = (uint16_t)(NVM_EEPROM_PAGE - ((mem + done) % NVM_EEPROM_PAGE));
        uint16_t n    = (uint16_t)(len - done);

        if(n > room) n = room;

        if(!ee_write_raw((uint16_t)(mem + done), buf + done, n)) return 0;

        done = (uint16_t)(done + n);
    }
    return 1;
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
    uint16_t mem;

    if(slot >= NVM_SLOT_COUNT) return 0;
    if(len >  NVM_SLOT_SIZE)   return 0;

    mem = (uint16_t)(NVM_SLOT_BASE + (uint16_t)slot * NVM_SLOT_SIZE);
    return ee_read_raw(mem, (uint8_t *)buf, (uint16_t)len);
}

uint8_t BMS_NVM_SlotWrite(uint8_t slot, const void *buf, uint32_t len)
{
    uint16_t mem;

    if(slot >= NVM_SLOT_COUNT) return 0;
    if(len >  NVM_SLOT_SIZE)   return 0;

    mem = (uint16_t)(NVM_SLOT_BASE + (uint16_t)slot * NVM_SLOT_SIZE);

    /* EEPROM / FRAM 字节可覆写, 不需要擦除; 直接写即可。
     * (对比: 内部 Flash 的实现必须"先擦整页再写", 见 port_nvm_stm32*.c) */
    return ee_write_paged(mem, (const uint8_t *)buf, (uint16_t)len);
}

/* =====================================================================
 * 注意
 *   1) I2C 挂了 / 器件没焊 -> 返回 0, bms_nvm 不清脏标记, 下个去抖
 *      周期自动重试。绝不会把状态搞坏。
 *   2) EEPROM 的写周期约 5ms, 一次 80 字节的落盘大约 2ms(I2C) + 5ms
 *      (写周期) = 7ms; FRAM 没有写周期, 约 2ms。都在主循环里跑没问题。
 *   3) 不要放在中断里 —— 阻塞式 I2C 会拖住中断。
 *   4) 总线要上拉 (4.7k 典型), 走线别和功率回路并排。
 * ===================================================================== */
