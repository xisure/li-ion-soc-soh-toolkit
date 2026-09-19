/*********************************************************************************
 * File Name          : bms_nvm.c
 * Description        : 电池 SOC / SOH 算法库 —— 掉电保持 (平台无关实现)
 *
 * 本文件不含任何平台专有代码: 介质擦写全部通过 bms_nvm.h 里那 3 个函数
 * (BMS_NVM_SlotCount / SlotRead / SlotWrite) 转发给移植层。
 *
 * ---------------------------------------------------------------------
 * 磨损均衡 (内部 Flash 上"尽量保证寿命"的三道措施)
 *
 *   1) 多槽轮转 —— 主要手段
 *      每次落盘写到"下一个槽"而不是同一个槽, 磨损均摊到 N 个独立擦除
 *      单元上。总寿命 = 单页擦写次数 x 槽数。
 *        内部 Flash 1 万次/页:  2 槽 -> 2 万次, 4 槽 -> 4 万次
 *        外部 EEPROM 10 万次:   按"字节"计, 轮转主要在防单点损坏
 *        外部 FRAM  几乎无限:   轮转意义不大, 但也不吃亏
 *      读取时扫描全部槽, 取"序号最大且校验通过"的那一份 —— 这样即使
 *      擦写中途断电, 也只是丢掉最新的那一槽, 上一槽仍然完好可用。
 *
 *   2) 内容未变就不写
 *      BMS_NVM_SKIP_IDENTICAL = 1 时, 落盘前把序列化结果与"上次落盘
 *      内容"逐字节比较, 相同直接返回成功。很多次置脏其实没改变数据
 *      (比如 R0 某格只动 0.1mΩ 被取整抹平), 这些都能省下一次擦写。
 *
 *   3) 落盘去抖
 *      BMS_NVM_Task 距上次落盘不足 BMS_NVM_DEBOUNCE_MS 不动手, 把一轮
 *      放电里的多次学习合并成一次写 (见 bms_nvm.h)。
 *
 *   估算: 60s 去抖 + 每天 10 次学习 -> 每天 10 次擦写;
 *         2 槽 / 1 万次 = 2 万次 -> 5.5 年; 4 槽 -> 11 年。
 *         改 300s 去抖还能再乘 5。对"一台设备用几年"绰绰有余。
 *
 * ---------------------------------------------------------------------
 * 掉电安全: 头里有 magic + 版本 + 长度 + CRC16。任何一项不符就判该槽
 * 无效并继续看别的槽; 全部无效才回退标称值。绝不会读进"半截垃圾"。
 * CRC 用定点算法 (不是浮点), 保证跨平台结果一致。
 *
 * 源码编码 UTF-8, 换行 LF。
 *
 * 许可: MIT (见仓库根 LICENSE)。Copyright (c) 2026 锂电池 SOC/SOH 算法库 贡献者。
 *******************************************************************************/

#include "bms_nvm.h"
#include "bms_port.h"   /* BMS_NowMs */
#include "bms_cal.h"    /* BMS_CAL_SetTempDc: 导入前先把标定表切回落盘温度 */
#include "soc.h"        /* SOC_Get/SetCapacityMAh, SOC_Get/SetR0At 等 */

#if BMS_USE_NVM

/* =====================================================================
 * 内部状态
 * ===================================================================== */
static uint8_t  s_slot_cnt;                          /* 槽数 (0 = 不可用) */
static uint8_t  s_last_slot = 0xFFu;                 /* 最近一次写/读的槽 */
static uint16_t s_seq;                               /* 当前落盘序号 */
static uint32_t s_save_cnt;                          /* 累计成功落盘次数 */
static uint32_t s_last_save_ms;                      /* 上次落盘时刻 */
static uint8_t  s_dirty;                             /* 库自己置的脏标记 */
static uint8_t  s_have_shadow;                       /* 影子缓冲是否有效 */
static uint8_t  s_buf[BMS_NVM_BLOCK_LEN];            /* 读写缓冲 (132B) */
static uint8_t  s_shadow[BMS_NVM_PAYLOAD_LEN];       /* 上次落盘内容 (120B) */

/* =====================================================================
 * 小工具 —— 全部自己实现, 不依赖 <string.h>
 * ===================================================================== */

static void nvm_copy(uint8_t *d, const uint8_t *s, uint32_t n)
{
    uint32_t i;
    for(i = 0; i < n; i++) d[i] = s[i];
}

static uint8_t nvm_same(const uint8_t *a, const uint8_t *b, uint32_t n)
{
    uint32_t i;
    for(i = 0; i < n; i++) if(a[i] != b[i]) return 0;
    return 1;
}

static void nvm_zero(uint8_t *d, uint32_t n)
{
    uint32_t i;
    for(i = 0; i < n; i++) d[i] = 0;
}

/* ---- 小端读写 (与目标平台字节序无关) ---- */
static uint16_t nvm_rd16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static void nvm_wr16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
}

static uint32_t nvm_rd32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void nvm_wr32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
    p[2] = (uint8_t)((v >> 16) & 0xFFu);
    p[3] = (uint8_t)((v >> 24) & 0xFFu);
}

static void nvm_wr_f32(uint8_t *p, float v)
{
    union { float f; uint32_t u; } c;
    c.f = v;
    nvm_wr32(p, c.u);
}

static float nvm_rd_f32(const uint8_t *p)
{
    union { float f; uint32_t u; } c;
    c.u = nvm_rd32(p);
    return c.f;
}

/* ---- CRC16-CCITT (poly 0x1021, init 0xFFFF, 不取反) ----
 * 定点实现, 任何平台结果一致。soft_i2c 那类工程里没有现成的, 这里自带。 */
static uint16_t nvm_crc16(const uint8_t *d, uint32_t n)
{
    uint16_t crc = 0xFFFFu;
    uint32_t i;
    uint8_t  b;

    for(i = 0; i < n; i++)
    {
        crc ^= (uint16_t)((uint16_t)d[i] << 8);
        for(b = 0; b < 8; b++)
        {
            if(crc & 0x8000u) crc = (uint16_t)((crc << 1) ^ 0x1021u);
            else              crc = (uint16_t)(crc << 1);
        }
    }
    return crc;
}

/* ---- u16 序号"环回安全"比较: a 比 b 新返回 1 ----
 * 序号一直 +1, 65535 之后回 0。直接比大小在跨越回绕时会判反,
 * 用半个量程的差值判就对了 (两槽序号不可能差到 32768)。 */
static uint8_t nvm_seq_newer(uint16_t a, uint16_t b)
{
    uint16_t d = (uint16_t)(a - b);
    return (uint8_t)((d != 0u) && (d < 0x8000u));
}

/* =====================================================================
 * 序列化 —— soh_param_t <-> 固定的 120 字节小端字节流
 * (偏移定义见 bms_nvm.h 的"载荷 120B"; 改这里必须同步改 Python 解析)
 *
 * ver 3 起载荷从 68 加到 120: 前 68 字节**逐字节不变**, 新增的充电
 * 方向 R0 与累计充放电全部追加在 68 之后。这样 PC 侧只读前 68 字节的
 * 老流程不受影响, 而新工具按 120 读能拿到全部字段。
 *
 * 这两个函数**对外公开** (不是 static): 除了落盘, 串口调参模块 bms_tune
 * 收发 "SOH 参数块" 用的也是同一份字节布局。放在这里是唯一的真源 ——
 * 否则两处各写一份序列化, 改了一处忘另一处, 上位机写进去的参数就是错位
 * 的垃圾, 而且这种错误在板子上极难定位。
 * ===================================================================== */

uint16_t BMS_NVM_ParamToBytes(const soh_param_t *p, uint8_t *buf, uint16_t cap)
{
    uint8_t i;

    if(cap < (uint16_t)BMS_NVM_PAYLOAD_LEN) return 0;

    nvm_zero(buf, BMS_NVM_PAYLOAD_LEN);

    nvm_wr32(buf + 0, p->cap_mah);
    for(i = 0; i < 11u; i++) nvm_wr16(buf + 4u  + (uint32_t)i * 2u, p->r0[i]);
    for(i = 0; i < 11u; i++) nvm_wr16(buf + 26u + (uint32_t)i * 2u, p->base[i]);
    for(i = 0; i < 11u; i++) buf[48u + i] = p->cnt[i];
    buf[59] = p->q_n;
    buf[60] = p->r0_any;
    buf[61] = p->q_any;
    nvm_wr16(buf + 62, (uint16_t)p->temp_dc);   /* 落盘温度 (0.1C, 有符号) */
    nvm_wr_f32(buf + 64, p->kf_p);

    /* ---- ver 3 追加 (偏移 68 起, 前 68 字节与 ver 2 逐字节相同) ---- */
    for(i = 0; i < 11u; i++) nvm_wr16(buf + 68u + (uint32_t)i * 2u, p->r0_chg[i]);
    for(i = 0; i < 11u; i++) buf[90u + i] = p->cnt_chg[i];
    buf[101] = p->r0_chg_any;
    buf[102] = p->cnt_any;
    buf[103] = 0u;                              /* pad0 */
    nvm_wr32(buf + 104, p->cum_chg_mah);
    nvm_wr32(buf + 108, p->cum_dis_mah);
    nvm_wr16(buf + 112, p->half_cycle);
    nvm_wr16(buf + 114, 0u);                    /* pad1 */
    nvm_wr32(buf + 116, 0u);                    /* pad2 */

    return (uint16_t)BMS_NVM_PAYLOAD_LEN;
}

uint8_t BMS_NVM_BytesToParam(soh_param_t *p, const uint8_t *buf, uint16_t len)
{
    uint8_t i;

    if(len < (uint16_t)BMS_NVM_PAYLOAD_LEN) return 0;

    p->cap_mah = nvm_rd32(buf + 0);
    for(i = 0; i < 11u; i++) p->r0[i]   = nvm_rd16(buf + 4u  + (uint32_t)i * 2u);
    for(i = 0; i < 11u; i++) p->base[i] = nvm_rd16(buf + 26u + (uint32_t)i * 2u);
    for(i = 0; i < 11u; i++) p->cnt[i]  = buf[48u + i];
    p->q_n     = buf[59];
    p->r0_any  = buf[60];
    p->q_any   = buf[61];
    p->temp_dc = (int16_t)nvm_rd16(buf + 62);
    p->kf_p    = nvm_rd_f32(buf + 64);

    /* ---- ver 3 追加 (偏移 68 起) ---- */
    for(i = 0; i < 11u; i++) p->r0_chg[i]  = nvm_rd16(buf + 68u + (uint32_t)i * 2u);
    for(i = 0; i < 11u; i++) p->cnt_chg[i] = buf[90u + i];
    p->r0_chg_any = buf[101];
    p->cnt_any    = buf[102];
    p->pad0       = 0u;
    p->cum_chg_mah = nvm_rd32(buf + 104);
    p->cum_dis_mah = nvm_rd32(buf + 108);
    p->half_cycle  = nvm_rd16(buf + 112);
    p->pad1        = 0u;
    p->pad2        = 0u;

    return 1;
}

/* =====================================================================
 * 块组装与校验
 * ===================================================================== */

static void nvm_build_block(uint8_t *b, uint16_t seq, const uint8_t *pay)
{
    b[0] = BMS_NVM_MAGIC_B0;
    b[1] = BMS_NVM_MAGIC_B1;
    b[2] = BMS_NVM_MAGIC_B2;
    b[3] = BMS_NVM_MAGIC_B3;
    nvm_wr16(b + 4,  BMS_NVM_VER);
    nvm_wr16(b + 6,  seq);
    nvm_wr16(b + 8,  nvm_crc16(pay, BMS_NVM_PAYLOAD_LEN));
    nvm_wr16(b + 10, (uint16_t)BMS_NVM_PAYLOAD_LEN);
    nvm_copy(b + BMS_NVM_HDR_LEN, pay, BMS_NVM_PAYLOAD_LEN);
}

/* 返回 1 = 该槽内容可信 */
static uint8_t nvm_block_ok(const uint8_t *b)
{
    if(b[0] != BMS_NVM_MAGIC_B0 || b[1] != BMS_NVM_MAGIC_B1 ||
       b[2] != BMS_NVM_MAGIC_B2 || b[3] != BMS_NVM_MAGIC_B3)          return 0;
    if(nvm_rd16(b + 4)  != (uint16_t)BMS_NVM_VER)                     return 0;
    if(nvm_rd16(b + 10) != (uint16_t)BMS_NVM_PAYLOAD_LEN)             return 0;
    if(nvm_rd16(b + 8)  != nvm_crc16(b + BMS_NVM_HDR_LEN,
                                     BMS_NVM_PAYLOAD_LEN))            return 0;
    return 1;
}

/* =====================================================================
 * 读取: 扫描所有槽, 取序号最大且校验通过的一份
 * ===================================================================== */
static uint8_t nvm_load_best(void)
{
    uint8_t    i, best = 0xFFu;
    uint16_t   best_seq = 0u, seq;
    soh_param_t p;
    uint8_t    pay[BMS_NVM_PAYLOAD_LEN];

    for(i = 0; i < s_slot_cnt; i++)
    {
        if(!BMS_NVM_SlotRead(i, s_buf, BMS_NVM_BLOCK_LEN)) continue;
        if(!nvm_block_ok(s_buf))                           continue;

        seq = nvm_rd16(s_buf + 6);
        if(best == 0xFFu || nvm_seq_newer(seq, best_seq))
        {
            best     = i;
            best_seq = seq;
            nvm_copy(pay, s_buf + BMS_NVM_HDR_LEN, BMS_NVM_PAYLOAD_LEN);
        }
    }

    if(best == 0xFFu) return 0;      /* 从没存过 / 全被擦坏 -> 保持标称 */

    (void)BMS_NVM_BytesToParam(&p, pay, (uint16_t)BMS_NVM_PAYLOAD_LEN);

    /* 关键顺序: 先把标定表切回落盘时的温度, 再导入那时学到的 R0。
     * R0 现在按"当前温度基准 + 老化增量"存, 用同一温度的基准去减, 得到的
     * 增量才是干净的老化量; 差一个温度就会混进基准差 (低温下十几 mΩ)。
     * 切完温度后主循环第一次 BMS_CAL_Update() 会自动切到当前真实温度。 */
    BMS_CAL_SetTempDc(p.temp_dc);

    SOH_ImportParam(&p);             /* 注意: 该函数内部会重建 R0 基线 */

    s_last_slot   = best;
    s_seq         = best_seq;
    s_have_shadow = 1;
    s_dirty       = 0;
    nvm_copy(s_shadow, pay, BMS_NVM_PAYLOAD_LEN);   /* 避免导入后白写一次 */
    return 1;
}

/* =====================================================================
 * 公开接口
 * ===================================================================== */

uint8_t BMS_NVM_Init(void)
{
    s_last_save_ms = BMS_NowMs();    /* 先记时基, 免得刚上电就落一次 */
    s_save_cnt     = 0u;
    s_dirty        = 0u;
    s_have_shadow  = 0u;
    s_last_slot    = 0xFFu;
    s_seq          = 0u;

    s_slot_cnt = BMS_NVM_SlotCount();
    if(s_slot_cnt == 0u) return 0;   /* 该平台没接持久化 */

    return nvm_load_best();
}

uint8_t BMS_NVM_Save(void)
{
    uint8_t    slot, ok;
    soh_param_t p;
    uint8_t    pay[BMS_NVM_PAYLOAD_LEN];

    if(s_slot_cnt == 0u) return 0;

    SOH_ExportParam(&p);
    (void)BMS_NVM_ParamToBytes(&p, pay, (uint16_t)BMS_NVM_PAYLOAD_LEN);

#if BMS_NVM_SKIP_IDENTICAL
    /* 内容与上次落盘一致 -> 一次擦写都不用花, 直接算达成目标 */
    if(s_have_shadow && nvm_same(pay, s_shadow, BMS_NVM_PAYLOAD_LEN))
    {
        SOH_ClearDirty();
        s_dirty = 0u;
        return 1u;
    }
#endif

    /* 轮转到下一个槽 (磨损均衡); s_last_slot 初值 0xFF 时 (0xFF+1)%N == 0 */
    slot = (uint8_t)((uint8_t)(s_last_slot + 1u) % s_slot_cnt);

    nvm_build_block(s_buf, (uint16_t)(s_seq + 1u), pay);

    ok = BMS_NVM_SlotWrite(slot, s_buf, BMS_NVM_BLOCK_LEN);
    if(ok)
    {
        s_seq++;
        s_last_slot = slot;
        s_save_cnt++;
        s_have_shadow = 1;
        nvm_copy(s_shadow, pay, BMS_NVM_PAYLOAD_LEN);
        SOH_ClearDirty();
        s_dirty = 0u;
    }
    /* 失败: 不清脏、不推进序号 -> 下次 Task 再试; 旧槽内容仍然完好 */
    return ok;
}

void BMS_NVM_Task(void)
{
    uint32_t now;

    if(!BMS_NVM_IsDirty()) return;

    now = BMS_NowMs();
    if((uint32_t)(now - s_last_save_ms) < BMS_NVM_DEBOUNCE_MS) return;

    /* 成败都推进计时: 失败也只在下个去抖周期重试, 不拖住主循环 */
    s_last_save_ms = now;
    (void)BMS_NVM_Save();
}

uint8_t BMS_NVM_IsDirty(void)
{
    return (uint8_t)(SOH_IsDirty() || s_dirty);
}

void BMS_NVM_MarkDirty(void)
{
    s_dirty = 1u;
}

uint32_t BMS_NVM_GetSaveCount(void)
{
    return s_save_cnt;
}

int8_t BMS_NVM_GetActiveSlot(void)
{
    return (s_last_slot == 0xFFu) ? -1 : (int8_t)s_last_slot;
}

uint16_t BMS_NVM_GetSeq(void)
{
    return s_seq;
}

#else /* BMS_USE_NVM = 0 */

/* 未启用持久化: 本文件不产生任何代码 (不占 Flash / RAM)。
 * 保留一个内部 typedef 避免"空翻译单元"告警。 */
typedef int bms_nvm_disabled_t;

#endif /* BMS_USE_NVM */
