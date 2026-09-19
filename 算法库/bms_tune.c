/*********************************************************************************
 * File Name          : bms_tune.c
 * Description        : 电池 SOC / SOH 算法库 —— 在线调参协议层 (通道无关)
 *
 * 帧格式 / 命令表 / 调用约定 见 bms_tune.h, 这里只讲实现上的取舍。
 *
 * 三条实现取舍:
 *
 *  1) 接收缓冲只有一份, 而且应答**复用**它。
 *     最大的命令是"写标定表整包" (264B), 收完 CRC 通过后立即执行, 执行完
 *     这份缓冲就没用了, 应答 (最多 265B) 直接写回同一块。省掉第二份
 *     264B 缓冲 —— 在 4~8KB RAM 的 MCU 上这 264 字节是要命的。
 *     复用安全的前提: 本模块只在主循环上下文被调用, 不存在"前一条命令
 *     还在用缓冲时后一条又进来"的可能。
 *
 *  2) 帧同步允许 AA AA 55 这种重复前导。
 *     收到 SOF0 之后如果又收到 SOF0, 停留在"等 SOF1"而不是退回起点。
 *     有些上位机/蓝牙透传会在帧前吐多余的同步字节, 这条能少丢帧。
 *
 *  3) 所有写操作都不做"范围合理性"判断, 只做"索引越界"判断。
 *     比如写容量 999999 mAh 是合法的 (库内部会钳到
 *     [SOC_CAP_LO_PCT, SOC_CAP_HI_PCT] x 标称); 写 R0 = 0 也合法。
 *     库不做"这个值看起来不像真的"的猜测 —— 那是上位机该管的事,
 *     库管的是"别让我越界访问内存"。
 *
 * 源码编码 UTF-8, 换行 LF。
 *
 * 许可: MIT (见仓库根 LICENSE)。Copyright (c) 2026 锂电池 SOC/SOH 算法库 贡献者。
 *******************************************************************************/

#include "bms_tune.h"

#if BMS_USE_TUNE

#include "soc.h"
#include "soh.h"
#include "bms_nvm.h"    /* 无条件包含: 里面的宏 (BMS_NVM_PAYLOAD_LEN 等)
                          * 与开关无关, 只有函数声明受 BMS_USE_NVM 门控 */

/* =====================================================================
 * CRC16-CCITT (poly 0x1021, init 0xFFFF, 输入/输出均不取反)
 * 参数与 bms_nvm.c 的 nvm_crc16 完全一致 —— 改一处必须改两处。
 * 没有抽成公共模块: 为 20 行代码多一个 .c/.h 不划算, 而且这一份还需要
 * 流式版本 (应答是边拼边算的), 公共模块反而更绕。
 * ===================================================================== */

static uint16_t tune_crc_up(uint16_t crc, uint8_t b)
{
    uint8_t i;
    crc ^= (uint16_t)((uint16_t)b << 8);
    for(i = 0u; i < 8u; i++)
    {
        if(crc & 0x8000u) crc = (uint16_t)((uint16_t)(crc << 1) ^ 0x1021u);
        else              crc = (uint16_t)(crc << 1);
    }
    return crc;
}

static uint16_t tune_crc_blk(const uint8_t *d, uint16_t n)
{
    uint16_t crc = 0xFFFFu;
    uint16_t i;
    for(i = 0u; i < n; i++) crc = tune_crc_up(crc, d[i]);
    return crc;
}

/* =====================================================================
 * 小端字节流读写 —— 显式逐字节, 不做 memcpy(结构体)
 * ===================================================================== */

static uint16_t rd16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t rd32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void wr16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)(v >> 8);
}

static void wr32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8)  & 0xFFu);
    p[2] = (uint8_t)((v >> 16) & 0xFFu);
    p[3] = (uint8_t)((v >> 24) & 0xFFu);
}

/* float32 —— 与 bms_nvm.c 用同一个写法 (union 换类型), 不做指针强转,
 * 也就不引 <string.h> 的 memcpy: 回避对齐与严格别名问题。
 * EKF 参数在协议上一律 IEEE-754 小端, 上位机用 struct 的 '<f' 直接解。 */
static float tune_rd_f32(const uint8_t *p)
{
    union { float f; uint32_t u; } c;

    c.u = rd32(p);
    return c.f;
}

static void tune_wr_f32(uint8_t *p, float v)
{
    union { float f; uint32_t u; } c;

    c.f = v;
    wr32(p, c.u);
}

/* =====================================================================
 * 状态机
 *
 * 缓冲布局 (刻意把帧头也存进缓冲, 这样 CRC 可以一次算完, 不用边收边算
 * 还额外维护中间状态):
 *   s_buf[0]     LEN  低字节
 *   s_buf[1]     LEN  高字节
 *   s_buf[2]     CMD
 *   s_buf[3..]   PAYLOAD
 * CRC 覆盖 s_buf[0 .. s_len+2], 共 s_len+3 字节。
 * ===================================================================== */

#define ST_SOF0     0u
#define ST_SOF1     1u
#define ST_LEN0     2u
#define ST_LEN1     3u
#define ST_CMD      4u
#define ST_PAY      5u
#define ST_CRC0     6u
#define ST_CRC1     7u

static bms_tune_tx_fn s_tx;
static uint8_t  s_st;
static uint8_t  s_cmd;
static uint16_t s_len;
static uint16_t s_idx;
static uint16_t s_crc;
static uint8_t  s_buf[BMS_TUNE_RX_MAX + 3u];
static uint16_t s_err;
static uint16_t s_cmd_n;

/* =====================================================================
 * 应答发送 —— 流式拼帧, 不占额外缓冲
 * ===================================================================== */

static void reply(uint8_t cmd, const uint8_t *pay, uint16_t n)
{
    uint16_t c = 0xFFFFu;
    uint16_t i;
    uint8_t  c8;

    if(!s_tx) return;                       /* 没注册发送回调: 丢弃应答 */

    c = tune_crc_up(c, (uint8_t)(n & 0xFFu));
    c = tune_crc_up(c, (uint8_t)(n >> 8));
    c8 = (uint8_t)(cmd | BMS_TUNE_ACK_FLAG);
    c = tune_crc_up(c, c8);
    for(i = 0u; i < n; i++) c = tune_crc_up(c, pay[i]);

    s_tx(BMS_TUNE_SOF0);
    s_tx(BMS_TUNE_SOF1);
    s_tx((uint8_t)(n & 0xFFu));
    s_tx((uint8_t)(n >> 8));
    s_tx(c8);
    for(i = 0u; i < n; i++) s_tx(pay[i]);
    s_tx((uint8_t)(c & 0xFFu));
    s_tx((uint8_t)(c >> 8));
}

static void reply_rc(uint8_t cmd, uint8_t rc)
{
    reply(cmd, &rc, 1u);
}

/* =====================================================================
 * 命令分发
 * ===================================================================== */

static void dispatch(void)
{
    uint8_t  rc  = BMS_TUNE_OK;
    uint8_t *p   = &s_buf[3u];      /* 下行 payload (注意: 写 s_buf[0..2] 安全) */
    uint8_t  ti, tbl, idx;
    uint16_t n, i, v16;
#if BMS_USE_NVM
    soh_param_t sp;
#endif

    switch(s_cmd)
    {
    /* ---------------- 链路 / 自检 ---------------- */
    case BMS_TUNE_CMD_PING:
        if(s_len != 0u) { reply_rc(s_cmd, BMS_TUNE_EBADLEN); break; }
        s_buf[0] = BMS_TUNE_OK;
        wr16(s_buf + 1u, (uint16_t)BMS_TUNE_PROTO_VER);
        reply(s_cmd, s_buf, 3u);
        break;

    case BMS_TUNE_CMD_INFO:
        if(s_len != 0u) { reply_rc(s_cmd, BMS_TUNE_EBADLEN); break; }
        s_buf[0]  = BMS_TUNE_OK;
        wr16(s_buf + 1u,  (uint16_t)BMS_TUNE_PROTO_VER);
        wr16(s_buf + 3u,  (uint16_t)BMS_NVM_VER);
        s_buf[5]  = (uint8_t)BMS_CAL_TEMP_N;
        s_buf[6]  = (uint8_t)BMS_CAL_NTBL;
        s_buf[7]  = (uint8_t)BMS_CAL_NSOC;
        wr16(s_buf + 8u,  (uint16_t)BMS_CAL_BYTES);
        s_buf[10] = (uint8_t)(BMS_USE_NVM ? 0x01u : 0x00u);   /* bit0: NVM 已启用 */
        s_buf[11] = 0u;                                       /* 保留 */
        wr32(s_buf + 12u, (uint32_t)SOC_CAPACITY_NOMINAL_MAH);
        reply(s_cmd, s_buf, 16u);
        break;

    /* ---------------- 标定表 ---------------- */
    case BMS_TUNE_CMD_RD_CAL_POINT:
        if(s_len != 3u) { reply_rc(s_cmd, BMS_TUNE_EBADLEN); break; }
        ti = p[0]; tbl = p[1]; idx = p[2];
        if(ti >= (uint8_t)BMS_CAL_TEMP_N || tbl >= (uint8_t)BMS_CAL_NTBL
                                         || idx >= (uint8_t)BMS_CAL_NSOC)
        {
            reply_rc(s_cmd, BMS_TUNE_EBADARG); break;
        }
        s_buf[0] = BMS_TUNE_OK;
        wr16(s_buf + 1u, BMS_CAL_GetPoint(ti, tbl, idx));
        reply(s_cmd, s_buf, 3u);
        break;

    case BMS_TUNE_CMD_WR_CAL_POINT:
        if(s_len != 5u) { reply_rc(s_cmd, BMS_TUNE_EBADLEN); break; }
        ti = p[0]; tbl = p[1]; idx = p[2]; v16 = rd16(p + 3u);
        if(ti >= (uint8_t)BMS_CAL_TEMP_N || tbl >= (uint8_t)BMS_CAL_NTBL
                                         || idx >= (uint8_t)BMS_CAL_NSOC)
        {
            reply_rc(s_cmd, BMS_TUNE_EBADARG); break;
        }
        BMS_CAL_SetPoint(ti, tbl, idx, v16);
        reply_rc(s_cmd, BMS_TUNE_OK);
        break;

    case BMS_TUNE_CMD_RD_CAL_ALL:
        if(s_len != 0u) { reply_rc(s_cmd, BMS_TUNE_EBADLEN); break; }
        /* 应答复用接收缓冲: s_buf[0] 留给 rc, 数据从 s_buf[1] 开始 */
        n = BMS_CAL_Serialize(s_buf + 1u, (uint16_t)(BMS_TUNE_RX_MAX + 2u));
        if(n == 0u) { reply_rc(s_cmd, BMS_TUNE_ENOSUP); break; }
        s_buf[0] = BMS_TUNE_OK;
        reply(s_cmd, s_buf, (uint16_t)(n + 1u));
        break;

    case BMS_TUNE_CMD_WR_CAL_ALL:
        if(s_len != (uint16_t)BMS_CAL_BYTES) { reply_rc(s_cmd, BMS_TUNE_EBADLEN); break; }
        /* Deserialize 内部还要校验一次长度, 双重保险; 失败时表保持原样 */
        if(!BMS_CAL_Deserialize(p, s_len)) { reply_rc(s_cmd, BMS_TUNE_EBADARG); break; }
        reply_rc(s_cmd, BMS_TUNE_OK);
        break;

    case BMS_TUNE_CMD_CAL_RESTORE:
        if(s_len != 0u) { reply_rc(s_cmd, BMS_TUNE_EBADLEN); break; }
        BMS_CAL_LoadDefault();
        reply_rc(s_cmd, BMS_TUNE_OK);
        break;

    case BMS_TUNE_CMD_RD_CAL_TEMPS:
        if(s_len != 0u) { reply_rc(s_cmd, BMS_TUNE_EBADLEN); break; }
        s_buf[0] = BMS_TUNE_OK;
        for(i = 0u; i < (uint16_t)BMS_CAL_TEMP_N; i++)
            wr16(s_buf + 1u + i * 2u, (uint16_t)BMS_CAL_GetTempAt((uint8_t)i));
        reply(s_cmd, s_buf, (uint16_t)(1u + (uint16_t)BMS_CAL_TEMP_N * 2u));
        break;

    case BMS_TUNE_CMD_RD_TEMP:
        if(s_len != 0u) { reply_rc(s_cmd, BMS_TUNE_EBADLEN); break; }
        s_buf[0] = BMS_TUNE_OK;
        wr16(s_buf + 1u, (uint16_t)BMS_CAL_GetTempDc());
        reply(s_cmd, s_buf, 3u);
        break;

    case BMS_TUNE_CMD_WR_TEMP:
        if(s_len != 2u) { reply_rc(s_cmd, BMS_TUNE_EBADLEN); break; }
        BMS_CAL_SetTempDc((int16_t)rd16(p));
        reply_rc(s_cmd, BMS_TUNE_OK);
        break;

    case BMS_TUNE_CMD_RD_CAL_ACTIVE:
        /* 当前温度插值出来的活跃表 —— 算法实际在用的那一张, 调试必备 */
        if(s_len != 0u) { reply_rc(s_cmd, BMS_TUNE_EBADLEN); break; }
        s_buf[0] = BMS_TUNE_OK;
        n = 1u;
        for(tbl = 0u; tbl < (uint8_t)BMS_CAL_NTBL; tbl++)
        {
            for(idx = 0u; idx < (uint8_t)BMS_CAL_NSOC; idx++)
            {
                wr16(s_buf + n, BMS_CAL_GetBase(tbl, idx));
                n += 2u;
            }
        }
        reply(s_cmd, s_buf, n);
        break;

    /* ---------------- R0 活跃值 (基准 + 老化增量) ---------------- */
    case BMS_TUNE_CMD_RD_R0:
        if(s_len != 1u) { reply_rc(s_cmd, BMS_TUNE_EBADLEN); break; }
        idx = p[0];
        if(idx >= (uint8_t)BMS_CAL_NSOC) { reply_rc(s_cmd, BMS_TUNE_EBADARG); break; }
        s_buf[0] = BMS_TUNE_OK;
        wr16(s_buf + 1u, SOC_GetR0At(idx));
        reply(s_cmd, s_buf, 3u);
        break;

    case BMS_TUNE_CMD_WR_R0:
        if(s_len != 3u) { reply_rc(s_cmd, BMS_TUNE_EBADLEN); break; }
        idx = p[0]; v16 = rd16(p + 1u);
        if(idx >= (uint8_t)BMS_CAL_NSOC) { reply_rc(s_cmd, BMS_TUNE_EBADARG); break; }
        SOC_SetR0At(idx, v16);          /* 传绝对值, 内部换算成相对基准的增量 */
        reply_rc(s_cmd, BMS_TUNE_OK);
        break;

    case BMS_TUNE_CMD_RD_R0_ALL:
        if(s_len != 0u) { reply_rc(s_cmd, BMS_TUNE_EBADLEN); break; }
        s_buf[0] = BMS_TUNE_OK;
        for(i = 0u; i < (uint16_t)BMS_CAL_NSOC; i++)
            wr16(s_buf + 1u + i * 2u, SOC_GetR0At((uint8_t)i));
        reply(s_cmd, s_buf, (uint16_t)(1u + (uint16_t)BMS_CAL_NSOC * 2u));
        break;

    case BMS_TUNE_CMD_WR_R0_ALL:
        if(s_len != (uint16_t)((uint16_t)BMS_CAL_NSOC * 2u))
        {
            reply_rc(s_cmd, BMS_TUNE_EBADLEN); break;
        }
        for(i = 0u; i < (uint16_t)BMS_CAL_NSOC; i++)
            SOC_SetR0At((uint8_t)i, rd16(p + i * 2u));
        reply_rc(s_cmd, BMS_TUNE_OK);
        break;

    /* ---------------- 容量 ---------------- */
    case BMS_TUNE_CMD_RD_CAP:
        if(s_len != 0u) { reply_rc(s_cmd, BMS_TUNE_EBADLEN); break; }
        s_buf[0] = BMS_TUNE_OK;
        wr32(s_buf + 1u, SOC_GetCapacityMAh());
        reply(s_cmd, s_buf, 5u);
        break;

    case BMS_TUNE_CMD_WR_CAP:
        if(s_len != 4u) { reply_rc(s_cmd, BMS_TUNE_EBADLEN); break; }
        SOC_SetCapacityMAh(rd32(p));    /* 越界值由 soc.c 钳到 [LO%, HI%] x 标称 */
        reply_rc(s_cmd, BMS_TUNE_OK);
        break;

    /* ---------------- SOC 运行态与 EKF 参数 ---------------- */
    case BMS_TUNE_CMD_RD_SOC:
        if(s_len != 0u) { reply_rc(s_cmd, BMS_TUNE_EBADLEN); break; }
        s_buf[0] = BMS_TUNE_OK;
        wr16(s_buf + 1u,  (uint16_t)SOC_GetPercent01());
        wr16(s_buf + 3u,  (uint16_t)SOC_GetBase01());
        wr32(s_buf + 5u,  (uint32_t)SOC_GetCoulombMAh());
        s_buf[9] = (uint8_t)(SOC_IsEkfReady() ? 0x01u : 0x00u);
        tune_wr_f32(s_buf + 10u, SOC_GetEkfVrc());
        tune_wr_f32(s_buf + 14u, SOC_GetEkfP11());
        tune_wr_f32(s_buf + 18u, SOC_GetEkfP22());
        reply(s_cmd, s_buf, 22u);
        break;

    case BMS_TUNE_CMD_WR_SOC:
        if(s_len != 2u) { reply_rc(s_cmd, BMS_TUNE_EBADLEN); break; }
        v16 = rd16(p);
        if(v16 > 10000u) { reply_rc(s_cmd, BMS_TUNE_EBADARG); break; }
        /* 只对齐估计器, 不动 SOH: 容量学习的两个 OCV 锚点记的是
         * SOC_VoltageToSoc01(静置端压) 而不是估计器的 SOC, 改这里
         * 影响不到 ΔSOC, 所以没有"作废当前静置窗口"的必要。 */
        SOC_ForceSetPercent01((int32_t)v16);
        reply_rc(s_cmd, BMS_TUNE_OK);
        break;

    case BMS_TUNE_CMD_RD_EKF:
        if(s_len != 0u) { reply_rc(s_cmd, BMS_TUNE_EBADLEN); break; }
        n = (uint16_t)SOC_EkfParamCount();
        s_buf[0] = BMS_TUNE_OK;
        for(i = 0u; i < n; i++)
        {
            tune_wr_f32(s_buf + 1u + i * 4u, SOC_EkfGetParam((uint8_t)i));
        }
        reply(s_cmd, s_buf, (uint16_t)(1u + n * 4u));
        break;

    case BMS_TUNE_CMD_WR_EKF:
        n = (uint16_t)SOC_EkfParamCount();
        if(s_len != (uint16_t)(n * 4u)) { reply_rc(s_cmd, BMS_TUNE_EBADLEN); break; }
        /* 两趟: 先整组校验, 有一个越界就整组不写 —— 不留"写进去一半"的
         * 参数组。上位机的用法是"读整组 -> 改 -> 写回整组", 半组状态最难查。 */
        rc = BMS_TUNE_OK;
        for(i = 0u; i < n; i++)
        {
            if(SOC_EkfParamOk((uint8_t)i, tune_rd_f32(p + i * 4u)) != 0u)
            {
                rc = BMS_TUNE_EBADARG;
                break;
            }
        }
        if(rc == BMS_TUNE_OK)
        {
            for(i = 0u; i < n; i++)
            {
                (void)SOC_EkfSetParam((uint8_t)i, tune_rd_f32(p + i * 4u));
            }
        }
        reply_rc(s_cmd, rc);
        break;

    case BMS_TUNE_CMD_EKF_RESET:
        if(s_len != 0u) { reply_rc(s_cmd, BMS_TUNE_EBADLEN); break; }
        SOC_EkfReset();
        reply_rc(s_cmd, BMS_TUNE_OK);
        break;

    /* ---------------- SOH 学习结果 ---------------- */
    case BMS_TUNE_CMD_RD_SOH_BLOB:
        if(s_len != 0u) { reply_rc(s_cmd, BMS_TUNE_EBADLEN); break; }
#if BMS_USE_NVM
        SOH_ExportParam(&sp);
        n = BMS_NVM_ParamToBytes(&sp, s_buf + 1u, (uint16_t)(BMS_TUNE_RX_MAX + 2u));
        if(n == 0u) { reply_rc(s_cmd, BMS_TUNE_ENOSUP); break; }
        s_buf[0] = BMS_TUNE_OK;
        reply(s_cmd, s_buf, (uint16_t)(n + 1u));
#else
        reply_rc(s_cmd, BMS_TUNE_ENOSUP);
#endif
        break;

    case BMS_TUNE_CMD_WR_SOH_BLOB:
        if(s_len != (uint16_t)BMS_NVM_PAYLOAD_LEN)
        {
            reply_rc(s_cmd, BMS_TUNE_EBADLEN); break;
        }
#if BMS_USE_NVM
        if(!BMS_NVM_BytesToParam(&sp, p, s_len)) { reply_rc(s_cmd, BMS_TUNE_EBADARG); break; }
        SOH_ImportParam(&sp);
        reply_rc(s_cmd, BMS_TUNE_OK);
#else
        reply_rc(s_cmd, BMS_TUNE_ENOSUP);
#endif
        break;

    case BMS_TUNE_CMD_RD_SOH_SUM:
        if(s_len != 0u) { reply_rc(s_cmd, BMS_TUNE_EBADLEN); break; }
        s_buf[0]  = BMS_TUNE_OK;
        wr16(s_buf + 1u,  (uint16_t)SOH_GetPercent01());
        s_buf[3]  = SOH_GetPercent();
        s_buf[4]  = SOH_GetCapacityPercent();
        s_buf[5]  = SOH_GetResistancePercent();
        wr32(s_buf + 6u,  SOH_GetLearnedCapacityMAh());
        wr16(s_buf + 10u, SOH_GetR0AvgMOhm());
        wr16(s_buf + 12u, (uint16_t)BMS_CAL_GetTempDc());
        s_buf[14] = (uint8_t)((SOH_IsCapacityValid() ? 0x01u : 0x00u)
                            | (SOH_IsR0Valid()       ? 0x02u : 0x00u)
                            | (SOH_IsR0ChgValid()    ? 0x04u : 0x00u)
                            | (SOH_IsCountValid()    ? 0x08u : 0x00u));
        reply(s_cmd, s_buf, 15u);
        break;

    case BMS_TUNE_CMD_SOH_RESET:
        /* 清学习值的正确顺序: 先把 R0 活跃值打回"当前温度的出厂基准",
         * 再 SOH_Init()。反过来做的话, SOH_Init() 会把"含老化量的 R0"
         * 快照成分母, SOH_R 从此恒等于 100% —— 这个坑和 bms_nvm 上电
         * 导入时"先切温度再导入"是同一类。 */
        if(s_len != 0u) { reply_rc(s_cmd, BMS_TUNE_EBADLEN); break; }
        /* 两个方向都要打回出厂基准 —— 只清放电方向的话, 充电方向那张表
         * 会留着上一次学到的值, 而 SOH_Init() 又把 base 快照成了新基准,
         * 于是充电方向的 SOH_R 显示 100% 但表已经被污染了。 */
        for(i = 0u; i < (uint16_t)BMS_CAL_NSOC; i++)
        {
            SOC_SetR0At((uint8_t)i,    BMS_CAL_GetBase(BMS_CAL_R0, (uint8_t)i));
            SOC_SetR0ChgAt((uint8_t)i, BMS_CAL_GetBase(BMS_CAL_R0, (uint8_t)i));
        }
        SOH_Init();
        reply_rc(s_cmd, BMS_TUNE_OK);
        break;

    case BMS_TUNE_CMD_RD_SOH_CNT:
        /* 累计充放电与循环计数 (bms_config.h [10])。5 个字段全是"从学到现在"
         * 的累计量, 不是瞬时值; cycle_milli 是等效满循环 x1000 (与 DOD 无关),
         * half_cycle 是摆幅法的半循环 (与电池手册循环寿命同口径)。 */
        if(s_len != 0u) { reply_rc(s_cmd, BMS_TUNE_EBADLEN); break; }
        s_buf[0] = BMS_TUNE_OK;
        wr32(s_buf + 1u,  SOH_GetCumChargeMAh());
        wr32(s_buf + 5u,  SOH_GetCumDischargeMAh());
        wr16(s_buf + 9u,  SOH_GetHalfCycleCount());
        wr32(s_buf + 11u, SOH_GetCycleMilli());
        s_buf[15] = SOH_IsCountValid();
        reply(s_cmd, s_buf, 16u);
        break;

    case BMS_TUNE_CMD_SOH_CNT_RESET:
        /* 只清计数, 学到的容量与 R0 一个不动 —— 换电芯归零用, 不该顺手把
         * 已经学了几小时的结果一起丢掉 (那是 SOH_RESET 的职责)。 */
        if(s_len != 0u) { reply_rc(s_cmd, BMS_TUNE_EBADLEN); break; }
        SOH_ResetCounters();
        reply_rc(s_cmd, BMS_TUNE_OK);
        break;

    case BMS_TUNE_CMD_RD_R0_CHG_ALL:
        /* 充电方向那张 R0 表。放电方向仍是 RD_R0_ALL (0x12)。 */
        if(s_len != 0u) { reply_rc(s_cmd, BMS_TUNE_EBADLEN); break; }
        s_buf[0] = BMS_TUNE_OK;
        for(i = 0u; i < (uint16_t)BMS_CAL_NSOC; i++)
            wr16(s_buf + 1u + i * 2u, SOC_GetR0ChgAt((uint8_t)i));
        reply(s_cmd, s_buf, (uint16_t)(1u + (uint16_t)BMS_CAL_NSOC * 2u));
        break;

    case BMS_TUNE_CMD_WR_R0_CHG_ALL:
        if(s_len != (uint16_t)((uint16_t)BMS_CAL_NSOC * 2u))
        {
            reply_rc(s_cmd, BMS_TUNE_EBADLEN); break;
        }
        for(i = 0u; i < (uint16_t)BMS_CAL_NSOC; i++)
            SOC_SetR0ChgAt((uint8_t)i, rd16(p + i * 2u));
        reply_rc(s_cmd, BMS_TUNE_OK);
        break;

    case BMS_TUNE_CMD_RD_CORR:
        /* 当前工况折算系数 K = f_T x f_I (ppm, 1e6 = 1.0) 与折算后的可用容量。
         * 调温度/倍率修正时盯这两个数: K 恒为 100% 就说明还没标定
         * (SOC_CAP_TEMP_PCT 全 100 且 SOC_PEUKERT_K = 1.0), 此时折算路径
         * 与不折算逐位相同。 */
        if(s_len != 0u) { reply_rc(s_cmd, BMS_TUNE_EBADLEN); break; }
        s_buf[0] = BMS_TUNE_OK;
        wr32(s_buf + 1u,  (uint32_t)SOC_GetCorrPpm());
        wr32(s_buf + 5u,  SOC_GetEffectiveCapacityMAh());
        wr16(s_buf + 9u,  (uint16_t)BMS_CAL_GetRefTempDc());
        wr32(s_buf + 11u, (uint32_t)BMS_CAL_GetRefCurMa());
        reply(s_cmd, s_buf, 15u);
        break;

    /* ---------------- 掉电保持 ---------------- */
    case BMS_TUNE_CMD_NVM_SAVE:
        if(s_len != 0u) { reply_rc(s_cmd, BMS_TUNE_EBADLEN); break; }
#if BMS_USE_NVM
        reply_rc(s_cmd, BMS_NVM_Save() ? BMS_TUNE_OK : BMS_TUNE_ENOSUP);
#else
        reply_rc(s_cmd, BMS_TUNE_ENOSUP);
#endif
        break;

    case BMS_TUNE_CMD_NVM_INFO:
        if(s_len != 0u) { reply_rc(s_cmd, BMS_TUNE_EBADLEN); break; }
#if BMS_USE_NVM
        s_buf[0] = BMS_TUNE_OK;
        wr32(s_buf + 1u, BMS_NVM_GetSaveCount());
        s_buf[5] = (uint8_t)BMS_NVM_GetActiveSlot();   /* -1 (无) 会变成 255 */
        wr16(s_buf + 6u, BMS_NVM_GetSeq());
        reply(s_cmd, s_buf, 8u);
#else
        reply_rc(s_cmd, BMS_TUNE_ENOSUP);
#endif
        break;

    /* ---------------- 实时量 ---------------- */
    case BMS_TUNE_CMD_RD_LIVE:
        if(s_len != 0u) { reply_rc(s_cmd, BMS_TUNE_EBADLEN); break; }
        s_buf[0] = BMS_TUNE_OK;
        wr16(s_buf + 1u,  (uint16_t)SOC_GetPercent01());
        wr16(s_buf + 3u,  (uint16_t)SOH_GetPercent01());
        wr32(s_buf + 5u,  SOH_GetLearnedCapacityMAh());
        wr16(s_buf + 9u,  SOH_GetR0AvgMOhm());
        wr16(s_buf + 11u, (uint16_t)BMS_CAL_GetTempDc());
        reply(s_cmd, s_buf, 13u);
        break;

    default:
        reply_rc(s_cmd, BMS_TUNE_ENOSUP);
        break;
    }

    (void)rc;   /* rc 只作初值/可读性用, 各分支都自己发应答 */
}

/* =====================================================================
 * 对外接口
 * ===================================================================== */

void BMS_TUNE_SetTx(bms_tune_tx_fn tx)
{
    s_tx = tx;
}

void BMS_TUNE_Feed(uint8_t b)
{
    switch(s_st)
    {
    case ST_SOF0:
        if(b == BMS_TUNE_SOF0) s_st = ST_SOF1;
        break;

    case ST_SOF1:
        /* 允许 AA AA 55 这种重复前导: 又收到 SOF0 就继续等 SOF1 */
        if(b == BMS_TUNE_SOF1)      s_st = ST_LEN0;
        else if(b == BMS_TUNE_SOF0) s_st = ST_SOF1;
        else                        s_st = ST_SOF0;
        break;

    case ST_LEN0:
        s_buf[0] = b;
        s_st = ST_LEN1;
        break;

    case ST_LEN1:
        s_buf[1] = b;
        s_len = rd16(s_buf);
        if(s_len > (uint16_t)BMS_TUNE_RX_MAX)
        {
            s_err++;                /* 超长: 不可能的帧, 直接丢 */
            BMS_TUNE_Reset();
            break;
        }
        s_st = ST_CMD;
        break;

    case ST_CMD:
        s_buf[2] = b;
        s_cmd = b;
        s_idx = 0u;
        s_st = (s_len > 0u) ? ST_PAY : ST_CRC0;
        break;

    case ST_PAY:
        s_buf[3u + s_idx] = b;
        s_idx++;
        if(s_idx >= s_len) s_st = ST_CRC0;
        break;

    case ST_CRC0:
        s_crc = (uint16_t)b;
        s_st = ST_CRC1;
        break;

    case ST_CRC1:
        s_crc |= (uint16_t)((uint16_t)b << 8);
        s_st = ST_SOF0;             /* 先复位, 这样 dispatch() 里再调 Feed 也不会重入 */
        if(s_crc == tune_crc_blk(s_buf, (uint16_t)(s_len + 3u)))
        {
            s_cmd_n++;
            dispatch();
        }
        else
        {
            s_err++;                /* CRC 错: 静默丢弃, 不应答 */
        }
        break;

    default:
        s_st = ST_SOF0;
        break;
    }
}

void BMS_TUNE_Reset(void)
{
    s_st  = ST_SOF0;
    s_len = 0u;
    s_idx = 0u;
}

uint16_t BMS_TUNE_GetErrCount(void) { return s_err; }
uint16_t BMS_TUNE_GetCmdCount(void) { return s_cmd_n; }

#endif /* BMS_USE_TUNE */
