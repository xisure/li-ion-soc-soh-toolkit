/*********************************************************************************
 * File Name          : bms_cal.c
 * Description        : 标定表管理 —— 多温度点 + 线性插值 (详见 bms_cal.h)
 *
 * 实现取舍说明:
 *
 *  1) 标定基准表放 RAM 而不是 const, 是为了让 BMS_CAL_SetPoint() 能在运行时
 *     改表 (后续串口改表要用)。代价 264 B RAM + 一份 264 B 的 Flash 出厂副本。
 *     用不起这 264 B 的平台: 把 s_ref 改成从 c_def_* 直接读 (去掉 set/load
 *     default 两个函数), 能省回 264 B, 但就不能运行时改表了。
 *
 *  2) 插值权重用 Q8 定点 (0~256) 而不是 float: 无 FPU 的 MCU 上省掉
 *     每点一次浮点乘加, 11 点 x 4 表 = 44 次。精度 1/256 对 mΩ / mV
 *     量级完全够 (R0 插值误差 < 0.02 mΩ)。
 *
 *  3) 端点平延不报错: 温度超出标定范围说明电芯在极端工况, 此时用最近的
 *     标定点比外推安全, 也不会让算法拿到离谱的数。
 *
 *  4) 本模块除了 4 张 SOC 标定表, 还管"工况折算系数" (容量随温度 / 倍率的
 *     修正, bms_config.h [10]): 容量温度系数表 c_cap_temp_pct 与
 *     BMS_CAL_TempFactorF / BMS_CAL_RateFactorF。放在这里是因为它们与标定表
 *     同属"电芯参数", 且都按同一批温度点插值。
 *
 * 源码编码 UTF-8, 换行 LF。
 *
 * 许可: MIT (见仓库根 LICENSE)。Copyright (c) 2026 锂电池 SOC/SOH 算法库 贡献者。
 *******************************************************************************/

#include "bms_cal.h"

#define NTEMP   BMS_CAL_TEMP_N
#define NTBL    BMS_CAL_NTBL
#define NSOC    BMS_CAL_NSOC

/* =====================================================================
 * 温度点与出厂标定值 (const, 放 Flash)
 * ===================================================================== */
static const int16_t c_temp_dc[NTEMP] = BMS_CAL_TEMP_DC;

static const uint16_t c_def_ocv[NTEMP][NSOC] = SOC_OCV_TABLE_MV;
static const uint16_t c_def_r0 [NTEMP][NSOC] = SOC_R0_TABLE_MOHM;
static const uint16_t c_def_r1 [NTEMP][NSOC] = SOC_R1_TABLE_MOHM;
static const uint16_t c_def_tau[NTEMP][NSOC] = SOC_TAU_TABLE_S;

/* 可用容量的温度系数 (%) —— 见 bms_config.h [10]。
 * 元素个数必须等于 NTEMP: 少了会编译报错 (bms_config.h 里有 #if 兜底),
 * 多了 GCC 只给 warning, 所以改 BMS_CAL_TEMP_DC 时两处一起改。 */
static const uint16_t c_cap_temp_pct[NTEMP] = SOC_CAP_TEMP_PCT;

/* 编译期断言: BMS_CAL_TEMP_N 与 BMS_CAL_TEMP_DC 的元素个数必须一致。
 * 表达式是整型常量, 不触发 -Wvariably-modified。 */
typedef char bms_cal_temp_n_check_t
    [(sizeof(c_temp_dc) / sizeof(c_temp_dc[0]) == NTEMP) ? 1 : -1];
/* 至少要两个温度点, 否则插值无从谈起 */
typedef char bms_cal_temp_min_check_t[(NTEMP >= 2) ? 1 : -1];

/* =====================================================================
 * 工作副本与活跃表 (RAM)
 * ===================================================================== */
static uint16_t s_ref[NTBL][NTEMP][NSOC];   /* 标定基准 (可被 SetPoint 改) */
static uint16_t s_act[NTBL][NSOC];          /* 当前温度插值出来的活跃表 */
static int16_t  s_cur_dc;                   /* s_act 对应的温度 */
static uint8_t  s_inited;
static uint8_t  s_dirty;

/* =====================================================================
 * 内部工具
 * ===================================================================== */

static uint16_t def_get(uint8_t tbl, uint8_t ti, uint8_t idx)
{
    switch(tbl)
    {
    case BMS_CAL_OCV: return c_def_ocv[ti][idx];
    case BMS_CAL_R0:  return c_def_r0[ti][idx];
    case BMS_CAL_R1:  return c_def_r1[ti][idx];
    default:          return c_def_tau[ti][idx];
    }
}

/* 线性插值: a + (b - a) * w / 256,  w 是 Q8 定点 */
static uint16_t lerp(uint16_t a, uint16_t b, int32_t w)
{
    int32_t v = (int32_t)a + ((((int32_t)b - (int32_t)a) * w) >> 8);
    if(v < 0)      v = 0;
    if(v > 65535)  v = 65535;
    return (uint16_t)v;
}

/* 按 s_cur_dc 重算 s_act */
static void refresh(void)
{
    uint8_t k, i, lo = 0;
    int32_t w = 0;

    if(s_cur_dc <= c_temp_dc[0])                 /* 低于最低点: 平延 */
    {
        lo = 0;
        w  = 0;
    }
    else if(s_cur_dc >= c_temp_dc[NTEMP - 1])    /* 高于最高点: 平延 */
    {
        lo = (uint8_t)(NTEMP - 2);
        w  = 256;
    }
    else
    {
        /* 找包住 s_cur_dc 的那个区间 [lo, lo+1]。
         *
         * 这里不能写成 "if (cur < c_temp_dc[k+1]) { lo = k; break; }" ——
         * 一旦 cur 落在最后一个区间 (25~45C) 里, 条件不成立、lo 就一直停在
         * 初值 0, 权重会按**第一个区间**的跨度去算: 35C 时
         * w = (350-50)*256/200 = 384 > 256, lerp 变成拿 5C->25C 的斜率
         * 往外推, R0 出 38mΩ 而正确值是 59mΩ (差 36%)。
         * 三点都填同一个占位值时看不出来 (b-a=0, 权重错也无所谓),
         * 一旦填入真实数据就是系统性偏差 —— 由 sim_cal.py 跑真码抓到。
         * 所以反过来: 默认 lo = 0, 每跨过一个温度点就往前推一格。 */
        lo = 0;
        for(k = 0; k + 2u < NTEMP; k++)
        {
            if(s_cur_dc < c_temp_dc[k + 1u]) break;
            lo = (uint8_t)(k + 1u);
        }
        w = ((int32_t)s_cur_dc - (int32_t)c_temp_dc[lo]) * 256
            / ((int32_t)c_temp_dc[lo + 1] - (int32_t)c_temp_dc[lo]);
    }

    for(k = 0; k < NTBL; k++)
    {
        for(i = 0; i < NSOC; i++)
        {
            s_act[k][i] = lerp(s_ref[k][lo][i], s_ref[k][lo + 1][i], w);
        }
    }
}

/* =====================================================================
 * 对外接口 —— 初始化 / 温度刷新
 * ===================================================================== */

void BMS_CAL_Init(void)
{
    uint8_t k, ti, i;

    for(k = 0; k < NTBL; k++)
    {
        for(ti = 0; ti < NTEMP; ti++)
        {
            for(i = 0; i < NSOC; i++)
            {
                s_ref[k][ti][i] = def_get(k, ti, i);
            }
        }
    }

    /* 默认用中间那个温度点 (3 点即 25C) —— 上电还没测温时最合理的选择 */
    s_cur_dc = c_temp_dc[NTEMP / 2];
    refresh();

    s_inited = 1;
    s_dirty  = 0;
}

void BMS_CAL_Update(int16_t temp_dc)
{
    int32_t d;

    if(!s_inited) return;               /* 忘了调 Init 就什么都不做, 宁可不动也不乱算 */

    d = (int32_t)temp_dc - (int32_t)s_cur_dc;
    if(d < 0) d = -d;

    if(d < BMS_CAL_TEMP_HYST_DC) return;   /* 滞回内: 温度没真变, 不动 */

    s_cur_dc = temp_dc;
    refresh();
}

void BMS_CAL_SetTempDc(int16_t temp_dc)
{
    if(!s_inited) return;
    s_cur_dc = temp_dc;
    refresh();
}

int16_t BMS_CAL_GetTempDc(void)
{
    return s_cur_dc;
}

int16_t BMS_CAL_GetTempAt(uint8_t ti)
{
    if(ti >= NTEMP) return 0;
    return c_temp_dc[ti];
}

/* =====================================================================
 * 自带的对数 / 指数近似 —— 工况折算系数用
 *
 * 为什么不用 libm: 折算系数是"标定结果", 必须**跨平台逐位一致**, 否则 PC 端
 * 复刻出来的 SOH 与板子对不上, 而 BMS_USE_LIBM_EXPF 是个可切换的开关
 * (板子上为了省 EKF 那次 expf 的开销可能会关掉)。所以这里自带一套,
 * 与那个开关无关。
 *
 * 精度: 先做范围缩减 (x = 2^m * x0, x0 落在 [1,2)), 再算
 *   ln x0 = 2*(z + z^3/3 + z^5/5 + ...), z = (x0-1)/(x0+1) 落在 [0, 1/3)
 * z 很小时级数收敛极快, 取到 z^11 项的相对误差 < 2e-6。
 * exp 的 6 阶泰勒与 soc.c 的 SOC_ExpNeg 同一套系数 (|y| <= 1 时 < 2e-4)。
 * 折算系数本身只要求 ~1e-3, 这个精度绰绰有余 (实测见 脚本/test/fit_temp_rate.py)。
 * ===================================================================== */

static float cal_expneg(float x)      /* exp(-x), x >= 0 */
{
    float y;
    int   k = 0;

    if(x <= 0.0f) return 1.0f;
    while(x > 1.0f && k < 6) { x *= 0.5f; k++; }

    y = 1.0f + x * (-1.0f + x * (0.5f + x * (-1.6666667e-1f +
              x * (4.1666667e-2f + x * (-8.3333333e-3f + x * 1.3888889e-3f)))));
    while(k-- > 0) y *= y;
    return y;
}

static float cal_expf(float y)        /* exp(y) */
{
    if(y >= 0.0f) return 1.0f / cal_expneg(y);
    return cal_expneg(-y);
}

static float cal_logf(float x)        /* ln(x), x > 0 */
{
    float z, z2, r;
    int   m = 0;

    if(x <= 0.0f) return 0.0f;

    while(x >= 2.0f) { x *= 0.5f; m++; }      /* 范围缩减到 [1, 2) */
    while(x <  1.0f) { x *= 2.0f; m--; }

    z  = (x - 1.0f) / (x + 1.0f);             /* [0, 1/3) */
    z2 = z * z;
    r  = z * (1.0f + z2 * (0.33333333f + z2 * (0.2f + z2 * (0.14285714f +
              z2 * (0.11111111f + z2 * 0.09090909f)))));
    return 2.0f * r + (float)m * 0.69314718f;
}

static float cal_powf(float b, float e)
{
    if(b <= 0.0f) return 1.0f;
    return cal_expf(e * cal_logf(b));
}

/* =====================================================================
 * 工况折算系数
 * ===================================================================== */

float BMS_CAL_TempFactorF(int16_t temp_dc)
{
    uint8_t lo = 0, k;
    float   w;

    if(NTEMP < 2u) return 1.0f;

    if(temp_dc <= c_temp_dc[0])          return (float)c_cap_temp_pct[0] / 100.0f;
    if(temp_dc >= c_temp_dc[NTEMP - 1])  return (float)c_cap_temp_pct[NTEMP - 1] / 100.0f;

    /* 与 refresh() 里同一个写法 (同一个坑): lo 默认 0, 每跨过一个温度点前推一格。
     * 写成"if(cur < c_temp_dc[k+1]) {lo=k; break;}" 会在最后一个区间失手。 */
    for(k = 0; k + 2u < NTEMP; k++)
    {
        if(temp_dc < c_temp_dc[k + 1u]) break;
        lo = (uint8_t)(k + 1u);
    }
    w = (float)((int32_t)temp_dc - (int32_t)c_temp_dc[lo])
      / (float)((int32_t)c_temp_dc[lo + 1u] - (int32_t)c_temp_dc[lo]);

    return ((float)c_cap_temp_pct[lo]
            + ((float)c_cap_temp_pct[lo + 1u] - (float)c_cap_temp_pct[lo]) * w)
           / 100.0f;
}

float BMS_CAL_RateFactorF(int32_t cur_ma)
{
    float a, ratio;

    /* k == 1.0 = 未标定 -> 恒等。写成 == 而不是 <= 是有意的:
     * 只有"正好等于占位值"才短路, 免得将来标定成 0.99 之类的异常值也走短路。 */
    if(SOC_PEUKERT_K == 1.0f) return 1.0f;

    a = (float)((cur_ma < 0) ? -cur_ma : cur_ma);
    if(a < (float)SOC_RATE_MIN_CUR_MA) return 1.0f;   /* 静置 / 漏电流不折算 */

    ratio = a / (float)BMS_REF_CUR_MA;
    if(ratio < SOC_RATE_RATIO_LO) ratio = SOC_RATE_RATIO_LO;
    if(ratio > SOC_RATE_RATIO_HI) ratio = SOC_RATE_RATIO_HI;

    return cal_powf(ratio, 1.0f - SOC_PEUKERT_K);
}

uint16_t BMS_CAL_GetCapTempPct(uint8_t ti)
{
    if(ti >= NTEMP) return 0;
    return c_cap_temp_pct[ti];
}

int16_t BMS_CAL_GetRefTempDc(void) { return (int16_t)BMS_REF_TEMP_DC; }
int32_t BMS_CAL_GetRefCurMa(void)  { return (int32_t)BMS_REF_CUR_MA;  }

/* =====================================================================
 * 对外接口 —— 读活跃表 (算法用)
 * ===================================================================== */

uint16_t BMS_CAL_GetBase(uint8_t tbl, uint8_t idx)
{
    if(tbl >= NTBL || idx >= NSOC) return 0;
    return s_act[tbl][idx];
}

/* =====================================================================
 * 对外接口 —— 读写标定点 (串口改表 / 标工具)
 * ===================================================================== */

uint16_t BMS_CAL_GetPoint(uint8_t ti, uint8_t tbl, uint8_t idx)
{
    if(ti >= NTEMP || tbl >= NTBL || idx >= NSOC) return 0;
    return s_ref[tbl][ti][idx];
}

void BMS_CAL_SetPoint(uint8_t ti, uint8_t tbl, uint8_t idx, uint16_t v)
{
    if(ti >= NTEMP || tbl >= NTBL || idx >= NSOC) return;   /* 越界: 静默丢弃 */
    if(!s_inited) return;

    s_ref[tbl][ti][idx] = v;
    s_dirty = 1;
    refresh();          /* 立刻生效, 不用等下一个温度刷新周期 */
}

void BMS_CAL_LoadDefault(void)
{
    uint8_t k, ti, i;

    for(k = 0; k < NTBL; k++)
    {
        for(ti = 0; ti < NTEMP; ti++)
        {
            for(i = 0; i < NSOC; i++)
            {
                s_ref[k][ti][i] = def_get(k, ti, i);
            }
        }
    }
    s_dirty = 1;
    refresh();
}

uint8_t BMS_CAL_IsDirty(void)   { return s_dirty; }
void    BMS_CAL_ClearDirty(void){ s_dirty = 0; }

/* =====================================================================
 * 序列化 —— 供后续串口收发 / 落盘用 (本轮只备好, 尚未接线)
 *
 * 布局: 温度点优先, [ti][tbl][idx], 每点 2 字节小端, 共 264 字节。
 * ===================================================================== */

uint16_t BMS_CAL_Serialize(uint8_t *buf, uint16_t cap)
{
    uint16_t ti, k, i, n = 0;

    if(buf == 0 || cap < BMS_CAL_BYTES) return 0;

    for(ti = 0; ti < NTEMP; ti++)
    {
        for(k = 0; k < NTBL; k++)
        {
            for(i = 0; i < NSOC; i++)
            {
                uint16_t v = s_ref[k][ti][i];
                buf[n++] = (uint8_t)(v & 0xFFu);
                buf[n++] = (uint8_t)(v >> 8);
            }
        }
    }
    return n;
}

uint8_t BMS_CAL_Deserialize(const uint8_t *buf, uint16_t len)
{
    uint16_t ti, k, i, n = 0;

    if(buf == 0 || len != BMS_CAL_BYTES) return 0;

    for(ti = 0; ti < NTEMP; ti++)
    {
        for(k = 0; k < NTBL; k++)
        {
            for(i = 0; i < NSOC; i++)
            {
                uint16_t v = (uint16_t)buf[n] | ((uint16_t)buf[n + 1] << 8);
                s_ref[k][ti][i] = v;
                n += 2;
            }
        }
    }
    s_dirty = 1;
    refresh();
    return 1;
}
