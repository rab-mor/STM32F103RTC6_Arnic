/**
 * @file  harmonics.c
 * @brief See harmonics.h.
 */

#include "harmonics.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <string.h>

#define Q                   29          /* coefficient fraction bits          */
#define IN_SHIFT            4           /* samples x16: less rounding noise   */
#define PI_F                3.14159265f

typedef struct {
    int32_t s1;
    int32_t s2;
} gz_t;

typedef enum { H_IDLE, H_CAPTURE, H_READY } hstate_t;

volatile bool g_harm_capturing;

static gz_t     s_gz[RELAY_COUNT][HARM_COUNT];
static int32_t  s_coef[HARM_COUNT];     /* 2 cos(w_h), Q29 */
static float    s_cos[HARM_COUNT];
static float    s_sin[HARM_COUNT];
static hstate_t s_state;
static uint32_t s_n;                    /* samples fed so far                 */
static uint32_t s_len;                  /* N for this capture                 */
static uint16_t s_f0_active;
static uint32_t s_last_start;
static volatile uint16_t s_f0_next = HARM_F0_DEFAULT;

static harm_set_t s_out[2];
static volatile uint8_t s_out_idx;      /* index of the latest complete set   */
static volatile bool    s_have_out;
static uint8_t  s_set_id;

void harm_init(void)
{
    memset(s_gz, 0, sizeof(s_gz));
    s_state          = H_IDLE;
    g_harm_capturing = false;
    s_have_out       = false;
    s_set_id         = 0u;
    s_last_start     = 0u;
}

void harm_set_f0(uint16_t f0_centi_hz)
{
    if (f0_centi_hz >= HARM_F0_MIN && f0_centi_hz <= HARM_F0_MAX) {
        s_f0_next = f0_centi_hz;
    }
}

static void start_capture(void)
{
    const uint16_t f0 = s_f0_next;
    s_f0_active = f0;

    /* N = round(HARM_CYCLES * fs / f0), f0 in 0.01 Hz. */
    const uint32_t num = HARM_CYCLES * ADC_SAMPLE_RATE_HZ * 100u;
    s_len = (num + f0 / 2u) / f0;

    for (uint32_t h = 0u; h < HARM_COUNT; h++) {
        const float w = 2.0f * PI_F * ((float)f0 * 0.01f) * (float)(h + 1u) / (float)ADC_SAMPLE_RATE_HZ;
        s_cos[h]  = cosf(w);
        s_sin[h]  = sinf(w);
        s_coef[h] = (int32_t)lrintf(2.0f * s_cos[h] * (float)(1ul << Q));
    }
    memset(s_gz, 0, sizeof(s_gz));
    s_n     = 0u;
    s_state = H_CAPTURE;
    __DMB();
    g_harm_capturing = true;
}

/* The hot loop: RELAY_COUNT x 9 updates per sample, 400-480 samples a
   second. Optimised even in Debug builds so it costs the same in both. */
__attribute__((optimize("O2")))
void harm_feed(const uint16_t *counts_by_relay)
{
    if (!g_harm_capturing) {
        return;
    }
    for (uint32_t r = 0u; r < RELAY_COUNT; r++) {
        const int32_t x = ((int32_t)counts_by_relay[r] - (int32_t)SENSE_ZERO_COUNTS) * (1 << IN_SHIFT);
        gz_t *g = s_gz[r];
        for (uint32_t h = 0u; h < HARM_COUNT; h++) {
            const int64_t p  = (int64_t)s_coef[h] * (int64_t)g[h].s1 + ((int64_t)1 << (Q - 1));
            const int32_t s0 = x + (int32_t)(p >> Q) - g[h].s2;
            g[h].s2 = g[h].s1;
            g[h].s1 = s0;
        }
    }
    if (++s_n >= s_len) {
        g_harm_capturing = false;
        s_state = H_READY;
    }
}

static int8_t phase_q(float rad)
{
    while (rad >  PI_F) { rad -= 2.0f * PI_F; }
    while (rad <= -PI_F) { rad += 2.0f * PI_F; }
    int32_t q = (int32_t)lrintf(rad * (128.0f / PI_F));
    if (q > 127)  { q -= 256; }         /* +180 degrees is -180 */
    return (int8_t)q;
}

static uint16_t sat16(float v)
{
    if (!(v > 0.0f))   { return 0u; }
    if (v >= 65535.0f) { return 65535u; }
    return (uint16_t)lrintf(v);
}

/* Amplitude (peak counts) and phase of harmonic h+1 of relay r. */
static void bin_result(uint32_t r, uint32_t h, float to_peak, float *amp, float *ph)
{
    /* y = s1 - e^-jw s2. The subtraction is done in integers: s1 and s2 are
       close for h1, and float would lose the difference. s_coef is 2cos(w)
       in Q29, i.e. cos(w) in Q30. */
    const gz_t   *g   = &s_gz[r][h];
    const int64_t re  = (int64_t)g->s1 - (((int64_t)g->s2 * s_coef[h] + ((int64_t)1 << 29)) >> 30);
    const float   fre = (float)re;
    const float   fim = (float)g->s2 * s_sin[h];
    *amp = sqrtf(fre * fre + fim * fim) * to_peak;
    *ph  = atan2f(fim, fre);
}

static void finish(void)
{
    harm_set_t *o = &s_out[s_have_out ? (1u - s_out_idx) : 0u];
    memset(o, 0, sizeof(*o));
    o->f0_centi_hz = s_f0_active;
    o->samples     = (uint16_t)s_len;

    /* |X| = A * N / 2 for a sinusoid of amplitude A (counts << IN_SHIFT). */
    const float to_peak = 2.0f / ((float)s_len * (float)(1 << IN_SHIFT));

    for (uint32_t r = 0u; r < RELAY_COUNT; r++) {
        float amp[HARM_COUNT];
        float ph[HARM_COUNT];

        bin_result(r, 0u, to_peak, &amp[0], &ph[0]);
        const float h1_ma = amp[0] * 0.70710678f / SENSE_LSB_PER_AMP * 1000.0f;
        o->rec[r].h1_ma = sat16(h1_ma);
        if (h1_ma < (float)HARM_MIN_H1_MA) {
            continue;                   /* idle relay: ratios and phases stay 0 */
        }
        for (uint32_t h = 1u; h < HARM_COUNT; h++) {
            bin_result(r, h, to_peak, &amp[h], &ph[h]);
        }
        for (uint32_t k = 0u; k < HARM_RATIOS; k++) {
            o->rec[r].ratio[k] = sat16(amp[k + 1u] / amp[0] * 10000.0f);
            o->rec[r].phase[k] = phase_q(ph[k + 1u] - (float)(k + 2u) * ph[0]);
        }
    }

    if (++s_set_id == 0u) {
        s_set_id = 1u;
    }
    o->set_id = s_set_id;

    taskENTER_CRITICAL();
    s_out_idx  = (uint8_t)(o - s_out);
    s_have_out = true;
    taskEXIT_CRITICAL();
}

void harm_service(uint32_t now_ms)
{
    if (s_state == H_READY) {
        finish();
        s_state = H_IDLE;
    }
    if (s_state == H_IDLE && (uint32_t)(now_ms - s_last_start) >= HARM_PERIOD_MS) {
        s_last_start = now_ms;
        start_capture();
    }
}

bool harm_get(harm_set_t *out)
{
    bool ok;
    taskENTER_CRITICAL();
    ok = s_have_out;
    if (ok) {
        *out = s_out[s_out_idx];
    }
    taskEXIT_CRITICAL();
    return ok;
}

uint8_t harm_latest_id(void)
{
    return s_have_out ? s_out[s_out_idx].set_id : 0u;
}
