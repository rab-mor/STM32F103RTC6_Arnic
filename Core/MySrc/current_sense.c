/**
 * current_sense.c - F103
 *
 * Port of the H7 current_sensors.c to a 72 MHz Cortex-M3 with a 12-bit
 * ADC.  The measurement is the same - AC RMS per channel with the DC
 * component removed - but the arithmetic is entirely different.
 *
 * WHY THE FLOAT PIPELINE DID NOT COME ACROSS
 *
 * The original ran an IIR DC blocker, a squarer and a 400-entry running
 * sum in float32 on an M4F where those are single-cycle.  On an M3 every
 * one is a soft-float library call of 30-100 cycles.  At 8 kHz x 8
 * channels that is a large fraction of the core, and the running sum
 * needed a full re-summation every 2400 calls to bound float32 drift.
 *
 * This version uses the variance identity instead:
 *
 *     n^2 * var = n * SUM(x^2) - (SUM x)^2
 *
 * Both accumulators are int64 and the subtraction is exact integer
 * arithmetic, so there is no cancellation error and no drift - the
 * 60-second resync disappears entirely.  DC removal is exact and
 * immediate rather than an IIR that has to settle.  Float is used once
 * per channel per window, for the square root, at 8 x 20 = 160 calls per
 * second.  Cost is roughly 6 cycles per channel per sample, under 1% of
 * the core.
 *
 * WHAT WAS KEPT
 *
 * The 400-sample window.  Squaring a sinusoid at f produces ripple at
 * 2f, and a 400-sample boxcar at 8 kHz has nulls at every multiple of
 * 20 Hz - which covers 100 Hz and 120 Hz, so the ripple is rejected for
 * 50 Hz and 60 Hz mains alike.  This is the reason for 400 and it is
 * not an arbitrary choice.
 *
 * The zero-current noise-gate calibration, with two corrections.  It
 * waits for relays_busy() to clear, and it no longer assumes the relays
 * are off: latching relays keep their positions through a reset and
 * nothing is pulsed at boot, so a load may be drawing current while the
 * gate is learned.  A channel whose relay is ON, or whose learned gate
 * exceeds SENSE_GATE_MAX_A, keeps the fixed SENSE_NOISE_FLOOR_A instead,
 * so real current is never learned as noise.
 *
 * WHAT WAS DROPPED
 *
 * GAIN_MULTIPLIER 1.058f.  That empirical trim was absorbing the
 * tolerance of the 1.5x divider ahead of the H7's ADC.  The ACS725 runs
 * on 3.3 V and drives the pin directly, so there is no divider and
 * nothing for a trim to correct.  The part is also ratiometric on the
 * same rail as VREF+, so counts-per-amp is exactly 81.92 regardless of
 * rail accuracy.  Do not reintroduce a calibration factor without
 * measuring against a reference first.
 */

#include "current_sense.h"
#include "relays.h"
#include "cmsis_os2.h"
#include "app_shared.h"
#include <math.h>
#include <string.h>

/* ======================================================================
 * Calibration tuning
 * ==================================================================== */

/* Windows of zero current to observe before fixing the gate.
 * One window is RMS_WINDOW_SAMPLES / ADC_SAMPLE_RATE_HZ = 50 ms,
 * so 60 windows is 3 seconds - matching the original's intent. */
#define SENSE_CAL_WINDOWS       60U

/* Gate at mean + k*sigma of the observed zero-current reading. */
#define SENSE_CAL_SIGMA         5.0f

/* ======================================================================
 * DMA buffer
 *
 * F103 has a single flat SRAM and DMA1 can reach all of it, so unlike
 * the H7 there is no section placement or cache maintenance to do.
 * ==================================================================== */

static uint32_t s_dma_buf[ADC_DMA_WORDS];

/* ======================================================================
 * State
 * ==================================================================== */

typedef struct {
    int64_t sumsq;      /* SUM(x^2) over the window */
    int32_t sum;        /* SUM(x)    over the window */
} sense_acc_t;

/* Indexed by SENSE_PAn, not by relay. */
static sense_acc_t s_acc[SENSE_COUNT];

static int32_t  s_rail24_sum;
static int32_t  s_rail5_sum;
static uint32_t s_window_samples;

static sense_result_t s_result;

static float    s_gate_a[RELAY_COUNT];
static float    s_cal_mean[RELAY_COUNT];
static float    s_cal_m2[RELAY_COUNT];
static uint32_t s_cal_n;
static uint8_t  s_cal_done;
static uint32_t s_last_activity;

static volatile uint8_t  s_half_ready;
static volatile uint8_t  s_full_ready;
static volatile uint32_t s_overruns;

static ADC_HandleTypeDef *s_hadc1;

/* ======================================================================
 * Accumulation
 * ==================================================================== */

static inline void acc_add(sense_acc_t *a, uint32_t x)
{
    a->sum   += (int32_t)x;
    a->sumsq += (int64_t)x * (int64_t)x;   /* UMULL + 64-bit add on M3 */
}

static void acc_reset(void)
{
    memset(s_acc, 0, sizeof(s_acc));
    s_rail24_sum     = 0;
    s_rail5_sum      = 0;
    s_window_samples = 0;
}

/**
 * Unpack one DMA half-buffer into the accumulators.
 *
 * Layout, per trigger, from the dual regular simultaneous configuration:
 *
 *   word[0] = PA4 (low half) | PA0 (high half)
 *   word[1] = PA5            | PA1
 *   word[2] = PA6            | PA2
 *   word[3] = PA7            | PA3
 *   word[4] = PC4 24V rail   | PC5 5V rail
 *
 * ADC1 is the master and always occupies the low half.
 */
static void accumulate(const uint32_t *words, uint32_t triggers)
{
    for (uint32_t t = 0U; t < triggers; t++) {
        const uint32_t *scan = &words[t * ADC_RANKS_PER_SCAN];

        for (uint32_t r = 0U; r < 4U; r++) {
            uint32_t w = scan[r];
            acc_add(&s_acc[SENSE_PA4 + r],  w         & 0x0FFFU);  /* ADC1 */
            acc_add(&s_acc[SENSE_PA0 + r], (w >> 16)  & 0x0FFFU);  /* ADC2 */
        }

        uint32_t rail = scan[ADC_RAIL_RANK];
        s_rail24_sum += (int32_t)( rail        & 0x0FFFU);
        s_rail5_sum  += (int32_t)((rail >> 16) & 0x0FFFU);
    }

    s_window_samples += triggers;
}

/* ======================================================================
 * Calibration
 * ==================================================================== */

static void cal_reset(void)
{
    memset(s_cal_mean, 0, sizeof(s_cal_mean));
    memset(s_cal_m2,   0, sizeof(s_cal_m2));
    s_cal_n = 0U;
}

static void cal_update(void)
{
    /* Welford, one pass, numerically stable at this sample count. */
    s_cal_n++;

    for (uint8_t i = 0U; i < RELAY_COUNT; i++) {
        float x = s_result.irms_a[i];
        float d = x - s_cal_mean[i];
        s_cal_mean[i] += d / (float)s_cal_n;
        s_cal_m2[i]   += d * (x - s_cal_mean[i]);
    }

    if (s_cal_n < SENSE_CAL_WINDOWS) {
        return;
    }

    for (uint8_t i = 0U; i < RELAY_COUNT; i++) {
        float var   = (s_cal_n > 1U)
                    ? (s_cal_m2[i] / (float)(s_cal_n - 1U))
                    : 0.0f;
        float gate  = s_cal_mean[i] + SENSE_CAL_SIGMA * sqrtf(var);

        if (relays_get_state(i) != 0U || gate > SENSE_GATE_MAX_A) {
            /* A load was (or may have been) drawing current: that is not
             * noise.  Keep the fixed floor for this channel. */
            s_gate_a[i] = SENSE_NOISE_FLOOR_A;
        } else {
            s_gate_a[i] = (gate > SENSE_NOISE_FLOOR_A) ? gate : SENSE_NOISE_FLOOR_A;
        }
    }
    s_cal_done = 1U;
}

/* ======================================================================
 * Window finalisation
 * ==================================================================== */

static void finalize(void)
{
    static const uint8_t k_map[RELAY_COUNT] = RELAY_CURRENT_SENSE_MAP;

    const int64_t n  = (int64_t)s_window_samples;
    const float   fn = (float)s_window_samples;

    for (uint8_t rly = 0U; rly < RELAY_COUNT; rly++) {
        const sense_acc_t *a = &s_acc[k_map[rly]];

        /*
         * rms_counts = sqrt(var) where n^2 * var = n*sumsq - sum^2.
         *
         * Both terms reach ~2.7e12 for a full-scale signal and their
         * difference can be small, but the subtraction is done in int64
         * so it is exact.  Only the result is converted to float, after
         * any cancellation has already happened losslessly.
         */
        int64_t num = n * a->sumsq - (int64_t)a->sum * (int64_t)a->sum;
        if (num < 0) {
            num = 0;    /* reachable only on a perfectly flat channel */
        }

        float rms_counts = sqrtf((float)num) / fn;
        float dc_counts  = ((float)a->sum / fn) - (float)SENSE_ZERO_COUNTS;

        s_result.irms_a[rly] = rms_counts * SENSE_AMPS_PER_LSB;
        s_result.idc_a[rly]  = dc_counts  * SENSE_AMPS_PER_LSB;
    }

    /* Rails: mean over the window, no RMS wanted. */
    s_result.rail_24v = ((float)s_rail24_sum / fn)
                      * ADC_MV_PER_LSB * 0.001f * RAIL_24V_DIVIDER;
    s_result.rail_5v  = ((float)s_rail5_sum / fn)
                      * ADC_MV_PER_LSB * 0.001f * RAIL_5V_DIVIDER;

    /*
     * Calibration wants quiet channels.  If a relay moves mid-calibration
     * (relays_busy() or a change in relays_activity()), discard what has
     * been gathered and start over.  Channels that were carrying current
     * are caught in cal_update() and keep the fixed floor.
     */
    if (s_cal_done == 0U) {
    	uint32_t act = relays_activity();
        if (relays_busy() != 0U || act != s_last_activity) {
            cal_reset();
        } else {
            cal_update();
        }
        s_last_activity = act;
    } else {
        for (uint8_t i = 0U; i < RELAY_COUNT; i++) {
            if (s_result.irms_a[i] < s_gate_a[i]) {
                s_result.irms_a[i] = 0.0f;
            }
            if (s_result.idc_a[i] < s_gate_a[i] &&
                s_result.idc_a[i] > -s_gate_a[i]) {
                s_result.idc_a[i] = 0.0f;
            }
        }
    }

    s_result.seq++;
    acc_reset();
}

/* ======================================================================
 * Public API
 * ==================================================================== */

void sense_init(void)
{
    acc_reset();
    cal_reset();
    s_last_activity = relays_activity();
    memset(&s_result, 0, sizeof(s_result));

    for (uint8_t i = 0U; i < RELAY_COUNT; i++) {
        s_gate_a[i] = SENSE_NOISE_FLOOR_A;
    }

    s_cal_done   = 0U;
    s_half_ready = 0U;
    s_full_ready = 0U;
    s_overruns   = 0U;
    s_hadc1      = NULL;
}

int sense_start(void *hadc1, void *hadc2, void *htim3)
{
    ADC_HandleTypeDef *a1 = (ADC_HandleTypeDef *)hadc1;
    ADC_HandleTypeDef *a2 = (ADC_HandleTypeDef *)hadc2;
    TIM_HandleTypeDef *t3 = (TIM_HandleTypeDef *)htim3;

    s_hadc1 = a1;

    /*
     * Self-calibration is mandatory on the F1 ADC and CubeMX does not
     * generate a call for it.  Skipping it leaves a large offset error
     * that looks exactly like a real DC bias on every channel - and the
     * DC-removal step partly masks it, which makes it worse to find.
     */
    if (HAL_ADCEx_Calibration_Start(a1) != HAL_OK) { return -1; }
    if (HAL_ADCEx_Calibration_Start(a2) != HAL_OK) { return -2; }

    /*
     * Slave before master.  Starting ADC1 first means the first frame or
     * two come back with the halves transposed.
     *
     * NOTE: some STM32F1 HAL revisions enable the slave inside
     * HAL_ADCEx_MultiModeStart_DMA().  If the call below returns an
     * error or HAL_BUSY, delete this HAL_ADC_Start() line - that is the
     * symptom of a HAL that is already doing it.
     */
    if (HAL_ADC_Start(a2) != HAL_OK) { return -3; }

    if (HAL_ADCEx_MultiModeStart_DMA(a1, s_dma_buf, ADC_DMA_WORDS) != HAL_OK) {
        return -4;
    }

    /* TIM3 last: nothing is sampled until TRGO starts firing. */
    if (HAL_TIM_Base_Start(t3) != HAL_OK) { return -5; }

    return 0;
}

uint8_t sense_service(void)
{
    uint8_t produced = 0U;

    if (s_half_ready != 0U) {
        s_half_ready = 0U;
        accumulate(&s_dma_buf[0], ADC_TRIGGERS_PER_HALF);
        if (s_window_samples >= RMS_WINDOW_SAMPLES) {
            finalize();
            produced = 1U;
        }
    }

    if (s_full_ready != 0U) {
        s_full_ready = 0U;
        accumulate(&s_dma_buf[ADC_HALF_WORDS], ADC_TRIGGERS_PER_HALF);
        if (s_window_samples >= RMS_WINDOW_SAMPLES) {
            finalize();
            produced = 1U;
        }
    }

    return produced;
}

const sense_result_t *sense_get(void)
{
    return &s_result;
}

uint8_t sense_calibrated(void)
{
    return s_cal_done;
}

uint32_t sense_overruns(void)
{
    return s_overruns;
}

/* ======================================================================
 * DMA callbacks
 *
 * Flag only.  All arithmetic happens in sense_service() on the main
 * loop, so the ISR stays a handful of cycles and never contends with
 * the relay scheduler or the link.
 * ==================================================================== */

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) {
        if (s_half_ready != 0U) {
            s_overruns++;   /* main loop missed the 12.5 ms deadline */
        }
        s_half_ready = 1U;
        osThreadFlagsSet(MeasureTaskHandle, MEAS_FLAG_WINDOW_DONE);
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) {
        if (s_full_ready != 0U) {
            s_overruns++;
        }
        s_full_ready = 1U;
        osThreadFlagsSet(MeasureTaskHandle, MEAS_FLAG_WINDOW_DONE);
    }
}
