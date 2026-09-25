/**
 * measure_task.c - F103
 *
 * MeasureTask: one app_snapshot_t per 50 ms measurement window, for LinkTask
 * to send to the H7. Readings come from current_sense.c (ADC1 + ADC2 in dual
 * simultaneous mode, triggered by TIM3 at 8 kHz, DMA in circular mode).
 *
 * The task sleeps until the ADC DMA reports a half-buffer (thread flag
 * MEAS_FLAG_WINDOW_DONE from current_sense.c's callbacks), lets
 * sense_service() do the arithmetic, and publishes once a whole window has
 * been measured.
 *
 * Faults, reported to the H7 in app_snapshot_t.fault_flags:
 *   FAULT_UNCALIBRATED  zero-current gate not learned yet (first ~3 s)
 *   FAULT_ADC_START     sense_start() failed: currents and rails read 0
 *   FAULT_ADC_STALL     no window for MEASURE_STALL_MS: DMA or TIM3 stopped
 *   FAULT_ADC_OVERRUN   a DMA half-buffer was lost since the last window
 *
 * The snapshot is double-buffered: this task fills g_snap[!active] and then
 * flips g_snap_active, so LinkTask never reads a half-written one.
 */

#include "cmsis_os2.h"
#include "board_config.h"
#include "app_shared.h"
#include "relays.h"
#include "current_sense.h"
#include "f103_tasks.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern TIM_HandleTypeDef htim3;

/* 400 samples at 8 kHz = 50 ms, three whole mains cycles. */
#define MEASURE_WINDOW_MS   ((RMS_WINDOW_SAMPLES * 1000u) / ADC_SAMPLE_RATE_HZ)
#define MEASURE_STALL_MS    (4u * MEASURE_WINDOW_MS)

measure_dbg_t     g_measure_dbg;
volatile uint32_t g_hb_measure;

static uint32_t s_window_id;

/* Stamp and hand over a filled snapshot. */
static void publish(app_snapshot_t *next)
{
    next->window_id      = ++s_window_id;
    next->relay_state    = (uint8_t)relays_get_state_mask();
    next->dropped_events = g_snap[g_snap_active].dropped_events;   /* kept by link_post_event */

    __DMB();                                        /* contents before the flip */
    g_snap_active = (uint8_t)(1u - g_snap_active);
}

/* No usable measurement: publish zeros with the reason, so the H7 and the
   dashboard show a fault instead of a believable 0 A. */
static void publish_fault(uint8_t faults)
{
    app_snapshot_t *s = &g_snap[1u - g_snap_active];

    for (uint8_t i = 0u; i < RELAY_COUNT; i++) {
        s->relay_ma[i] = 0;
    }
    s->rail_24V_mv = 0u;
    s->rail_5V_mv  = 0u;
    s->fault_flags = faults;
    publish(s);
}

static uint16_t volts_to_mv(float v)
{
    const float mv = v * 1000.0f + 0.5f;
    if (!(mv > 0.0f))   { return 0u; }
    if (mv > 65535.0f)  { return 65535u; }
    return (uint16_t)mv;
}

void MeasureTask_Run(void *argument)
{
    (void)argument;

    sense_init();
    g_measure_dbg.start_rc = (int32_t)sense_start(&hadc1, &hadc2, &htim3);

    if (g_measure_dbg.start_rc != 0) {
        /* No ADC. Keep publishing, flagged, so the H7 sees why the readings
           are zero, and keep the heartbeat going: relays still work, and a
           watchdog reset would not fix a peripheral that refuses to start. */
        for (;;) {
            osDelay(MEASURE_WINDOW_MS);
            g_hb_measure++;
            publish_fault((uint8_t)(FAULT_ADC_START | FAULT_UNCALIBRATED));
        }
    }

    uint32_t last_window   = HAL_GetTick();
    uint32_t seen_overruns = 0u;

    for (;;) {
        /* Woken by each DMA half-buffer (every 12.5 ms); the timeout only
           matters if the ADC has stopped. */
        (void)osThreadFlagsWait(MEAS_FLAG_WINDOW_DONE, osFlagsWaitAny, MEASURE_WINDOW_MS);
        g_hb_measure++;

        const uint32_t now = HAL_GetTick();

        if (sense_service() != 0u) {
            last_window = now;

            const sense_result_t *r = sense_get();
            app_snapshot_t       *s = &g_snap[1u - g_snap_active];

            for (uint8_t i = 0u; i < RELAY_COUNT; i++) {
                s->relay_ma[i] = (int32_t)(r->irms_a[i] * 1000.0f + 0.5f);
                g_measure_dbg.relay_ma[i] = s->relay_ma[i];
            }
            s->rail_24V_mv = volts_to_mv(r->rail_24v);
            s->rail_5V_mv  = volts_to_mv(r->rail_5v);

            uint8_t faults = 0u;
            if (sense_calibrated() == 0u) {
                faults |= FAULT_UNCALIBRATED;
            }
            const uint32_t ovr = sense_overruns();
            if (ovr != seen_overruns) {
                seen_overruns = ovr;
                faults |= FAULT_ADC_OVERRUN;
            }
            s->fault_flags = faults;
            publish(s);

            g_measure_dbg.windows++;
            g_measure_dbg.calibrated  = sense_calibrated();
            g_measure_dbg.overruns    = ovr;
            g_measure_dbg.rail_24v_mv = s->rail_24V_mv;
            g_measure_dbg.rail_5v_mv  = s->rail_5V_mv;

        } else if ((uint32_t)(now - last_window) >= MEASURE_STALL_MS) {
            last_window = now;                /* report once per stall period */
            g_measure_dbg.stalls++;
            publish_fault((uint8_t)(FAULT_ADC_STALL |
                                    ((sense_calibrated() != 0u) ? 0u : FAULT_UNCALIBRATED)));
        }
    }
}
