/**
 * current_sense.h - F103 header file
 *
 * Eight-channel AC RMS current measurement from the dual-simultaneous
 * ADC pipeline, plus the 24 V and 5 V rail monitors.
 *
 * Super-loop usage:
 *
 *     sense_init();
 *     sense_start(&hadc1, &hadc2, &htim3);
 *     for (;;) {
 *         relays_tick();
 *         if (sense_service()) {
 *             const sense_result_t *r = sense_get();
 *             ...
 *         }
 *     }
 *
 * sense_service() does the arithmetic and must be called from the main
 * loop, not from an ISR.  The DMA callbacks only set a flag.
 */

#ifndef CURRENT_SENSE_H
#define CURRENT_SENSE_H

#include "board_config.h"
#include <stdint.h>

typedef struct {
    /* Indexed by RELAY, 0..RELAY_COUNT-1 - not by sense pin.  The
     * translation through RELAY_CURRENT_SENSE_MAP has already happened. */
    float    irms_a[RELAY_COUNT];   /* AC RMS current, DC component removed */
    float    idc_a[RELAY_COUNT];    /* mean offset from the 1.65 V zero    */

    float    rail_24v;              /* volts */
    float    rail_5v;               /* volts */

    uint32_t seq;                   /* increments once per completed window */
} sense_result_t;

/** Zero all accumulators.  Call before sense_start(). */
void sense_init(void);

/**
 * Calibrate both ADCs, then start TIM3 and the dual-mode DMA stream.
 *
 * @return 0 on success, negative on HAL error
 */
int sense_start(void *hadc1, void *hadc2, void *htim3);

/**
 * Process any DMA half-buffer that has become ready.
 *
 * @return 1 if a measurement window completed and sense_get() has new
 *         data, 0 otherwise.  Returns 1 roughly every 50 ms.
 */
uint8_t sense_service(void);

/** Most recent completed measurement.  Never NULL. */
const sense_result_t *sense_get(void);

/**
 * Non-zero once the zero-current noise gate has been established.
 * Readings before this are unfiltered and will show a small standing
 * offset from converter noise.
 */
uint8_t sense_calibrated(void);

/**
 * Count of DMA half-buffers that became ready while a previous one was
 * still unprocessed.  Any non-zero value means the main loop missed the
 * 12.5 ms deadline and samples were lost.
 */
uint32_t sense_overruns(void);

#endif /* CURRENT_SENSE_H */
