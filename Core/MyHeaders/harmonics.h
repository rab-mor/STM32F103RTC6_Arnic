#ifndef HARMONICS_H
#define HARMONICS_H
/**
 * @file  harmonics.h
 * @brief Per-relay current harmonics h1..h9, measured on the F103 itself.
 *        Shared verbatim by both F103 projects (RELAY_COUNT and the ADC
 *        constants come from each board's board_config.h).
 *
 * Once every HARM_PERIOD_MS, the next HARM_CYCLES line cycles of every
 * relay's current are run through a Goertzel detector per harmonic, placed
 * exactly on h x f0, where f0 is the ADE9000's line frequency passed down by
 * the H7. The window is N = round(HARM_CYCLES x 8000 / f0) samples (400 at
 * 60 Hz, 480 at 50 Hz): a whole number of cycles, so no window function is
 * needed and neighbouring harmonics do not leak into each other.
 *
 * Why here and not on the H7: the H7 would need every raw sample of every
 * relay (8 kHz x 12 bit x up to 200 relays), which neither the SPI link nor
 * the RS485 bus can carry. A set of results is 26 bytes per relay per second.
 *
 * Per relay, per set:
 *   h1_ma      fundamental, mA RMS: the size of the load
 *   ratio[k]   h(k+2) RMS / h1 RMS in 0.01 % units: the shape of the load
 *   phase[k]   phase of h(k+2) minus (k+2) x phase of h1, in 360/256 degree
 *              steps: independent of where the window started, so it
 *              compares between sets and between relays
 * ratio and phase are zero when h1 is below HARM_MIN_H1_MA.
 *
 * Integer Goertzel (Q29 coefficients, 64-bit products) in the sample loop:
 * no FPU on the M3. Float only for the final magnitude and phase, 9 x
 * RELAY_COUNT times a second.
 *
 * Calls, all from MeasureTask:
 *   harm_service(now)          every wake: starts a capture when one is due,
 *                              finishes one that is complete
 *   harm_capturing()/harm_feed() from current_sense.c's sample loop
 * From LinkTask: harm_set_f0(), harm_get().
 */

#include <stdint.h>
#include <stdbool.h>
#include "board_config.h"

#define HARM_COUNT          9u          /* h1..h9                           */
#define HARM_RATIOS         8u          /* h2..h9                           */
#define HARM_CYCLES         3u
#define HARM_PERIOD_MS      1000u
#define HARM_MIN_H1_MA      100u
#define HARM_F0_DEFAULT     6000u       /* 0.01 Hz, used until the H7 sends one */
#define HARM_F0_MIN         4500u
#define HARM_F0_MAX         6600u

typedef struct {
    uint16_t h1_ma;
    uint16_t ratio[HARM_RATIOS];
    int8_t   phase[HARM_RATIOS];
} harm_rec_t;

typedef struct {
    uint8_t    set_id;                  /* +1 per set, never 0 once valid   */
    uint16_t   f0_centi_hz;             /* frequency the detectors sat on   */
    uint16_t   samples;                 /* N                                */
    harm_rec_t rec[RELAY_COUNT];
} harm_set_t;

void harm_init(void);

/* 0.01 Hz. Out-of-range values (including 0 = unknown) are ignored. */
void harm_set_f0(uint16_t f0_centi_hz);

extern volatile bool g_harm_capturing;
static inline bool harm_capturing(void) { return g_harm_capturing; }

/* One sample set, raw 12-bit ADC counts, indexed by relay. */
void harm_feed(const uint16_t *counts_by_relay);

void harm_service(uint32_t now_ms);

/* Latest complete set. false until the first one is ready. */
bool    harm_get(harm_set_t *out);
uint8_t harm_latest_id(void);           /* 0 = none yet */

#endif /* HARMONICS_H */
