/**
 * board_config.h - F103 header file
 *
 * Board constants for the SPI-linked STM32F103RCT6 relay/sense board.
 *
 * This file holds only values that are fixed by the hardware or by the
 * CubeMX configuration.  Anything that differs between the SPI board and
 * the UART board belongs here, so the two build targets share every other
 * source file unchanged.
 *
 * Target: STM32F103RCT6, LQFP64, 72 MHz from 8 MHz HSE.
 */

#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#if defined(STM32F1)
#include "stm32f1xx_hal.h"
#elif defined(STM32F4)
#include "stm32f4xx_hal.h"
#endif
#include "main.h"
#include <stdint.h>

/* ======================================================================
 * Board identity
 * ==================================================================== */

#define BOARD_LINK_SPI          1   /* this target talks to the H7 over SPI2 */
#define BOARD_LINK_UART         0

/* ======================================================================
 * Clock tree
 *
 * HSE 8 MHz crystal -> PLL x9 -> SYSCLK 72 MHz
 *   AHB  /1  -> 72 MHz
 *   APB1 /2  -> 36 MHz
 *   APB2 /1  -> 72 MHz
 *   ADC  /6  -> 12 MHz  (14 MHz is the hard ceiling on F1)
 * ==================================================================== */

#define BOARD_HSE_HZ            8000000UL
#define BOARD_SYSCLK_HZ         72000000UL
#define BOARD_APB1_HZ           36000000UL
#define BOARD_APB2_HZ           72000000UL
#define BOARD_ADCCLK_HZ         12000000UL

/*
 * TIM3 sits on APB1.  When an APB prescaler is anything other than /1 the
 * timer clock on that bus is DOUBLED.  So TIM3 counts at 72 MHz, not 36.
 * Deriving the prescaler from 36 MHz gives 4 kHz and every RMS reading
 * comes out low by a factor that still looks plausible.
 */
#define BOARD_APB1_TIMER_HZ     72000000UL

_Static_assert(BOARD_ADCCLK_HZ <= 14000000UL, "F103 ADC clock must not exceed 14 MHz");

/* ======================================================================
 * ADC sampling
 *
 * TIM3 TRGO (update event) triggers one dual-simultaneous 5-rank scan.
 *
 *   PSC = 71  ->  72 MHz / 72   = 1 MHz  (1 tick = 1 us)
 *   ARR = 124 ->  125 ticks     = 125 us = 8.000 kHz
 *
 * Both registers are N-1 because the counter includes zero.  The 1 MHz
 * intermediate is deliberate: ARR then reads directly in microseconds.
 * ==================================================================== */

#define ADC_SAMPLE_RATE_HZ      8000UL
#define TIM3_PRESCALER          71U
#define TIM3_PERIOD             124U

_Static_assert((BOARD_APB1_TIMER_HZ / (TIM3_PRESCALER + 1UL))
                   / (TIM3_PERIOD + 1UL) == ADC_SAMPLE_RATE_HZ, "TIM3 PSC/ARR do not yield ADC_SAMPLE_RATE_HZ");

/*
 * Dual regular simultaneous mode.  ADC1 is master, ADC2 is slave.
 *
 * Both sequences must have the same rank count, which is why the 24 V and
 * 5 V rail monitors ride along as rank 5.  They get sampled at 8 kHz, far
 * more often than needed; decimate in software.
 *
 *   rank | ADC1 (low half)     | ADC2 (high half)
 *   -----+---------------------+---------------------
 *     1  | PA4  ch4            | PA0  ch0
 *     2  | PA5  ch5            | PA1  ch1
 *     3  | PA6  ch6            | PA2  ch2
 *     4  | PA7  ch7            | PA3  ch3
 *     5  | PC4  ch14  (24 V)   | PC5  ch15  (5 V)
 *
 * ADC2 has no DMA channel on F103.  In dual mode ADC1->DR is a 32-bit
 * register holding both results, so one DMA1 Channel 1 transfer per rank
 * carries both.  Unpack with (w & 0xFFFF) and (w >> 16).
 */
#define ADC_RANKS_PER_SCAN      5U

#define ADC_TRIGGERS_PER_HALF   100U
#define ADC_HALF_WORDS          (ADC_RANKS_PER_SCAN * ADC_TRIGGERS_PER_HALF)
#define ADC_DMA_WORDS           (ADC_HALF_WORDS * 2U)   /* 1000 words = 4 KB */

/* Half-buffer interval: 100 triggers @ 8 kHz = 12.5 ms.  This is the
 * hard deadline for process_buffer() to complete. */
#define ADC_HALF_PERIOD_MS      ((ADC_TRIGGERS_PER_HALF * 1000U) / ADC_SAMPLE_RATE_HZ)

/* ======================================================================
 * Sense channel indexing
 *
 * The unpacked sample array is indexed by PIN, not by rank, so that this
 * mapping can be checked against the schematic by reading pin names
 * rather than counting array positions.
 * ==================================================================== */

#define SENSE_PA0               0U
#define SENSE_PA1               1U
#define SENSE_PA2               2U
#define SENSE_PA3               3U
#define SENSE_PA4               4U
#define SENSE_PA5               5U
#define SENSE_PA6               6U
#define SENSE_PA7               7U
#define SENSE_COUNT             8U

/* Rank 5 carries the rails; both live in word[4] of each scan. */
#define ADC_RAIL_RANK           4U

/*
 * Maps relay index 0..7 to the sense index carrying its Relay-CurrentN net.
 *
 * The order is REVERSED - Relay-Current1 is on PA7, not PA0.  Confirmed
 * from the MCU sheet pin numbers:
 *
 *     pin 14 = PA0 = Relay-Current8       pin 20 = PA4 = Relay-Current4
 *     pin 15 = PA1 = Relay-Current7       pin 21 = PA5 = Relay-Current3
 *     pin 16 = PA2 = Relay-Current6       pin 22 = PA6 = Relay-Current2
 *     pin 17 = PA3 = Relay-Current5       pin 23 = PA7 = Relay-Current1
 *
 * Cross-checked against the K-designator placement: sensor N sits on K
 * designator {K1,K3,K5,K7,K2,K4,K6,K8} for N = 1..8, and those carry
 * RELAY1..RELAY8 respectively.  So Relay-CurrentN belongs to RELAY N in
 * every case; the K numbering is board placement only.
 *
 * Consequence for the dual-mode buffer layout:
 *     word[0] = relay4 (low) | relay8 (high)
 *     word[1] = relay3       | relay7
 *     word[2] = relay2       | relay6
 *     word[3] = relay1       | relay5
 *     word[4] = 24 V rail    | 5 V rail
 */
#define RELAY_CURRENT_SENSE_MAP                                            \
    { SENSE_PA7, SENSE_PA6, SENSE_PA5, SENSE_PA4,                          \
      SENSE_PA3, SENSE_PA2, SENSE_PA1, SENSE_PA0 }

/* ======================================================================
 * RMS window
 *
 * 400 samples @ 8 kHz = 50 ms = exactly 3 mains cycles at 60 Hz.
 * A window spanning whole cycles has no partial-cycle error, which is
 * what keeps the RMS value from wobbling with window phase.
 * ==================================================================== */

#define RMS_WINDOW_SAMPLES      400U

_Static_assert(RMS_WINDOW_SAMPLES % ADC_TRIGGERS_PER_HALF == 0U, "RMS window must be a whole number of DMA half-buffers");

/* ======================================================================
 * Scaling
 *
 * The F103 ADC is 12-bit.  The previous H7 design used 16-bit constants;
 * every one of these had to be rederived.
 *
 *   LSB = 3300 mV / 4096 = 0.8057 mV
 * ==================================================================== */

#define ADC_VREF_MV             3300.0f
#define ADC_FULL_SCALE          4096.0f
#define ADC_MV_PER_LSB          (ADC_VREF_MV / ADC_FULL_SCALE)   /* 0.8057 mV */

/*
 * Sense element: ACS725LLCTR-20AB-T, one per relay, no gain stage.
 *
 *   +/-20 A bidirectional, 66 mV/A, VCC 3.0-3.6 V, DC to 120 kHz
 *   Zero-current output = 0.5 x VCC for a bidirectional device
 *
 * The part is RATIOMETRIC: both the zero point and the sensitivity scale
 * with VCC.  Provided VREF+ is the same 3.3 V rail that supplies the
 * sensors, the conversion in ADC counts is exact and independent of how
 * accurate that rail actually is:
 *
 *   counts per amp = 4096 * (66 / 3300) = 81.92 exactly
 *   zero current   = 4096 / 2           = 2048 counts exactly
 *
 * So do not calibrate against a measured VREF - that would reintroduce
 * an error the ratiometric design already cancels.  If VREF+ is ever
 * fed from a separate reference instead of the sensor rail, this stops
 * holding and the constants below need rederiving.
 */
#define SENSE_MV_PER_AMP        66.0f
#define SENSE_GAIN              1.0f
#define SENSE_LSB_PER_AMP       81.92f
#define SENSE_AMPS_PER_LSB      (1.0f / SENSE_LSB_PER_AMP)   /* 0.012207 A */
#define SENSE_ZERO_COUNTS       2048
#define SENSE_FULL_SCALE_A      20.0f

/* +/-20 A spans 2048 +/- 1638 counts, so 410..3686.  Comfortable margin
 * at both rails; a reading pinned near 0 or 4095 means a fault, not a
 * large current. */
#define SENSE_COUNTS_AT_FS      1638

/*
 * Readings below this are reported as zero.
 *
 * At 81.92 counts/A this is only ~4 LSB - tighter than the ~9 LSB the
 * 100 mV/A placeholder implied, because 66 mV/A is a weaker signal.
 * Averaging over RMS_WINDOW_SAMPLES helps, but converter noise adds to
 * the sum of squares rather than cancelling, so it shows up as a small
 * positive RMS bias.  Measure the actual zero-current reading on
 * hardware and raise this if it sits above 0.05 A.
 */
#define SENSE_NOISE_FLOOR_A     0.05f

/* A learned gate above this is real current, not noise (relays are left as
 * they were at boot, so a load may be running during calibration).  That
 * channel keeps SENSE_NOISE_FLOOR_A instead. */
#define SENSE_GATE_MAX_A        0.25f

/* Current evidence (relay_task.c).  A relay believed OFF that carries at
 * least EVIDENCE_ON_MA for EVIDENCE_WINDOWS windows in a row (0.5 s) is
 * really ON; a relay commanded OFF that still carries it did not open. */
#define EVIDENCE_ON_MA          300
#define EVIDENCE_WINDOWS        10U

/* Rail divider ratios (top + bottom) / bottom:
   24 V: 100k over 10k -> 11.0.   5 V: 270k over 56k -> 5.8214. */
#define RAIL_24V_DIVIDER        11.0f
#define RAIL_5V_DIVIDER         5.8214f

/* ======================================================================
 * Relays
 * ==================================================================== */

#define RELAY_COUNT             8U

/* Coil energised time for the ADJH23012 latching relays. */
#define RELAY_PULSE_MS          80U

/* Minimum idle time after a pulse before the same relay may pulse again.
 * Protects against a rapid ON/OFF/ON sequence chain-pulsing the coils. */
#define RELAY_INTERPULSE_MS     20U

/*
 * Maximum coils energised at once.  Eight simultaneous coils is a large
 * step load on the 24 V rail - which is presumably why ADC-Voltage-24V
 * exists.  Raise this only after measuring the rail sag under load.
 */
#define RELAY_MAX_CONCURRENT    2U

/*
 * Bit N set => relay N+1 has its coil purpose reversed relative to the
 * schematic's "pin 1 = Reset, pin 2 = Set" convention.
 *
 * So "pin 1 = Reset, pin 2 = Set" holds for all eight relays, and ON is
 * always coil B.  If a relay turns out inverted during bring-up, set its
 * bit here - do not add an inversion at the call site, which is how the
 */
#define RELAY_REVERSED_MASK     0x00U
/* Relay record in the AT24C64 (relay_store.c).  A save waits until no relay
 * has moved for RSTORE_SETTLE_MS; a failed save is retried after
 * RSTORE_RETRY_MS. */
#define RSTORE_SETTLE_MS        250U
#define RSTORE_RETRY_MS         5000U

/* Bench switches. Real PCB: RELAY_DRIVE_COILS 1, MEASURE_SIMULATE 0. */
#define RELAY_DRIVE_COILS       1   /* 0 = pulses timed and tracked, coil pins never driven */
#define MEASURE_SIMULATE        0   /* 1 = synthetic currents/rails, 0 = real ADC (current_sense.c) */



/* External watchdog on WDOG (PB2). SupervisorTask toggles it every
   WDOG_KICK_MS while RelayTask and MeasureTask are alive. Must be well
   inside the watchdog chip's timeout. */
#define WDOG_KICK_MS        50u
#define WDOG_STALL_MS       1000u


#endif /* BOARD_CONFIG_H */
