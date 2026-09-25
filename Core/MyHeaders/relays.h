/**
 * relays.h - F103 header file
 *
 * Non-blocking driver for eight dual-coil latching relays (ADJH23012)
 * driven through two ULN2803 Darlington arrays.
 *
 * Usage from the super-loop:
 *
 *     relays_init();            // once, after MX_GPIO_Init()
 *     for (;;) {
 *         relays_tick();        // every pass, no delay needed
 *         ...
 *     }
 *
 * Commands arriving from the link call relays_request().  They return
 * immediately; the coil pulse is scheduled and completed by relays_tick().
 */

#ifndef RELAYS_H
#define RELAYS_H

#include <stdint.h>

#define RELAYS_OK               0
#define RELAYS_ERR_INDEX       -1
#define RELAYS_ERR_STATE       -2

/**
 * Drive every coil low and queue all relays to the OFF state.
 *
 * Does NOT block.  The eight de-energising pulses are staggered by the
 * scheduler over roughly (8 / RELAY_MAX_CONCURRENT) * RELAY_PULSE_MS.
 * Poll relays_busy() if you need to know when the board has settled.
 *
 * Call after MX_GPIO_Init().
 */
void relays_init(void);

/**
 * Advance the pulse scheduler.  Call from the main loop as often as
 * possible; resolution is the 1 ms HAL tick.
 */
void relays_tick(void);

/**
 * Request a relay state.  Non-blocking.
 *
 * @param idx    relay index, 0..RELAY_COUNT-1
 * @param state  1 = ON, 0 = OFF
 *
 * A request for the state a relay is already known to be in is a no-op -
 * no coil is pulsed.  A request that arrives while that relay is mid-pulse
 * is recorded and acted on after the current pulse completes.
 *
 * @return RELAYS_OK, or RELAYS_ERR_INDEX / RELAYS_ERR_STATE
 */
int relays_request(uint8_t idx, uint8_t state);

/**
 * Apply a batch of relay changes from a 32-bit update/state mask pair.
 * Bit N of update_mask selects relay N; bit N of state_mask is its state.
 *
 * @return count of relays that failed validation (0 on success)
 */
uint32_t relays_apply_mask(uint32_t update_mask, uint32_t state_mask);

/**
 * Last known state of a relay.  Only meaningful once relays_state_known()
 * returns non-zero for that index.
 */
uint8_t relays_get_state(uint8_t idx);

/**
 * Non-zero once a completed pulse has established a known position for
 * this relay.  Zero from reset until the first pulse finishes - a
 * latching relay holds its position across a power cycle, so firmware
 * cannot infer position at boot.
 */
uint8_t relays_state_known(uint8_t idx);

/**
 * Non-zero when this relay has nothing left to do: no coil energised, its
 * position is known, and that position is the last one requested.
 */
uint8_t relays_settled(uint8_t idx);

/**
 * Packed shadow state, bit N = relay N.  Suitable for the status pay-load
 * pushed back over the link.  Pair it with relays_known_mask() so the
 * host can tell "off" from "not yet established".
 */
uint32_t relays_get_state_mask(void);
uint32_t relays_known_mask(void);
uint32_t relays_activity(void);

/**
 * Non-zero if any coil is energized or any request is still outstanding.
 */
uint8_t relays_busy(void);

#endif /* RELAYS_H */
