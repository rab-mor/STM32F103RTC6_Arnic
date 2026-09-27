/**
 * relays.h - F103 header file (main board and expansion module)
 *
 * Non-blocking driver for the dual-coil latching relays (ADJH23012)
 * driven through ULN2803 Darlington arrays: 8 on the main board, 12 on the
 * expansion module (RELAY_COUNT, board_config.h).
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
 * Drive every coil low and assume every relay is OFF, without pulsing
 * anything.  Nothing moves until a command arrives.
 *
 * Call after MX_GPIO_Init(), then relays_restore() with the saved record.
 */
void relays_init(void);

/**
 * Take the positions from the saved record: bit N of state_mask is relay N's
 * position, bit N of established_mask says it was known (not assumed) when
 * saved.  No coil is pulsed.  A relay that already has a command in flight
 * keeps it.
 */
void relays_restore(uint32_t state_mask, uint32_t established_mask);

/**
 * Current evidence: the relay is really in `state` (for example, current is
 * flowing through a relay believed OFF).  Updates the position without a
 * pulse.  Ignored while that relay is moving.
 *
 * @return 1 if the recorded position changed, 0 otherwise
 */
uint8_t relays_adopt(uint8_t idx, uint8_t state);

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
 * Non-zero once this relay's position is established: a pulse completed,
 * the saved record said so, or current evidence showed it.  Zero while the
 * position is only assumed (first boot, no record).
 */
uint8_t relays_state_known(uint8_t idx);

/**
 * Non-zero when this relay has nothing left to do: no coil energised, no
 * command waiting for a pulse, and its position is the last one requested.
 */
uint8_t relays_settled(uint8_t idx);

/**
 * Packed shadow state, bit N = relay N.  Suitable for the status pay-load
 * pushed back over the link.  relays_known_mask() is the established mask,
 * so the host can tell "off" from "assumed off".
 */
uint32_t relays_get_state_mask(void);
uint32_t relays_known_mask(void);
uint32_t relays_activity(void);

/**
 * Non-zero if any coil is energized or any request is still outstanding.
 */
uint8_t relays_busy(void);

#endif /* RELAYS_H */
