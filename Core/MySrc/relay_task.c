/**
 * relay_task.c - F103
 *
 * RelayTask: runs the relay scheduler and reports each command's outcome
 * to the H7.
 *
 * LinkTask has already answered each command with ACCEPTED (queued, coil
 * not fired yet). From there, per command:
 *
 *   1. relays_request() records the target; relays_tick() pulses the coil
 *      for RELAY_PULSE_MS, at most RELAY_MAX_CONCURRENT coils at once.
 *   2. Once the relay has settled on the command's target, wait for the
 *      next measurement window from MeasureTask, so the reply carries a
 *      current reading taken with the contacts in their new position.
 *   3. Post VERIFIED with that reading. A command that has not got that far
 *      within RELAY_VERIFY_TIMEOUT_MS gets FAULT instead.
 *
 * The H7 sends one relay at a time and waits for the answer, so normally
 * only one command is outstanding. If a newer command for the same relay
 * arrives first, the older one is answered FAULT (it never completed as
 * asked) and the newer one takes over.
 *
 * VERIFIED is position-based: a latching relay has no contact feedback, and a
 * circuit with nothing plugged in legitimately reads 0 A, so the current cannot
 * prove the contacts moved. The reading is reported so the H7 can see it.
 *
 * Only this task calls relays_request() and relays_tick().
 */

#include "cmsis_os2.h"
#include "board_config.h"
#include "app_shared.h"
#include "F10Slave.h"       /* link_post_event() */
#include "relays.h"
#include "f103_tasks.h"

#define RELAY_TICK_MS             2u      /* scheduler resolution; a pulse is 80 ms */
#define RELAY_VERIFY_TIMEOUT_MS   1000u   /* the H7 gives up after 1500 ms          */

typedef struct {
    bool     active;
    bool     settled;       /* at the target, waiting for a fresh reading */
    uint8_t  target;
    uint16_t seq;           /* the H7's cmd_seq, echoed in every event    */
    uint32_t window;        /* measurement window id when it settled      */
    uint32_t t_start;
} pending_t;

relay_task_dbg_t  g_relay_dbg;
volatile uint32_t g_hb_relay;       /* SupervisorTask watches this */

static pending_t s_pend[RELAY_COUNT];

static void post(link_event_type_t type, uint8_t idx, uint16_t seq, int32_t ma)
{
    const link_event_t e = {
        .type        = (uint8_t)type,
        .relay_idx   = idx,
        .seq         = seq,
        .measured_ma = ma,
    };
    link_post_event(&e);
}

static void accept(const relay_cmd_t *cmd, uint32_t now)
{
    g_relay_dbg.cmds++;

    if (relays_request(cmd->relay_idx, cmd->target) != RELAYS_OK) {
        /* LinkTask validates index and target, so this should not happen. */
        post(LINK_EVENT_REJECTED, cmd->relay_idx, cmd->seq, 0);
        g_relay_dbg.rejected++;
        return;
    }

    pending_t *p = &s_pend[cmd->relay_idx];
    if (p->active) {
        post(LINK_EVENT_FAULT, cmd->relay_idx, p->seq, 0);
        g_relay_dbg.superseded++;
    }

    p->active  = true;
    p->settled = false;
    p->target  = cmd->target;
    p->seq     = cmd->seq;
    p->window  = 0u;
    p->t_start = now;
}

static void check_pending(uint32_t now)
{
    const app_snapshot_t *snap = &g_snap[g_snap_active];

    for (uint8_t i = 0u; i < RELAY_COUNT; i++) {
        pending_t *p = &s_pend[i];
        if (!p->active) {
            continue;
        }

        if (!p->settled) {
            if (relays_settled(i) && (relays_get_state(i) == p->target)) {
                p->settled = true;
                p->window  = snap->window_id;
            }
        } else if (snap->window_id != p->window) {
            /* A full window has been measured since the contacts moved. */
            post(LINK_EVENT_VERIFIED, i, p->seq, snap->relay_ma[i]);
            p->active = false;
            g_relay_dbg.verified++;
            continue;
        }

        if ((uint32_t)(now - p->t_start) >= RELAY_VERIFY_TIMEOUT_MS) {
            post(LINK_EVENT_FAULT, i, p->seq, 0);
            p->active = false;
            g_relay_dbg.faults++;
        }
    }
}

void RelayTask_Run(void *argument)
{
    (void)argument;

    relays_init();      /* queues every relay to OFF; the scheduler drives them there */

    for (;;) {
        relay_cmd_t cmd;

        /* Wait up to one tick for a command, then take any others queued. */
        if (osMessageQueueGet(relay_cmd_qHandle, &cmd, NULL, RELAY_TICK_MS) == osOK) {
            const uint32_t t = HAL_GetTick();
            accept(&cmd, t);
            while (osMessageQueueGet(relay_cmd_qHandle, &cmd, NULL, 0u) == osOK) {
                accept(&cmd, t);
            }
        }

        relays_tick();
        check_pending(HAL_GetTick());
        g_hb_relay++;

        g_relay_dbg.state_mask = relays_get_state_mask();
        g_relay_dbg.known_mask = relays_known_mask();
    }
}
