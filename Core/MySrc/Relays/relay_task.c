/**
 * relay_task.c - F103
 *
 * RelayTask: owns the relays. It restores their positions at boot, runs the
 * pulse scheduler, reports each command's outcome to the H7, keeps the saved
 * record up to date, and corrects positions from current evidence.
 *
 * Boot (nothing is ever pulsed here):
 *   1. relays_init() assumes every relay OFF without touching a coil.
 *   2. The newest record is read from the AT24C64 and relays_restore() takes
 *      those positions. No EEPROM, or no record: they stay assumed OFF.
 *
 * Commands. LinkTask has already answered each one with ACCEPTED. Then:
 *   1. relays_request() records the target; relays_tick() pulses the coil for
 *      RELAY_PULSE_MS, at most RELAY_MAX_CONCURRENT coils at once. A command
 *      always pulses, even if the relay is believed to be there already.
 *   2. Once the relay has settled, wait until a measurement window that
 *      started after the contacts moved has been published (two window ids
 *      later), so the reading is clean.
 *   3. OFF commands: if that reading still shows EVIDENCE_ON_MA or more, the
 *      contacts did not open. Post FAULT with the reading and record the
 *      relay as ON, because the load is still powered.
 *      Everything else: post VERIFIED with the reading. An ON command with no
 *      current is still VERIFIED: nothing may be plugged in.
 *   4. A command that has not got that far within RELAY_VERIFY_TIMEOUT_MS
 *      gets FAULT.
 *
 * Current evidence, every window: a relay believed OFF, not moving and with
 * no command pending, that carries EVIDENCE_ON_MA or more for
 * EVIDENCE_WINDOWS windows in a row is really ON. Its position is corrected
 * without a pulse (relays_adopt()). The reverse cannot be concluded: an ON
 * relay with no current may just have no load.
 *
 * Saving: when the positions (or which ones are established) change, the
 * record is written once no relay has moved for RSTORE_SETTLE_MS, and only
 * while no coil is energised, so the ~8 ms EEPROM write never stretches a
 * pulse. A failed write is retried after RSTORE_RETRY_MS.
 *
 * Only this task calls relays_request(), relays_tick(), relays_adopt() and
 * the relay store.
 */

#include "cmsis_os2.h"
#include "board_config.h"
#include "app_shared.h"
#include "F10Slave.h"       /* link_post_event() */
#include "relays.h"
#include "relay_store.h"
#include "f103_tasks.h"

#define RELAY_TICK_MS             2u      /* scheduler resolution; a pulse is 80 ms */
#define RELAY_VERIFY_TIMEOUT_MS   1000u   /* the H7 gives up after 1500 ms          */

extern I2C_HandleTypeDef hi2c2;           /* i2c.c: the AT24C64 */

typedef struct {
    bool     active;
    bool     settled;       /* at the target, waiting for a clean reading */
    uint8_t  target;
    uint16_t seq;           /* the H7's cmd_seq, echoed in every event    */
    uint32_t window;        /* measurement window id when it settled      */
    uint32_t t_start;
} pending_t;

relay_task_dbg_t     g_relay_dbg;
/* MeasureTask copies this into g_snap. PRESENT until the boot read says
   otherwise, so the first windows do not report a false FAULT_EEPROM. */
relay_truth_t        g_relay_truth = { .store_flags = STORE_F_PRESENT };
volatile uint32_t    g_hb_relay;          /* SupervisorTask watches this */

static pending_t s_pend[RELAY_COUNT];
static uint8_t   s_evidence_run[RELAY_COUNT];

/* Saving */
static uint32_t s_saved_state;
static uint32_t s_saved_known;
static uint32_t s_last_change_ms;
static uint32_t s_retry_at_ms;
static uint32_t s_last_activity;

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

    /* A command replaces whatever the evidence said about this relay. */
    g_relay_truth.evidence_mask &= (uint16_t)~(1u << cmd->relay_idx);
    s_evidence_run[cmd->relay_idx] = 0u;
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
        } else if ((uint32_t)(snap->window_id - p->window) >= 2u) {
            /* The window published at settle time, and the one after it,
               both hold samples from before the contacts moved. This one
               does not. */
            const int32_t ma = snap->relay_ma[i];

            if (p->target == 0u && ma >= EVIDENCE_ON_MA) {
                /* Commanded OFF, still conducting: the contacts did not open. */
                (void)relays_adopt(i, 1u);
                g_relay_truth.evidence_mask |= (uint16_t)(1u << i);
                post(LINK_EVENT_FAULT, i, p->seq, ma);
                g_relay_dbg.stuck_on++;
            } else {
                post(LINK_EVENT_VERIFIED, i, p->seq, ma);
                g_relay_dbg.verified++;
            }
            p->active = false;
            continue;
        }

        if ((uint32_t)(now - p->t_start) >= RELAY_VERIFY_TIMEOUT_MS) {
            post(LINK_EVENT_FAULT, i, p->seq, 0);
            p->active = false;
            g_relay_dbg.faults++;
        }
    }
}

/* Once per new measurement window. */
static void check_evidence(const app_snapshot_t *snap)
{
    for (uint8_t i = 0u; i < RELAY_COUNT; i++) {
        const bool candidate = !s_pend[i].active &&
                               relays_settled(i) &&
                               (relays_get_state(i) == 0u) &&
                               (snap->relay_ma[i] >= EVIDENCE_ON_MA);
        if (!candidate) {
            s_evidence_run[i] = 0u;
            continue;
        }
        if (++s_evidence_run[i] >= EVIDENCE_WINDOWS) {
            s_evidence_run[i] = 0u;
            if (relays_adopt(i, 1u) != 0u) {
                g_relay_truth.evidence_mask |= (uint16_t)(1u << i);
                g_relay_dbg.evidence_on++;
            }
        }
    }
}

static void update_store_flags(bool pending, bool failed)
{
    uint8_t f = g_relay_truth.store_flags & (uint8_t)(STORE_F_PRESENT | STORE_F_LOADED);
    if (pending) { f |= STORE_F_PENDING; }
    if (failed)  { f |= STORE_F_SAVE_FAILED; }
    g_relay_truth.store_flags = f;
}

static void maybe_save(uint32_t now)
{
    const uint32_t state = relays_get_state_mask();
    const uint32_t known = relays_known_mask();
    const uint32_t act   = relays_activity();

    if (act != s_last_activity) {
        s_last_activity  = act;
        s_last_change_ms = now;             /* something moved: wait for quiet */
    }

    const bool dirty = (state != s_saved_state) || (known != s_saved_known);
    const bool failed = (g_relay_truth.store_flags & STORE_F_SAVE_FAILED) != 0u;

    if (!dirty) {
        update_store_flags(false, failed);
        return;
    }
    if ((g_relay_truth.store_flags & STORE_F_PRESENT) == 0u) {
        update_store_flags(true, false);    /* nowhere to save it */
        return;
    }
    update_store_flags(true, failed);

    if (relays_busy() != 0u ||
        (uint32_t)(now - s_last_change_ms) < RSTORE_SETTLE_MS ||
        (int32_t)(now - s_retry_at_ms) < 0) {
        return;
    }

    if (rstore_save((uint16_t)state, (uint16_t)known)) {
        s_saved_state = state;
        s_saved_known = known;
        g_relay_dbg.saves++;
        update_store_flags(false, false);
    } else {
        s_retry_at_ms = now + RSTORE_RETRY_MS;
        g_relay_dbg.save_errors++;
        update_store_flags(true, true);
    }
}

static void restore_positions(void)
{
    uint16_t state = 0u, known = 0u;
    const rstore_load_t r = rstore_load(&hi2c2, (uint8_t)RELAY_COUNT, &state, &known);

    uint8_t flags = 0u;
    if (r != RSTORE_NO_CHIP) { flags |= STORE_F_PRESENT; }
    if (r == RSTORE_LOADED)  { flags |= STORE_F_LOADED;  }
    g_relay_truth.store_flags = flags;

    relays_restore(state, known);

    /* What is in the chip now is what we just read: nothing to save until a
       relay changes. With no record, the assumed-OFF state is saved once so
       the next boot finds one. */
    s_saved_state = (r == RSTORE_LOADED) ? state : 0xFFFFFFFFu;
    s_saved_known = (r == RSTORE_LOADED) ? known : 0xFFFFFFFFu;
    g_relay_dbg.restored_state = state;
    g_relay_dbg.restored_known = known;
    g_relay_dbg.store_result   = (uint32_t)r;
}

void RelayTask_Run(void *argument)
{
    (void)argument;

    relays_init();          /* assumes OFF, pulses nothing */
    restore_positions();    /* ~60 ms of EEPROM reads      */

    s_last_activity  = relays_activity();
    s_last_change_ms = HAL_GetTick();
    s_retry_at_ms    = s_last_change_ms;

    uint32_t last_window = g_snap[g_snap_active].window_id;

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

        const uint32_t now = HAL_GetTick();
        check_pending(now);

        const app_snapshot_t *snap = &g_snap[g_snap_active];
        if (snap->window_id != last_window) {
            last_window = snap->window_id;
            if ((snap->fault_flags & (FAULT_ADC_START | FAULT_ADC_STALL)) == 0u) {
                check_evidence(snap);
            }
        }

        maybe_save(now);
        g_hb_relay++;

        g_relay_dbg.state_mask = relays_get_state_mask();
        g_relay_dbg.known_mask = relays_known_mask();
    }
}
