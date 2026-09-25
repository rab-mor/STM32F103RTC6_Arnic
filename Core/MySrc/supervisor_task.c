/**
 * supervisor_task.c - F103
 *
 * SupervisorTask: services the external watchdog on the WDOG pin (PB2).
 *
 * The watchdog chip resets the F103 unless WDOG toggles often enough. This
 * task toggles it every WDOG_KICK_MS, but only while RelayTask, MeasureTask
 * and LinkTask are all still making progress (their heartbeat counters keep
 * moving). If any one hangs for WDOG_STALL_MS the kicks stop and the chip
 * resets the board, which is the point of having it.
 *
 * LinkTask wakes every 100 ms whether or not the H7 is polling, so a silent
 * H7 does not stop the kicks; only a stuck LinkTask does.
 *
 * Debugging: halting the F103 at a breakpoint stops the kicks too, so the
 * watchdog chip resets it once its timeout passes. Live Expressions (which
 * does not halt the core) is unaffected.
 *
 * Timing: WDOG_KICK_MS must be comfortably shorter than the watchdog chip's
 * timeout. Check it against the chip's datasheet (board_config.h). A
 * "windowed" watchdog also has a minimum time between kicks; if the part is
 * one of those, WDOG_KICK_MS must sit inside its window.
 */

#include <stdbool.h>
#include "cmsis_os2.h"
#include "main.h"
#include "board_config.h"
#include "f103_tasks.h"

#ifndef WDOG_KICK_MS
#error "Add WDOG_KICK_MS and WDOG_STALL_MS to board_config.h"
#endif

supervisor_dbg_t g_sup_dbg;

typedef struct {
    const volatile uint32_t *hb;
    uint32_t                 last;
    uint32_t                 since;
} watch_t;

static inline void kick(void)
{
#if defined(WDOG_Pin) && defined(WDOG_GPIO_Port)
    HAL_GPIO_TogglePin(WDOG_GPIO_Port, WDOG_Pin);
#endif
}

void SupervisorTask_Run(void *argument)
{
    (void)argument;

#if defined(WDOG_Pin) && defined(WDOG_GPIO_Port)
    g_sup_dbg.has_pin = 1u;
#else
    g_sup_dbg.has_pin = 0u;     /* no pin labelled WDOG in CubeMX: nothing to kick */
#endif

    watch_t w[] = {
        { &g_hb_relay,   0u, 0u },      /* bit 0 */
        { &g_hb_measure, 0u, 0u },      /* bit 1 */
        { &g_hb_link,    0u, 0u },      /* bit 2 */
    };
    const uint32_t n = (uint32_t)(sizeof(w) / sizeof(w[0]));

    uint32_t now = HAL_GetTick();
    for (uint32_t i = 0u; i < n; i++) {
        w[i].last  = *w[i].hb;
        w[i].since = now;               /* WDOG_STALL_MS of grace at start-up */
    }

    uint32_t next = osKernelGetTickCount();

    for (;;) {
        next += WDOG_KICK_MS;
        (void)osDelayUntil(next);

        now = HAL_GetTick();
        bool all_alive = true;

        for (uint32_t i = 0u; i < n; i++) {
            const uint32_t v = *w[i].hb;
            if (v != w[i].last) {
                w[i].last  = v;
                w[i].since = now;
            } else if ((uint32_t)(now - w[i].since) >= WDOG_STALL_MS) {
                all_alive = false;
                g_sup_dbg.stalled_mask |= (1u << i);
            }
        }

        if (all_alive) {
            kick();
            g_sup_dbg.kicks++;
        } else {
            g_sup_dbg.withheld++;       /* the watchdog chip will reset us */
        }
    }
}
