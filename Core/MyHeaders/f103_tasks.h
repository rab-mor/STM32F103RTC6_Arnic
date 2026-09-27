/**
 * f103_tasks.h - F103 header file
 *
 * RelayTask, MeasureTask and SupervisorTask entry points, and their live stats.
 *
 * All three are created from freertos.c (USER CODE BEGIN RTOS_THREADS).
 *
 * Ownership:
 *   LinkTask       - SPI2 to the H7; ACCEPTED/REJECTED for each command
 *   RelayTask      - relays_request() / relays_tick(); VERIFIED / FAULT
 *   MeasureTask    - fills g_snap[] once per 50 ms measurement window
 *   SupervisorTask - kicks the external watchdog (WDOG, PB2) while RelayTask,
 *                    MeasureTask and LinkTask are all still running
 */

#ifndef F103_TASKS_H
#define F103_TASKS_H

#include <stdint.h>

/* All loop forever. */
void RelayTask_Run(void *argument);
void MeasureTask_Run(void *argument);
void SupervisorTask_Run(void *argument);

/* Heartbeats: each task bumps its own counter every pass. SupervisorTask stops
   kicking the watchdog if any one stops moving for WDOG_STALL_MS. LinkTask
   wakes every 100 ms even when the H7 is silent, so an idle H7 does not look
   like a hung LinkTask. */
extern volatile uint32_t g_hb_relay;     /* relay_task.c,   every 2 ms   */
extern volatile uint32_t g_hb_measure;   /* measure_task.c, every 12.5 ms */
extern volatile uint32_t g_hb_link;      /* F10Slave.c,     <= 100 ms     */

/* ------------------------------------------------------------------------
 * RelayTask stats. Add "g_relay_dbg" to Live Expressions.
 * ---------------------------------------------------------------------- */
typedef struct {
    uint32_t cmds;          /* commands taken from relay_cmd_q              */
    uint32_t verified;      /* VERIFIED sent to the H7                      */
    uint32_t faults;        /* FAULT sent: not settled within the timeout   */
    uint32_t superseded;    /* FAULT sent: a newer command for that relay   */
    uint32_t rejected;      /* REJECTED sent: relays_request() refused it   */
    uint32_t stuck_on;      /* FAULT sent: still conducting after OFF       */
    uint32_t evidence_on;   /* positions corrected to ON by current         */
    uint32_t saves;         /* relay records written                        */
    uint32_t save_errors;   /* relay record writes that failed              */
    uint32_t store_result;  /* boot: 0 no EEPROM, 1 no record, 2 restored   */
    uint32_t restored_state;/* boot: positions read from the record        */
    uint32_t restored_known;
    uint32_t state_mask;    /* bit n = relay n ON, as far as we know        */
    uint32_t known_mask;    /* bit n = relay n's position is established    */
} relay_task_dbg_t;

extern relay_task_dbg_t g_relay_dbg;

/* ------------------------------------------------------------------------
 * MeasureTask stats. Add "g_measure_dbg" to Live Expressions.
 * ---------------------------------------------------------------------- */
typedef struct {
    int32_t  start_rc;      /* sense_start(): 0 = running. -1/-2 ADC calibration,
                               -3 ADC2 start, -4 DMA start, -5 TIM3 start   */
    uint32_t windows;       /* 50 ms windows published                      */
    uint32_t calibrated;    /* 1 once the zero-current gate is learned
                               (about 3 s after the relays settle at boot)  */
    uint32_t overruns;      /* DMA halves lost: MeasureTask fell behind     */
    uint32_t stalls;        /* times no window arrived for 200 ms           */
    int32_t  relay_ma[8];   /* last published currents, mA                  */
    uint32_t rail_24v_mv;
    uint32_t rail_5v_mv;
} measure_dbg_t;

extern measure_dbg_t g_measure_dbg;

/* ------------------------------------------------------------------------
 * SupervisorTask stats. Add "g_sup_dbg" to Live Expressions.
 * ---------------------------------------------------------------------- */
typedef struct {
    uint32_t kicks;         /* WDOG toggles                                 */
    uint32_t withheld;      /* passes where a task had stalled: no kick     */
    uint32_t stalled_mask;  /* bit 0 RelayTask, 1 MeasureTask, 2 LinkTask; sticky */
    uint32_t has_pin;       /* 0 = no WDOG pin in this build (nothing kicked) */
} supervisor_dbg_t;

extern supervisor_dbg_t g_sup_dbg;

#endif /* F103_TASKS_H */
