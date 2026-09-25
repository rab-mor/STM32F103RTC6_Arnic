#ifndef APP_SHARED_H
#define APP_SHARED_H
/* Shared Application Structures and Queues (between F103 and H755)
 * This header is necessary for the shared queue handles that are in freertos.c as well as the shared structs between the STMH7 and the F103.
 *
 *
 *
 * Includes*/
#include <stdint.h>
#include <stdbool.h>
#include "cmsis_os2.h"
#include "board_config.h"
#include "link_proto.h"
/*end includes*/

/* Macro Definitions
* SPI
* bit 0 --> set when an RX frame is receieved
* bit 1 --> set when a TX frame is ready to be sent */
#define LINK_FLAG_FRAME_RX 		(1u << 0) /* EXTI rising edge: frame in rx Buf */
#define LINK_FLAG_TX_READY 		(1u << 1) /* MeasureTask new snapshot ready to be transferred to STMH7  */
#define LINK_FLAG_SPI_ERROR 	(1u << 2)

#define FAULT_UNCALIBRATED (1u << 0)
#define FAULT_ADC_START    (1u << 1)   /* sense_start() failed: no measurements   */
#define FAULT_ADC_STALL    (1u << 2)   /* no ADC window for 200 ms                 */
#define FAULT_ADC_OVERRUN  (1u << 3)   /* DMA half-buffer lost: MeasureTask late   */

// ADC
#define MEAS_FLAG_WINDOW_DONE   (1u << 0)  /* ADC DMA TC: window closed         */
#define MEAS_FLAG_HALF          (1u << 1)  /* ADC DMA HT: reserved, unused      */


/*---------------- external handles and structs--------------- */
extern osThreadId_t LinkTaskHandle;
extern osThreadId_t MeasureTaskHandle;
extern osThreadId_t RelayTaskHandle;
extern osThreadId_t SupervisorTaskHandle;

extern osMessageQueueId_t relay_cmd_qHandle; // freertos.c
extern osMessageQueueId_t event_qHandle; 	// freertos.c

/* ---------- relay_cmd_q: LinkTask -> RelayTask ---------- */

typedef struct {
	uint8_t 	relay_idx; // which relay index 0-15
	uint8_t 	target; // 1 = on , 0 = off
	uint16_t	seq;  	// H7's sequence number echoed in both acks
} relay_cmd_t;
// Assert Size at Compile time
_Static_assert(sizeof(relay_cmd_t)  == 4, "relay_cmd_t size changed");


/* ---------- event_q: RelayTask -> LinkTask ---------- */


typedef struct {
    uint8_t  type;        // accepted / verified / rejected / fault
    uint8_t  relay_idx;
    uint16_t seq;
    int32_t  measured_ma; // valid for 'verified'
} link_event_t;
// Assert Size at Compile time
_Static_assert(sizeof(link_event_t) == 8, "link_event_t size changed");


/*---------------Snapshot: MeasureTask --------------------------*/

typedef struct {
	uint32_t	window_id;
	int32_t 	relay_ma[RELAY_COUNT];
	uint16_t 	rail_24V_mv;
	uint16_t 	rail_5V_mv;
	uint8_t 	relay_state;
	uint8_t 	fault_flags;
	uint16_t 	dropped_events;

} app_snapshot_t;
_Static_assert(sizeof(app_snapshot_t) == 44, "app_snapshot_t layout changed");


extern app_snapshot_t    g_snap[2];
extern volatile uint8_t g_snap_active;
extern volatile uint16_t g_dropped_events;


#endif /*APP_SHARED_H */
