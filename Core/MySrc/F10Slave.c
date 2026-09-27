/**
 * @file    F10Slave.c - F103
 * @brief   SPI slave side of the H755 <-> F103 link.
 *
 * Keeps a frame armed in the SPI DMA at all times. The H7 (master) polls on a
 * fixed cadence; there is no DRDY line.
 *
 * Transaction lifecycle:
 *
 *   1. LinkTask builds a frame, computes its CRC, arms the DMA.
 *   2. The H7 pulls CS low and clocks 64 bytes both directions.
 *   3. TxRxCplt fires; LinkTask is woken.
 *   4. LinkTask processes the received frame, builds the next one, re-arms.
 *
 * The H7 holds CS high for >= 50 us between transactions, which is the window
 * LinkTask has to re-arm in.
 *
 * What the next frame carries (build_tx_frame), in this order:
 *   1. OTA_STATUS, right after an OTA request: the result of that request.
 *   2. METER_DATA when there is a window the H7 has not been sent yet, or
 *      relay events waiting. The H7 polls every 20 ms and a window lasts
 *      50 ms, so every window goes out at least once.
 *   3. HARMONICS: the next part of the newest harmonics set, in the polls
 *      that would otherwise repeat a window.
 *   4. METER_DATA again.
 *
 * The H7 passes the line frequency in every POLL and COMMAND; it goes to the
 * harmonic detectors. After the reply to a successful OTA END has been
 * clocked out, the board resets into the bootloader, which installs it.
 *
 * Usage (freertos.c, StartLinkTask):
 *
 *      F10Slave_CsExtiInit();
 *      F10Slave_Init(&hspi2);          // SPI2 on the real F103 board
 *      for (;;) { F10Slave_Service(); }
 *
 * The HAL callbacks are defined at the bottom of this file, so delete any
 * CubeMX-generated stubs for them in main.c.
 *
 * Portability: builds for the F103 (the real board) and the F411 (the bench
 * stub). The only chip-specific part is the CS EXTI routing, which lives in
 * AFIO on the F1 and SYSCFG on the F4.
 */

#include <string.h>
#include "app_shared.h"
#include "cmsis_os2.h"
#include "F10Slave.h"
#include "f103_tasks.h"     /* g_hb_link */
#include "harmonics.h"
#include "f1_ota.h"

/* LinkTask wakes at least this often even when the H7 is silent, so a failed
   DMA arm is retried and SupervisorTask can see that LinkTask is alive. */
#define F10SLAVE_IDLE_MS    100u


_Static_assert(RELAY_COUNT == LINK_RELAY_COUNT, "relay count must match the wire format");
/* ==========================================================================
 * State
 * ========================================================================== */

static SPI_HandleTypeDef *s_hspi = NULL;

static F10Comm_Frame_t    s_tx_frame[2];
static F10Comm_Frame_t    s_rx_frame[2];
static uint8_t            s_tx_seq = 0u;

static volatile uint8_t s_active   = 0u;   /* buffer currently in the DMA */
static volatile bool    s_armed    = false; /* a transfer is waiting for the H7 */

volatile uint32_t g_hb_link;               /* SupervisorTask watches this */

// counters for relay commands received and relay status transferred
static volatile uint8_t relay_cmds_rx = 0;

static uint32_t          s_last_sent_window;   /* window_id of the last METER_DATA built */
static bool              s_ota_reply;          /* next frame is OTA_STATUS                */
static link_ota_status_t s_ota_status;
static bool              s_reboot;             /* OTA END accepted                        */
static uint8_t           s_reboot_stage;       /* 1 reply armed, 2 reply clocked out      */
static uint32_t          s_reboot_armed_ms;

static harm_set_t        s_harm;               /* the set being sent, copied once         */
static uint8_t           s_harm_id;            /* its set_id                              */
static uint8_t           s_harm_part = LINK_HARM_PARTS;   /* next part; PARTS = all sent  */

_Static_assert(LINK_OTA_QUERY == F1OTA_OP_QUERY && LINK_OTA_BEGIN == F1OTA_OP_BEGIN &&
               LINK_OTA_DATA == F1OTA_OP_DATA && LINK_OTA_END == F1OTA_OP_END &&
               LINK_OTA_ABORT == F1OTA_OP_ABORT, "OTA op numbers must match f1_ota.h");
_Static_assert(HARM_RATIOS == LINK_HARM_RATIOS, "harmonics record layout");


/* Diagnostics -- deliberately non-static so Live Expressions can reach them. */
volatile uint32_t dbg_s_truncated  = 0u; /**< CS went high mid frame           */
volatile uint32_t dbg_s_xfers      = 0u;  /**< Completed SPI transactions.      */
volatile uint32_t dbg_s_armed      = 0u;  /**< Successful DMA arms.             */
volatile uint32_t dbg_s_arm_fail   = 0u;  /**< HAL refused to arm the DMA.      */
volatile uint32_t dbg_s_spi_errors = 0u;  /**< SPI/DMA error callbacks.         */
volatile uint32_t dbg_s_crc_errors = 0u;  /**< Master frames failing CRC.       */
volatile uint32_t dbg_s_bad_magic  = 0u;  /**< Master frames with wrong magic.  */
volatile uint8_t  dbg_s_last_type  = 0u;  /**< Type of last valid master frame. */
volatile uint32_t dbg_s_cs_edges   = 0u;  /**< CS rising edges seen by EXTI.    */

/* ==========================================================================
 * Helpers
 * ========================================================================== */

void link_post_event(const link_event_t *evt) {

	if(osMessageQueuePut(event_qHandle, evt, 0u, 0u) != osOK) {
		link_event_t discard;
		(void)osMessageQueueGet(event_qHandle, &discard, NULL, 0u);
		g_snap[0].dropped_events++;
		g_snap[1].dropped_events++;
		(void)osMessageQueuePut(event_qHandle, evt, 0u, 0u);
	}
}


static void build_meter(F10Comm_Frame_t *f)
{
    f->type = (uint8_t)F10_TYPE_METER_DATA;

    /* Running state (int32_t mA) -> this frame's wire view (int16_t mA). */
    const app_snapshot_t *s = &g_snap[g_snap_active];
    s_last_sent_window = s->window_id;

    link_status_t st;
    memset(&st, 0, sizeof(st));

    st.window_id = s->window_id;

    for (uint8_t i = 0u; i < LINK_RELAY_COUNT; i++) {
        /* Clamp rather than truncate: a wrapped int16_t reads as a sign
         * flip on the H7, which looks like a real measurement. */
        int32_t ma = s->relay_ma[i];
        if (ma >  32767)  { ma =  32767; }
        if (ma < -32768)  { ma = -32768; }
        st.relay_ma[i] = (int16_t)ma;
    }

    st.rail_24v_mv    = s->rail_24V_mv;
    st.rail_5v_mv     = s->rail_5V_mv;
    st.relay_state    = s->relay_state;
    st.fault_flags    = s->fault_flags;
    st.dropped_events = s->dropped_events;
    st.relay_known    = s->relay_known;
    st.relay_evidence = s->relay_evidence;
    st.store_flags    = s->store_flags;

    /* Events are per-frame, not running state, so they are drained here and
     * not from g_snap. Timeout 0: if the queue is empty, event_count stays 0. */
    link_event_t evt;
    while ((st.event_count < LINK_MAX_EVENTS_PER_FRAME) &&
           (osMessageQueueGet(event_qHandle, &evt, NULL, 0u) == osOK)) {
        st.events[st.event_count].type        = evt.type;
        st.events[st.event_count].relay_idx   = evt.relay_idx;
        st.events[st.event_count].cmd_seq     = evt.seq;
        st.events[st.event_count].measured_ma = evt.measured_ma;
        st.event_count++;
    }

    link_encode_status(f->payload, &st);
}

/* A harmonics part is due: a newer set exists, or parts of this one are left. */
static bool harm_part_due(void)
{
    const uint8_t id = harm_latest_id();
    if (id != 0u && id != s_harm_id) {
        if (harm_get(&s_harm)) {
            s_harm_id   = s_harm.set_id;
            s_harm_part = 0u;
        }
    }
    return s_harm_part < LINK_HARM_PARTS;
}

static void build_harm(F10Comm_Frame_t *f)
{
    link_harm_part_t hp;
    hp.set_id      = s_harm.set_id;
    hp.part        = s_harm_part;
    hp.f0_centi_hz = s_harm.f0_centi_hz;
    for (uint8_t i = 0u; i < LINK_HARM_RELAYS_PER_PART; i++) {
        const harm_rec_t *h = &s_harm.rec[s_harm_part * LINK_HARM_RELAYS_PER_PART + i];
        hp.rec[i].h1_ma = h->h1_ma;
        for (uint8_t k = 0u; k < LINK_HARM_RATIOS; k++) {
            hp.rec[i].ratio[k] = h->ratio[k];
            hp.rec[i].phase[k] = h->phase[k];
        }
    }
    s_harm_part++;
    f->type = (uint8_t)F10_TYPE_HARMONICS;
    link_encode_harm_part(f->payload, &hp);
}

static void build_tx_frame(uint8_t idx)
{
    F10Comm_Frame_t *f = &s_tx_frame[idx];

    /* Note sizeof(*f), not sizeof(s_tx_frame): the latter is the whole
     * two-element array and would run off the end when idx == 1. */
    memset(f, 0, sizeof(*f));

    f->magic    = F10COMM_MAGIC;
    f->version  = F10COMM_PROTO_VERSION;
    f->seq      = s_tx_seq++;
    f->reserved = 0u;

    const bool new_window = (g_snap[g_snap_active].window_id != s_last_sent_window);
    const bool events     = (osMessageQueueGetCount(event_qHandle) > 0u);

    if (s_ota_reply) {
        s_ota_reply = false;
        f->type = (uint8_t)F10_TYPE_OTA_STATUS;
        link_encode_ota_status(f->payload, &s_ota_status);
        if (s_reboot && s_reboot_stage == 0u) {
            s_reboot_stage    = 1u;         /* the END reply is armed */
            s_reboot_armed_ms = HAL_GetTick();
        }
    } else if (new_window || events || !harm_part_due()) {
        build_meter(f);
    } else {
        build_harm(f);
    }

    /* CRC spans everything ahead of the crc16 field itself. */
    f->crc16 = F10Comm_Crc16((const uint8_t *)f, F10COMM_CRC_SPAN);
}

/**
 * @brief Build the next frame and arm the DMA.
 * @return true if the DMA armed successfully. else false
 */
static bool arm_next_transfer(void)
{
    build_tx_frame(s_active);
    memset(&s_rx_frame[s_active], 0, sizeof(s_rx_frame[s_active]));

    if (HAL_SPI_TransmitReceive_DMA(s_hspi,
                                    (uint8_t *)&s_tx_frame[s_active],
                                    (uint8_t *)&s_rx_frame[s_active],
                                    F10COMM_FRAME_SIZE) != HAL_OK) {
        dbg_s_arm_fail++;
        s_armed = false;                /* F10Slave_Service retries it */
        return false;
    }

    dbg_s_armed++;
    s_armed = true;
    return true;
}

/**
 * @brief Validate and act on the frame the master just clocked in.
 */
static void process_rx_frame(void)
{
    const F10Comm_Frame_t *f = &s_rx_frame[s_active];

    if (f->magic != F10COMM_MAGIC) {
        /* Master polled before we were armed, or sent filler. Expected
         * occasionally - the master re-polls. Not an error on its own. */
        dbg_s_bad_magic++;
        return;
    }

    if (f->version != F10COMM_PROTO_VERSION) {
        dbg_s_bad_magic++;
        return;
    }

    const uint16_t calc = F10Comm_Crc16((const uint8_t *)f, F10COMM_CRC_SPAN);
    if (calc != f->crc16) {
        dbg_s_crc_errors++;
        return;
    }

    dbg_s_last_type = f->type;

    switch (f->type) {

        case F10_TYPE_COMMAND: {   /* braced: C forbids a declaration
                                    * immediately after a case label */
            link_cmd_t wire;
            relay_cmds_rx++;
            if (link_decode_cmd(f->payload, &wire) != 0) {
                /* Bad relay index or target. link_decode_cmd fills the fields
                 * before validating, so the seq is still good to echo back. */
                link_event_t evt = {
                    .type        = (uint8_t)LINK_EVENT_REJECTED,
                    .relay_idx   = wire.relay_idx,
                    .seq         = wire.cmd_seq,
                    .measured_ma = 0
                };
                link_post_event(&evt);
                break;
            }

            harm_set_f0(link_get_f0(f->payload, LINK_F0_OFF_COMMAND));

            relay_cmd_t cmd = {
                .relay_idx = wire.relay_idx,
                .target    = wire.target,
                .seq       = wire.cmd_seq
            };

            link_event_t evt = {
                .relay_idx   = cmd.relay_idx,
                .seq         = cmd.seq,
                .measured_ma = 0
            };

            /* Phase one of the ack: queued, coil not yet fired. RelayTask
             * posts VERIFIED or FAULT later against this same seq. */
            if (osMessageQueuePut(relay_cmd_qHandle, &cmd, 0u, 0u) != osOK) {
                evt.type = (uint8_t)LINK_EVENT_REJECTED;   /* queue full */
            } else {
                evt.type = (uint8_t)LINK_EVENT_ACCEPTED;
            }
            link_post_event(&evt);
            break;
        }

        case F10_TYPE_POLL:
            /* Master just wanted our data; the reply is already armed. */
            harm_set_f0(link_get_f0(f->payload, LINK_F0_OFF_POLL));
            break;

        case F10_TYPE_OTA: {
            link_ota_req_t r;
            uint8_t res = F1OTA_ERR_PARAM;
            if (link_decode_ota_req(f->payload, &r) == 0) {
                res = f1_ota_request(r.op, r.offset, r.data, r.len, &s_reboot);
            }
            s_ota_status.op_seq      = r.op_seq;
            s_ota_status.result      = res;
            s_ota_status.state       = f1_ota_state();
            s_ota_status.board_type  = (uint8_t)F1_THIS_BOARD;
            s_ota_status.next_offset = f1_ota_next();
            s_ota_status.fw_version  = (uint16_t)((FW_VERSION_MAJOR << 8) | FW_VERSION_MINOR);
            s_ota_reply = true;             /* goes out in the very next frame */
            break;
        }

        default:
            break;
    }
}

/* ==========================================================================
 * Public
 * ========================================================================== */

/* CubeMX can't express "SPI NSS pin + EXTI", so the EXTI routing is written
 * here by hand. The GPIO mode register is untouched, so the SPI keeps
 * hardware NSS framing; EXTI taps the pin's input path, which stays live.
 * Rising edge only: CS is active low, so rising = end of transaction.
 * Call once, after MX_GPIO_Init() and MX_SPIx_Init(). */
void F10Slave_CsExtiInit(void)
{
    uint32_t pin_num = 0u;
    while (((F_CS_Pin >> pin_num) & 1u) == 0u) { pin_num++; }

    /* GPIOA=0, GPIOB=1, ... - the ports sit 0x400 apart on both families. */
    const uint32_t port_idx = (((uint32_t)F_CS_GPIO_Port) - GPIOA_BASE) >> 10u;
    const uint32_t shift    = (pin_num & 3u) * 4u;

#if defined(STM32F1)
    /* F1: the line-to-port routing is in AFIO. */
    __HAL_RCC_AFIO_CLK_ENABLE();
    uint32_t reg = AFIO->EXTICR[pin_num >> 2u];
    reg &= ~(0x0FuL << shift);
    reg |=  (port_idx << shift);
    AFIO->EXTICR[pin_num >> 2u] = reg;
#else
    /* F4: the same routing lives in SYSCFG. */
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    uint32_t reg = SYSCFG->EXTICR[pin_num >> 2u];
    reg &= ~(0x0FuL << shift);
    reg |=  (port_idx << shift);
    SYSCFG->EXTICR[pin_num >> 2u] = reg;
#endif

    EXTI->RTSR |=  (uint32_t)F_CS_Pin;   /* CS deasserted */
    EXTI->FTSR &= ~(uint32_t)F_CS_Pin;   /* assert edge not needed */
    EXTI->PR    =  (uint32_t)F_CS_Pin;   /* discard any edge latched at boot */
    EXTI->IMR  |=  (uint32_t)F_CS_Pin;

    IRQn_Type irq;
    if      (pin_num <= 4u) { irq = (IRQn_Type)(EXTI0_IRQn + pin_num); }
    else if (pin_num <= 9u) { irq = EXTI9_5_IRQn; }
    else                    { irq = EXTI15_10_IRQn; }

    /* Priority 5 = configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY at CubeMX's
     * FreeRTOS defaults. Anything more urgent may not call osThreadFlagsSet. */
    HAL_NVIC_SetPriority(irq, 5, 0);
    HAL_NVIC_EnableIRQ(irq);
}


void F10Slave_Init(SPI_HandleTypeDef *hspi)
{
    s_hspi   = hspi;
    s_tx_seq = 0u;
    s_active = 0u;

    (void)arm_next_transfer();
}


void F10Slave_Service(void) {
	/* wait on a received frame or if there is an SPI error */
	const uint32_t flags = osThreadFlagsWait(LINK_FLAG_FRAME_RX | LINK_FLAG_SPI_ERROR, osFlagsWaitAny, F10SLAVE_IDLE_MS);
	g_hb_link++;

	/* The H7 never clocked the END reply out: install anyway. */
	if (s_reboot_stage == 1u && (uint32_t)(HAL_GetTick() - s_reboot_armed_ms) > 2000u) {
		f1_ota_reboot(&g_hb_link);
	}

	if ((flags & osFlagsError) != 0u) {
        /* Timeout (the H7 is idle) or a flags error. The error codes have
         * their high bits set, so they must never be tested as flags.
         * If the last arm failed nothing is waiting for the H7 and no
         * callback will ever come: arm again. */
        if (!s_armed) {
            (void)HAL_SPI_Abort(s_hspi);
            (void)arm_next_transfer();
        }
        return;
    }

	if (flags & LINK_FLAG_SPI_ERROR) {
        /* Abort tears down a half-finished transfer so the peripheral is not
         * left busy. Without it, the re-arm below fails and the link stops
         * dead after the first glitch. */
        dbg_s_spi_errors++;
        (void)HAL_SPI_Abort(s_hspi);

        (void)arm_next_transfer();
        return;
    }

	if (flags & LINK_FLAG_FRAME_RX) {
		/* The frame just clocked out was the reply to OTA END: install. */
		if (s_reboot_stage == 1u) {
			s_reboot_stage = 2u;
			f1_ota_reboot(&g_hb_link);
		}
		process_rx_frame();
		(void)arm_next_transfer();
	}
}

/* ==========================================================================
 * HAL callbacks and the CS interrupt
 *
 * Delete any CubeMX-generated stubs for these in main.c.
 * ========================================================================== */

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi != s_hspi) { return; }

    dbg_s_xfers++;
    s_armed = false;

    /* No re-arm here on purpose: the DMA would start overwriting
     * s_rx_frame[s_active] while LinkTask is still reading it. LinkTask
     * re-arms once it has processed the frame, and the master's >= 50 us
     * CS-high gap is what covers that window. */
    osThreadFlagsSet(LinkTaskHandle, LINK_FLAG_FRAME_RX);
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi != s_hspi) {
        return;
    }

    /* Counting and recovery happen in LinkTask - HAL_SPI_Abort() must not
     * run from the ISR that reported the error. */
    s_armed = false;
    osThreadFlagsSet(LinkTaskHandle, LINK_FLAG_SPI_ERROR);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	dbg_s_cs_edges++;
    if (GPIO_Pin != F_CS_Pin) { return; } // return if wrong pin was passed into function.
    if (s_hspi == NULL)       { return; } // CS edge before F10Slave_Init(): nothing armed yet
    if (!s_armed)             { return; } // DMA idle: its counter says nothing about this edge

    if (HAL_GPIO_ReadPin(F_CS_GPIO_Port, F_CS_Pin) == GPIO_PIN_RESET) {
        return;   /* CS is low again: that edge was a glitch, not a deselect */
    }

    const uint32_t rem = __HAL_DMA_GET_COUNTER(s_hspi->hdmarx);
    /* rem == 0                    -> completed, TxRxCplt owns it.
     * rem == F10COMM_FRAME_SIZE   -> CS pulsed, nothing clocked. Still armed.
     * otherwise                   -> truncated. */

    if ((rem != 0u) && (rem != F10COMM_FRAME_SIZE)) {
        dbg_s_truncated++;
        osThreadFlagsSet(LinkTaskHandle, LINK_FLAG_SPI_ERROR);
    }
}

#if defined(STM32F1)
/* On the real board F_CS is PB12 (SPI2_NSS), EXTI line 12. CubeMX generates
 * no handler for it because the pin is an SPI pin, not a GPIO_EXTI pin.
 * If CubeMX ever generates EXTI15_10_IRQHandler (another EXTI pin on lines
 * 10-15), delete this one and add HAL_GPIO_EXTI_IRQHandler(F_CS_Pin) to the
 * generated handler in stm32f1xx_it.c instead. */
void EXTI15_10_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(F_CS_Pin);
}
#endif
