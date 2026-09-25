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


static void build_tx_frame(uint8_t idx)
{
    F10Comm_Frame_t *f = &s_tx_frame[idx];

    /* Note sizeof(*f), not sizeof(s_tx_frame): the latter is the whole
     * two-element array and would run off the end when idx == 1. */
    memset(f, 0, sizeof(*f));

    f->magic    = F10COMM_MAGIC;
    f->version  = F10COMM_PROTO_VERSION;
    f->type     = (uint8_t)F10_TYPE_METER_DATA;
    f->seq      = s_tx_seq++;
    f->reserved = 0u;

    /* Running state (int32_t mA) -> this frame's wire view (int16_t mA). */
    const app_snapshot_t *s = &g_snap[g_snap_active];

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
            break;

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
