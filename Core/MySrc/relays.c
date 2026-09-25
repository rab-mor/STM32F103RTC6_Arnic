/**
 * relays.c - F103
 *
 * Non-blocking scheduler for eight dual-coil latching relays.
 *
 * Design notes, and why this differs from the H7 implementation it
 * replaces:
 *
 *  - The old relay_pulse_start() indexed active_pulses[] by relay but
 *    stored the coil pin in that entry.  A second command for the same
 *    relay inside the 80 ms window overwrote the entry with the other
 *    coil's pin, leaving the first coil energised with nothing tracking
 *    it.  It never went low.  Here the pulse record holds the relay's
 *    own state machine and a coil selector, so a mid-pulse request is
 *    recorded as a target and acted on only after the coil drops.
 *
 *  - Concurrency is capped at RELAY_MAX_CONCURRENT.  The old code would
 *    energise all eight coils at once given an eight-bit update mask.
 *
 *  - Shadow state is tracked, so redundant commands do not re-pulse and
 *    the link can report real positions.
 *
 *  - relays_init() does not block.  The old RELAYS_GPIO_Init() spent
 *    8 x 80 ms in HAL_Delay(), which would trip the external watchdog
 *    on this board.
 *
 *  - Only one coil per relay can ever be energised, structurally: each
 *    relay owns a single coil selector, not two independent pins.
 *
 *  - RELAY_DRIVE_COILS (board_config.h) = 0 on the bench: every pulse is
 *    scheduled, timed and tracked exactly as on the real board, but the
 *    coil pins are never written.  coil_write() is the only place a coil
 *    is driven.
 */

#include "relays.h"
#include "board_config.h"

#ifndef RELAY_DRIVE_COILS
#error "Add RELAY_DRIVE_COILS to board_config.h (0 = bench, 1 = real relays)"
#endif

typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
} gpio_pin_t;

/* Coil selector values. */
#define COIL_A                  0U      /* connector pin 1, "Reset" */
#define COIL_B                  1U      /* connector pin 2, "Set"   */

/* ======================================================================
 * Coil GPIO map
 *
 * IMPORTANT: this is NOT a transcription of the RLY*_A / RLY*_B net
 * names.  The two ULN2803s are wired differently.
 *
 * U14 (relays 1-4) is straight through: In -> On, and the net names
 * correspond.  Driving RLY1_A actuates RELAY1_A.
 *
 * U15 (relays 5-8) is not.  The ULN2803 pairs input pin n with output
 * pin 19-n, and on U15 the input and output net names do not correspond:
 *
 *     drive      pin      pin      actuates
 *     RLY5_B  ->  1   ->  18  ->   RELAY8_A
 *     RLY5_A  ->  2   ->  17  ->   RELAY8_B
 *     RLY6_B  ->  3   ->  16  ->   RELAY7_A
 *     RLY6_A  ->  4   ->  15  ->   RELAY7_B
 *     RLY7_B  ->  5   ->  14  ->   RELAY6_A
 *     RLY7_A  ->  6   ->  13  ->   RELAY6_B
 *     RLY8_B  ->  7   ->  12  ->   RELAY5_A
 *     RLY8_A  ->  8   ->  11  ->   RELAY5_B
 *
 * So relays 5-8 are both index-reversed and A/B swapped.  This is
 * confirmed as the true wiring, not a drafting error, so the mapping is
 * absorbed here rather than corrected in hardware.
 *
 * The tables below are indexed by PHYSICAL RELAY (K1..K8 => 0..7) and
 * hold the MCU pin that actuates that relay's coil.  Every entry carries
 * the net name and ULN path it came from so it can be checked line by
 * line against the schematic.
 * ==================================================================== */

/* Coil A - connector pin 1 - "Reset" per the schematic note. */
static const gpio_pin_t k_coil_a[RELAY_COUNT] = {
    { GPIOC, GPIO_PIN_2  },  /* K1 RELAY1_A <- RLY1_A  U14 I7->O7 (PC2)  */
    { GPIOC, GPIO_PIN_0  },  /* K2 RELAY2_A <- RLY2_A  U14 I5->O5 (PC0)  */
    { GPIOC, GPIO_PIN_14 },  /* K3 RELAY3_A <- RLY3_A  U14 I3->O3 (PC14) */
    { GPIOB, GPIO_PIN_9  },  /* K4 RELAY4_A <- RLY4_A  U14 I1->O1 (PB9)  */
    { GPIOB, GPIO_PIN_7  },  /* K5 RELAY5_A <- RLY8_B  U15 I7->O7 (PB7)  SWAP */
    { GPIOB, GPIO_PIN_5  },  /* K6 RELAY6_A <- RLY7_B  U15 I5->O5 (PB5)  SWAP */
    { GPIOB, GPIO_PIN_3  },  /* K7 RELAY7_A <- RLY6_B  U15 I3->O3 (PB3)  SWAP */
    { GPIOA, GPIO_PIN_15 },  /* K8 RELAY8_A <- RLY5_B  U15 I1->O1 (PA15) SWAP */
};

/* Coil B - connector pin 2 - "Set" per the schematic note. */
static const gpio_pin_t k_coil_b[RELAY_COUNT] = {
    { GPIOC, GPIO_PIN_3  },  /* K1 RELAY1_B <- RLY1_B  U14 I8->O8 (PC3)  */
    { GPIOC, GPIO_PIN_1  },  /* K2 RELAY2_B <- RLY2_B  U14 I6->O6 (PC1)  */
    { GPIOC, GPIO_PIN_15 },  /* K3 RELAY3_B <- RLY3_B  U14 I4->O4 (PC15) */
    { GPIOC, GPIO_PIN_13 },  /* K4 RELAY4_B <- RLY4_B  U14 I2->O2 (PC13) */
    { GPIOB, GPIO_PIN_8  },  /* K5 RELAY5_B <- RLY8_A  U15 I8->O8 (PB8)  SWAP */
    { GPIOB, GPIO_PIN_6  },  /* K6 RELAY6_B <- RLY7_A  U15 I6->O6 (PB6)  SWAP */
    { GPIOB, GPIO_PIN_4  },  /* K7 RELAY7_B <- RLY6_A  U15 I4->O4 (PB4)  SWAP */
    { GPIOD, GPIO_PIN_2  },  /* K8 RELAY8_B <- RLY5_A  U15 I2->O2 (PD2)  SWAP */
};

/*
 * PA15, PB3 and PB4 appear above.  They are JTAG pins at reset and only
 * respond as GPIO once __HAL_AFIO_REMAP_SWJ_NOJTAG() has run.  CubeMX
 * normally emits that call into MX_GPIO_Init(); if it is missing, relays
 * 7 and 8 will silently do nothing.
 *
 * PC13/PC14/PC15 are supplied through the backup domain and are limited
 * to ~3 mA and 2 MHz.  Sourcing into a ULN2803 input is ~0.7 mA, which
 * is comfortable, but their GPIO speed must stay Low.
 */
static volatile uint32_t s_activity = 0U;

/* ====================================================================== */

typedef struct {
    uint8_t  target;        /* requested state, 1 = ON                   */
    uint8_t  known;         /* last established position                 */
    uint8_t  known_valid;   /* 0 until a pulse has completed             */
    uint8_t  pulsing;       /* a coil is energised right now             */
    uint8_t  coil;          /* which coil, COIL_A or COIL_B              */
    uint32_t t_start;       /* tick at which the coil went high          */
    uint32_t t_ready;       /* earliest tick a new pulse may start       */
} relay_ctx_t;

static relay_ctx_t s_ctx[RELAY_COUNT];

/* ---------------------------------------------------------------------- */

static const gpio_pin_t *coil_pin(uint8_t idx, uint8_t coil)
{
    return (coil == COIL_B) ? &k_coil_b[idx] : &k_coil_a[idx];
}

/**
 * Which coil drives the requested user-visible state.
 *
 * Normally ON is the Set coil (B) and OFF is the Reset coil (A).  For a
 * relay flagged in RELAY_REVERSED_MASK the roles swap.  This is the only
 * place the inversion lives - do not add a second one at the call site,
 * which is how the H7 code ended up with two cancelling errors.
 */
static uint8_t coil_for_state(uint8_t idx, uint8_t state)
{
    uint8_t on_coil = ((RELAY_REVERSED_MASK >> idx) & 1U) ? COIL_A : COIL_B;

    if (state != 0U) {
        return on_coil;
    }
    return (on_coil == COIL_B) ? COIL_A : COIL_B;
}

/** The position a relay latches into after a pulse on this coil. */
static uint8_t state_for_coil(uint8_t idx, uint8_t coil)
{
    return (coil == coil_for_state(idx, 1U)) ? 1U : 0U;
}

static void coil_write(uint8_t idx, uint8_t coil, GPIO_PinState level)
{
    const gpio_pin_t *p = coil_pin(idx, coil);
#if RELAY_DRIVE_COILS
    HAL_GPIO_WritePin(p->port, p->pin, level);
#else
    /* Bench build: the pulse is scheduled and timed, but no pin is driven. */
    (void)p;
    (void)level;
#endif
}

/* ---------------------------------------------------------------------- */

void relays_init(void)
{
    uint32_t now = HAL_GetTick();

    /* Every coil low before anything else.  MX_GPIO_Init() should already
     * have left them low, but a reset that does not power-cycle the board
     * can leave a coil energised from before. */
    for (uint8_t i = 0U; i < RELAY_COUNT; i++) {
        coil_write(i, COIL_A, GPIO_PIN_RESET);
        coil_write(i, COIL_B, GPIO_PIN_RESET);

        s_ctx[i].target      = 0U;   /* queue everything to OFF */
        s_ctx[i].known       = 0U;
        s_ctx[i].known_valid = 0U;   /* position genuinely unknown at boot */
        s_ctx[i].pulsing     = 0U;
        s_ctx[i].coil        = COIL_A;
        s_ctx[i].t_start     = now;
        s_ctx[i].t_ready     = now;
    }

    /* No pulsing here.  known_valid == 0 with target == 0 makes the
     * scheduler drive all eight to OFF, staggered, from relays_tick(). */
}

void relays_tick(void)
{
    uint32_t now    = HAL_GetTick();
    uint8_t  active = 0U;

    /* Phase 1: drop coils whose pulse has expired. */
    for (uint8_t i = 0U; i < RELAY_COUNT; i++) {
        if (s_ctx[i].pulsing == 0U) {
            continue;
        }

        if ((uint32_t)(now - s_ctx[i].t_start) >= RELAY_PULSE_MS) {
            coil_write(i, s_ctx[i].coil, GPIO_PIN_RESET);
            s_activity++;
            s_ctx[i].pulsing     = 0U;
            /* The relay latched where THIS coil sends it.  Not the target:
             * a request that arrived mid-pulse may have changed that, and
             * it still needs a pulse of its own. */
            s_ctx[i].known       = state_for_coil(i, s_ctx[i].coil);
            s_ctx[i].known_valid = 1U;
            s_ctx[i].t_ready     = now + RELAY_INTERPULSE_MS;
        }
    }

    /* Phase 2: count what is still energised. */
    for (uint8_t i = 0U; i < RELAY_COUNT; i++) {
        if (s_ctx[i].pulsing != 0U) {
            active++;
        }
    }

    /* Phase 3: start pulses for relays that need one, up to the cap. */
    for (uint8_t i = 0U; i < RELAY_COUNT; i++) {
        if (active >= RELAY_MAX_CONCURRENT) {
            break;
        }
        if (s_ctx[i].pulsing != 0U) {
            continue;
        }
        /* Already where it should be, and we know that for a fact. */
        if (s_ctx[i].known_valid != 0U && s_ctx[i].known == s_ctx[i].target) {
            continue;
        }
        /* Signed compare so the tick wrap at 49.7 days is handled. */
        if ((int32_t)(now - s_ctx[i].t_ready) < 0) {
            continue;
        }

        s_ctx[i].coil    = coil_for_state(i, s_ctx[i].target);
        s_ctx[i].t_start = now;
        s_ctx[i].pulsing = 1U;
        coil_write(i, s_ctx[i].coil, GPIO_PIN_SET);
        s_activity++;
        active++;
    }
}



uint32_t relays_activity(void) {
	return s_activity;
}


/* ---------------------------------------------------------------------- */

int relays_request(uint8_t idx, uint8_t state)
{
    if (idx >= RELAY_COUNT) {
        return RELAYS_ERR_INDEX;
    }
    if (state > 1U) {
        return RELAYS_ERR_STATE;
    }

    /* Record only.  If a pulse is in flight it finishes first; the
     * scheduler picks this up on the pass after that. */
    s_ctx[idx].target = state;
    return RELAYS_OK;
}

uint32_t relays_apply_mask(uint32_t update_mask, uint32_t state_mask)
{
    uint32_t errors = 0U;

    for (uint8_t i = 0U; i < RELAY_COUNT; i++) {
        if ((update_mask & (1UL << i)) == 0UL) {
            continue;
        }
        uint8_t state = ((state_mask & (1UL << i)) != 0UL) ? 1U : 0U;
        if (relays_request(i, state) != RELAYS_OK) {
            errors++;
        }
    }
    return errors;
}


uint8_t relays_get_state(uint8_t idx)
{
    return (idx < RELAY_COUNT) ? s_ctx[idx].known : 0U;
}


uint8_t relays_state_known(uint8_t idx)
{
    return (idx < RELAY_COUNT) ? s_ctx[idx].known_valid : 0U;
}


uint8_t relays_settled(uint8_t idx)
{
    if (idx >= RELAY_COUNT) {
        return 0U;
    }
    const relay_ctx_t *c = &s_ctx[idx];
    return (c->pulsing == 0U && c->known_valid != 0U && c->known == c->target) ? 1U : 0U;
}


uint32_t relays_get_state_mask(void)
{
    uint32_t m = 0U;
    for (uint8_t i = 0U; i < RELAY_COUNT; i++) {
        if (s_ctx[i].known != 0U) {
            m |= (1UL << i);
        }
    }
    return m;
}


uint32_t relays_known_mask(void)
{
    uint32_t m = 0U;
    for (uint8_t i = 0U; i < RELAY_COUNT; i++) {
        if (s_ctx[i].known_valid != 0U) {
            m |= (1UL << i);
        }
    }
    return m;
}

uint8_t relays_busy(void)
{
    for (uint8_t i = 0U; i < RELAY_COUNT; i++) {
        if (s_ctx[i].pulsing != 0U) {
            return 1U;
        }
        if (s_ctx[i].known_valid == 0U || s_ctx[i].known != s_ctx[i].target) {
            return 1U;
        }
    }
    return 0U;
}
