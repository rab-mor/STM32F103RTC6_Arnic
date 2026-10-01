/**
 * relay_store.c - F103
 *
 * See relay_store.h. Record layout, 16 bytes at the start of each slot page:
 *
 *   off size field
 *    0   2   magic    'RS' (0x5352)
 *    2   1   version  1
 *    3   1   count    relays this board has (8 or 12); a mismatch is ignored
 *    4   4   seq      +1 per save; the highest valid one is the newest
 *    8   2   state    bit n = relay n on
 *   10   2   known    bit n = relay n's position was established
 *   12   2   reserved 0
 *   14   2   crc16    CCITT-FALSE over bytes 0..13
 *
 * An erased chip reads 0xFF everywhere, which fails the magic check.
 */

#include "relay_store.h"
#include "eeprom_at24.h"
#include <string.h>

#define RSTORE_MAGIC        0x5352u
#define RSTORE_VERSION      1u
#define RSTORE_REC_BYTES    16u

_Static_assert((RSTORE_BASE % AT24_PAGE_BYTES) == 0u, "slots must start on a page");
_Static_assert(RSTORE_BASE + RSTORE_SLOTS * AT24_PAGE_BYTES <= AT24_SIZE_BYTES, "slots overrun the chip");

static at24_t          s_ee;
static rstore_stats_t  s_stats;
static uint8_t         s_count;
static uint8_t         s_next_slot;
static uint32_t        s_next_seq = 1u;

static uint16_t crc16(const uint8_t *d, uint32_t n)
{
    uint16_t crc = 0xFFFFu;
    for (uint32_t i = 0u; i < n; i++) {
        crc ^= (uint16_t)d[i] << 8;
        for (uint8_t b = 0u; b < 8u; b++) {
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static inline uint16_t get16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static inline uint32_t get32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static inline void put16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static inline void put32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}

static uint16_t slot_addr(uint8_t slot)
{
    return (uint16_t)(RSTORE_BASE + (uint32_t)slot * AT24_PAGE_BYTES);
}

static bool record_valid(const uint8_t *r, uint8_t count)
{
    return (get16(&r[0]) == RSTORE_MAGIC) &&
           (r[2] == RSTORE_VERSION) &&
           (r[3] == count) &&
           (get16(&r[14]) == crc16(r, 14u));
}

rstore_load_t rstore_load(I2C_HandleTypeDef *hi2c, uint8_t relay_count,
                          uint16_t *state, uint16_t *known)
{
    *state = 0u;
    *known = 0u;
    s_count = relay_count;
    s_next_slot = 0u;
    s_next_seq  = 1u;

    s_stats.present = at24_probe(&s_ee, hi2c);
    if (!s_stats.present) {
        return RSTORE_NO_CHIP;
    }

    bool     found = false;
    uint32_t best_seq = 0u;
    uint8_t  best_slot = 0u;
    uint8_t  best[RSTORE_REC_BYTES];

    for (uint8_t slot = 0u; slot < RSTORE_SLOTS; slot++) {
        uint8_t r[RSTORE_REC_BYTES];
        if (!at24_read(&s_ee, slot_addr(slot), r, sizeof(r)) || !record_valid(r, relay_count)) {
            continue;
        }
        const uint32_t seq = get32(&r[4]);
        /* Signed difference: all live records are within RSTORE_SLOTS of each
           other, so this orders them correctly across a 32-bit wrap too. */
        if (!found || ((int32_t)(seq - best_seq) > 0)) {
            found     = true;
            best_seq  = seq;
            best_slot = slot;
            memcpy(best, r, sizeof(best));
        }
    }

    if (!found) {
        return RSTORE_EMPTY;
    }

    const uint16_t mask = (uint16_t)((1u << relay_count) - 1u);
    *state = get16(&best[8])  & mask;
    *known = get16(&best[10]) & mask;

    s_stats.loads_ok++;
    s_stats.seq  = best_seq;
    s_stats.slot = best_slot;
    s_next_slot  = (uint8_t)((best_slot + 1u) % RSTORE_SLOTS);
    s_next_seq   = best_seq + 1u;
    return RSTORE_LOADED;
}

bool rstore_save(uint16_t state, uint16_t known)
{
    if (!s_stats.present) {
        return false;
    }

    uint8_t r[RSTORE_REC_BYTES];
    memset(r, 0, sizeof(r));
    put16(&r[0], RSTORE_MAGIC);
    r[2] = RSTORE_VERSION;
    r[3] = s_count;
    put32(&r[4], s_next_seq);
    put16(&r[8], state);
    put16(&r[10], known);
    put16(&r[14], crc16(r, 14u));

    const uint8_t slot = s_next_slot;
    /* Whatever happens, the next save uses the following slot: a page that
       failed to write is simply skipped over. */
    s_next_slot = (uint8_t)((slot + 1u) % RSTORE_SLOTS);

    uint8_t back[RSTORE_REC_BYTES];
    if (at24_write_page(&s_ee, slot_addr(slot), r, sizeof(r)) &&
        at24_read(&s_ee, slot_addr(slot), back, sizeof(back)) &&
        (memcmp(r, back, sizeof(r)) == 0)) {
        s_stats.saves_ok++;
        s_stats.seq  = s_next_seq;
        s_stats.slot = slot;
        s_next_seq++;
        return true;
    }

    s_stats.save_errors++;
    return false;
}

void rstore_get_stats(rstore_stats_t *out)
{
    *out = s_stats;
}
