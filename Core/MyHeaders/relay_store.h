/**
 * relay_store.h - F103 header file
 *
 * The last known relay positions, kept in the AT24C64 so a reset or power
 * cut never needs a relay to be pulsed to find out where it is.
 *
 * Wear levelling: RSTORE_SLOTS records of 16 bytes, one per 32-byte page,
 * written round-robin with an increasing sequence number. The newest valid
 * record wins at boot. At 1 million cycles per page that is ~32 million
 * saves, far beyond the relays' own life.
 *
 * Shared verbatim by the main board (8 relays) and the expansion module
 * (12 relays): the record holds up to 16.
 */

#ifndef RELAY_STORE_H
#define RELAY_STORE_H

#include <stdint.h>
#include <stdbool.h>
#include "main.h"

#define RSTORE_BASE         0x0000u     /* first page used                */
#define RSTORE_SLOTS        32u         /* 1 KB of the 8 KB chip          */

typedef enum {
    RSTORE_NO_CHIP  = 0,                /* nothing answered on the bus    */
    RSTORE_EMPTY    = 1,                /* chip found, no valid record    */
    RSTORE_LOADED   = 2,                /* record found and returned      */
} rstore_load_t;

typedef struct {
    bool     present;                   /* EEPROM answered at boot        */
    uint32_t loads_ok;
    uint32_t saves_ok;
    uint32_t save_errors;
    uint32_t seq;                       /* sequence of the newest record  */
    uint8_t  slot;                      /* where it lives                 */
} rstore_stats_t;

/* Probe the EEPROM and read the newest record. On RSTORE_LOADED, *state and
   *known hold bit n = relay n (on / position established). Otherwise both
   are 0. Blocks for ~60 ms at 100 kHz. */
rstore_load_t rstore_load(I2C_HandleTypeDef *hi2c, uint8_t relay_count,
                          uint16_t *state, uint16_t *known);

/* Write a new record. Blocks for ~8 ms. Returns false if the chip is missing
   or the write/verify failed (the next save tries the following slot). */
bool rstore_save(uint16_t state, uint16_t known);

void rstore_get_stats(rstore_stats_t *out);

#endif /* RELAY_STORE_H */
