/**
 * eeprom_at24.h - F103 header file (main board and expansion module)
 *
 * Minimal blocking driver for the AT24C64 (8 KB, 32-byte pages, 16-bit
 * memory address, 5 ms write cycle). Shared verbatim by the main board and
 * the expansion module.
 *
 * Only one task may use a given at24_t: nothing here takes a lock.
 *
 * The chip's A0..A2 straps are not known from the firmware side, so
 * at24_probe() looks for it at all eight addresses (0x50..0x57) and keeps
 * the first that answers.
 */

#ifndef EEPROM_AT24_H
#define EEPROM_AT24_H

#include <stdint.h>
#include <stdbool.h>
#include "main.h"

#define AT24_SIZE_BYTES     8192u
#define AT24_PAGE_BYTES     32u
#define AT24_WRITE_MS       5u      /* datasheet maximum write cycle */

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint16_t           dev;         /* 8-bit HAL address, 0 = not found */
    uint32_t           errors;      /* failed transfers, for the debugger */
    uint32_t           recoveries;  /* bus recoveries performed          */
} at24_t;

/* Find the chip. Returns true if one answered. */
bool at24_probe(at24_t *e, I2C_HandleTypeDef *hi2c);

/* Read len bytes starting at addr. */
bool at24_read(at24_t *e, uint16_t addr, void *dst, uint16_t len);

/* Write len bytes (1..32) starting at addr, all inside one 32-byte page,
   then wait for the chip to finish its write cycle (up to ~6 ms, yielding). */
bool at24_write_page(at24_t *e, uint16_t addr, const void *src, uint16_t len);

#endif /* EEPROM_AT24_H */
