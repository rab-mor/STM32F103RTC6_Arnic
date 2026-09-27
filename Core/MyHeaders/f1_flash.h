#ifndef F1_FLASH_H
#define F1_FLASH_H
/**
 * @file  f1_flash.h
 * @brief Erase and program the F103's own flash, register level. Shared
 *        verbatim by both F103 projects. Used only by the update path
 *        (f1_ota.c), which never writes the region the code runs from.
 *
 * The F103 has one flash bank: while a page erase (20-40 ms) or a half-word
 * program (~50 us) runs, the CPU stalls on its next instruction fetch,
 * interrupts included. The DMA streams (ADC, SPI, UART) keep going, but a
 * half-buffer that lands during an erase is processed late.
 */

#include <stdint.h>
#include <stdbool.h>

bool f1_flash_erase_page(uint32_t addr);

/* addr must be even. An odd len is padded with 0xFF. Reads every half-word
   back. False on a flash error or a mismatch. */
bool f1_flash_program(uint32_t addr, const void *data, uint32_t len);

#endif /* F1_FLASH_H */
