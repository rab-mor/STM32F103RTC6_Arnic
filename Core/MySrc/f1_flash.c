/**
 * @file  f1_flash.c
 * @brief See f1_flash.h.
 */

#include "f1_flash.h"
#include "stm32f1xx.h"

#define FLASH_KEY_1     0x45670123u
#define FLASH_KEY_2     0xCDEF89ABu
#define SR_ERRORS       (FLASH_SR_PGERR | FLASH_SR_WRPRTERR)

static void unlock(void)
{
    if ((FLASH->CR & FLASH_CR_LOCK) != 0u) {
        FLASH->KEYR = FLASH_KEY_1;
        FLASH->KEYR = FLASH_KEY_2;
    }
}

static void lock(void)
{
    FLASH->CR |= FLASH_CR_LOCK;
}

/* The loop itself stalls on its fetch while the flash is busy, so it only
   spins for the last few cycles of an operation. */
static bool wait_done(void)
{
    while ((FLASH->SR & FLASH_SR_BSY) != 0u) {
    }
    const uint32_t sr = FLASH->SR;
    FLASH->SR = SR_ERRORS | FLASH_SR_EOP;          /* write-1-to-clear */
    return (sr & SR_ERRORS) == 0u;
}

bool f1_flash_erase_page(uint32_t addr)
{
    unlock();
    (void)wait_done();
    FLASH->CR |= FLASH_CR_PER;
    FLASH->AR  = addr;
    FLASH->CR |= FLASH_CR_STRT;
    const bool ok = wait_done();
    FLASH->CR &= ~FLASH_CR_PER;
    lock();

    /* An erased page reads all ones. */
    const volatile uint32_t *p = (const volatile uint32_t *)addr;
    for (uint32_t i = 0u; ok && i < (0x800u / 4u); i++) {
        if (p[i] != 0xFFFFFFFFu) {
            return false;
        }
    }
    return ok;
}

bool f1_flash_program(uint32_t addr, const void *data, uint32_t len)
{
    if ((addr & 1u) != 0u) {
        return false;
    }
    const uint8_t *src = (const uint8_t *)data;
    bool ok = true;

    unlock();
    (void)wait_done();
    for (uint32_t i = 0u; ok && i < len; i += 2u) {
        const uint16_t hw = (uint16_t)(src[i] | ((i + 1u < len) ? ((uint16_t)src[i + 1u] << 8) : 0xFF00u));
        volatile uint16_t *dst = (volatile uint16_t *)(addr + i);
        FLASH->CR |= FLASH_CR_PG;
        *dst = hw;
        ok = wait_done();
        FLASH->CR &= ~FLASH_CR_PG;
        if (ok && *dst != hw) {
            ok = false;
        }
    }
    lock();
    return ok;
}
