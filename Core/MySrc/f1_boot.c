/**
 * @file  f1_boot.c
 * @brief Bootloader for both F103 boards. Shared verbatim by both projects.
 *        Flow and layout: f1_image.h.
 *
 * It is compiled with the application but lives on its own: the linker
 * script puts the .boot.* sections in the first 16 KB (BOOT region), and the
 * application's vector table starts at 0x08004000. Flashing the project from
 * CubeIDE therefore writes both, and an update image (tools/make_ota.py)
 * contains only the application.
 *
 * Rules that keep it independent of the application it overwrites:
 *   - every function and constant here is in a .boot.* section (BOOT_FN /
 *     BOOT_RO), and nothing here calls anything outside this file;
 *   - no global or static variables: the application's startup code is what
 *     initialises .data and .bss, and it has not run;
 *   - no library calls (no memcpy, no 64-bit division, no float, no strings).
 * tools/check_boot.py checks the first rule on the built ELF.
 *
 * Runs on the 8 MHz HSI straight out of reset (flash writes need the HSI on
 * anyway) and toggles the MAX706 watchdog input (WDOG pin from main.h)
 * around every flash page, so a long copy never trips it.
 */

#include "main.h"       /* WDOG_Pin / WDOG_GPIO_Port, the CMSIS device header */
#include "f1_image.h"
#include "board_config.h"

#define BOOT_FN   __attribute__((section(".boot.text"), used, noinline))
#define BOOT_RO   __attribute__((section(".boot.rodata"), used))

#define BOOT_INSTALL_TRIES   3u

extern uint32_t _estack;                /* top of RAM, from the linker script */

void f1_boot_reset(void);
void f1_boot_fault(void);

/* Only the entries a Cortex-M3 can take before the application sets VTOR. */
__attribute__((section(".boot.vectors"), used))
const uint32_t g_f1_boot_vectors[8] = {
    (uint32_t)&_estack,
    (uint32_t)f1_boot_reset,
    (uint32_t)f1_boot_fault,            /* NMI         */
    (uint32_t)f1_boot_fault,            /* HardFault   */
    (uint32_t)f1_boot_fault,            /* MemManage   */
    (uint32_t)f1_boot_fault,            /* BusFault    */
    (uint32_t)f1_boot_fault,            /* UsageFault  */
    0u,
};

BOOT_RO static const uint32_t k_boot_crc_nibble[16] = {
    0x00000000u, 0x1DB71064u, 0x3B6E20C8u, 0x26D930ACu,
    0x76DC4190u, 0x6B6B51F4u, 0x4DB26158u, 0x5005713Cu,
    0xEDB88320u, 0xF00F9344u, 0xD6D6A3E8u, 0xCB61B38Cu,
    0x9B64C2B0u, 0x86D3D2D4u, 0xA00AE278u, 0xBDBDF21Cu,
};

/* ------------------------------ Watchdog --------------------------------- */

BOOT_FN static uint32_t pin_number(void)
{
    uint32_t n = 0u;
    while ((((uint32_t)WDOG_Pin >> n) & 1u) == 0u && n < 15u) {
        n++;
    }
    return n;
}

BOOT_FN static void wdog_init(void)
{
    /* GPIOA..GPIOG are 0x400 apart; their clock enables are APB2ENR bits 2..8. */
    const uint32_t port = ((uint32_t)WDOG_GPIO_Port - GPIOA_BASE) >> 10;
    RCC->APB2ENR |= (1u << (2u + port));
    (void)RCC->APB2ENR;

    const uint32_t pin   = pin_number();
    volatile uint32_t *cr = (pin < 8u) ? &WDOG_GPIO_Port->CRL : &WDOG_GPIO_Port->CRH;
    const uint32_t shift = (pin & 7u) * 4u;
    *cr = (*cr & ~(0xFu << shift)) | (0x2u << shift);   /* push-pull output, 2 MHz */
}

BOOT_FN static void wdog_kick(void)
{
    WDOG_GPIO_Port->ODR ^= (uint32_t)WDOG_Pin;
}

/* -------------------------------- CRC ------------------------------------ */

BOOT_FN static uint32_t boot_crc32(const volatile uint8_t *p, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFFu;
    for (uint32_t i = 0u; i < len; i++) {
        crc ^= p[i];
        crc = (crc >> 4) ^ k_boot_crc_nibble[crc & 0x0Fu];
        crc = (crc >> 4) ^ k_boot_crc_nibble[crc & 0x0Fu];
        if ((i & 0x3FFFu) == 0u) {
            wdog_kick();
        }
    }
    return ~crc;
}

/* -------------------------------- Flash ---------------------------------- */

BOOT_FN static bool flash_wait(void)
{
    while ((FLASH->SR & FLASH_SR_BSY) != 0u) {
    }
    const uint32_t sr = FLASH->SR;
    FLASH->SR = FLASH_SR_PGERR | FLASH_SR_WRPRTERR | FLASH_SR_EOP;
    return (sr & (FLASH_SR_PGERR | FLASH_SR_WRPRTERR)) == 0u;
}

BOOT_FN static void flash_unlock(void)
{
    if ((FLASH->CR & FLASH_CR_LOCK) != 0u) {
        FLASH->KEYR = 0x45670123u;
        FLASH->KEYR = 0xCDEF89ABu;
    }
}

BOOT_FN static bool flash_erase(uint32_t addr)
{
    (void)flash_wait();
    FLASH->CR |= FLASH_CR_PER;
    FLASH->AR  = addr;
    FLASH->CR |= FLASH_CR_STRT;
    const bool ok = flash_wait();
    FLASH->CR &= ~FLASH_CR_PER;
    return ok;
}

BOOT_FN static bool flash_half(uint32_t addr, uint16_t v)
{
    volatile uint16_t *dst = (volatile uint16_t *)addr;
    FLASH->CR |= FLASH_CR_PG;
    *dst = v;
    const bool ok = flash_wait();
    FLASH->CR &= ~FLASH_CR_PG;
    return ok && (*dst == v);
}

/* -------------------------------- Install -------------------------------- */

/* Copy `size` staged bytes over the application, page by page, and check the
   copy against the record's CRC. */
BOOT_FN static bool install(uint32_t size, uint32_t crc)
{
    const volatile uint8_t *src = (const volatile uint8_t *)F1_STAGE_BASE;
    bool ok = true;

    flash_unlock();
    for (uint32_t off = 0u; ok && off < size; off += F1_PAGE_SIZE) {
        wdog_kick();
        ok = flash_erase(F1_APP_BASE + off);
        wdog_kick();
        const uint32_t end = ((size - off) < F1_PAGE_SIZE) ? size : (off + F1_PAGE_SIZE);
        for (uint32_t i = off; ok && i < end; i += 2u) {
            const uint16_t hi = (i + 1u < size) ? (uint16_t)src[i + 1u] : 0xFFu;
            ok = flash_half(F1_APP_BASE + i, (uint16_t)(src[i] | (hi << 8)));
        }
    }
    FLASH->CR |= FLASH_CR_LOCK;

    return ok && (boot_crc32((const volatile uint8_t *)F1_APP_BASE, size) == crc);
}

BOOT_FN static bool app_vectors_ok(void)
{
    const uint32_t sp = *(const volatile uint32_t *)F1_APP_BASE;
    const uint32_t pc = *(const volatile uint32_t *)(F1_APP_BASE + 4u);
    return (sp > F1_RAM_BASE) && (sp <= F1_RAM_END) && ((sp & 3u) == 0u) &&
           ((pc & 1u) != 0u) && (pc > F1_APP_BASE) && (pc < F1_APP_BASE + F1_APP_MAX);
}

BOOT_FN static void start_app(void)
{
    const uint32_t sp = *(const volatile uint32_t *)F1_APP_BASE;
    const uint32_t pc = *(const volatile uint32_t *)(F1_APP_BASE + 4u);

    SCB->VTOR = F1_APP_BASE;
    __asm volatile ("dsb\n isb\n msr msp, %0\n bx %1" : : "r"(sp), "r"(pc) : "memory");
    for (;;) {
    }
}

BOOT_FN void f1_boot_reset(void)
{
    wdog_init();
    wdog_kick();

    const volatile f1_meta_t *m = (const volatile f1_meta_t *)F1_META_BASE;
    const uint32_t size = m->size;
    const uint32_t crc  = m->crc32;

    /* A record the application wrote for this board, for an image that is
       still intact in staging. */
    bool staged = (m->magic == F1_META_MAGIC) &&
                  (size != 0u) && (size <= F1_APP_MAX) &&
                  (m->board_type == F1_THIS_BOARD) &&
                  (boot_crc32((const volatile uint8_t *)F1_META_BASE, 20u) == m->rec_crc32);
    if (staged) {
        staged = (boot_crc32((const volatile uint8_t *)F1_STAGE_BASE, size) == crc);
    }

    /* Install a pending update, or reinstall the last one over a broken
       application. */
    const bool pending = staged && (m->installed == F1_META_PENDING);
    if (pending || (staged && !app_vectors_ok())) {
        for (uint32_t t = 0u; t < BOOT_INSTALL_TRIES; t++) {
            if (install(size, crc)) {
                if (pending) {
                    flash_unlock();
                    (void)flash_half(F1_META_BASE + 24u, F1_META_INSTALLED);
                    FLASH->CR |= FLASH_CR_LOCK;
                }
                break;
            }
        }
    }

    if (app_vectors_ok()) {
        start_app();
    }

    /* No application to run: stay here, watchdog fed, ready for SWD. */
    for (;;) {
        for (volatile uint32_t i = 0u; i < 100000u; i++) {
        }
        wdog_kick();
    }
}

/* NVIC_SystemReset() is not force-inlined in every CMSIS version, and at -O0
   a copy of it would land in the application's .text: write AIRCR here. */
BOOT_FN void f1_boot_fault(void)
{
    __asm volatile ("dsb" : : : "memory");
    SCB->AIRCR = (0x5FAu << SCB_AIRCR_VECTKEY_Pos) | (SCB->AIRCR & SCB_AIRCR_PRIGROUP_Msk) |
                 SCB_AIRCR_SYSRESETREQ_Msk;
    __asm volatile ("dsb" : : : "memory");
    for (;;) {
    }
}
