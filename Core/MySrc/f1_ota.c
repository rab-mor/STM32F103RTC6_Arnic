/**
 * @file  f1_ota.c
 * @brief See f1_ota.h and f1_image.h.
 */

#include "f1_ota.h"
#include "f1_image.h"
#include "f1_flash.h"
#include "board_config.h"
#include "app_shared.h"
#include "relays.h"
#include "cmsis_os2.h"

typedef struct {
    uint8_t  state;
    uint32_t size;
    uint32_t crc32;
    uint32_t fw_version;
    uint8_t  board_type;
    uint32_t next;                  /* image bytes written so far    */
    uint32_t erased_to;             /* staging erased below this address */
} session_t;

static session_t s_ota;

/* Diagnostics for Live Expressions. */
volatile uint32_t dbg_ota_sessions;
volatile uint32_t dbg_ota_last_err;

static uint8_t fail(uint8_t err)
{
    dbg_ota_last_err = err;
    if (err == F1OTA_ERR_FLASH || err == F1OTA_ERR_CRC) {
        s_ota.state = F1OTA_ST_FAILED;
    }
    return err;
}

uint8_t f1_ota_begin(const f1_ota_begin_t *b)
{
    if (b->board_type != F1_THIS_BOARD) {
        return fail(F1OTA_ERR_BOARD);
    }
    if (b->size == 0u || b->size > F1_STAGE_MAX) {
        return fail(F1OTA_ERR_PARAM);
    }

    /* The same session again (its first reply was lost): carry on. */
    if (s_ota.state == F1OTA_ST_RECEIVING && s_ota.size == b->size &&
        s_ota.crc32 == b->crc32 && s_ota.next == 0u) {
        return F1OTA_OK;
    }

    /* Nothing may point at staging while it is being rewritten. */
    if (!f1_flash_erase_page(F1_META_BASE)) {
        return fail(F1OTA_ERR_FLASH);
    }

    s_ota.state      = F1OTA_ST_RECEIVING;
    s_ota.size       = b->size;
    s_ota.crc32      = b->crc32;
    s_ota.fw_version = b->fw_version;
    s_ota.board_type = b->board_type;
    s_ota.next       = 0u;
    s_ota.erased_to  = F1_STAGE_BASE;
    dbg_ota_sessions++;
    return F1OTA_OK;
}

uint8_t f1_ota_write(uint32_t offset, const uint8_t *data, uint32_t len)
{
    if (s_ota.state != F1OTA_ST_RECEIVING) {
        return fail(F1OTA_ERR_STATE);
    }
    if (len == 0u || offset + len > s_ota.size) {
        return fail(F1OTA_ERR_PARAM);
    }
    if (offset + len <= s_ota.next) {
        return F1OTA_OK;                /* a repeat: already written */
    }
    if (offset != s_ota.next) {
        return fail(F1OTA_ERR_OFFSET);
    }
    /* Every chunk but the last must be even: flash takes half-words. */
    if ((len & 1u) != 0u && offset + len != s_ota.size) {
        return fail(F1OTA_ERR_PARAM);
    }

    const uint32_t addr = F1_STAGE_BASE + offset;
    while (s_ota.erased_to < addr + len) {           /* one page at a time */
        if (!f1_flash_erase_page(s_ota.erased_to)) {
            return fail(F1OTA_ERR_FLASH);
        }
        s_ota.erased_to += F1_PAGE_SIZE;
    }
    if (!f1_flash_program(addr, data, len)) {
        return fail(F1OTA_ERR_FLASH);
    }
    s_ota.next = offset + len;
    return F1OTA_OK;
}

uint8_t f1_ota_end(void)
{
    if (s_ota.state == F1OTA_ST_DONE) {
        return F1OTA_OK;                /* a repeat of an END already done */
    }
    if (s_ota.state != F1OTA_ST_RECEIVING || s_ota.next != s_ota.size) {
        return fail(F1OTA_ERR_STATE);
    }
    if (f1_crc32(0u, (const void *)F1_STAGE_BASE, s_ota.size) != s_ota.crc32) {
        return fail(F1OTA_ERR_CRC);
    }

    f1_meta_t m;
    m.magic      = F1_META_MAGIC;
    m.size       = s_ota.size;
    m.crc32      = s_ota.crc32;
    m.fw_version = s_ota.fw_version;
    m.board_type = s_ota.board_type;
    m.rec_crc32  = f1_crc32(0u, &m, 20u);
    m.installed  = F1_META_PENDING;
    m._pad       = 0xFFFFu;

    /* installed and _pad stay erased: the bootloader writes installed. */
    if (!f1_flash_program(F1_META_BASE, &m, 24u)) {
        return fail(F1OTA_ERR_FLASH);
    }
    s_ota.state = F1OTA_ST_DONE;
    return F1OTA_OK;
}

void f1_ota_abort(void)
{
    if (s_ota.state != F1OTA_ST_DONE) {
        s_ota.state = F1OTA_ST_IDLE;
    }
}

uint32_t f1_ota_next(void)  { return s_ota.next; }
uint8_t  f1_ota_state(void) { return s_ota.state; }

void f1_ota_reboot(void)
{
    for (uint32_t t = 0u; t < 2000u; t += 10u) {
        const bool settled = (relays_busy() == 0u) &&
                             ((g_relay_truth.store_flags & STORE_F_PENDING) == 0u);
        if (settled) {
            break;
        }
        osDelay(10u);
    }
    osDelay(20u);                       /* the reply's last bytes, the EEPROM write */
    NVIC_SystemReset();
}
