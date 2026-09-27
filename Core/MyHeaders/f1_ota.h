#ifndef F1_OTA_H
#define F1_OTA_H
/**
 * @file  f1_ota.h
 * @brief Receives an application update into the staging area (f1_image.h)
 *        while the application keeps running. Shared verbatim by both F103
 *        projects; the transport (SPI link on the main board, RS485 on the
 *        expansion module) only moves bytes and calls these.
 *
 *   begin  size, CRC, version and board type from the package header
 *   write  image bytes in order; offset must equal f1_ota_next() (a repeat
 *          of bytes already written is accepted and ignored)
 *   end    CRC over staging, write the metadata record, then the transport
 *          sends its reply and calls f1_ota_reboot(); the bootloader installs
 *   abort  drop the session (staging is left as it is; nothing points at it)
 *
 * Every call returns F1OTA_*; with F1OTA_ERR_OFFSET the reply's next offset
 * tells the H7 where to resume.
 */

#include <stdint.h>
#include <stdbool.h>

#define F1OTA_OK            0u
#define F1OTA_ERR_STATE     1u      /* no session, or already ended            */
#define F1OTA_ERR_PARAM     2u      /* size / length out of range              */
#define F1OTA_ERR_OFFSET    3u      /* not the next byte: resume from next     */
#define F1OTA_ERR_FLASH     4u      /* erase or program failed                 */
#define F1OTA_ERR_CRC       5u      /* staging does not match the header's CRC */
#define F1OTA_ERR_BOARD     6u      /* the image is for the other board type   */

#define F1OTA_ST_IDLE       0u
#define F1OTA_ST_RECEIVING  1u
#define F1OTA_ST_DONE       2u      /* staged; resetting into the bootloader   */
#define F1OTA_ST_FAILED     3u

typedef struct {
    uint32_t size;
    uint32_t crc32;
    uint32_t fw_version;
    uint8_t  board_type;            /* F1_BOARD_*                              */
} f1_ota_begin_t;

uint8_t  f1_ota_begin(const f1_ota_begin_t *b);
uint8_t  f1_ota_write(uint32_t offset, const uint8_t *data, uint32_t len);
uint8_t  f1_ota_end(void);
void     f1_ota_abort(void);

uint32_t f1_ota_next(void);         /* next image byte wanted                  */
uint8_t  f1_ota_state(void);        /* F1OTA_ST_*                              */

/* After f1_ota_end() returned OK and the reply is on its way: waits (up to
   2 s) for relay pulses to finish and the relay record to be saved, then
   resets into the bootloader. Does not return. */
void     f1_ota_reboot(void);

#endif /* F1_OTA_H */
