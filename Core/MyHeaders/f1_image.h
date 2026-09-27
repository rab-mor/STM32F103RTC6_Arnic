#ifndef F1_IMAGE_H
#define F1_IMAGE_H
/**
 * @file  f1_image.h
 * @brief Flash layout and update image format of the two F103 boards (main
 *        board F103RCT6, expansion module F103VCT6; both 256 KB flash, 2 KB
 *        pages). Shared verbatim by both F103 projects and the H7 (Common/),
 *        and built by tools/make_ota.py. No HAL, no RTOS, no project includes.
 *
 *   address      size     contents
 *   0x08000000    16 KB   bootloader (f1_boot.c, linked into the same ELF as
 *                         the application, so one flash writes both)
 *   0x08004000   118 KB   application (vector table first)
 *   0x08021800   118 KB   staging: an update is written here while the
 *                         application keeps running
 *   0x0803F000     4 KB   metadata: which image is staged, and whether the
 *                         bootloader has installed it yet
 *
 * Update flow
 *   1. The H7 streams the image; f1_ota.c writes it to staging.
 *   2. f1_ota_end() checks the CRC over staging, writes the metadata record
 *      with installed = 0xFFFF (pending) and the board resets itself.
 *   3. The bootloader sees a pending record whose CRC matches staging, copies
 *      staging over the application, checks the copy, marks the record
 *      installed (0x0000) and starts the application.
 *   Losing power during 3 only means 3 runs again at the next power-up: the
 *   record stays pending until the copy is verified, and staging is untouched.
 *   A missing or broken application with an installed record whose image is
 *   still intact in staging is copied again the same way.
 *
 * Relays do not move during any of this: they are latching, the coil pins
 * reset low, and the application restores its record without pulsing.
 */

#include <stdint.h>
#include <stdbool.h>

#define F1_FLASH_BASE        0x08000000u
#define F1_PAGE_SIZE         0x800u          /* 2 KB */

#define F1_BOOT_BASE         0x08000000u
#define F1_BOOT_SIZE         0x00004000u     /* 16 KB  */
#define F1_APP_BASE          0x08004000u
#define F1_APP_MAX           0x0001D800u     /* 118 KB */
#define F1_STAGE_BASE        0x08021800u
#define F1_STAGE_MAX         F1_APP_MAX
#define F1_META_BASE         0x0803F000u
#define F1_META_SIZE         0x00001000u

#define F1_RAM_BASE          0x20000000u
#define F1_RAM_END           0x2000C000u     /* 48 KB on both parts */

_Static_assert(F1_APP_BASE + F1_APP_MAX == F1_STAGE_BASE, "app and staging must touch");
_Static_assert(F1_STAGE_BASE + F1_STAGE_MAX == F1_META_BASE, "staging and metadata must touch");
_Static_assert(F1_META_BASE + F1_META_SIZE == 0x08040000u, "layout must end at 256 KB");

/* Board types: the same numbers as rs485_info_t.board_type. */
#define F1_BOARD_EXPANSION   1u
#define F1_BOARD_MAIN        2u

/* ---------------------------------------------------------------------------
 * Update package: f1_image_hdr_t, then `size` bytes of application image
 * (what the linker put at F1_APP_BASE onwards). The H7 reads the header and
 * sends its fields to the board in the OTA BEGIN message; the image bytes
 * follow in DATA messages, offset 0 = F1_APP_BASE.
 * ------------------------------------------------------------------------- */
#define F1_IMAGE_MAGIC       0x4D493146u     /* 'F1IM' */
#define F1_IMAGE_HDR_VERSION 1u

typedef struct {
    uint32_t magic;             /* F1_IMAGE_MAGIC                          */
    uint16_t hdr_version;       /* F1_IMAGE_HDR_VERSION                    */
    uint8_t  board_type;        /* F1_BOARD_*                              */
    uint8_t  flags;             /* 0                                       */
    uint32_t fw_version;        /* major << 16 | minor << 8 | patch        */
    uint32_t size;              /* image bytes, <= F1_APP_MAX              */
    uint32_t crc32;             /* CRC-32 (zlib) of the image              */
    uint32_t build_unix;
    uint32_t reserved;
    uint32_t hdr_crc32;         /* CRC-32 of the 28 bytes before it        */
} f1_image_hdr_t;

_Static_assert(sizeof(f1_image_hdr_t) == 32u, "f1_image_hdr_t is a wire format");

/* ---------------------------------------------------------------------------
 * Metadata record at F1_META_BASE. Written by the application (f1_ota.c)
 * after staging checks out; `installed` is the only field the bootloader
 * writes (0xFFFF -> 0x0000, which flash allows without an erase).
 * ------------------------------------------------------------------------- */
#define F1_META_MAGIC        0x444D3146u     /* 'F1MD' */
#define F1_META_PENDING      0xFFFFu
#define F1_META_INSTALLED    0x0000u

typedef struct {
    uint32_t magic;             /* F1_META_MAGIC                           */
    uint32_t size;              /* staged image bytes                      */
    uint32_t crc32;             /* of the staged image                     */
    uint32_t fw_version;
    uint32_t board_type;
    uint32_t rec_crc32;         /* CRC-32 of the 20 bytes before it        */
    uint16_t installed;         /* F1_META_PENDING / F1_META_INSTALLED     */
    uint16_t _pad;              /* left erased                             */
} f1_meta_t;

_Static_assert(sizeof(f1_meta_t) == 28u, "f1_meta_t is a flash format");

/* CRC-32 as zlib / Python's binascii.crc32. Pass crc = 0 to start and feed
   the result back to continue. f1_image.c; the bootloader has its own copy. */
uint32_t f1_crc32(uint32_t crc, const void *data, uint32_t len);

/* A vector table the application could start from. */
static inline bool f1_app_vectors_ok(uint32_t base)
{
    const uint32_t sp = *(const volatile uint32_t *)base;
    const uint32_t pc = *(const volatile uint32_t *)(base + 4u);
    return (sp > F1_RAM_BASE) && (sp <= F1_RAM_END) && ((sp & 3u) == 0u) &&
           ((pc & 1u) != 0u) && (pc > base) && (pc < base + F1_APP_MAX);
}

#endif /* F1_IMAGE_H */
