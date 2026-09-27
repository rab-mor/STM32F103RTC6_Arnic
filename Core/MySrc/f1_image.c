/**
 * @file  f1_image.c
 * @brief f1_crc32() for the application side. See f1_image.h.
 */

#include "f1_image.h"

/* Half-byte table: 64 bytes instead of 1 KB. */
static const uint32_t k_crc_nibble[16] = {
    0x00000000u, 0x1DB71064u, 0x3B6E20C8u, 0x26D930ACu,
    0x76DC4190u, 0x6B6B51F4u, 0x4DB26158u, 0x5005713Cu,
    0xEDB88320u, 0xF00F9344u, 0xD6D6A3E8u, 0xCB61B38Cu,
    0x9B64C2B0u, 0x86D3D2D4u, 0xA00AE278u, 0xBDBDF21Cu,
};

uint32_t f1_crc32(uint32_t crc, const void *data, uint32_t len)
{
    const uint8_t *p = (const uint8_t *)data;
    crc = ~crc;
    for (uint32_t i = 0u; i < len; i++) {
        crc ^= p[i];
        crc = (crc >> 4) ^ k_crc_nibble[crc & 0x0Fu];
        crc = (crc >> 4) ^ k_crc_nibble[crc & 0x0Fu];
    }
    return ~crc;
}
