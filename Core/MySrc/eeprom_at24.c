/**
 * eeprom_at24.c - F103
 *
 * See eeprom_at24.h. Blocking HAL I2C calls with short timeouts; the write
 * cycle is waited out with osDelay so the calling task yields.
 *
 * Bus recovery: an I2C slave that was mid-byte when the F103 reset can hold
 * SDA low forever, and the F1 I2C peripheral can latch BUSY after a glitch.
 * On any failed transfer nine clocks are sent on SCL by hand, which releases
 * a stuck slave, and the peripheral is initialised again.
 */

#include "eeprom_at24.h"
#include "cmsis_os2.h"
#include <string.h>

#define AT24_BASE_ADDR      0xA0u   /* 0x50 << 1 */
#define AT24_XFER_TIMEOUT   25u     /* ms: 36 bytes at 100 kHz is ~3.3 ms */
#define AT24_READY_TRIES    4u      /* write cycle polls after the first wait */

static void delay_us(uint32_t us)
{
    /* ~72 cycles per us at 72 MHz; the loop is 4-6 cycles. Rough is fine:
       it only paces a bit-banged clock that the slave accepts at any speed. */
    volatile uint32_t n = us * 12u;
    while (n-- > 0u) { __NOP(); }
}

/* Reset the peripheral and clock the bus free. Uses the EEPROM_SCL/SDA pin
   labels both boards share in CubeMX. */
static void bus_recover(at24_t *e)
{
    I2C_HandleTypeDef *h = e->hi2c;
    e->recoveries++;

    (void)HAL_I2C_DeInit(h);                 /* pins go back to inputs */

    GPIO_InitTypeDef g = {0};
    g.Pin   = EEPROM_SCL_Pin;
    g.Mode  = GPIO_MODE_OUTPUT_OD;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(EEPROM_SCL_GPIO_Port, &g);

    g.Pin  = EEPROM_SDA_Pin;
    g.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init(EEPROM_SDA_GPIO_Port, &g);

    for (uint8_t i = 0u; i < 9u; i++) {
        HAL_GPIO_WritePin(EEPROM_SCL_GPIO_Port, EEPROM_SCL_Pin, GPIO_PIN_RESET);
        delay_us(6u);
        HAL_GPIO_WritePin(EEPROM_SCL_GPIO_Port, EEPROM_SCL_Pin, GPIO_PIN_SET);
        delay_us(6u);
        if (HAL_GPIO_ReadPin(EEPROM_SDA_GPIO_Port, EEPROM_SDA_Pin) == GPIO_PIN_SET) {
            break;                           /* slave let go of SDA */
        }
    }

    /* HAL_I2C_Init() pulses CR1.SWRST, which clears a latched BUSY (F1
       errata), and MspInit puts the pins back on their I2C function. */
    (void)HAL_I2C_Init(h);
}

static bool ready(at24_t *e, uint32_t trials)
{
    return HAL_I2C_IsDeviceReady(e->hi2c, e->dev, trials, 2u) == HAL_OK;
}

bool at24_probe(at24_t *e, I2C_HandleTypeDef *hi2c)
{
    e->hi2c       = hi2c;
    e->dev        = 0u;
    e->errors     = 0u;
    e->recoveries = 0u;

    for (uint8_t attempt = 0u; attempt < 2u; attempt++) {
        for (uint16_t a = 0u; a < 8u; a++) {
            const uint16_t dev = (uint16_t)(AT24_BASE_ADDR + (a << 1));
            if (HAL_I2C_IsDeviceReady(hi2c, dev, 2u, 2u) == HAL_OK) {
                e->dev = dev;
                return true;
            }
        }
        bus_recover(e);                      /* maybe the bus was stuck */
    }
    return false;
}

bool at24_read(at24_t *e, uint16_t addr, void *dst, uint16_t len)
{
    if ((e->dev == 0u) || (len == 0u) || ((uint32_t)addr + len > AT24_SIZE_BYTES)) {
        return false;
    }
    for (uint8_t attempt = 0u; attempt < 2u; attempt++) {
        if (HAL_I2C_Mem_Read(e->hi2c, e->dev, addr, I2C_MEMADD_SIZE_16BIT,
                             (uint8_t *)dst, len, AT24_XFER_TIMEOUT) == HAL_OK) {
            return true;
        }
        e->errors++;
        bus_recover(e);
    }
    return false;
}

bool at24_write_page(at24_t *e, uint16_t addr, const void *src, uint16_t len)
{
    if ((e->dev == 0u) || (len == 0u) || (len > AT24_PAGE_BYTES) ||
        ((addr / AT24_PAGE_BYTES) != ((addr + len - 1u) / AT24_PAGE_BYTES)) ||
        ((uint32_t)addr + len > AT24_SIZE_BYTES)) {
        return false;
    }

    uint8_t buf[AT24_PAGE_BYTES];            /* HAL wants a non-const pointer */
    memcpy(buf, src, len);

    for (uint8_t attempt = 0u; attempt < 2u; attempt++) {
        if (HAL_I2C_Mem_Write(e->hi2c, e->dev, addr, I2C_MEMADD_SIZE_16BIT,
                              buf, len, AT24_XFER_TIMEOUT) == HAL_OK) {
            /* The chip ignores its address until the write cycle is over. */
            osDelay(AT24_WRITE_MS + 1u);
            for (uint8_t t = 0u; t < AT24_READY_TRIES; t++) {
                if (ready(e, 1u)) {
                    return true;
                }
                osDelay(2u);
            }
            e->errors++;
            return false;
        }
        e->errors++;
        bus_recover(e);
    }
    return false;
}
