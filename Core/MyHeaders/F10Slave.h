// F10Slave.h - F103 header file
#ifndef F10SLAVE_H
#define F10SLAVE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"          /* SPI_HandleTypeDef, F_CS_Pin */
#include "app_shared.h"    /* link_event_t; pulls in link_proto.h */

/* Enable a rising-edge EXTI on the SPI NSS pin without disturbing its
 * alternate-function config. Call before F10Slave_Init(). */
void F10Slave_CsExtiInit(void);

/* Initialise the slave and arm the first transfer. Call after MX_SPIx_Init()
 * and after the RTOS queues exist, i.e. from inside LinkTask. */
void F10Slave_Init(SPI_HandleTypeDef *hspi);

/* Blocks on the thread flags set by the SPI ISRs, processes the completed
 * frame and re-arms the DMA. Call in a loop from LinkTask. */
void F10Slave_Service(void);

void link_post_event(const link_event_t *ev);

#ifdef __cplusplus
}
#endif

#endif /* F10SLAVE_H */
