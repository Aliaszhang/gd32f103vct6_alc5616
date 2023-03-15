#ifndef __SPI_DRIVER_H__
#define __SPI_DRIVER_H__

#include "gd32f10x.h"

void i2s_dma_config(uint16_t *i2s1_send_array, uint32_t send_size);
int i2s1_init(void);
#endif

