#ifndef __GPIO_DRIVER_H__
#define __GPIO_DRIVER_H__

#include "gd32f10x.h"

void gpio_toggle_pin(uint32_t gpio_periph, uint32_t pin);
int board_gpio_init(void);

#endif

