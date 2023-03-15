#include "gpio_driver.h"
#include "gd32f10x_gpio.h"

int board_gpio_init(void)
{
    // MIC_PWR_CTL
    gpio_init(GPIOE, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_2);
    // LED
    gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_0);
    
    gpio_bit_reset(GPIOE, GPIO_PIN_2); //alc5616 poweron

    return 0;
}

void gpio_toggle_pin(uint32_t gpio_periph, uint32_t pin)
{
    gpio_bit_write(gpio_periph, pin, (bit_status)(1 - gpio_input_bit_get(gpio_periph, pin)));
}


