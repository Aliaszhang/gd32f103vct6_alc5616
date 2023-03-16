#include "gd32f10x.h"
#include "gpio_driver.h"
#include "i2c_driver.h"
#include "fmc_operation.h"
#include "spi_driver.h"
#include "alc5616.h"
#include <stdio.h>
#include <string.h>
#include "systick.h"

static int uart_init(void)
{
    // rcu_periph_clock_enable(RCU_GPIOA);
    // rcu_periph_clock_enable(RCU_USART0);
 
    /* connect port to USARTx_Tx */
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);

    /* connect port to USARTx_Rx */
    gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_10);
    
    usart_deinit(USART0);
    usart_baudrate_set(USART0, 115200U);
    usart_word_length_set(USART0, USART_WL_8BIT);
    usart_stop_bit_set(USART0, USART_STB_1BIT);
    usart_parity_config(USART0, USART_PM_NONE);
    usart_hardware_flow_rts_config(USART0, USART_RTS_DISABLE);
    usart_hardware_flow_cts_config(USART0, USART_CTS_DISABLE);
    usart_receive_config(USART0, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);
    usart_enable(USART0);
    
    return 0;
}

int main(void)
{
    systick_config();
    rcu_periph_clock_enable(RCU_AF);
    rcu_periph_clock_enable(RCU_DMA0);
    rcu_periph_clock_enable(RCU_DMA1);
    rcu_periph_clock_enable(RCU_PMU);
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_GPIOD);
    rcu_periph_clock_enable(RCU_GPIOE);
    rcu_periph_clock_enable(RCU_USART0);
    rcu_periph_clock_enable(RCU_I2C0);   
    rcu_periph_clock_enable(RCU_SPI1);

    board_gpio_init();
    uart_init();
    i2c_config();
    spi_i2s_init();
    alc5616_init();
    
    printf("CK_SYS is %d\r\n", rcu_clock_freq_get(CK_SYS));
    printf("CK_AHB is %d\r\n", rcu_clock_freq_get(CK_AHB));
    printf("CK_APB1 is %d\r\n", rcu_clock_freq_get(CK_APB1));
    printf("CK_APB2 is %d\r\n", rcu_clock_freq_get(CK_APB2));

    
    while (1)
    {
        gpio_toggle_pin(GPIOB, GPIO_PIN_0);
        i2s_transfer_dma(AUDIO_1);
        delay_1ms(1000);

        gpio_toggle_pin(GPIOB, GPIO_PIN_0);
        i2s_transfer_dma(AUDIO_2);
        delay_1ms(1000);

        gpio_toggle_pin(GPIOB, GPIO_PIN_0);
        i2s_transfer_dma(AUDIO_3);
        delay_1ms(1000);

        gpio_toggle_pin(GPIOB, GPIO_PIN_0);
        i2s_transfer_dma(AUDIO_4);
        delay_1ms(1000);
    }
    
}



int fputc(int ch, FILE *f)
{
    usart_data_transmit(USART0, (uint8_t)ch);
    while(RESET == usart_flag_get(USART0, USART_FLAG_TBE));
    return ch;
}



