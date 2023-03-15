#include "spi_driver.h"
#include "gd32f10x_dma.h"
#include "gd32f10x_spi.h"
#include "gd32f10x_gpio.h"


void spi_gpio_config(void)
{
    /* configure I2S1 GPIO: I2S1_WS/PB12, I2S1_CK/PB13, I2S_SD/PB15, I2S_MCLK/PC6 */
    gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_15);
    gpio_init(GPIOC, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_6);
}

void i2s_dma_config(uint16_t *i2s1_send_array, uint32_t send_size)
{
    dma_parameter_struct dma_init_struct;
    dma_struct_para_init(&dma_init_struct);

    /* configure I2S1 transmit DMA: DMA0, DMA_CH4 */
    dma_deinit(DMA0, DMA_CH4);

    dma_init_struct.periph_addr  = (uint32_t)&SPI_DATA(SPI1);
    dma_init_struct.memory_addr  = (uint32_t)i2s1_send_array;
    dma_init_struct.direction    = DMA_MEMORY_TO_PERIPHERAL;
    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_16BIT;
    dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_16BIT;
    dma_init_struct.priority     = DMA_PRIORITY_ULTRA_HIGH;
    dma_init_struct.number       = send_size;
    dma_init_struct.periph_inc   = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.memory_inc   = DMA_MEMORY_INCREASE_ENABLE;
    dma_init(DMA0, DMA_CH4, &dma_init_struct);
    /* configure DMA mode */
    dma_circulation_disable(DMA0, DMA_CH4);
    dma_memory_to_memory_disable(DMA0, DMA_CH4);
    
    spi_dma_enable(SPI1, SPI_DMA_TRANSMIT);
    dma_channel_enable(DMA0, DMA_CH4);
}

/*!
    \brief      configure the SPI peripheral
    \param[in]  none
    \param[out] none
    \retval     none
*/
void i2s1_config(void)
{
    spi_i2s_deinit(SPI1);

    /* configure I2S1 */
    i2s_init(SPI1, I2S_MODE_MASTERTX, I2S_STD_PHILLIPS, I2S_CKPL_LOW);
    i2s_psc_config(SPI1, I2S_AUDIOSAMPLE_8K, I2S_FRAMEFORMAT_DT16B_CH16B, I2S_MCKOUT_ENABLE);
}


int i2s1_init(void)
{
    /* configure GPIO */
    spi_gpio_config();
    /* configure DMA */
    // i2s_dma_config();
    /* configure SPI */
    i2s1_config();

    /* enable I2S1 */
    // i2s_enable(SPI1);

    return 0;
}

/*-------------------------------------------------------------------------------*/



