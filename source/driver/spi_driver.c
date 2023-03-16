#include "spi_driver.h"
#include "gd32f10x_dma.h"
#include "gd32f10x_spi.h"
#include "gd32f10x_gpio.h"


void spi_gpio_config(void)
{
    gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_4);  // spi0_cs

    /* configure I2S1 GPIO: I2S1_WS/PB12, I2S1_CK/PB13, I2S_SD/PB15, I2S_MCLK/PC6 */
    gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_15);
    gpio_init(GPIOC, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_6);

    gpio_pin_remap_config(GPIO_SWJ_SWDPENABLE_REMAP, ENABLE);
    gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_15); // spi2_cs
    
    GPIO_BOP(GPIOA) = GPIO_PIN_4; // cs_high
    GPIO_BOP(GPIOA) = GPIO_PIN_15;
   
    /* config SPI0 GPIO: SCK/PA5, MISO/PA6, MOSI/PA7 */
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_5 | GPIO_PIN_7);
    gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_6);
    
    // gpio_pin_remap_config(GPIO_SPI2_REMAP, ENABLE);
    gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_3 | GPIO_PIN_5);
    gpio_init(GPIOB, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_4);
}

void spi_dma_config(uint32_t spi_periph, uint32_t addr, uint32_t size)
{
    dma_parameter_struct dma_init_struct;
    dma_struct_para_init(&dma_init_struct);
    
    dma_deinit(DMA0, DMA_CH1);

    dma_init_struct.periph_addr  = (uint32_t)&SPI_DATA(spi_periph);
    dma_init_struct.memory_addr  = (uint32_t)addr;
    dma_init_struct.direction    = DMA_PERIPHERAL_TO_MEMORY;
    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_16BIT;
    dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_16BIT;
    dma_init_struct.priority     = DMA_PRIORITY_HIGH;
    dma_init_struct.number       = size;
    dma_init_struct.periph_inc   = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.memory_inc   = DMA_PERIPH_INCREASE_DISABLE;

    dma_init(DMA0, DMA_CH1, &dma_init_struct);
    /* configure DMA mode */
    dma_circulation_disable(DMA0, DMA_CH1);
    dma_memory_to_memory_disable(DMA0, DMA_CH1);
    
    dma_channel_enable(DMA0, DMA_CH4);
    spi_dma_enable(spi_periph, SPI_DMA_RECEIVE);
}

void i2s_dma_config(uint32_t i2s1_send_array, uint32_t send_size)
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
void spi02_i2s1_config(void)
{
    spi_parameter_struct spi_init_struct;
    /* deinitilize SPI and the parameters */
    spi_i2s_deinit(SPI0);
    spi_i2s_deinit(SPI1);
    spi_i2s_deinit(SPI2);

    spi_struct_para_init(&spi_init_struct);

    /* configure SPI0 parameter */
    spi_init_struct.trans_mode           = SPI_TRANSMODE_FULLDUPLEX;
    spi_init_struct.device_mode          = SPI_MASTER;
    spi_init_struct.frame_size           = SPI_FRAMESIZE_8BIT;
    spi_init_struct.clock_polarity_phase = SPI_CK_PL_LOW_PH_1EDGE;
    spi_init_struct.nss                  = SPI_NSS_SOFT;
    spi_init_struct.prescale             = SPI_PSC_8;
    spi_init_struct.endian               = SPI_ENDIAN_MSB;
    spi_init(SPI0, &spi_init_struct);

    /* configure SPI2 parameter */
    spi_init_struct.trans_mode           = SPI_TRANSMODE_FULLDUPLEX;
    spi_init_struct.clock_polarity_phase = SPI_CK_PL_LOW_PH_1EDGE;
    spi_init_struct.nss                  = SPI_NSS_SOFT;

    spi_init(SPI2, &spi_init_struct);

    /* configure I2S1 */
    i2s_init(SPI1, I2S_MODE_MASTERTX, I2S_STD_PHILLIPS, I2S_CKPL_LOW);
    i2s_psc_config(SPI1, I2S_AUDIOSAMPLE_8K, I2S_FRAMEFORMAT_DT16B_CH16B, I2S_MCKOUT_ENABLE);
}


int spi_i2s_init(void)
{
    /* configure GPIO */
    spi_gpio_config();
    /* configure DMA */
    // i2s_dma_config();
    /* configure SPI */
    spi02_i2s1_config();

    /* enable SPI */
    // spi_enable(SPI2);
    // i2s_enable(SPI1);
    spi_enable(SPI0);

    return 0;
}

uint8_t spi_send_byte(uint32_t spi_periph, uint8_t byte)
{
    /* loop while data register in not empty */
    while(RESET == spi_i2s_flag_get(spi_periph, SPI_FLAG_TBE));

    /* send byte through the SPI5 peripheral */
    spi_i2s_data_transmit(spi_periph, byte);

    /* wait to receive a byte */
    while(RESET == spi_i2s_flag_get(spi_periph, SPI_FLAG_RBNE));

    /* return the byte read from the SPI bus */
    return(spi_i2s_data_receive(spi_periph));
}
/*-------------------------------------------------------------------------------*/



