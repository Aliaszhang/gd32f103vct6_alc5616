#include "i2c_driver.h"
#include "gd32f10x_i2c.h"
#include "gd32f10x_dma.h"
#include "gd32f10x_gpio.h"
#include "stdio.h"


/*!
    \brief      configure the I2CX interfaces
    \param[in]  none
    \param[out] none
    \retval     0
*/
int i2c_config(void)
{
    /* connect I2C_SCL_PIN to I2C_SCL */
    /* connect I2C_SDA_PIN to I2C_SDA */

    gpio_pin_remap_config(GPIO_I2C0_REMAP, ENABLE);
    gpio_init(I2C_SCL_PORT, GPIO_MODE_AF_OD, GPIO_OSPEED_50MHZ, I2C_SCL_PIN);
    gpio_init(I2C_SDA_PORT, GPIO_MODE_AF_OD, GPIO_OSPEED_50MHZ, I2C_SDA_PIN);
    
    /* configure I2C clock */
    i2c_clock_config(I2CX, I2C_SPEED, I2C_DTCY_2);
    /* configure I2C address */
    i2c_mode_addr_config(I2CX, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS, 0);
    /* enable I2CX */
    i2c_enable(I2CX);
    /* enable acknowledge */
    i2c_ack_config(I2CX, I2C_ACK_ENABLE);

    return 0;
}

/*!
    \brief      reset i2c bus
    \param[in]  none
    \param[out] none
    \retval     none
*/
void i2c_bus_reset(void)
{
    i2c_deinit(I2CX);
    /* configure SDA/SCL for GPIO */
    GPIO_BC(I2C_SCL_PORT) |= I2C_SCL_PIN;
    GPIO_BC(I2C_SDA_PORT) |= I2C_SDA_PIN;
    gpio_init(I2C_SCL_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, I2C_SCL_PIN);
    gpio_init(I2C_SDA_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, I2C_SDA_PIN);
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    GPIO_BOP(I2C_SCL_PORT) |= I2C_SCL_PIN;
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    GPIO_BOP(I2C_SDA_PORT) |= I2C_SDA_PIN;
    /* configure the I2CX interface */
    i2c_config();
}

uint8_t i2c_bytes_write_timeout(uint32_t i2cx, uint32_t slave_addr, uint8_t *p_buffer, uint8_t write_address, uint8_t number_of_byte)
{
    uint8_t   state = I2C_START;
    uint16_t  timeout = 0;
    uint8_t   i2c_timeout_flag = 0;

    /* enable acknowledge */
    i2c_ack_config(i2cx, I2C_ACK_ENABLE);
    while(!(i2c_timeout_flag)) 
    {
        switch(state) 
        {
        case I2C_START:
            /* i2c master sends start signal only when the bus is idle */
            while(i2c_flag_get(i2cx, I2C_FLAG_I2CBSY) && (timeout < I2C_TIME_OUT)) 
            {
                timeout++;
            }
            if(timeout < I2C_TIME_OUT) 
            {
                i2c_start_on_bus(i2cx);
                timeout = 0;
                state = I2C_SEND_ADDRESS;
            } 
            else 
            {
                i2c_bus_reset();
                timeout = 0;
                state = I2C_START;
                printf("i2c bus is busy in WRITE!\n");
            }
            break;
        case I2C_SEND_ADDRESS:
            /* i2c master sends START signal successfully */
            while((! i2c_flag_get(i2cx, I2C_FLAG_SBSEND)) && (timeout < I2C_TIME_OUT)) 
            {
                timeout++;
            }
            if(timeout < I2C_TIME_OUT) 
            {
                i2c_master_addressing(i2cx, slave_addr, I2C_TRANSMITTER);
                timeout = 0;
                state = I2C_CLEAR_ADDRESS_FLAG;
            } 
            else 
            {
                timeout = 0;
                state = I2C_START;
                printf("i2c master sends start signal timeout in WRITE!\n");
            }
            break;
        case I2C_CLEAR_ADDRESS_FLAG:
            /* address flag set means i2c slave sends ACK */
            while((! i2c_flag_get(i2cx, I2C_FLAG_ADDSEND)) && (timeout < I2C_TIME_OUT)) 
            {
                timeout++;
            }
            if(timeout < I2C_TIME_OUT) 
            {
                i2c_flag_clear(i2cx, I2C_FLAG_ADDSEND);
                timeout = 0;
                state = I2C_TRANSMIT_DATA;
            } 
            else 
            {
                timeout = 0;
                state = I2C_START;
                printf("i2c master clears address flag timeout in WRITE!\n");
            }
            break;
        case I2C_TRANSMIT_DATA:
            /* wait until the transmit data buffer is empty */
            while((! i2c_flag_get(i2cx, I2C_FLAG_TBE)) && (timeout < I2C_TIME_OUT)) 
            {
                timeout++;
            }
            if(timeout < I2C_TIME_OUT) 
            {
                /* send register address, only one byte */
                i2c_data_transmit(i2cx, write_address);
                timeout = 0;
            } 
            else 
            {
                timeout = 0;
                state = I2C_START;
                printf("i2c master sends EEPROM's internal address timeout in WRITE!\n");
            }
            /* wait until BTC bit is set */
            while((!i2c_flag_get(i2cx, I2C_FLAG_BTC)) && (timeout < I2C_TIME_OUT)) {
                timeout++;
            }
            if(timeout < I2C_TIME_OUT) {
                timeout = 0;
            } else {
                timeout = 0;
                state = I2C_START;
                printf("i2c master sends data timeout in WRITE!\n");
            }

            while(number_of_byte--) {
                i2c_data_transmit(i2cx, *p_buffer);
                /* point to the next byte to be written */
                p_buffer++;
                /* wait until BTC bit is set */
                while((!i2c_flag_get(i2cx, I2C_FLAG_BTC)) && (timeout < I2C_TIME_OUT)) {
                    timeout++;
                }
                if(timeout < I2C_TIME_OUT) {
                    timeout = 0;
                } else {
                    timeout = 0;
                    state = I2C_START;
                    printf("i2c master sends data timeout in WRITE!\n");
                }
            }
            timeout = 0;
            state = I2C_STOP;
            break;
        case I2C_STOP:
            /* send a stop condition to I2C bus */
            i2c_stop_on_bus(i2cx);
            /* i2c master sends STOP signal successfully */
            while((I2C_CTL0(i2cx) & I2C_CTL0_STOP) && (timeout < I2C_TIME_OUT)) {
                timeout++;
            }
            if(timeout < I2C_TIME_OUT) {
                timeout = 0;
                state = I2C_END;
                i2c_timeout_flag = I2C_OK;
            } else {
                timeout = 0;
                state = I2C_START;
                printf("i2c master sends stop signal timeout in WRITE!\n");
            }
            break;
        default:
            state = I2C_START;
            i2c_timeout_flag = I2C_OK;
            timeout = 0;
            printf("i2c master sends start signal in WRITE.\n");
            break;
        }
    }
    return I2C_END;
}

uint8_t i2c_bytes_read_timeout(uint32_t i2cx, uint32_t slave_addr, uint8_t *p_buffer, uint8_t read_address, uint8_t number_of_byte)
{
    uint8_t   state = I2C_START;
    uint8_t   read_cycle = 0;
    uint16_t  timeout = 0;
    uint8_t   i2c_timeout_flag = 0;

    /* enable acknowledge */
    i2c_ack_config(i2cx, I2C_ACK_ENABLE);
    while(!(i2c_timeout_flag)) 
    {
        switch(state) 
        {
        case I2C_START:
            if(RESET == read_cycle) 
            {
                /* i2c master sends start signal only when the bus is idle */
                while(i2c_flag_get(i2cx, I2C_FLAG_I2CBSY) && (timeout < I2C_TIME_OUT)) 
                {
                    timeout++;
                }
                if(timeout < I2C_TIME_OUT) 
                {
                    /* whether to send ACK or not for the next byte */
                    if(2 == number_of_byte) 
                    {
                        i2c_ackpos_config(i2cx, I2C_ACKPOS_NEXT);
                    }
                } 
                else 
                {
                    i2c_bus_reset();
                    timeout = 0;
                    state = I2C_START;
                    printf("i2c bus is busy in READ!\n");
                }
            }
            /* send the start signal */
            i2c_start_on_bus(i2cx);
            timeout = 0;
            state = I2C_SEND_ADDRESS;
            break;
        case I2C_SEND_ADDRESS:
            /* i2c master sends START signal successfully */
            while((! i2c_flag_get(i2cx, I2C_FLAG_SBSEND)) && (timeout < I2C_TIME_OUT)) 
            {
                timeout++;
            }
            if(timeout < I2C_TIME_OUT) 
            {
                if(RESET == read_cycle) 
                {
                    i2c_master_addressing(i2cx, slave_addr, I2C_TRANSMITTER);
                    state = I2C_CLEAR_ADDRESS_FLAG;
                } 
                else 
                {
                    i2c_master_addressing(i2cx, slave_addr, I2C_RECEIVER);
                    if(number_of_byte < 3) 
                    {
                        /* disable acknowledge */
                        i2c_ack_config(i2cx, I2C_ACK_DISABLE);
                    }
                    state = I2C_CLEAR_ADDRESS_FLAG;
                }
                timeout = 0;
            } 
            else 
            {
                timeout = 0;
                state = I2C_START;
                read_cycle = 0;
                printf("i2c master sends start signal timeout in READ!\n");
            }
            break;
        case I2C_CLEAR_ADDRESS_FLAG:
            /* address flag set means i2c slave sends ACK */
            while((!i2c_flag_get(i2cx, I2C_FLAG_ADDSEND)) && (timeout < I2C_TIME_OUT)) 
            {
                timeout++;
            }
            if(timeout < I2C_TIME_OUT) 
            {
                i2c_flag_clear(i2cx, I2C_FLAG_ADDSEND);
                if((SET == read_cycle) && (1 == number_of_byte)) 
                {
                    /* send a stop condition to I2C bus */
                    i2c_stop_on_bus(i2cx);
                }
                timeout = 0;
                state   = I2C_TRANSMIT_DATA;
            } 
            else 
            {
                timeout = 0;
                state   = I2C_START;
                read_cycle = 0;
                printf("i2c master clears address flag timeout in READ!\n");
            }
            break;
        case I2C_TRANSMIT_DATA:
            if(RESET == read_cycle) 
            {
                /* wait until the transmit data buffer is empty */
                while((! i2c_flag_get(i2cx, I2C_FLAG_TBE)) && (timeout < I2C_TIME_OUT)) 
                {
                    timeout++;
                }
                if(timeout < I2C_TIME_OUT) 
                {
                    /* send the EEPROM's internal address to write to : only one byte address */
                    i2c_data_transmit(i2cx, read_address);
                    timeout = 0;
                } 
                else 
                {
                    timeout = 0;
                    state = I2C_START;
                    read_cycle = 0;
                    printf("i2c master wait data buffer is empty timeout in READ!\n");
                }
                /* wait until BTC bit is set */
                while((!i2c_flag_get(i2cx, I2C_FLAG_BTC)) && (timeout < I2C_TIME_OUT)) 
                {
                    timeout++;
                }
                if(timeout < I2C_TIME_OUT) 
                {
                    timeout = 0;
                    state = I2C_START;
                    read_cycle++;
                } else 
                {
                    timeout = 0;
                    state = I2C_START;
                    read_cycle = 0;
                    printf("i2c master sends EEPROM's internal address timeout in READ!\n");
                }
            } 
            else 
            {
                while(number_of_byte) 
                {
                    timeout++;
                    if(3 == number_of_byte) 
                    {
                        /* wait until BTC bit is set */
                        while(!i2c_flag_get(i2cx, I2C_FLAG_BTC));
                        /* disable acknowledge */
                        i2c_ack_config(i2cx, I2C_ACK_DISABLE);
                    }
                    if(2 == number_of_byte) 
                    {
                        /* wait until BTC bit is set */
                        while(!i2c_flag_get(i2cx, I2C_FLAG_BTC));
                        /* send a stop condition to I2C bus */
                        i2c_stop_on_bus(i2cx);
                    }
                    /* wait until RBNE bit is set */
                    if(i2c_flag_get(i2cx, I2C_FLAG_RBNE)) 
                    {
                        /* read a byte from the EEPROM */
                        *p_buffer = i2c_data_receive(i2cx);

                        /* point to the next location where the byte read will be saved */
                        p_buffer++;

                        /* decrement the read bytes counter */
                        number_of_byte--;
                        timeout = 0;
                    }
                    if(timeout > I2C_TIME_OUT) 
                    {
                        timeout = 0;
                        state = I2C_START;
                        read_cycle = 0;
                        printf("i2c master sends data timeout in READ!\n");
                    }
                }
                timeout = 0;
                state = I2C_STOP;
            }
            break;
        case I2C_STOP:
            /* i2c master sends STOP signal successfully */
            while((I2C_CTL0(i2cx) & I2C_CTL0_STOP) && (timeout < I2C_TIME_OUT)) 
            {
                timeout++;
            }
            if(timeout < I2C_TIME_OUT) 
            {
                timeout = 0;
                state = I2C_END;
                i2c_timeout_flag = I2C_OK;
            } 
            else 
            {
                timeout = 0;
                state = I2C_START;
                read_cycle = 0;
                printf("i2c master sends stop signal timeout in READ!\n");
            }
            break;
        default:
            state = I2C_START;
            read_cycle = 0;
            i2c_timeout_flag = I2C_OK;
            timeout = 0;
            printf("i2c master sends start signal in READ.\n");
            break;
        }
    }
    return I2C_END;
}





































