

#ifndef __I2C_DRIVER_H__
#define __I2C_DRIVER_H__

#include "gd32f10x.h"

typedef enum {
    I2C_START = 0,
    I2C_SEND_ADDRESS,
    I2C_CLEAR_ADDRESS_FLAG,
    I2C_TRANSMIT_DATA,
    I2C_STOP,
} i2c_process_enum;

#define I2C_TIME_OUT           (uint16_t)(5000)

#define I2C_OK                 1
#define I2C_FAIL               0
#define I2C_END                1

#define I2C_SPEED               100000
#define I2CX_AUDIO_ADDRESS      0x36
#define I2CX_AHT20_ADDRESS      0x38

#define I2CX                    I2C0
#define I2C_SCL_PORT            GPIOB
#define I2C_SDA_PORT            GPIOB
#define I2C_SCL_PIN             GPIO_PIN_8
#define I2C_SDA_PIN             GPIO_PIN_9

void i2c_bus_reset(void);
int i2c_config(void);
uint8_t i2c_bytes_read_timeout(uint32_t i2cx, uint32_t slave_addr, uint8_t *p_buffer, uint8_t read_address, uint8_t number_of_byte);
uint8_t i2c_bytes_write_timeout(uint32_t i2cx, uint32_t slave_addr, uint8_t *p_buffer, uint8_t write_address, uint8_t number_of_byte);

#endif

