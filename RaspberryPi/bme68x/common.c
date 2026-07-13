/**
 * This is a derived copy of the original from Bosch Sensortec.
 * 
 * Modified: 2026-07-05
 * 
 * This was modified to work with the WiringPi i2c routine on Raspberry Pi, 
 * some work around to address limitations.
 * 
 * Copyright (C) 2023 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * 
 * 
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <stdbool.h>
#include <string.h>

#include "bme68x.h"
#include "bme68x_defs.h"
//#include "coines.h"
#include "common.h"
#include <wiringPi.h>
#include "wiringPiI2C.h"

#define COINES_SUCCESS                             0
#define COINES_E_FAILURE                           -1
enum coines_multi_io_pin {
    COINES_SHUTTLE_PIN_7 = 0x09
};

void swap_endianness(uint16_t* dest_arr, uint16_t* src_arr, uint16_t count);
bool is_system_little_endian();

/******************************************************************************/
/*!                 Macro definitions                                         */
/*! BME68X shuttle board ID */
#define BME68X_SHUTTLE_ID  0x93
#define SWAP_BYTES(x) ((((x) >> 8) | ((x) << 8)) & 0xffff)

/******************************************************************************/
/*!                Static variable definition                                 */
static uint8_t dev_addr;

/******************************************************************************/
/*!                User interface functions                                   */

/*!
 * I2C read function map to wiringPi 
 */
BME68X_INTF_RET_TYPE bme68x_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{

    int device_addr = *(int*)intf_ptr;
    int ret = 0;

    //ret = wiringPiI2CWriteReg8(device_addr, 0xCC, reg_addr);
    if (len == 1)
    {
        reg_data[0] = wiringPiI2CReadReg8(device_addr, reg_addr);
        ret = reg_data[0] < 0 ? reg_data[0] : 1;
    }
    else
    {
        ret = wiringPiI2CReadBlockData(device_addr, reg_addr, reg_data, (uint8_t)len);
    }        
    /* 
    printf("i2c read len=%d data: ", len );
    for (int i=0; i< len; i++ )
        printf(" 0x%02X", reg_data[i]);
    printf(" ret=%d\n", ret);
    */ 
    return ret < 0;
}

/*!
 * I2C write function map to COINES platform
 */
BME68X_INTF_RET_TYPE bme68x_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    int device_addr = *(int*)intf_ptr;
    int ret = 0;
    uint8_t buffer[len+1];

    /* 
       reference datasheets : 
           https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme688-ds000.pdf
           revision: BST-BME688-DS000-03 Revision_1.3 022024

       6.2.1 I2C write
       "...master sends pairs of register addresses and register data..."
    */
        
    buffer[0]=reg_addr;
    memcpy( buffer+1, reg_data, len );

    /*
    printf("i2c write len=%d, data: ", len );
    for (int i=0; i< len; i++ )
        printf(" 0x%02X", reg_data[i]);
    */   
    ret = wiringPiI2CWriteBlockData(device_addr, (int) reg_addr, (uint8_t*) buffer, (uint8_t)len+1);
       
    //printf(" ret=%d\n", ret );

    usleep(2);
    return ret < 0;
}

void bme68x_delay_us(uint32_t period, void *intf_ptr)
{
    //printf("bme.delay_us=%d\n", period );
    usleep(period); 
}

void bme68x_check_rslt(const char api_name[], int8_t rslt)
{
    switch (rslt)
    {
        case BME68X_OK:

            /* Do nothing */
            break;
        case BME68X_E_NULL_PTR:
            printf("API name [%s]  Error [%d] : Null pointer\r\n", api_name, rslt);
            break;
        case BME68X_E_COM_FAIL:
            printf("API name [%s]  Error [%d] : Communication failure\r\n", api_name, rslt);
            break;
        case BME68X_E_INVALID_LENGTH:
            printf("API name [%s]  Error [%d] : Incorrect length parameter\r\n", api_name, rslt);
            break;
        case BME68X_E_DEV_NOT_FOUND:
            printf("API name [%s]  Error [%d] : Device not found\r\n", api_name, rslt);
            break;
        case BME68X_E_SELF_TEST:
            printf("API name [%s]  Error [%d] : Self test error\r\n", api_name, rslt);
            break;
        case BME68X_W_NO_NEW_DATA:
            printf("API name [%s]  Warning [%d] : No new data found\r\n", api_name, rslt);
            break;
        default:
            printf("API name [%s]  Error [%d] : Unknown error code\r\n", api_name, rslt);
            break;
    }
}

int8_t bme68x_interface_init(struct bme68x_dev *bme, uint8_t intf)
{
    int8_t rslt = BME68X_OK;

    if (bme != NULL)
    {
       usleep(29);
       if (is_system_little_endian())
           printf("little endian\n");

        /* Bus configuration : I2C */
        if (intf == BME68X_I2C_INTF)
        {
            printf("I2C Interface selected.\n");
            //dev_addr = BME68X_I2C_ADDR_LOW;
            bme->read = bme68x_i2c_read;
            bme->write = bme68x_i2c_write;
            bme->intf = BME68X_I2C_INTF;

            /* SDO pin is made low */
            //(void)coines_set_pin_config(COINES_SHUTTLE_PIN_SDO, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_LOW);

            //(void)coines_config_i2c_bus(COINES_I2C_BUS_0, COINES_I2C_STANDARD_MODE);
        }
        /* Bus configuration : SPI 
        else if (intf == BME68X_SPI_INTF)
        {
            printf("SPI Interface selected.\n");
            //dev_addr = COINES_SHUTTLE_PIN_7;
            bme->read = bme68x_spi_read;
            bme->write = bme68x_spi_write;
            bme->intf = BME68X_SPI_INTF;
         //   (void)coines_config_spi_bus(COINES_SPI_BUS_0, COINES_SPI_SPEED_7_5_MHZ, COINES_SPI_MODE0);
        } */

        bme->delay_us = bme68x_delay_us;
        //bme->intf_ptr = &dev_addr;
        bme->amb_temp = 25; /* The ambient temperature in deg C is used for defining the heater temperature */
    }
    else
    {
        rslt = BME68X_E_NULL_PTR;
    }

    return rslt;
}

void bme68x_deinit(void * intf_ptr)
{
    int device_addr = *(int*)intf_ptr;
    uint8_t i2c_reset = BME68X_SOFT_RESET_CMD;
    (void)fflush(stdout);
   
    bme68x_i2c_write( 0xE0, &i2c_reset, 1, intf_ptr ); 
    /* Coines interface reset */
    //coines_soft_reset();
    //coines_delay_msec(1000);
    usleep(1000);
    close(device_addr);    
}

/*!
 * @brief This API is used to swap the endianness of the 16-bit data
 */
void swap_endianness(uint16_t* dest_arr, uint16_t* src_arr, uint16_t count)
{
    for (uint16_t i = 0; i < count; i++) {
        dest_arr[i] = SWAP_BYTES(src_arr[i]);
    }
}

/*!
 *  @brief This API is used to check if the system is little-endian or big-endian
 *
 */
bool is_system_little_endian() 
{
    uint16_t value = 0x1;  
    // 2-byte integer with the least significant byte set to 1
    // Pointer to the first byte of 'value'
    uint8_t *byte_ptr = (uint8_t*)&value;  
    // If the least significant byte is 1, the system is little-endian
    return byte_ptr[0] == 1;
}

