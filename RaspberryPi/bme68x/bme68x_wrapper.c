//example: https://github.com/hivemq-cloud/paho-C-mqtt-client-example/blob/master/main.c

#include <assert.h>
#include <ctype.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <time.h>

#include <wiringPi.h>
//#include <wiringSerial.h>
#include "wiringPiI2C.h"
#include "mqtt.h"
#include "bme68x.h"
#include "common.h"
#include "bme68x_wrapper.h"

int bme68x_forced_mode(struct bme68x_dev *bme, struct bme68x_avg * results) 
{
    int8_t rslt;
    struct bme68x_conf conf;
    struct bme68x_heatr_conf heatr_conf;
    struct bme68x_data data;
    uint32_t del_period;
    uint32_t time_ms = 0;
    uint8_t n_fields;
    uint16_t sample_count = 1;
    time_t begin, current;

    time(&begin);

    rslt = bme68x_get_conf(&conf, bme);
    bme68x_check_rslt("bme68x_get_conf", rslt);

    /* Check if rslt == BME68X_OK, report or handle if otherwise */
    conf.filter = BME68X_FILTER_OFF;
    conf.odr = BME68X_ODR_NONE; 
    conf.os_hum = BME68X_OS_16X;
    conf.os_pres = BME68X_OS_1X;
    conf.os_temp = BME68X_OS_2X;
    rslt = bme68x_set_conf(&conf, bme);
    bme68x_check_rslt("bme68x_set_conf", rslt);
    printf("done set conf\n");

    /* Check if rslt == BME68X_OK, report or handle if otherwise */
    heatr_conf.enable = BME68X_ENABLE;
    heatr_conf.heatr_temp = 300;
    heatr_conf.heatr_dur = 100;
    rslt = bme68x_set_heatr_conf(BME68X_FORCED_MODE, &heatr_conf, bme);
    bme68x_check_rslt("bme68x_set_heatr_conf", rslt);
    printf("set heater conf completed.\n");

    /* Check if rslt == BME68X_OK, report or handle if otherwise */
    rslt = bme68x_set_op_mode(BME68X_FORCED_MODE, bme);
    bme68x_check_rslt("bme68x_set_op_mode", rslt);
    printf("set op mode FORCED completed.\n");

    /* Check if rslt == BME68X_OK, report or handle if otherwise */
    //printf( "Sample, TimeStamp(ms), Temperature(deg C), Pressure(Pa), Humidity(%%), Gas resistance(ohm), Status, Profile index, Measurement index\n");

    /* Calculate delay period in microseconds */
    del_period = bme68x_get_meas_dur(BME68X_FORCED_MODE, &conf, bme) + (heatr_conf.heatr_dur * 1000);
    bme->delay_us(del_period, bme->intf_ptr);
    time(&current);
    time_ms = (long) difftime(current,begin) * 1000L;

    rslt = bme68x_get_data(BME68X_FORCED_MODE, &data, &n_fields, bme);
    bme68x_check_rslt("bme68x_get_data", rslt);
    //printf("forced mode get_data, rslt=%d, n_fields=%d\n", rslt, n_fields);

    /* Check if rslt == BME68X_OK, report or handle if otherwise */
#ifdef BME68X_USE_FPU
    /*
    //printf("%lu, %.2f, %.2f, %.2f, %.2f, 0x%x, %d, %d\n",
           (long unsigned int)time_ms + (del_period / 2000),
           data.temperature,
           data.pressure,
           data.humidity,
           data.gas_resistance,
           data.status,
           data.gas_index,
           data.meas_index); */
#else
    /*printf("%lu, %d, %lu, %lu, %lu, 0x%x, %d, %d\n",
           (long unsigned int)time_ms + (del_period / 2000),
           (data.temperature / 100),
           (long unsigned int)data.pressure,
           (long unsigned int)(data.humidity / 1000),
           (long unsigned int)data.gas_resistance,
           data.status,
           data.gas_index,
           data.meas_index);*/
#endif

    results->temperature = data.temperature;
    results->pressure = data.pressure;
    results->humidity = data.humidity;
    results->gas_resistance = data.gas_resistance,
    results->status = data.status;
    results->gas_index = data.gas_index;
    results->meas_index = data.meas_index;
    return 0;
}

int bme68x_parallel_mode(struct bme68x_dev *bme, struct bme68x_avg *results) 
{
    int8_t rslt;
    struct bme68x_conf conf;
    struct bme68x_heatr_conf heatr_conf;
    struct bme68x_data data[3];
    uint32_t del_period;
    uint32_t time_ms = 0;
    uint8_t n_fields;
    uint16_t sample_count = 0;
    time_t begin, current;

    /* Heater temperature in degree Celsius */
    uint16_t temp_prof[10] = { 320, 100, 100, 100, 200, 200, 200, 320, 320, 320 };
    /* Multiplier to the shared heater duration */
    uint16_t mul_prof[10] = { 5, 2, 10, 30, 5, 5, 5, 5, 5, 5 };
    const size_t profile_length = 10;

    time(&begin);

    rslt = bme68x_get_conf(&conf, bme);
    bme68x_check_rslt("bme68x_get_conf", rslt);

    /* Check if rslt == BME68X_OK, report or handle if otherwise */
    conf.filter = BME68X_FILTER_OFF;
    conf.odr = BME68X_ODR_NONE; 
    conf.os_hum = BME68X_OS_16X;
    conf.os_pres = BME68X_OS_1X;
    conf.os_temp = BME68X_OS_2X;
    rslt = bme68x_set_conf(&conf, bme);
    bme68x_check_rslt("bme68x_set_conf", rslt);
    printf("done set conf\n");

    /* Check if rslt == BME68X_OK, report or handle if otherwise */
    heatr_conf.enable = BME68X_ENABLE;
    heatr_conf.heatr_temp_prof = temp_prof;
    heatr_conf.heatr_dur_prof = mul_prof;
    /* Shared heating duration in milliseconds */
    heatr_conf.shared_heatr_dur = (uint16_t)(140 - (bme68x_get_meas_dur(BME68X_PARALLEL_MODE, &conf, bme) / 1000));
    heatr_conf.profile_len = profile_length;

    rslt = bme68x_set_heatr_conf(BME68X_PARALLEL_MODE, &heatr_conf, bme);
    bme68x_check_rslt("bme68x_set_heatr_conf", rslt);
    printf("set heater conf completed.\n");

    /* Check if rslt == BME68X_OK, report or handle if otherwise */
    rslt = bme68x_set_op_mode(BME68X_PARALLEL_MODE, bme);
    bme68x_check_rslt("bme68x_set_op_mode", rslt);
    printf("set op mode PARALLEL completed.\n");

    /* Check if rslt == BME68X_OK, report or handle if otherwise */
    // getchar();

    while (sample_count <= SAMPLE_COUNT)
    {
        /* Calculate delay period in microseconds */
        del_period = bme68x_get_meas_dur(BME68X_PARALLEL_MODE, &conf, bme) + (heatr_conf.shared_heatr_dur * 1000);
        bme->delay_us(del_period, bme->intf_ptr);
        time(&current);
        time_ms = (long) difftime(current,begin) * 1000L;

        rslt = bme68x_get_data(BME68X_PARALLEL_MODE, data, &n_fields, bme);
        bme68x_check_rslt("bme68x_get_data", rslt);
        //printf("parallel mode get_data, rslt=%d, n_fields=%d\n", rslt, n_fields);

        /* Check if rslt == BME68X_OK, report or handle if otherwise */
        //printf( "sam, time(ms), temp(C), press(Pa), Humid(%%), voc(ohm), status, nprof, nmeas\n");
        for (uint8_t i = 0; i < n_fields; i++)
        {
#ifdef BME68X_USE_FPU
            /*printf("%u, %lu, %.2f, %.2f, %.2f, %.2f, 0x%x, %d, %d\n",
                   sample_count,
                   (long unsigned int)time_ms + (i * (del_period / 2000)),
                   data[i].temperature - TEMPERATURE_OFFSET,
                   data[i].pressure / 1000,
                   data[i].humidity,
                   data[i].gas_resistance / 1000,
                   data[i].status,
                   data[i].gas_index,
                   data[i].meas_index); */
#else
            /*printf("%u, %lu, %d, %lu, %lu, %lu, 0x%x, %d, %d\n",
                   sample_count,
                   (long unsigned int)time_ms + (i * (del_period / 2000)),
                   (data[i].temperature / 100),
                   (long unsigned int)data[i].pressure,
                   (long unsigned int)(data[i].humidity / 1000),
                   (long unsigned int)data[i].gas_resistance,
                   data[i].status,
                   data[i].gas_index,
                   data[i].meas_index);*/
#endif
            sample_count++;
            results->temperature += data[i].temperature;
            results->pressure += data[i].pressure;
            results->humidity += data[i].humidity;
            results->gas_resistance += data[i].gas_resistance,
            results->status = data[i].status;
            results->gas_index = data[i].gas_index;
            results->meas_index = data[i].meas_index;
        }
        /*
        for(int i = 0; i < n_fields; i++) {
            results[data[i].gas_index] = data[i];
            if(data[i].gas_index == profile_length - 1) return true;
        }*/
    }
    
    results->temperature /= sample_count;
    results->pressure /= sample_count;
    results->humidity /= sample_count;
    results->gas_resistance /= sample_count; 

    return 0;
}


int bme68x_sequential_mode(struct bme68x_dev *bme, struct bme68x_avg * results) 
{
    int8_t rslt;
    struct bme68x_conf conf;
    struct bme68x_heatr_conf heatr_conf;
    struct bme68x_data data[3];
    uint32_t del_period;
    uint32_t time_ms = 0;
    uint8_t n_fields;
    uint16_t sample_count = 0;
    time_t begin, current;
    const size_t profile_length = 10;

    /* Heater temperature in degree Celsius */
    uint16_t temp_prof[10] = { 200, 240, 280, 320, 360, 360, 320, 280, 240, 200 };

    /* Heating duration in milliseconds */
    uint16_t dur_prof[10] = { 100, 100, 100, 100, 100, 100, 100, 100, 100, 100 };

    time(&begin);

    rslt = bme68x_get_conf(&conf, bme);
    bme68x_check_rslt("bme68x_get_conf", rslt);

    /* Check if rslt == BME68X_OK, report or handle if otherwise */
    conf.filter = BME68X_FILTER_OFF;
    conf.odr = BME68X_ODR_NONE; 
    conf.os_hum = BME68X_OS_16X;
    conf.os_pres = BME68X_OS_1X;
    conf.os_temp = BME68X_OS_2X;
    rslt = bme68x_set_conf(&conf, bme);
    bme68x_check_rslt("bme68x_set_conf", rslt);
    printf("done set conf\n");

    /* Check if rslt == BME68X_OK, report or handle if otherwise */
    heatr_conf.enable = BME68X_ENABLE;
    heatr_conf.heatr_temp_prof = temp_prof;
    heatr_conf.heatr_dur_prof = dur_prof;
    heatr_conf.profile_len = 10;
    rslt = bme68x_set_heatr_conf(BME68X_SEQUENTIAL_MODE, &heatr_conf, bme);
    bme68x_check_rslt("bme68x_set_heatr_conf", rslt);
    printf("set heater conf completed.\n");

    /* Check if rslt == BME68X_OK, report or handle if otherwise */
    rslt = bme68x_set_op_mode(BME68X_SEQUENTIAL_MODE, bme);
    bme68x_check_rslt("bme68x_set_op_mode", rslt);
    printf("set op mode SEQUENTIAL completed.\n");

    /* Check if rslt == BME68X_OK, report or handle if otherwise */

    while (sample_count <= SAMPLE_COUNT)
    {
        /* Calculate delay period in microseconds */
        del_period = bme68x_get_meas_dur(BME68X_SEQUENTIAL_MODE, &conf, bme) + (heatr_conf.heatr_dur_prof[0] * 1000);
        bme->delay_us(del_period, bme->intf_ptr);
        time(&current);
        time_ms = (long) difftime(current,begin) * 1000L;

        rslt = bme68x_get_data(BME68X_SEQUENTIAL_MODE, data, &n_fields, bme);
        bme68x_check_rslt("bme68x_get_data", rslt);
        //printf("sequential mode get_data, rslt=%d, n_fields=%d\n", rslt, n_fields);

        /* Check if rslt == BME68X_OK, report or handle if otherwise */
        //printf( "sam, time(ms), temp(C), press(Pa), Humid(%%), voc(ohm), status, nprof, nmeas\n");
        for (uint8_t i = 0; i < n_fields; i++)
        {
#ifdef BME68X_USE_FPU
           /* 
            printf("%u, %lu, %.2f, %.2f, %.2f, %.2f, 0x%x, %d, %d\n",
                   sample_count,
                   (long unsigned int)time_ms + (i * (del_period / 2000)),
                   data[i].temperature,
                   data[i].pressure,
                   data[i].humidity,
                   data[i].gas_resistance,
                   data[i].status,
                   data[i].gas_index,
                   data[i].meas_index); */
#else
            /*printf("%u, %lu, %d, %lu, %lu, %lu, 0x%x, %d, %d\n",
                   sample_count,
                   (long unsigned int)time_ms + (i * (del_period / 2000)),
                   (data[i].temperature / 100),
                   (long unsigned int)data[i].pressure,
                   (long unsigned int)(data[i].humidity / 1000),
                   (long unsigned int)data[i].gas_resistance,
                   data[i].status,
                   data[i].gas_index,
                   data[i].meas_index);*/
#endif
            sample_count++;
            results->temperature += data[i].temperature;
            results->pressure += data[i].pressure;
            results->humidity += data[i].humidity;
            results->gas_resistance += data[i].gas_resistance,
            results->status = data[i].status;
            results->gas_index = data[i].gas_index;
            results->meas_index = data[i].meas_index;
        }
        /*
        for(int i = 0; i < n_fields; i++) {
            results[data[i].gas_index] = data[i];
            if(data[i].gas_index == profile_length - 1) return true;
        }*/
        //getchar();
        
    }
    
    results->temperature /= sample_count;
    results->pressure /= sample_count;
    results->humidity /= sample_count;
    results->gas_resistance /= sample_count;

    return 0;
}

