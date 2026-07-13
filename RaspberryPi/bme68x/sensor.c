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
#include "wiringPiI2C.h"
#include "sensor.h"
#include "mqtt.h"
#include "bme68x.h"
#include "common.h"
#include "bme68x_wrapper.h"

int main(int argc, char *argv[]) {

    int i = 0;
    int scan_count = 0;
    char payload[512];
    struct bme68x_dev bme;
    struct bme68x_avg results;
    int8_t rslt;
    int file;    
    char *i2c_name = "/dev/i2c-1";
    int i2c_addr =  BME68X_I2C_ADDR_LOW;  // 0x76 I2C address of the device

    wiringPiSetupGpio();
    /* power for i2c device. you can put bme68x on non dedicated 3.3v power pin. */
    pinMode(PWR_PIN, OUTPUT);
    digitalWrite(PWR_PIN, HIGH);  // Turn on 
    sleep(1); // wait for sensor to power up.

    int fd = wiringPiI2CSetupInterface(i2c_name, i2c_addr);

    bme.intf_ptr = &fd;
    assert(*(int *)bme.intf_ptr == fd );

    /* Interface preference is updated as a parameter
     * For I2C : BME68X_I2C_INTF
     * For SPI : BME68X_SPI_INTF  // this project did not do spi. 
     */

    do 
    {
        rslt = bme68x_interface_init(&bme, BME68X_I2C_INTF);
        bme68x_check_rslt("bme68x_interface_init", rslt);

        rslt = bme68x_init(&bme);
        bme68x_check_rslt("bme68x_init", rslt);
        printf("init complete chip_id=0x%02X\n", bme.chip_id );

        /*
        rslt = bme68x_selftest_check(&bme);
        bme68x_check_rslt("bme68x_selftest_check", rslt);
        if (rslt != BME68X_OK )
        {
            return rslt;
        }
        printf("self test passed.\n" );
        */

        results.status = 0x80;
        // choose and uncomment one and only one of the following scan methods.
        for (int j = 0; j < 5 && results.status == 0x80; j++ ) // 0x80 data doesn't seem right.
        { 
            //bme68x_forced_mode( &bme, &results );   
            //bme68x_parallel_mode( &bme, &results );
            bme68x_sequential_mode( &bme, &results );
        }
        if ( results.status != 0x80 )
        {
            results.temperature -= TEMPERATURE_OFFSET;
            results.pressure /= 1000;
            results.gas_resistance /= 1000.0;
            //results.humidity = results.humidity;  // no change.

            printf( "temp(C), press(Pa), Humid(%%), voc(ohm), status, nprof, nmeas\n");
            printf("  %.2f,    %.2f,    %.2f, %.2f,  0x%x,  %d,   %d\n",
                   results.temperature, results.pressure, results.humidity, 
                   results.gas_resistance, results.status, results.gas_index,
                   results.meas_index);

            sprintf( payload, msg_template, STATION, DEVICE, 
                     results.temperature, results.pressure, 
                     results.humidity, results.gas_resistance );
            mqtt_publish( TOPIC, payload, strlen(payload));
        }
        bme68x_soft_reset(&bme);
        sleep(UPDATE_INTERVAL);
    }
    while (1);

    bme68x_deinit(bme.intf_ptr);
    digitalWrite(PWR_PIN, LOW);  // Turn off 
    printf("shutting down\n");

    return 0;
}

