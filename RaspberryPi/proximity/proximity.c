#include <ctype.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>

#include <wiringPi.h>
#include <wiringSerial.h>

#include "proximity.h"
#include "mqtt.h"
#include "mmwave.h"
#include "toolbox.h"

int scan(int serial_port) {
    int i = 0;
    int range = 0;
    time_t time1, time2;

    time(&time1);
    time(&time2);
    while(difftime(time2, time1) <= SCAN_INTERVAL){
       i = get_range(serial_port);
       range = range < i ? i : range;
       usleep(7);
       time(&time2);
    }  
    return range;
}

int main(void) 
{
    int i = 0;
    int scan_count = 0;
    char payload[BUFFER_LEN];
    int range = 0;
    int serial_port = 0;

    wiringPiSetupPinType(WPI_PIN_BCM);
    printf("GPIO setup complete\n");

    serial_port = init_mmWave(SERIAL_PORT, BAUD_RATE);
    if (serial_port < 0)
       return -1;
    printf("mmWave setup complete\n");

    while(1) {
       scan_count = (scan_count +1) % HEARTBEAT_UPDATE;
       range = scan(serial_port);
       if ( range > 0 || scan_count <= 0 ) {
          sprintf( payload, msg_template, STATION, range, DEVICE);
          mqtt_publish( TOPIC, payload);  
       }
       sleep(UPDATE_INTERVAL);
    }
    return 0;
}
