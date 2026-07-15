// reference : https://github.com/mackron/miniaudio/tree/master#building

#include <ctype.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include <wiringPi.h>
#include <wiringSerial.h>
#include "mmwave.h"
#include "toolbox.h"

int get_range( int serial_port ) 
{
    char buffer[BUFFER_LEN];
    char *p = NULL;
    int dist = 0;

    buffer[0]=0;
    read_serial_data(serial_port, buffer);
   
    if ((p = strstr(buffer, "ON")) != NULL) 
    {
       read_serial_data(serial_port, buffer);
      
       p = strstr(buffer, "Range");
       if (p == NULL) 
       {
           p = strstr(buffer, "N");
           if (p != NULL)
               p = p + 1;
       }
       else
           p = p + 5;
       if (p) 
       {
           p = crunch_blanks(p);
           if (is_numeric(p)) 
           { 
               dist = atoi(p);
               if (dist > 0 ) //movement detected
               {
                   return dist;
                }
           }
       }
   }
   
   return -1;
}

// Read and print data from serial
void read_serial_data(int serial_port, char *buffer) 
{
   int i = 0;
 
    buffer[0] = 0;
    while (serialDataAvail (serial_port) ) 
    {
        char dat = serialGetchar (serial_port);   /* receive character serially*/
        // printf("%c", dat);
        // fflush (stdout) ;

        if (dat == '\n')
            return;

        if (isprint(dat)) 
        {
            if (i < (BUFFER_LEN - 2)) 
            {
                buffer[i] = dat;
                buffer[i+1] = 0;
                i++;
            }
            else
                return;
        }
        delay(1);
    }
}

int init_mmWave(char *device_name, int baud_rate) 
{
    int serial_port;
    char *hex_to_send = "FDFCFBFA0800120000006400000004030201";

    if ((serial_port = serialOpen("/dev/ttyAMA0", 115200)) < 0)  
    {
        fprintf( stderr, "Unable to open serial device %s\n", strerror(errno));
        return -1;
    }
    
    if (wiringPiSetup () == -1) { /* initializes wiringPi setup */
        fprintf (stdout, "Unable to start wiringPi: %s\n", strerror (errno)) ;
        return -1 ;
    }

    // Power on the mmWave sensor. 
    serialPuts(serial_port, hex_to_send);
    // Wait for the serial port to initialize

    return serial_port;
}

