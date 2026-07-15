#if !defined(MMWAVE_H)
#define MMWAVE_H

   #define BUFFER_LEN       250

   extern int get_range( int serial_port );

   // Read and print data from serial
   extern void read_serial_data(int serial_port, char *buffer); 

   extern int init_mmWave(char *device_name, int baud_rate); 

#endif
