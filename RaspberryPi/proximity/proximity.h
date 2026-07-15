#if !defined(PROXIMITY_H)
#define PROXIMITY_H

   #define PROXIMITY_CLEAR 0
   #define BUFFER_LEN 250
   #define HEARTBEAT_UPDATE 180
   #define UPDATE_INTERVAL  180
   #define SCAN_INTERVAL    17
   #define STATION          "bedroom1"
   #define DEVICE           "Raspberry Pi Zero 2W"
   #define SERIAL_PORT      "/dev/ttyAMA0"        // setup serial port in raspi-config, remove bt in config.txt
   #define BAUD_RATE        115200

   #define TOPIC       "cafruit/feeds/upstairs"
 
   const char* msg_template = "{\"value\":{\"station\":\"%s\",\"range\":%i,\"dev\":\"%s\"}}";

#endif
