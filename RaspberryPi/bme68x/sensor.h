#if !defined(SENSOR_H)
#define SENSOR_H

   #define TEMPERATURE_OFFSET 6

   #define UPDATE_INTERVAL  60 * 30
   #define STATION          "room1"
   #define DEVICE           "Raspberry Pi Zero 2W"

   #define PWR_PIN          17 
   #define TOPIC            "<MQTT by Key>"
 
   const char* msg_template = "{\"value\":{\"station\":\"%s\",\"dev\":\"%s\", \"temp\": %.2f, \"pres\": %.2f, \"humd\": %.2f, \"voc\": %.2f }}";

#endif
