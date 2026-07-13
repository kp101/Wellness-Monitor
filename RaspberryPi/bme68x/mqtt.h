#if !defined(MQTT_H)
#define MQTT_H

   #define MQTT_BROKER "ssl://io.adafruit.com:8883"
   #define MQTT_UID    "ADAFRUIT_AIO_USERNAME"
   #define MQTT_PWD    "ADAFRUIT_AIO_KEY"

   extern int mqtt_publish( char *topic, char *payload, int len);

#endif
