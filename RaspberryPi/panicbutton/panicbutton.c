#include <ctype.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>

#include <wiringPi.h>
#include "panicbutton.h"
#include "mqtt.h"

int main(void) {

   int i = 0;
   bool triggered = false;
   char payload[512];

   wiringPiSetupPinType(WPI_PIN_BCM);
   pinMode(BUTTON_PIN, INPUT);
   pullUpDnControl(BUTTON_PIN, PUD_DOWN);

   while(1) {
      i = digitalRead(BUTTON_PIN);
      // printf("Button %i\n", i );
      if (i == HIGH) {
         if (!triggered) {
            printf("Alarm Activated %i\n", i );
            mqtt_publish( TOPIC, DISTRESS_SIG );
            triggered = true;
         }
         delay(5);
      }
      else {
         if (triggered) {
            printf("Cancelled alarm %i\n", i );
            mqtt_publish( TOPIC, CANCEL_SIG, strlen(CANCEL_SIG));
            triggered = false;
         }
         delay(5);
      }
   }

   return 0;
}
