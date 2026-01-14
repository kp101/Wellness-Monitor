The goal of this project is to implement a collection of IoT devices around the house for remote monitoring of seniors aging in place or children returning from school. This is a low cost, open source, customizable solution for monitoring activities, alarms and alerts.

The strategy here is the use of MQTT as a central broker for reporting. We are reporting:
  1. the occupants' movement from room to room (we assume a senior moving from room to room is okay),
  2. the temperature, pressure, humidity of each room,
  3. alerting the hearing impaired with blinking led if the doorbell message is receive,
  4. more to come...

Minimal requirements: 
  1. Adafruit IO account (not neccesary but sufficient. You can use other MQTT brokers),
  2. one of these microcontrollers: Raspberry Pi Pico 2W, Pimoroni Pico Plus 2W, Adafruit ESP32 Feather v2,
  3. a mmWave detector (Waveshare mmWave), bosch BME688 or 690 or 280 (if voc is not needed.),

     
