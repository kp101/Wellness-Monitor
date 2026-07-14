# WELLNESS MONITOR with IOT DEVICES

The goal of this project is to implement a collection of IoT devices around the house for remote monitoring of activities for seniors aging in place/living alone or children returning from school. This is a low cost, open source, customizable solution for monitoring activities, alarms and alerts. We assume no pets.

The core of the monitoring does not depend on cameras, only movements. The destination of the data is chosen by the implementer. 

Additional features like environmental sensors(bme68x, bme690), oled display, speakers, plotting, etc. are available for more advanced implementers.

## Sample report:

 ![motus](images/adafruitio-motus-20260714.png)

## Assumptions
1. we assume a senior moving from room to room in his/her daily activity is relatively healthy.
2. No pets.

## Organization

In each folder, there is a type of IoT device which will be used to collect movements. 

- Only one type of device is needed to begin...[RaspberryPi-Pico2W](https://github.com/kp101/Wellness-Monitor/tree/50d20936e2d2d9d425f359b2aec37112ab954b6e/RaspberryPi-Pico2W).

- The [adafruitio](https://github.com/kp101/Wellness-Monitor/tree/d90f61e64aa972bd3db0047025a08c983d82ec8a/adafruitio) folder contain instructions for setting up your online MQTT broker.

- The [iot-monitor](https://github.com/kp101/Wellness-Monitor/tree/4c8350d6da55e2aa0cc7e07d48e445fc9e42534d/iot-monitor) is for monitoring the monitoring devices, reboots, system status, etc.

Additional features such as environmental sensor, display, audio out, camera are for more advaned users. A generic Raspberry Pi (ZeroW,2B,3B,...) is the most flexible but most complicated. Details are in each folder.

> [!TIP]
> The most cost efficient is the RaspberryPi Pico 2W ~ $10 CAD, mmWave ~ $5 CAD, plus an old phone charger and a mqtt online broker for remote monitoring. Start [here](https://github.com/kp101/Wellness-Monitor/tree/50d20936e2d2d9d425f359b2aec37112ab954b6e/RaspberryPi-Pico2W).

## Advanced Features

These are features not turnon by default and requires additional customizations. See respective project details.

1. Environmental sensors bme680, bme690 to report temperature, pressure, humidity and voc,
2. oled display with controller (ssd1306 or ssd1327),
3. Pi camera,
4. Raspberry Pi Audio,
5. RF receiver and RF remote (requires level shifter).

## Minimal requirements: 
  1. Adafruit IO account (not neccesary but sufficient. You can use other MQTT brokers but a commercial broker has features to trigger alarms and send text/email in defined scenarios),
  2. one of these microcontrollers: Raspberry Pi Pico 2W, Pimoroni Pico Plus 2W, Adafruit ESP32 Feather v2, Raspberry Pi ZeroW/Zero2W/2B/3B/3B+,
  3. a mmWave detector (Waveshare mmWave). 


