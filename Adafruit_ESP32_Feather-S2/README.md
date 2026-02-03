<b>Adafruit ESP32 Feather S2 w/bme280 + mmWave</b>

The following is a "C/C++" implementation using a Adafruit ESP32 Feather S2 with embedded bme280 sensor and an optional mmWave sensor. Currently is is coded for remote monitoring of outdoor conditions with a battery pack.

This IoT updates Adafruit IO MQTT in the cloud. It will also work with local mosquitto mqtt. Adafruit IO is nice because in my district, triggers can be created to send email/text messages to my phone. This along with others in the project for Wellness Monitoring allows me to check on outside conditions.

This implementation is similar to the Raspberry Pi Pico 2W's except for the programming language and the subscribing of a topic. But in terms of movement detection and how i2c was connected and bme688 it is the same. This implementation uses deepsleep in non-mmWave implementation. This amazing little device will last for months on a charge of battery. optional mmWave sensor draws too much power to be on battery.

Features:

1. Adjustable scanning periods,
2. optional - movement scanning and enviornmental readings,
3. publishing to a broker only. 
4. secured MQTT connection using 8883 port,
5. Settings can be changed in config.h.

Hardware dependencies:

1. Adafruit ESP32-S2 Feather with BME280 Sensor/STEMMA QT/4MB Flash+2MB PSRAM,
2. optional - HMMD mmWave Sensor refer:https://www.waveshare.com/wiki/HMMD_mmWave_Sensor,
3. battery pack or a 5v power adaptor (needed if used with mmWave, a used phone charger between 1A-2A is sufficient.), cable should be as short as possible to avoid brownout.

Software dependencies:

Arduino IDE

Attaching mmWave sensor (~50mA) optional:

- TX pin 39 on esp32 is attached to RX pin on mmWave sensor,
- RX pin 38 on esp32 is attached to TX pin on mmWave sensor,
- 3.3v pin on Pico is attached to 3.3v pin on mmWave sensor,
- GND pin on Pico is attached to GND pin on mmWave sensor.

note: the connection of TX to RX pins. Also, this sensor alone will mean running on batteries is impractical.

On communicating with Adafruit IO's MQTT:

The attaching of rootcert is required for secured connection. See code for detail,
should leave client_id="" to avoid collision on multiple client with same client_id triggering random disconnects. (don't ask me how I know.)
when sending JSON to Adafruit IO mqtt, 'value={"xxx":yy}' is needed. Otherwise, send only number and text. 
alternative to mqtt, it is also possible to use the restful api on Adafruit IO.

Installation Instructions:

hold reset and connect usb to computer, wait for device to show.
open Arduino IDE,
change parameters in config.h
compile, upload and test.

This is part of a larger project designed for a customizable wellness remote monitoring.
