<b>Pimoroni Pico Plus 2W + bme690 + mmWave</b>

The following is a "Micropython" implementation using a Pimoroni Pico Plus 2W with 2 sensors to update Adafruit IO MQTT in the cloud. 
An account in Adafruit IO is sufficient but not neccessary. Other MQTT broker will work as well. Tested with Eclipse Mosquitto MQTT broker running locally using port 1883. 
Adafruit IO is nice because in my district, triggers can be created to send email/text messages to my phone.
This along with others in the project for Wellness Monitoring allows me to check on comfort and movements of the inhabitants remotely.

This implementation is the same as the Raspberry Pi Pico 2W except for the firmware image, how i2c was connected and bme690.

Features:
  1. Adjustable scanning periods,
  2. movement scanning and enviornmental readings,
  3. S.O.S. blinker if errors occurs prior to a soft reset,
  4. exception handling and reset if necessary,
  5. daily reboot in case of memory leaks,
  6. secured MQTT connection using 8883 port,
  7. Settings can be changed in config.yml.


Hardware dependencies:
  1. Pimoroni Pico Plus 2W, 
  2. HMMD mmWave Sensor refer:https://www.waveshare.com/wiki/HMMD_mmWave_Sensor,
  3. BME690 4-in-1 Air Quality Breakout (Gas, Temperature, Pressure, Humidity) refer: [https://shop.pimoroni.com/products/bme690-breakout?variant=55129358467451]
  4. a 5v power adaptor (a used phone charger between 1A-2A is sufficient.), cable should be as short as possible to avoid brownout.


Software dependencies:
  1. Pimoroni Pico Plus 2W compiled by Pimoroni with added libraries. refer: https://github.com/pimoroni/pimoroni-pico-rp2350/releases/tag/v1.26.1
  2. umqtt.simple (a tested and modified copy provided for convenience and consistency),
  3. network_manager (a tested copy provided for convenience and consistency),
  4. yaml (a tested copy provided for convenience and consistency),
  5. robust (a tested copy provided for convenience and consistency),


Attaching mmWave sensor (~50mA):
  1. TX pin 26 on Pico is attached to RX pin on mmWave sensor,
  2. RX pin 27 on Pico is attached to TX pin on mmWave sensor,
  3. 3.3v pin on Pico is attached to 3.3v pin on mmWave sensor,
  4. GND pin on Pico is attached to GND pin on mmWave sensor.

note: the connection of TX to RX pins. Also, this sensor alone will mean running on batteries is impractical. 


Attaching Bosch's BME690 sensor over I2C (requires ~3.1 mA):
  1. Attach bme688 via Qw/ST(Qwiic/STEMMA QT) connector,
  2. attach pin SDA (pin 4) on Pico to SDA pin on bme688,
  3. attach pin SCL (pin 5) on Pico to SCL pin on bme688,
  4. depending on what else is attached to I2C, there is a secondary address for bme688 (see config.yml).

note: This sensor has a heater, it should have a breakin period and a scan period meeting the requirements of Bosch for accuracy.


On communicating with Adafruit IO's MQTT:
  1. SSL=True is needed to use port 8883 for secured connection (there is a version of umqtt.Simple that imports uSSL which doesn't work in this firmware.)
  2. leave client_id="" to avoid collision on multiple client with same client_id triggering random disconnects. (don't ask me how I know.)
  3. when sending JSON to Adafruit IO mqtt, 'value={"xxx":yy}' is needed. Otherwise, send number and text directly.
alternative to mqtt, it is also possible to use the restful api on Adafruit IO.

Installation Instructions:
  1. hold reset and connect usb to computer, wait for drive to show and copy pimoroni firmware listed above to usb drive.
  2. open Thonny, create new files and copy contents into each file and save with same name on device.
  3. change parameters in config.yml
  4. save and test.

This is part of a larger project designed for a customizable wellness remote monitoring. 
