# RASPBERRY PI ZERO 2W + BME68X

This is a implementation of a c program to pull temperature/pressure/humidity/voc from a Bosch BME68x sensor via i2c and report it an external mqtt broker.
Adafruit IO MQTT was chosen and tested with this but it may work with other brokers. Adafruit IO is nice because in my district, triggers, weather forecast (paid) can be created to send email/text messages to my phone. This along with others in the project for Wellness Monitoring allows me to check on comfort and movements of the inhabitants remotely.

This is an alternative to the usual python routine, it is more stable...it has less dependencies so less affected by breaking changes. As a bonus, it is much faster and consumes less resources.

>[!NOTE]
> This project depends on codes from [Bosch Sensortec](https://github.com/boschsensortec/BME68x_SensorAPI)
> Certain portion of codes are reproduced with modification to work around limitations.

## Features

- Adjustable scanning periods,
- publishing environmental data to a broker. 
- secured MQTT connection using 8883 port,
- Settings can be changed in mqtt.h and sensor.h headers prior to compiling.

## Hardware dependencies

1. BME688 4-in-1 Air Quality Breakout (Gas, Temperature, Pressure, Humidity) refer: [bme688breakout](https://shop.pimoroni.com/products/bme688-breakout)
2. a qwiic connect cable to pinout for sensor with qwiic connector,
3. a 5v power adaptor (a used phone charger approx 2A is sufficient.), cable should be as short as possible to avoid brownout.
4. a Raspberry Pi ZeroW  or Pi Zero2W.

## Software dependencies

1. WiringPi library, (see install_wiringpi.txt),
2. paho-mqtt library. (see install_paho.txt),
3. sudo raspi-config to enable i2c && sudo reboot,
4. sudo apt install i2c-tools -y && sudo i2cdetect -y 1 for troubleshooting or setup confirmation.

> [!CAUTION]
> A word on communicating with Adafruit IO's MQTT. Should leave client_id="" to avoid collision with multiple client with same client_id triggering random disconnects. (don't ask me how I know.)
> when sending JSON to Adafruit IO mqtt, 'value={"xxx":yy}' is needed. Otherwise, send only number and text. 
> alternative to mqtt, it is also possible to use the restful api on Adafruit IO.
    
## Attaching Bosch's BME688 sensor over I2C (requires ~3.1 mA)

1. Attach bme688 via Qw/ST to breakout connector. STEMMA QT/Qwiic JST SH 4-pin Cable with Premium Female Sockets. pinout typically includes four pins arranged as follows: 
- Black = GND, 
- Red = 3.3V, 
- Blue = i2c SDA data, 
- Yellow = i2c SCL clock. 
2. Connect Pin 17 to power pin on bme688,
3. connect pin SDA (i2c 1) on Pico to SDA pin on bme688,
4. connect pin SCL (i2c 1) on Pico to SCL pin on bme688,
  
> [!TIP]
> For bme68x,depending on what else is attached to I2C, there is a secondary address for bme688...0x76,
> use i2cdetect to check or trouble shoot.
> This sensor has a heater, it should have a breakin period and a scan period meeting the requirements of Bosch for accuracy. see Bosch doc.

## Compiling and Installation Instructions

1. Minimum customization should be made to sensor.h, mqtt.h and bme68x.service. Then compile with Makefile, 
2. Once sucessfully compiled:
- sudo cp the bme68x.service file to /etc/systemd/system directory.
- sudo systemctl enable bme68x,
- sudo systemctl start bme68x.
   
## Verify
- login to your mqtt broker account to verify data are posting. By default it is 30 minutes.
