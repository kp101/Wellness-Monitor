<b>Raspberry Pi Zero 2W + bme688</b>

This is a implementation of a c program to pull temperature/pressure/humidity/voc from a Bosch BME68x sensor via i2c and report it an external mqtt broker.
Adafruit IO MQTT was chosen and tested with this but it may work with other brokers. Adafruit IO is nice because in my district, triggers, weather forecast (paid) can be created to send email/text messages to my phone. This along with others in the project for Wellness Monitoring allows me to check on comfort and movements of the inhabitants remotely.

Features:

    Adjustable scanning periods,
    publishing environmental data to a broker. 
    secured MQTT connection using 8883 port,
    Settings can be changed in mqtt.h and sensor.h headers prior to compiling.

Hardware dependencies:

    1. BME688 4-in-1 Air Quality Breakout (Gas, Temperature, Pressure, Humidity) 
    refer: https://shop.pimoroni.com/products/bme688-breakout?variant=39336951709779,
    2. a 5v power adaptor (a used phone charger approx 2A is sufficient.), cable should be as short as possible to avoid brownout.
    3. a Raspberry Pi ZeroW  or Pi Zero2W

Software dependencies:

    1. WiringPi, (see install_wiringpi.sh)
    2. paho-mqtt. (see install_paho.sh)
    
Attaching Bosch's BME688 sensor over I2C (requires ~3.1 mA):

    1. Attach bme688 via Qw/ST to breakout connector. Qwiic I2C connector pinout typically includes four pins arranged as follows: 
        a. Black = GND, 
        b. Red = 3.3V, 
        c. Blue = SDA, 
        d. Yellow = SCL. 
    2. Connect Pin 17 to power pin on bme688,
    3. connect pin SDA (i2c 1) on Pico to SDA pin on bme688,
    4. connect pin SCL (i2c 1) on Pico to SCL pin on bme688,
    note: depending on what else is attached to I2C, there is a secondary address for bme688...0x76

note: This sensor has a heater, it should have a breakin period and a scan period meeting the requirements of Bosch for accuracy. see Bosch doc.

On communicating with Adafruit IO's MQTT:

    should leave client_id="" to avoid collision with multiple client with same client_id triggering random disconnects. (don't ask me how I know.)
    when sending JSON to Adafruit IO mqtt, 'value={"xxx":yy}' is needed. Otherwise, send only number and text. 
    alternative to mqtt, it is also possible to use the restful api on Adafruit IO.

Compile and Installation Instructions:

    1. Customize, compile with Makefile, 
    2. Once sucessfully compiled, sudo cp the bme68x.service file to /etc/systemd/system directory. sudo systemctl enable bme68x, sudo systemctl start bme68x.
    3. login to your mqtt broker account to verify data are posting. By default it is 30 minutes.
