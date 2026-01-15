/*
MIT License

Copyright (c) 2026 kp101

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.


Adafruit ESP32 Feather V2 Huzzah with QT/Stemma, 8 MB Flash and 2 MB PSRAM, NeoPixel mini RGB LED, 
==================================================================================================
 *
 * ESP32 Dual core 240MHz Xtensa® processor, 8 MB of SPI Flash, 2 MB of PSRAM, a tuned PCB antenna, both WiFi and Bluetooth Classic/LE support,
 * The CH9102F USB to serial converter can handle 50bps to 4Mbps max rate. (Boards shipped before May 19, 2022 used the CP2102N chip, good to 3Mbps.) ,
 * Separate controllable 3.3V power supply for STEMMA QT to allow for ultra low power consumption even with sensors are attached,
 * Additional user button tactile switch on input pin 38
 * Designed for low power usage: verified with a PPK to draw 70uA from the Lipoly battery in deep sleep and 1.2mA in light sleep.
 * LiPoly battery monitor with two 200K resistor divider,
 * 3V - This is the output from the 3.3V regulator. The regulator can supply 500mA peak but half of that is drawn by the ESP32, 
   and it's a fairly power-hungry chip. So if you need a ton of power for stuff like LEDs, motors, etc. use the USB or BAT pins, and 
   an additional regulator.
 * LiPoly Battery Monitor - The lipoly battery monitor, located to the left of the STEMMA QT connector, towards the center of the board, 
   has a two 200K resistor divider. VOLTAGE_MONITOR pin (GPI35) - This ADC pin can be used to read the data from the battery monitor. 
   Basically perform an analog read and multiply by two, to get a rough sense of the lipoly battery voltage.
 * CHG LED - This LED, located below the USB-C connector, and labeled CHG on the silk, lights up while the battery is charging. 
   It is normal for it to possibly flicker while no battery is plugged in.
 * STEMMA QT connector, labeled QT I2C on the silk. This connector allows you to connect a variety of sensors and breakouts 
   with STEMMA QT connectors using various associated cables. In CircuitPython, use board.STEMMA_I2C(). In Arduino, use Wire.
 * NeoPixel RGB LED - This RGB LED, on NEOPIXEL or pin 0, can be controlled in code to light up any color of the rainbow. 
   Treat it like a normal WS2812B. It is available in CircuitPython is board.NEOPIXEL, and Arduino as PIN_NEOPIXEL.
 * NeoPixel Power Pin - There is a power pin that must be pulled high for the NeoPixel to work. This is done automatically by CircuitPython and Arduino. 
   The pin is available in CircuitPython and in Arduino as NEOPIXEL_I2C_POWER or pin 2.
 * Red LED - This little red LED, labeled #13 on the silk, is pin 13. It can be controlled in code like any LED, set high to turn on. 
   It is available in CircuitPython as board.LED, and Arduino as LED_BUILTIN.
 * SW38 - This is a user readable button switch to use as an input, labeled SW38 on the silk. You can read it on pin GPI 38. 
   Note this is an input-only pin. There is a pull-up on board. It is available in CircuitPython as board.BUTTON, and in Arduino as BUTTON.
 * Reset - The reset button, labeled reset on the silk, is used to reboot the board.
 * LiPoly batteries are 'maxed out' at 4.2V and stick around 3.7V for much of the battery life, then slowly sink down to 3.2V or so 
   before the protection circuitry cuts it off. By measuring the voltage you can quickly tell when you're heading below 3.7V.
 * For permanent installations, a 5V 1A USB wall adapter will let you plug in a USB cable for reliable power
 * For mobile use, where you don't want a LiPoly, use a USB battery pack!
 * If you have a higher voltage power supply, use a 5V buck converter and wire it to a USB cable's 5V and GND input 

IMPORTANT

 * For running in low power mode, you can disable (set output and LOW) the NEOPIXEL_I2C_POWER pin, 
   this will turn off the separate 3.3V regulator that powers the QT connector's red wire, you need to turn it back on later if not in deepsleep.

 * Do not use alkaline or NiMH batteries and connect to the battery port - this will destroy the LiPoly charger
 * Do not use 7.4V RC batteries on the battery port - this will destroy the board


reference : https://learn.adafruit.com/adafruit-esp32-feather-v2/overview
            https://learn.adafruit.com/adafruit-esp32-feather-v2/power-management
*/

#include <Adafruit_BME680.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_I2CRegister.h>
#include <ctype.h>
#include <time.h>
#include <string.h>
#include <stdlib.h> 
#include "Adafruit_LC709203F.h"
#include "Adafruit_MAX1704X.h"
#include <Adafruit_NeoPixel.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include "WiFiClientSecure.h"
#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>
#include <Wire.h>

#include "config.h"

#if defined(ADAFRUIT_FEATHER_ESP32_V2) or defined(ARDUINO_ADAFRUIT_ITSYBITSY_ESP32)
  #define VBATPIN A13
  #define PIN_NEOPIXEL 0
  #define NEOPIXEL_I2C_POWER 2
#endif

#if defined(PIN_NEOPIXEL)
  Adafruit_NeoPixel pixel(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
#endif

//hmmd_mmWave
HardwareSerial mySerial(1);
#define BUFFER_LEN 255

// WiFiFlientSecure for SSL/TLS support
WiFiClientSecure espClient;
//WiFiClient espClient;

// Create an MQTT client
Adafruit_MQTT_Client mqtt(&espClient, AIO_SERVER, AIO_SERVER_PORT, AIO_USERNAME, AIO_KEY);

// Define the feed
Adafruit_MQTT_Publish sensor = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME AIO_FEED);
Adafruit_MQTT_Subscribe perimeter = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME AIO_PERIMETER, MQTT_QOS_1);
//LC709203 Battery Monitor
Adafruit_MAX17048 maxlipo;
Adafruit_LC709203F lc;
//MAX17048 i2c address
bool addr9x36 = true;

float temp=0.0, humd=0.0, pres=0.0, voc=0.0, alt=0.00;
RTC_DATA_ATTR int cycle_counter = 0L;
RTC_DATA_ATTR int reboot_countdown = REBOOT_INTERVAL;
unsigned long delayTime;

#define DEFAULT_I2C_PORT &Wire
// Some boards have TWO I2C ports, how nifty. We should scan both
#if defined(ARDUINO_ARCH_RP2040) \
    || defined(ARDUINO_ADAFRUIT_QTPY_ESP32S2) \
    || defined(ARDUINO_ADAFRUIT_QTPY_ESP32S3_NOPSRAM) \
    || defined(ARDUINO_ADAFRUIT_QTPY_ESP32S3_N4R2) \
    || defined(ARDUINO_ADAFRUIT_QTPY_ESP32_PICO) \
    || defined(ARDUINO_SAM_DUE) \
    || defined(ARDUINO_ARCH_RENESAS_UNO)
  #define SECONDARY_I2C_PORT &Wire1
  Wire1.setPins(SDA, SCL);
#endif

// Bosch BME 688 integrated temperature, humidity and pressure sensor.
Adafruit_BME680 bme(&Wire); // I2C

void scani2c() {
  byte error, address;
  int nDevices;
  Serial.println("Scanning...");
  nDevices = 0;
  //Wire.begin(I2C_SDA, I2C_SCL);
  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
      nDevices++;
    }
    else if (error==4) {
      Serial.print("Unknow error at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  }
  else {
    Serial.println("done\n");
  }
  delay(5000); 
}
void sendHexData(String hexString) {
  // Convert hex string to bytes
  int hexStringLength = hexString.length();
  byte hexBytes[hexStringLength / 2];
  for (int i = 0; i < hexStringLength; i += 2) {
    hexBytes[i / 2] = strtoul(hexString.substring(i, i + 2).c_str(), NULL, 16);
  }

  // Send bytes through software serial
  mySerial.write(hexBytes, sizeof(hexBytes));
}

// Read and print data from serial
void readSerialData(char *buffer) {
  int i = 0;

  while (mySerial.available() > 0) {
    char incomingByte = mySerial.read();
    //Serial.print(incomingByte);
    if (incomingByte == '\n')
      return;

    if (isprint(incomingByte)) {
      if (i < (BUFFER_LEN - 2)) {
        buffer[i] = incomingByte;
        buffer[i+1] = 0;
        i++;
      }
      else
        return;
    }
  }
}

void initmmWave(void) {
  int retry_counter = 10;

  // Power on the mmWave sensor. 
  Serial.print("initializing mmWave...");
  // Wait for the serial port to initialize

  mySerial.begin(BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);  // RX, TX
    // Wait for the serial port to initialize
  while (!mySerial) {
    Serial.print(".");
    delay(100);
    retry_counter --;
    if (retry_counter < 0) {
      sos(10);
      ESP.restart(); // Restart the ESP32
    }
  }

  Serial.println("setting mmWave mode...");
  // Hex string to send
  String hex_to_send = "FDFCFBFA0800120000006400000004030201";
  sendHexData(hex_to_send);
}

void sos(int repeat) {
  int i=0, j=0;

  for (i=0; i< repeat; i++) {
    for (i=0; j< 3; j++) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(250);  // wait 5 seconds
      digitalWrite(LED_BUILTIN, LOW);
      delay(500);  // wait 5 seconds
    }
    for (i=0; j< 3; j++) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(1000);  // wait 5 seconds
      digitalWrite(LED_BUILTIN, LOW);
      delay(500);  // wait 5 seconds
    }
    for (i=0; j< 3; j++) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(250);  // wait 5 seconds
      digitalWrite(LED_BUILTIN, LOW);
      delay(500);  // wait 5 seconds
    }
  }
}
// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;

  //Stop if already connected.
  if (mqtt.connected()) {
    return;
  }
  Serial.print(F("Connecting to MQTT... "));

  digitalWrite(LED_BUILTIN, HIGH);
  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println(F("Retrying MQTT connection in 5 seconds..."));   
    mqtt.disconnect();
    delay(5000);  // wait 5 seconds
    retries--;
    if (retries == 0) {
         // basically die and wait for WDT to reset me
      digitalWrite(LED_BUILTIN, LOW);
      sos(10);
      ESP.restart();
      //deepSleep();
      //while (1);
    }
  }
  Serial.println(F("MQTT Connected!"));
  digitalWrite(LED_BUILTIN, LOW);

}

void doorbell_flasher(int repeat) {
  int i = 0;

  Serial.print("do doorbell flashes, repeat:");
  Serial.print(repeat);
  Serial.println("");
  for (i=1; i<= repeat; i++) {
    Serial.print(i);
    LEDon();
    delay(2000);
    LEDoff();
    delay(2000);  
  }
}

void perimeter_callback(char * data, uint16_t len) {
  char * sptr = strstr(data, DOORBELL);

  if (sptr)
      doorbell_flasher(DOORBELL_FLASHES);
  
  Serial.print("perimeter callback value=");
  Serial.println(data);
}

void initbme688() {
  Serial.println("initializing bme688...");

  if (!bme.begin(0x76)) {   // must supply address, it won't find it on its own.
    Serial.println(F("Could not find a valid BME680 sensor, check wiring!"));
    //while (1);
    sos(10);
    ESP.restart();
  }
  Serial.println("Set up oversampling and filter initialization...");
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms
}

void bme688() {
  unsigned long endTime = bme.beginReading();

  if (endTime == 0) {
    Serial.println(F("Failed to begin reading :("));
    return;
  }
  if (!bme.endReading()) {
    Serial.println(F("Failed to complete reading :("));
    return;
  }

  temp = bme.temperature - TEMPERATURE_OFFSET;
  pres = bme.pressure / 1000.0F;
  humd = bme.humidity;
  voc  = bme.gas_resistance / 1000.0;
  alt  = bme.readAltitude(SEALEVELPRESSURE_HPA);

  //temp = temp * 9.0 / 5.0 + 32;  //convert to celcius
  Serial.print("Temperature = ");
  Serial.print(temp);
  Serial.println(" *C");
  //temperature->save(temp);

  Serial.print("Pressure = ");
  Serial.print(pres);
  Serial.println(" kPa");
  //pressure->save(pres);

  Serial.print("Humidity = ");
  Serial.print(humd);
  Serial.println(" %");
  //humidity->save(hum);

  Serial.print("Gas Resistance = ");
  Serial.print(voc);
  Serial.println(" ohm");
  //voc->save(voc);

  Serial.print("Approx. Altitude = ");
  Serial.print(alt);
  Serial.println(" m");

  Serial.println();
}

float check_battery() {
  float measuredvbat = analogReadMilliVolts(VBATPIN);
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat /= 1000; // convert to volts!
  return measuredvbat;
}

// Neopixel 
void LEDon() {
#if defined(PIN_NEOPIXEL)
  //pixel.begin(); // INITIALIZE NeoPixel
  //pixel.setBrightness(20); // not so bright
  //pixel.setPixelColor(0, 0xFF0000); // red
  //pixel.setPixelColor(0, pixel.Color(0,255,0)); // green
  pixel.setPixelColor(0, pixel.Color(0,0,255)); // blue
  pixel.show();
#endif
}
void LEDoff() {
#if defined(PIN_NEOPIXEL)
  pixel.setPixelColor(0, 0x0);
  pixel.show();
#endif
}

void enableInternalPower() {
#if defined(NEOPIXEL_POWER)
  pinMode(NEOPIXEL_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_POWER, HIGH);
#endif
#if defined(NEOPIXEL_I2C_POWER)
  pinMode(NEOPIXEL_I2C_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_I2C_POWER, HIGH);
#endif
}

// disabling power affects the bme690 sensor even if it was turned back on subsequently.
void disableInternalPower() {
#if defined(NEOPIXEL_POWER)
  pinMode(NEOPIXEL_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_POWER, LOW);
#endif
#if defined(NEOPIXEL_I2C_POWER)
  pinMode(NEOPIXEL_I2C_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_I2C_POWER, LOW);
#endif
}

void deepSleep() {
  delay(500);  
  //esp_sleep_enable_timer_wakeup(300000000); // 5 minutes
  esp_sleep_enable_timer_wakeup(SECONDS_SLEEP * uS_TO_S_FACTOR); // in microseconds
  esp_deep_sleep_start();
}
void lightSleep() {
  delay(500);  
  //esp_sleep_enable_timer_wakeup(300000000); // 5 minutes
  esp_sleep_enable_timer_wakeup(SECONDS_SLEEP * uS_TO_S_FACTOR); // in microseconds
  esp_light_sleep_start();
}

void initWiFi() {
  int retry_counter = 10;

  digitalWrite(LED_BUILTIN, HIGH);
  // connect to wifi ap.
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(F("."));
    retry_counter--;
    if (retry_counter <=0 ) {
      delay(10000); // wait a bit before restart.
      sos(10);
      ESP.restart();
    }
  }
  Serial.println(F("WiFi connected"));
  Serial.println(F("IP address: ")); Serial.println(WiFi.localIP());

  // Set Adafruit IO's root CA
  espClient.setCACert(adafruitio_root_ca);
  digitalWrite(LED_BUILTIN, LOW);
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  // Start the serial communication with a baud rate of 115200
  Serial.begin(115200);
  while (!Serial) {  // Wait for the serial port to initialize
    delay(100);
  }
  // Turn on any internal power switches for TFT, NeoPixels, I2C, etc!
  enableInternalPower();
#if defined(PIN_NEOPIXEL)
  pixel.begin(); // INITIALIZE NeoPixel
#endif

  //connect wifi.
  initWiFi();

  //checkbattery();
  Wire.begin();            // init i2c
  //scani2c();
  initbme688();
  initmmWave();

  perimeter.setCallback(perimeter_callback);
  // Setup MQTT subscription for perimeter feed.
  mqtt.subscribe(&perimeter);
}

void loop() {
  char buffer[BUFFER_LEN];
  char payload[250];
  int retry = 0;
  char * p = 0;
  int dist = 0;
  float measuredvbat = 0.0;
  time_t start_time, end_time;
  int diff = 0;

  measuredvbat = check_battery();
  Serial.print("VBat: " ); Serial.println(measuredvbat);

  if (WiFi.status() != WL_CONNECTED)
    initWiFi();

  MQTT_connect();

  dist = 0;
  for (int i=1; i <=5; i++ ) {
    readSerialData(buffer);
    if (strcmp(buffer, "ON") == 0) {
      readSerialData(buffer);
      p = strstr(buffer, "Range ");
      if (p) 
        dist = atoi(p+6);
    }     
  }  
  if (dist > 0 ){
    Serial.print("Movement detected: ");
    Serial.print(dist);
    Serial.println(".");
  }
  else
    Serial.println("No movement detected.");
  delay(500);  

  cycle_counter = (cycle_counter + 1) % HEARTBEAT_UPDATE;
  Serial.print("cycle_counter=");
  Serial.print(cycle_counter);
  Serial.println("");

  if (cycle_counter == 1) {
    bme688();
    delay(500);  
    sprintf( payload, "{ \"value\":{\"dev\": \"%s\", \"station\": \"%s\", \"temp\": %.2f, \"pres\": %.2f, \"humd\": %.2f, \"voc\": %.2f, \"alt\": %.2f, \"range\": %i, \"vbat\": %.2f}}", 
                                  DEVICE, STATION, temp, pres, humd, voc, alt, dist, measuredvbat); 
    Serial.println(payload);
    digitalWrite(LED_BUILTIN, HIGH);
    sensor.publish(payload);  
    digitalWrite(LED_BUILTIN, LOW);
  }
  else if (dist > 0) {
    sprintf( payload, "{ \"value\":{\"dev\": \"%s\", \"station\": \"%s\", \"range\": %i}}", DEVICE, STATION, dist); 
    Serial.println(payload);
    digitalWrite(LED_BUILTIN, HIGH);
    sensor.publish(payload);  
    digitalWrite(LED_BUILTIN, LOW);
  }
  else
    Serial.println("dist <=0, no mqtt update.");

  // Get the current time
  time(&start_time);
  do {
    // in between getting sensor data,...
    // wait for incoming subscription packets and trigger callback subloop.
    // testing found 10,000ms or 20,000 to be reliably return, larger values have unpredictable wait times.
    // this is a blocking wait.
    Serial.println("processing packets...");
    mqtt.processPackets(20000);
    Serial.println("dones...");
    time(&end_time);
  } while ((int) difftime(end_time, start_time) < (UPDATE_INTERVAL / 1000));

  // ping the server to keep the mqtt connection alive
  // NOT required if you are publishing once every KEEPALIVE seconds
  if(! mqtt.ping()) {
    mqtt.disconnect();
  }

  //digitalWrite(LED_BUILTIN, LOW);  // turn the LED off by making the voltage LOW
  //disableInternalPower();        //disabling power will mess up the voc results.

  reboot_countdown--;
  Serial.print("reboot_countdown=");
  Serial.print(reboot_countdown);
  Serial.println("");
  delay(500);
  if (reboot_countdown <= 0)
    ESP.restart();
  
  // not using deep/light sleep in this app.
  /* 
  else {
    deepSleep();
  //lightSleep();
    #if defined(DEBUGGING)
      //delay(20000);  
      // delay(UPDATE_INTERVAL);
      //lightSleep();
    #endif
    #ifndef DEBUGGING
      //deepSleep();
      //delay(UPDATE_INTERVAL);
      //lightSleep();
    #endif
  }*/

}
