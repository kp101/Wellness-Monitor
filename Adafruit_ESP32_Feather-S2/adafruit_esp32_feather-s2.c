#include <ctype.h>
#include <string.h>
#include <stdlib.h> 
// #include "Adafruit_LC709203F.h" //obsolete as of 2023, replaced by MAX1704X
#include "Adafruit_MAX1704X.h"
#include <Adafruit_NeoPixel.h>
#include <HardwareSerial.h>
#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <WiFi.h>
#include "WiFiClientSecure.h"
#include <SPI.h>
#include <Wire.h>
#include "config.h"

//hmmd_mmWave
HardwareSerial mySerial(1);
#define BUFFER_LEN 255

//Wifi 
//WiFiClient espClient;
// WiFiFlientSecure for SSL/TLS support
WiFiClientSecure espClient;

// Create an MQTT client
Adafruit_MQTT_Client mqtt(&espClient, AIO_SERVER, AIO_SERVER_PORT, AIO_USERNAME, AIO_KEY);
// Define the feed
//AdafruitIO_Feed *temperature = io.feed("bedroom1-temp");
Adafruit_MQTT_Publish sensor = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME AIO_FEED);

//LC709203 Battery Monitor
Adafruit_MAX17048 maxlipo;
//Adafruit_LC709203F lc;
//MAX17048 i2c address
bool addr9x36 = true;

// Bosch BME 280 integrated temperature, humidity and pressure sensor.
Adafruit_BME280 bme; // I2C
Adafruit_NeoPixel pixel(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

float temp=0.0, humd=0.0, pres=0.0, volt=0.0;
RTC_DATA_ATTR int wake_count;

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
#endif


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
  // Power on the mmWave sensor. 
  Serial.println("initializing mmWave...");
  mySerial.begin(BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);  // RX, TX
    // Wait for the serial port to initialize
  while (!mySerial) {
    delay(100);
  }

  Serial.println("setting mmWave mode...");
  // Hex string to send
  String hex_to_send = "FDFCFBFA0800120000006400000004030201";
  sendHexData(hex_to_send);
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
  uint8_t retries = 5;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println(F("Retrying MQTT connection in 5 seconds..."));   
    mqtt.disconnect();
    delay(5000);  // wait 5 seconds
    retries--;
    if (retries == 0) {
      // basically die and wait for WDT to reset me
      deepSleep();  // or take a nap and retry later. could be power outage.
      //while (1);
    }
  }
  Serial.println(F("MQTT Connected!"));
}

void bme280() {
  
  temp = bme.readTemperature() - TEMPERATURE_OFFSET;
  pres = bme.readPressure() / 1000.0F;
  humd = bme.readHumidity();
  // shhh time to close your eyes
  bme.setSampling(Adafruit_BME280::MODE_SLEEP,
                  Adafruit_BME280::SAMPLING_X16, Adafruit_BME280::SAMPLING_X16, Adafruit_BME280::SAMPLING_X16,
                  Adafruit_BME280::FILTER_OFF,
                  Adafruit_BME280::STANDBY_MS_1000);

  //temp = temp * 9.0 / 5.0 + 32;
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
  Serial.println();
}

void checkbattery() {
  float cellVoltage = 0.0;

  if (!maxlipo.begin()) {
    Serial.println(F("Couldn't find Adafruit MAX17048..."));
  }
  else {
    cellVoltage = maxlipo.cellVoltage();
    if (isnan(cellVoltage)){
      Serial.println("battery not connected...");
    }
    else {
      Serial.print("Cell Percentage: ");
      Serial.print(maxlipo.cellPercent());
      Serial.println("%");
      Serial.print("Cell Voltage: ");
      maxlipo.cellVoltage();
      Serial.println("V");
      volt = maxlipo.cellVoltage();
    }
  }
}

void deepSleep() {
  //pinMode(NEOPIXEL_POWER, OUTPUT);
  //digitalWrite(NEOPIXEL_POWER, LOW); // off
  //digitalWrite(LED_BUILTIN, LOW);
  //digitalWrite(PIN_NEOPIXEL, LOW);
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

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  //mmWave power pin.
  //pinMode(NEOPIXEL_POWER, OUTPUT);
  //digitalWrite(NEOPIXEL_POWER, HIGH);
  
  // initialize NeoPixel strip object (REQUIRED)
  //pixel.begin();  
  //pixel.setBrightness(10);  // not so bright
  //pixel.setPixelColor(0, 0xFF0000); // red
  //pixel.setPixelColor(0, pixel.Color(0,255,0)); // green
  //pixel.setPixelColor(0, pixel.Color(0,0,255)); // blue
  //pixel.show();

// initialize serial monitor for debugging, not sensor serial port.
  Serial.begin(115200);
  Serial.println("opening serial port...");
  Serial.println("opening mmWave serial port...");
  //  Wait for the serial port to initialize
  /*
  while (!Serial) {
    delay(100);
  }
  */

  //pinMode(PIN_NEOPIXEL, OUTPUT);
  //digitalWrite(PIN_NEOPIXEL, HIGH);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    #if defined(DEBUGGING)
      Serial.print(F("."));
    #endif
  }
  Serial.println(F("WiFi connected"));
  Serial.println(F("IP address: ")); Serial.println(WiFi.localIP());

  // Set Adafruit IO's root CA
  espClient.setCACert(adafruitio_root_ca);

#if defined(ARDUINO_ADAFRUIT_QTPY_ESP32S2) || \
    defined(ARDUINO_ADAFRUIT_QTPY_ESP32S3_NOPSRAM) || \
    defined(ARDUINO_ADAFRUIT_QTPY_ESP32S3_N4R2) || \
    defined(ARDUINO_ADAFRUIT_QTPY_ESP32_PICO)
  // ESP32 is kinda odd in that secondary ports must be manually
  // assigned their pins with setPins()!
  Wire1.setPins(SDA1, SCL1);
#endif

#if defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2)
  // turn on the I2C power by setting pin to opposite of 'rest state'
  //pinMode(PIN_I2C_POWER, INPUT);
  //delay(1);
  //bool polarity = digitalRead(PIN_I2C_POWER);
  //pinMode(PIN_I2C_POWER, OUTPUT);
  //digitalWrite(PIN_I2C_POWER, !polarity);
#endif

#if defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2_TFT)
  pinMode(TFT_I2C_POWER, OUTPUT);
  digitalWrite(TFT_I2C_POWER, HIGH);
#endif
#if defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2_REVTFT)
  pinMode(TFT_I2C_POWER, OUTPUT);
  digitalWrite(TFT_I2C_POWER, HIGH);
#endif

  // Turn on the I2C power by pulling pin HIGH.
  //  pinMode(NEOPIXEL_I2C_POWER, OUTPUT);
  //  digitalWrite(NEOPIXEL_I2C_POWER, HIGH);
  checkbattery();

  if (! bme.begin()) {
    Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
    /*
    while( 1 ) { // don't continue.
      delay(100);
    }
    */
    deepSleep(); //take a nap and try again later.
  }
  Serial.println("Found BME280");
  Serial.println();
  // initmmWave();

}

void loop() {
  char buffer[BUFFER_LEN];
  char payload[250];
  int retry = 0;
  char * p = 0;
  int dist = 0;

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  MQTT_connect();
  bme280();

  /* get mmWave sensor results.
  for (int i=1; i <=5; i++ ) {
    readSerialData(buffer);
    if (strcmp(buffer, "ON") == 0) {
      readSerialData(buffer);
      p = strstr(buffer, "Range ");
      if (p) 
        dist = atoi(p+6);
    }     
  } 
  //readSerialData(buffer);
  Serial.println(buffer);
  */
  
  // Read battery and print data
  /*
  if (lc.begin())
    sprintf( payload, "{\"sensor\": %s, \"temp\": %.2f, \"press\": %.2f, \"humd\": %.2f, \"volt\": %.2f, \"perc\": %.2f, \"temp\": %.2f}", 
                      SENSOR, temp, pres, humd, lc.cellVoltage(), lc.cellPercent(), lc.getCellTemperature());
  else

    sprintf( payload, "{\"sensor\": \"%s\", \"temp\": %.2f, \"pres\": %.2f, \"humd\": %.2f, \"dist\": %i}", SENSOR, temp, pres, humd, dist);
  */

  sprintf( payload, "{ \"value\":{\"station\": \"%s\", \"dev\": \"%s\", \"temp\": %.2f, \"pres\": %.2f, \"humd\": %.2f, \"range\": %i, \"vbat\": %.2f}}", 
                                  STATION, DEVICE, temp, pres, humd, dist, volt); 
  sensor.publish(payload); 
  Serial.println(payload);

  // ping the server to keep the mqtt connection alive
  // NOT required if you are publishing once every KEEPALIVE seconds
  /*
  if(! mqtt.ping()) {
    mqtt.disconnect();
  }
  */
  mqtt.disconnect();

  //pinMode(NEOPIXEL_POWER, OUTPUT);
  //digitalWrite(NEOPIXEL_POWER, LOW);
  // pixel.clear();
  //pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
    //mmWave power pin.

     //digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
  //delay(20000);  
  deepSleep();
}
