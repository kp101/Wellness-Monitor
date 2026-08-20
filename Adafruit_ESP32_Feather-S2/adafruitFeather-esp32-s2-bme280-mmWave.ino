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
#include "config.h"

HardwareSerial mySerial(1);
#define BUFFER_LEN 255

WiFiClientSecure espClient;

// Create an MQTT client
Adafruit_MQTT_Client mqtt(&espClient, AIO_SERVER, AIO_SERVER_PORT, AIO_USERNAME, AIO_KEY);
// Define the feed
//AdafruitIO_Feed *temperature = io.feed("bedroom1-temp");
Adafruit_MQTT_Publish mqtt_temp = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME AIO_FEED_TEMP);
Adafruit_MQTT_Publish mqtt_pres = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME AIO_FEED_PRES);
Adafruit_MQTT_Publish mqtt_humd = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME AIO_FEED_HUMD);

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
  char payload[250];
  
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

// initialize serial monitor for debugging, not sensor serial port.
  Serial.begin(115200);
  Serial.println("opening serial port...");

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

#if defined(ARDUINO_ADAFRUIT_QTPY_ESP32S2) 
  // ESP32 is kinda odd in that secondary ports must be manually
  // assigned their pins with setPins()!
  Wire1.setPins(SDA1, SCL1);
#endif

#if defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2_TFT)
  pinMode(TFT_I2C_POWER, OUTPUT);
  digitalWrite(TFT_I2C_POWER, HIGH);
#endif
#if defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2_REVTFT)
  pinMode(TFT_I2C_POWER, OUTPUT);
  digitalWrite(TFT_I2C_POWER, HIGH);
#endif

  checkbattery();

  if (! bme.begin()) {
    Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
    deepSleep(); //take a nap and try again later.
  }
  Serial.println("Found BME280");
  Serial.println();

}

void loop() {
  char payload[250];

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  MQTT_connect();
  bme280(); 

  sprintf( payload, "%.2f", temp);
  mqtt_temp.publish(payload); 
  delay(1000);
  sprintf( payload, "%.2f", pres);
  mqtt_pres.publish(payload); 
  delay(1000);
  sprintf( payload, "%.2f", humd);
  mqtt_humd.publish(payload); 
    
  Serial.println("upload completed.");
  mqtt.disconnect();
  digitalWrite(LED_BUILTIN, LOW);

  deepSleep();
}
