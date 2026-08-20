# Adafruit ESP32 Feather S2 w/bme280

The following is a "C/C++" implementation using a Adafruit ESP32 Feather S2 with embedded bme280 sensor. Currently is is coded for remote monitoring of outdoor conditions with a battery pack. Contrary to popular criticism, this is actually very efficiently little device -- capable of 3 months or more running only on batteries.

This IoT updates Adafruit IO MQTT in the cloud. It will also work with local mosquitto mqtt. 

## Features:

1. Adjustable scanning periods,
2. publishing to a secured MQTT broker with a secured 8883 connection,
4. Settings can be changed in config.h.

## Hardware dependencies:

1. Adafruit ESP32-S2 Feather with BME280 Sensor/STEMMA QT/4MB Flash+2MB PSRAM,
2. battery pack or an old 5v/1-2A phone adapter. 

## Software dependencies:

- Arduino IDE
- On communicating with Adafruit IO's MQTT:

> [!TIP]
> The attaching of rootcert is required for secured connection. See code for detail,
> alternative to mqtt, it is also possible to use the restful api on Adafruit IO.

>[!CAUTION]
> should leave client_id="" to avoid collision on multiple client with same client_id triggering random disconnects. (don't ask me how I know.)
> when sending JSON to Adafruit IO mqtt, 'value={"xxx":yy}' is needed. Otherwise, send only number and text. 

## Installation Instructions:

hold reset and connect usb to computer, wait for device to show.
open Arduino IDE,
change parameters in config.h
compile, upload and test.

This is part of a larger project designed for a customizable wellness remote monitoring.
