#!/bin/bash
cd /home/pi/sensors
## https://gpiozero.readthedocs.io/en/latest/api_pins.html#changing-the-pin-factory
export GPIOZERO_PIN_FACTORY=native
source ./venv/bin/activate
python sensors.py
