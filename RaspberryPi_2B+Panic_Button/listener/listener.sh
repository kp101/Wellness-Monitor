#!/bin/bash
export GPIOZERO_PIN_FACTORY=native
cd /home/pi/listener
source ./venv/bin/activate
python listener.py
