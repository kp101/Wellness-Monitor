"""
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

prerequisites: The following requires micropython image uf2 from pimoroni 
               which included the micopython breakout_bme68x module.
               This breakout_bme68x is a dependency library used for 
               environmental readings and it was generously provided by pimoroni.
               
reference: https://github.com/pimoroni/pimoroni-pico/tree/main/micropython/examples/breakout_bme68x
tested with:
    https://github.com/pimoroni/pimoroni-pico-rp2350/releases/tag/v1.26.1
    
"""
#import simple
from robust import MQTTClient
from network_manager import NetworkManager
import machine
import time
from breakout_bme68x import BreakoutBME68X, STATUS_HEATER_STABLE, \
     FILTER_COEFF_3, STANDBY_TIME_1000_MS, OVERSAMPLING_16X, OVERSAMPLING_2X, OVERSAMPLING_1X, \
     I2C_ADDRESS_DEFAULT, I2C_ADDRESS_ALT
import uasyncio
import utime
import binascii
import yaml
import ujson

def load_config(filename=None):
    path = filename or "config.yml"
   
    return yaml.load(path)
    
config = load_config()
WIFI_SID = config['wifi']['ssid']
WIFI_PSK = config['wifi']['psk']
WIFI_REGION = config['wifi']['country']

MQTT_ADR = config['mqtt']['broker']
MQTT_UID  = config['mqtt']['uid']
MQTT_PWD  = config['mqtt']['pwd']
MQTT_TOPIC= config['mqtt']['topic']
MQTT_PORT = config['mqtt']['port']

SCAN_INTERVAL = config['scan_interval']
HEARTBEAT_UPDATE = config['heartbeat_update']
REBOOT_CYCLE = config['reboot_cycle']
WIFI_RETRY = config['wifi_retry']

STATION = config['station']
DEVICE  = config['device']

BME_I2C_ADR = config['bme_i2c']
BME_SCAN_DURATION = config['bme_scan_period']
TEMPERATURE_OFFSET = config['bme_temp_offset']
SDA = config['sda']
SCL = config['scl']
POWER_PIN = config['power_pin']

MMWAVE_SCAN_DURATION = config['mmwave_scan_period']
TX_PIN = config['tx_pin']
RX_PIN = config['rx_pin']
UART_NO = config['uart']
BAUD_RATE = config['baudrate']

led = machine.Pin('LED', machine.Pin.OUT) # only one led onboard.

scan_count = 0
net_retry = WIFI_RETRY
reboot_countdown = REBOOT_CYCLE

def send_hex_string(hex_string):
    hex_bytes = binascii.unhexlify(hex_string)
    soft_uart.write(hex_bytes)

def read_serial_data():
    start_time = time.ticks_ms()
    dist = 0
    
    # check for 9 seconds for movements within the room.
    while time.ticks_ms() - start_time < MMWAVE_SCAN_DURATION :
        try:
            if soft_uart.any():
                data = soft_uart.readline()
                print("Received:", data)
                if data:
                    data = data.decode('utf-8')
                    data = data.replace("Range","").strip()
                    data = data.replace("ON","").strip()
                    data = data.replace("N","").strip()
                    print("data:", data )
                    if data.isdigit():
                        dist = int(data)
                    elif data.count('OFF') > 0:
                        dist = 0
                    else:
                        pass
                 
        except (TypeError, UnicodeError) as e:
            print(f"Error: {e}")
            pass
        utime.sleep_ms(100)
        
    return dist
    
def mmwave():
    hex_to_send = "FDFCFBFA0800120000006400000004030201"
    
    send_hex_string(hex_to_send)
    
    return read_serial_data()

def status_handler(mode, status, ip):  # 
    global net_retry
    
    print("Connecting...")
    led.value(1)   
    if status is not None:
        if status:
            status_text = "Wifi Connection successful!"
            net_retry = WIFI_RETRY
        else:
            status_text = "Wifi Connection failed!"
            net_retry = net_retry - 1
            if net_retry <= 0:
                machine.reset()
        print(status_text)
    print("IP: {}".format(ip))
    time.sleep(1)
    led.value(0)

def sos(repeat):
    led.value(0)
    time.sleep_ms(1500)
    for j in range(repeat):
        for i in range(3):
            led.value(1)
            time.sleep_ms(250)
            led.value(0)
            time.sleep_ms(500)    
        for i in range(3):
            led.value(1)
            time.sleep_ms(1000)
            led.value(0)
            time.sleep_ms(500)
        for i in range(3):
            led.value(1)
            time.sleep_ms(250)
            led.value(0)
            time.sleep_ms(500)
        time.sleep_ms(1500)

# gather data from sensors. two sensors, mmwave and bme6xx.
def check_sensors(timer):
    heater = "Unstable"    
    distance = 0
    start_time = time.ticks_ms() 
    global scan_count
    global reboot_countdown
        
    try:
        mqtt_client = MQTTClient(client_id="", user=MQTT_UID, password=MQTT_PWD, server=MQTT_ADR, port=MQTT_PORT, ssl=True)

        distance = mmwave()
        payload = {"value": {"range": distance, "dev": DEVICE, "station": STATION }}
        
        scan_count = (scan_count + 1) % HEARTBEAT_UPDATE
        reboot_countdown = reboot_countdown - 1
        
        if scan_count == 1: # do additional scans, must update
            # read it a few times to allow heater to warm up. some recommends 30 secs. ref. see bosch website.
            while time.ticks_ms() - start_time < BME_SCAN_DURATION : 
                # read BME688
                temperature, pressure, humidity, gas_resistance, status, gas_index, meas_index = bme.read(heater_temp=250, heater_duration=50)
                heater = "Stable" if status & STATUS_HEATER_STABLE else "Unstable"

                if heater == "Stable":
                    # correct temperature and humidity using an offset
                    corrected_temperature = temperature - TEMPERATURE_OFFSET
                    dewpoint = temperature - ((100 - humidity) / 5)
                    corrected_humidity = 100 - (5 * (corrected_temperature - dewpoint))
                    payload["value"]["temp"] = corrected_temperature
                    payload["value"]["pres"] = pressure / 1000          
                    payload["value"]["humd"] = corrected_humidity
                    payload["value"]["voc"]  = gas_resistance / 1000
                else:
                    print("waiting for bme688 to stablize...") # heater element 
                time.sleep(1.0)
            
            print(payload)
            led.value(1)
            mqtt_client.connect()
            mqtt_client.publish(topic=MQTT_TOPIC, msg=ujson.dumps(payload))
            time.sleep_ms(500)            
            mqtt_client.disconnect()
            led.value(0)            
        elif distance > 0: 
            print(payload)
            led.value(1)
            mqtt_client.connect()
            mqtt_client.publish(topic=MQTT_TOPIC, msg=ujson.dumps(payload))
            time.sleep_ms(500)            
            mqtt_client.disconnect()
            led.value(0)
            
    except Exception as e:  
        print(e)
        sos(10)
        time.sleep(3)
        machine.reset()
    finally:
        if reboot_countdown <= 0:
            time.sleep(10)
            machine.reset()        
        
try:
    # initialize bme68x sensor. requires ~ 15mA, so a power from a GPIO pin should be sufficient.
    pwr = machine.Pin(POWER_PIN, machine.Pin.OUT)
    pwr.value(1)
    i2c = machine.I2C(1, scl=machine.Pin(SCL), sda=machine.Pin(SDA))
    #i2c = machine.I2C(1, scl=SCL, sda=SDA)
    #devices = i2c.scan()	# for troubleshooting...
    #print(devices)         # This will print the addresses of connected I2C devices.

    # bme could be on 0x76 or 0x77 change in config.yml as needed or use I2C_ADDRESS_DEFAULT, I2C_ADDRESS_ALT
    bme = BreakoutBME68X(i2c, BME_I2C_ADR)
    bme.configure(FILTER_COEFF_3, STANDBY_TIME_1000_MS, OVERSAMPLING_16X, OVERSAMPLING_2X, OVERSAMPLING_1X)
    print("bme init completed...")

    # setup serial port.
    soft_uart = machine.UART(UART_NO, baudrate=BAUD_RATE, tx=TX_PIN, rx=RX_PIN)

    # set up wifi
    network_manager = NetworkManager(WIFI_REGION, status_handler=status_handler)
    # connect to wifi
    uasyncio.get_event_loop().run_until_complete(network_manager.client(WIFI_SID, WIFI_PSK))
    pwr.value(0)

    # Initialize Timer 0
    timer = machine.Timer()

    # Set the timer to periodic mode with a period of nx1000 milliseconds and attach the callback function
    timer.init(mode=machine.Timer.PERIODIC, period=SCAN_INTERVAL, callback=check_sensors)

    while True:
        time.sleep(1)
        
except Exception as e:  
    print(e)
    sos(10)
    time.sleep(3)
    machine.reset()
    
except KeyboardInterrupt:
    timer.deinit()
    print("Timer stopped.")
finally:
    print("shutting down.")
