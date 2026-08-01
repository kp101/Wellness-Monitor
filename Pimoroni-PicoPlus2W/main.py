#!/micropython
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

reference: https://github.com/pimoroni/pimoroni-pico/tree/main/micropython/examples/breakout_bme690
reference: https://github.com/pimoroni/bme690-python/blob/main/examples/indoor-air-quality.py
reference: https://www.bosch-sensortec.com/products/environmental-sensors/gas-sensors/bme690/

Runs the sensor for a burn-in period, then uses a
combination of relative humidity and gas resistance
to estimate indoor air quality as a percentage.

Last update: July 31, 2026
version 2.31, Advanced, added oled, clean up codes.
"""
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
import ssd1306

def load_config(filename=None):
    path = filename or "config.yml"
   
    return yaml.load(path)
    
config = load_config()
WIFI_SID = config['wifi']['ssid']
WIFI_PSK = config['wifi']['psk']
WIFI_REGION = config['wifi']['country']
WIFI_RETRY = config['wifi']['retry']

MQTT_ADR = config['mqtt']['broker']
MQTT_UID  = config['mqtt']['uid']
MQTT_PWD  = config['mqtt']['pwd']
MQTT_PORT = config['mqtt']['port']
MQTT_TOPIC_MOTUS= config['mqtt']['topic_motus']
MQTT_TOPIC_TEMP= config['mqtt']['topic_temp']
MQTT_TOPIC_PRES= config['mqtt']['topic_pres']
MQTT_TOPIC_HUMD= config['mqtt']['topic_humd']
MQTT_TOPIC_VOC = config['mqtt']['topic_voc']

PROXIMITY_CHECK = config['proximity_check']
ENVIRO_UPDATE = config['enviro_update']
REBOOT_CYCLE = config['reboot_cycle']

STATION = config['station']
DEVICE  = config['device']

BME_I2C = config['bme']['i2c']
BME_I2C_ADR = config['bme']['i2c_adr']
BME_SCAN_DURATION = config['bme']['scan_duration']
BME_TEMPERATURE_OFFSET = config['bme']['temp_offset']
BME_HEATER_TEMP = config['bme']['heater_temp']
BME_HEATER_DURATION = config['bme']['heater_duration']
BME_SDA = config['bme']['sda']
BME_SCL = config['bme']['scl']
BME_POWER_PIN = config['bme']['pwr_pin']

MMWAVE_SCAN_DURATION = config['mmwave']['scan_duration']
MMWAVE_TX = config['mmwave']['tx_pin']
MMWAVE_RX = config['mmwave']['rx_pin']
MMWAVE_UART = config['mmwave']['uart']
MMWAVE_BAUD = config['mmwave']['baudrate']

OLED_POWER = config['oled']['pwr_pin']
OLED_I2C = config['oled']['i2c']
OLED_I2C_ADDR = config['oled']['i2c_adr']
OLED_SDA = config['oled']['sda']
OLED_SCL = config['oled']['scl']
OLED_WIDTH = config['oled']['width']
OLED_HEIGHT = config['oled']['height']

led = machine.Pin('LED', machine.Pin.OUT) # only one led onboard.

scan_count = 0
net_retry = WIFI_RETRY
reboot_countdown = REBOOT_CYCLE
## global variable for oled 
temperature = 0.00
humidity = 0.00
gas_resistance = 0.00
   
def send_hex_string(hex_string):
    global soft_uart
    
    hex_bytes = binascii.unhexlify(hex_string)
    soft_uart.write(hex_bytes)

def read_serial_data():
    global soft_uart
    
    start_time = time.ticks_ms()
    r = 0
    
    # check for x seconds for movements within the room.
    while time.ticks_ms() - start_time < MMWAVE_SCAN_DURATION :
        try:
            if soft_uart.any():
                data = soft_uart.readline()
                if data:
                    data = data.decode('ascii')
                    print("Received:", data)
                    data = data.replace("Range","").strip()
                    data = data.replace("ON","").strip()
                    data = data.replace("N","").strip()
                    if data is None:
                        pass
                    if data.isdigit():
                        r = int(data)
                        print(f"distance:{r}" )

                    elif data.count('OFF') > 0:
                        r = 0
                    else:
                        pass

        except Exception as e:  
            print(e)
        except (TypeError, UnicodeError) as e:
            print(f"Error: {e}")
            pass
        finally:
            utime.sleep_ms(20)
        
    return r
    
def mmwave():
    global soft_uart
    
    hex_to_send = "FDFCFBFA0800120000006400000004030201"
    try:
        soft_uart = machine.UART(MMWAVE_UART, baudrate=MMWAVE_BAUD, bits=8, parity=None, stop=1, tx=machine.Pin(MMWAVE_TX), rx=machine.Pin(MMWAVE_RX))
        send_hex_string(hex_to_send)
        print("initialize mmWave...")
        r = read_serial_data()
    except Exception as e:  
        print(e)
    finally:
        return r
    
def status_handler(mode, status, ip):  # 
    global net_retry
    global oled
    global led
    
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
    oled.fill(0)
    oled.text("IP:{}".format(ip), 0, 0)
    oled.show()    
    time.sleep(1)
    led.value(0)

def sos(repeat):
    global oled
    global led
    
    oled.fill(0)    
    oled.text('sos', 0, 0)
    oled.show()
    
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
    time.sleep(5)
    
# gather data from sensors. two sensors, mmwave and bme6xx.
def check_sensors(timer):
    heater = "Unstable"    
    distance = 0
    start_time = time.ticks_ms() 
    global scan_count
    global reboot_countdown
    global oled 
    global temperature
    global humidity
    global gas_resistance
    global bme
    
    try:

        distance = mmwave()
        print( f"mmwave:{distance}")
        #payload = {"value": {"range": distance, "dev": DEVICE, "station": STATION }}
        
        scan_count = (scan_count + 1 ) % ENVIRO_UPDATE
        reboot_countdown = reboot_countdown - 1
        
        if scan_count == 1: # do additional scans, must update
            oled.fill(0)
            oled.text("Sensing...", 0,0)
            oled.show()
            # read it a few times to allow heater to warm up. some recommends 30 secs. ref. see bosch website.
            while time.ticks_ms() - start_time < BME_SCAN_DURATION : 
                # read BME688
                temperature, pressure, humidity, gas_resistance, status, gas_index, meas_index = bme.read(heater_temp=BME_HEATER_TEMP, heater_duration=BME_HEATER_DURATION)

                heater = "Stable" if status & STATUS_HEATER_STABLE else "Unstable"

                if heater == "Stable":
                    # correct temperature and humidity using an offset
                    temperature = temperature - BME_TEMPERATURE_OFFSET
                    dewpoint = temperature - ((100 - humidity) / 5)
                    humidity = 100 - (5 * (temperature - dewpoint))
                    
                else:
                    print("waiting for bme688 to stablize...") # heater element 
                time.sleep(1.0)
            
        time.sleep(1)
        if distance > 0 or scan_count == 1:
            # set up wifi
            network_manager = NetworkManager(WIFI_REGION, status_handler=status_handler)
            uasyncio.get_event_loop().run_until_complete(network_manager.client(WIFI_SID, WIFI_PSK))
            time.sleep(3)
            mqtt_client = MQTTClient(client_id="", user=MQTT_UID, password=MQTT_PWD, server=MQTT_ADR, port=MQTT_PORT, ssl=True)
            mqtt_client.connect()
            oled.fill(0)
            oled.text("uploading...", 0,10)
            oled.show()

            if distance > 0:
                mqtt_client.publish(topic=MQTT_TOPIC_MOTUS, msg=STATION)               
                time.sleep(2)
                
            if scan_count == 1:               
                mqtt_client.publish(topic=MQTT_TOPIC_TEMP, msg="{:.2f}".format(temperature))
                time.sleep(2)
                mqtt_client.publish(topic=MQTT_TOPIC_PRES, msg="{:.2f}".format(pressure/1000))
                time.sleep(2)
                mqtt_client.publish(topic=MQTT_TOPIC_HUMD, msg="{:.2f}".format(humidity))
                time.sleep(2)
                mqtt_client.publish(topic=MQTT_TOPIC_VOC,  msg="{:.2f}".format(gas_resistance/1000))

            mqtt_client.disconnect()
            
        if distance > 0:            
            oled.fill(0)
            oled.text("temp:{:.2f} C".format(temperature), 0,0)
            oled.text("humd:{:.2f} %".format(humidity), 0,10)
            oled.text("voc:{:.2f} Kohm".format(gas_resistance/1000.00), 0,20)
            oled.show()
        else:
            oled.fill(0)
            oled.show()            
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
    pwr_bme = machine.Pin(BME_POWER_PIN, machine.Pin.OUT)
    pwr_bme.value(1)
    time.sleep(2)
    bme_i2c = machine.I2C(BME_I2C, scl=machine.Pin(BME_SCL), sda=machine.Pin(BME_SDA))
    #devices = bme_i2c.scan() # for troubleshooting...
    #print(devices)           # This will print the addresses of connected I2C devices.
    # bme could be on 0x76 or 0x77 change in config.yml as needed or use I2C_ADDRESS_DEFAULT, I2C_ADDRESS_ALT
    bme = BreakoutBME68X(bme_i2c, BME_I2C_ADR)
    bme.configure(FILTER_COEFF_3, STANDBY_TIME_1000_MS, OVERSAMPLING_16X, OVERSAMPLING_2X, OVERSAMPLING_1X)
    ## oled display SSD1306 on I2C
    pwr_oled = machine.Pin(OLED_POWER, machine.Pin.OUT)
    pwr_oled.value(2) 
    time.sleep(1)  # wait for power to stabilize.
    oled_i2c = machine.I2C(OLED_I2C, scl=machine.Pin(OLED_SCL), sda=machine.Pin(OLED_SDA))
    ## devices = oled_i2c.scan()	# for troubleshooting...
    ## print(devices)         # This will print the addresses of connected I2C devices.
    oled = ssd1306.SSD1306_I2C(OLED_WIDTH, OLED_HEIGHT, oled_i2c, addr = OLED_I2C_ADDR)
    # setup serial port.
    #soft_uart = machine.UART(MMWAVE_UART, baudrate=MMWAVE_BAUD, bits=8, parity=None, stop=1, tx=machine.Pin(MMWAVE_TX), rx=machine.Pin(MMWAVE_RX))
    #soft_uart = machine.UART(MMWAVE_UART, baudrate=MMWAVE_BAUD, tx=machine.Pin(MMWAVE_TX), rx=machine.Pin(MMWAVE_RX))    
    # set up wifi
    network_manager = NetworkManager(WIFI_REGION, status_handler=status_handler)
    uasyncio.get_event_loop().run_until_complete(network_manager.client(WIFI_SID, WIFI_PSK))   
    time.sleep(3)

    # Initialize Timer 0
    timer = machine.Timer()
    # Set the timer to periodic mode with a period of nx1000 milliseconds and attach the callback function
    timer.init(mode=machine.Timer.PERIODIC, period=PROXIMITY_CHECK, callback=check_sensors)

    while True:
        time.sleep(60)
        
except Exception as e:  
    print(e)
    sos(10)
    time.sleep(11)
    machine.reset()
    
except KeyboardInterrupt:
    timer.deinit()
    print("Timer stopped.")
finally:
    print("shutting down.")

