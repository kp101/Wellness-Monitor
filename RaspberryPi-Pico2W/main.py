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

prerequisites: The following requires micropython firmware image uf2 from pimoroni 
               which included the micopython breakout_bme68x module.
               This breakout_bme68x is a dependency library used for 
               environmental readings and it was generously provided by pimoroni.
               
reference: https://github.com/pimoroni/pimoroni-pico/tree/main/micropython/examples/breakout_bme68x
tested with:
    https://github.com/pimoroni/pimoroni-pico-rp2350/releases/tag/v1.26.1
    
version: Basic, 2.22
"""
import time
from machine import Pin, ADC, UART, Timer, lightsleep
from robust import MQTTClient
from network_manager import NetworkManager
import uasyncio
import utime
import binascii
import yaml
import ujson

led = Pin("LED", Pin.OUT)

def load_config(filename=None):
    path = filename or "config.yml"
   
    return yaml.load(path)
    
config = load_config()
wifi_sid = config['wifi']['ssid']
wifi_psk = config['wifi']['psk']
wifi_reg = config['wifi']['country']

mqtt_adr = config['mqtt']['broker']
mqtt_uid  = config['mqtt']['uid']
mqtt_pwd  = config['mqtt']['pwd']
mqtt_topic= config['mqtt']['topic']
feed_aggregate = config['mqtt']['aggregate']

update_interval = config['update_interval']
station = config['station']
device  = config['device']

tx_pin_number = config['tx_pin']
rx_pin_number = config['rx_pin']
uart          = config['uart']

tx_pin = machine.Pin(tx_pin_number)
rx_pin = machine.Pin(rx_pin_number)

scan_duration = config['mmwave_scan_duration']
reboot_countdown = config['reboot_cycle']
wifi_retry = config['wifi']['retry']

def send_hex_string(soft_uart, hex_string):
    hex_bytes = binascii.unhexlify(hex_string)
    soft_uart.write(hex_bytes)

def read_serial_data(soft_uart):
    start_time = time.ticks_ms()
    distance = 0
    
    while time.ticks_ms() - start_time < scan_duration :
        try:
            if soft_uart.any():
                data = soft_uart.readline()
                print("Received:", data)
                if data:
                    data = data.decode('utf-8')
                    data = data.replace("Range","").strip()
                    data = data.replace("ON","").strip()
                    data = data.replace("N","").strip()

                    if data.isdigit():
                        distance = int(data)
                    elif data.count('OFF') > 0:
                        distance = 0
                    else:
                        pass
                 
        except (TypeError, UnicodeError) as e:
            print(f"Error: {e}")
            pass
        utime.sleep_ms(100)
        
    return distance
    
def mmwave():
    hex_to_send = "FDFCFBFA0800120000006400000004030201"
    
    soft_uart = machine.UART(uart, baudrate=115200, tx=tx_pin, rx=rx_pin)         
    send_hex_string(soft_uart, hex_to_send)
    print("initialized mmWave.")
    
    return read_serial_data(soft_uart)

def sos(repeat):
    if OLED is "ssd1306":    
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

def status_handler(mode, status, ip):  # 
    global net_retry
    
    print("Connecting...")
    led.toggle()   
    if status is not None:
        if status:
            status_text = "Wifi Connection successful!"
            net_retry = wifi_retry
        else:
            status_text = "Wifi Connection failed!"
            net_retry = net_retry - 1
            if net_retry <= 0:
                machine.reset()
        print(status_text)
    print("IP: {}".format(ip))
    time.sleep(1)
    led.toggle()
        

def check_sensors(timer):   
    global reboot_countdown
    
    try:
        
        distance = mmwave()
        print("got range:", distance )
        
        if distance > 0:
            # set up wifi
            network_manager = NetworkManager(wifi_reg, status_handler=status_handler)   
            # connect to wifi
            uasyncio.get_event_loop().run_until_complete(network_manager.client(wifi_sid, wifi_psk))

            # sets up MQTT
            mqtt_client = MQTTClient(client_id=mqtt_uid, server=mqtt_adr, user=mqtt_uid, password=mqtt_pwd, keepalive=30)
            time.sleep(0.5)
            
            mqtt_client.connect()
            if feed_aggregate :
                payload = {"value": {"range": distance, "dev": device, "station": station }}
                print(payload)
                mqtt_client.publish(topic=mqtt_topic, msg=ujson.dumps(payload))
                
            else:
                mqtt_client.publish(topic=mqtt_topic, msg=station)

            mqtt_client.disconnect()
        
    except Exception as e:  # noqa: BLE001
        print(e)
  
try:
    
    #Initialize Timer 
    timer = machine.Timer()
    
    # Set the timer to periodic mode with a period of nx1000 milliseconds and attach the callback function
    timer.init(mode=machine.Timer.PERIODIC, period=update_interval, callback=check_sensors)
    
    while True:
        lightsleep(update_interval)

except KeyboardInterrupt:
    timer.deinit()
    print("Timer stopped.")
finally:
    print("shutting down.")

