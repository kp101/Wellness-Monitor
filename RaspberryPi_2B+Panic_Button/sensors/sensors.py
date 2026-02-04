#!/usr/bin/python3
import asyncio
import aiomqtt
import serial_asyncio
import logging
from time import time 
import yaml
import json
import RPi.GPIO as GPIO
import ssl

with open('sensors.yml', mode='r') as f:
    config = yaml.safe_load(f)
    MQTT_BROKER = config['mqtt']['broker']
    MQTT_PORT = config['mqtt']['port']
    MQTT_TOPIC_MOVEMENTS = config['mqtt']['topic1']
    MQTT_TOPIC_ALARM = config['mqtt']['topic2']
    DISTRESS_SIGNAL = config['mqtt']['distress_signal']
    CANCEL_SIGNAL = config['mqtt']['cancel_signal']
    STATION = config['mqtt']['station']
    DEVICE = config['mqtt']['device']
    UPDATE_INTERVAL = config['mqtt']['update_interval']
    HEARTBEAT_UPDATE = config['mqtt']['heartbeat_update']
    UID = config['mqtt']['uid']
    PWD = config['mqtt']['pwd']
    SERIAL_PORT = config['mmwave']['serial_port']
    BAUD_RATE = config['mmwave']['baud_rate']
    BUTTON_PIN = config['button']
    REPEAT_INTERVAL = config['repeat_interval']

tls_params = aiomqtt.TLSParameters(
    ca_certs=None,
    certfile=None,
    keyfile=None,
    cert_reqs=ssl.CERT_REQUIRED,
    tls_version=ssl.PROTOCOL_TLS,
    ciphers=None,
)

logger = logging.getLogger(__name__)

# setup panic button trigger pin.
GPIO.setwarnings(True)
GPIO.setmode(GPIO.BCM)   
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
logger.info("button setup completed...")

lock = asyncio.Lock()
# shared resource.
# proximity = 0  # range of mmwave, zero indicate infinity.

async def broadcast(topic, msg):
    try:
        async with lock:
            async with aiomqtt.Client(MQTT_BROKER, port=MQTT_PORT, \
                                  username=UID, password=PWD, \
                                  tls_params=tls_params) as client:
                # qos -> exactly once, no repeat.
                await client.publish(topic, payload=msg, qos=2) 
                logger.debug(f"Proximity {msg}.")

    except aiomqtt.MqttError:
        logger.info("MqttError")
    except Exception as e:
        logger.info(f"An unexpected error : {e}")

async def panic_button():
    """
    monitor remote control.
    """
    triggered = False
    start_time = time()
    try:
        while True:
            logger.info("panic button:", GPIO.input(BUTTON_PIN))
            logger.debug(f"GPIO.input(BUTTON_PIN): {GPIO.input(BUTTON_PIN)}")
            if GPIO.input(BUTTON_PIN)==GPIO.HIGH: # GPIO pin button pressed.
                if not triggered:
                    await broadcast(MQTT_TOPIC_ALARM, DISTRESS_SIGNAL)
                    logger.info("panic button:alarm broadcast")
                    triggered = True
                    start_time = time()
                    logger.debug(f"panic button triggered")
                elif time() - start_time > REPEAT_INTERVAL:  #reset and rebroadcast distress signal.
                    triggered = False
            elif triggered:
                await broadcast(MQTT_TOPIC_ALARM, CANCEL_SIGNAL)
                logger.info("panic button:cancel broadcast")
                triggered = False

            await asyncio.sleep(0.1)
    except Exception as e:
        logger.info(f"An unexpected error : {e}")

async def mmwave():
    """
    inform other IoT interested party to take action based on perimeter status. 
    """
    scan_count = 0

    try:
        while True:
            scan_count = (scan_count + 1) % HEARTBEAT_UPDATE
            range = await read_serial_data()
            if range > 0 or scan_count <= 0: 
                await broadcast(MQTT_TOPIC_MOVEMENTS, json.dumps({"value":
                    {"station": STATION, "range": range, "dev": DEVICE }}))
            logger.debug(f"logged : {range}")
            await asyncio.sleep(UPDATE_INTERVAL)
    except Exception as e:
        logger.info(f"An unexpected error : {e}")

async def read_serial_data():
    """
    Asynchronously reads data from a serial port and prints it.
    """
    start_time = time()
    range = 0
    try:
        reader, writer = await serial_asyncio.open_serial_connection(
            url=SERIAL_PORT, baudrate=BAUD_RATE
        )
        logger.debug(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud.")
        writer.write(b'FDFCFBFA0800120000006400000004030201')

        logger.info( "serial port opened, waiting and checking sensor...")
        while time() - start_time < 17.0:
            data = None
            data = await reader.readline()  # Read until a newline character
            logger.debug( "data {data}")
            if data:
                data = data.decode('utf-8', errors='ignore').strip()
                logger.debug( f"Received: {data}                ")
                data = data.replace("Range","").strip()
                data = data.replace("ON","").strip()
                data = data.replace("N","").strip()
                if data.isnumeric():
                   if int(data) > 0:
                       range = int(data)
                elif data.count('OFF') > 0:
                   #range = 0
                   pass
                else:
                   pass
            logger.debug( f"data:{data}")
            await asyncio.sleep(0.1)  # Small delay to prevent busy-waiting

    except ConnectionRefusedError:
        logger.debug( f"Connection refused. Serial device na or permissions denied.")
    except KeyboardInterrupt:
        logger.info( f"Keyboard interrupted")
    except Exception as e:
        logger.debug( f"An unexpected error occurred: {e}")
    finally:
        if 'writer' in locals() and writer:
            writer.close()
            await writer.wait_closed()
        logger.debug("Serial connection closed. ")
        return range


async def main():
    """
    Main function to run the asyncio event loop and serial reading.
    """
    logging.basicConfig(
        level=logging.INFO,
        #level=logging.DEBUG,
        format="%(asctime)s-%(relativeCreated)6d %(threadName)s %(message)s",
        filename='sensors.log'
    )


    try:
         async with asyncio.TaskGroup() as tg:
             tg.create_task(mmwave())
             tg.create_task(panic_button())
             #tg.create_task(bme688())

    #except TerminateTaskGroup:
    #    logger.info( f"task group terminated.")
    except KeyboardInterrupt:
        logger.info( f"keyboard interrupt")
    except Exception as e:
        logger.debug( f"An unexpected error occurred: {e}")
    finally:
        GPIO.cleanup()  # Clean up GPIO on exit

if __name__ == "__main__":
    asyncio.run(main())
