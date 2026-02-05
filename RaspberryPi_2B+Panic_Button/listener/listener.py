#!/bin/python3
import io
import sys
import base64
import asyncio
import aiomqtt
from aiomqtt_router import AiomqttRouter
import logging
from datetime import datetime
from zoneinfo import ZoneInfo
from time import sleep, time
import json
import sys
import yaml
from os import environ
environ['PYGAME_HIDE_SUPPORT_PROMPT'] = '1'
import warnings
warnings.filterwarnings("ignore", category=UserWarning)
import pygame
from picamera2 import Picamera2
from libcamera import Transform
from PIL import Image
import ssl

logger = logging.getLogger(__name__)

with open('kitchen.yml', mode='r') as f:
    config = yaml.safe_load(f)
    TZ = ZoneInfo(config['timezone'])
    STATION = config['station']
    MQTT_BROKER = config['mqtt']['broker']
    MQTT_PORT = config['mqtt']['port']
    MQTT_TOPIC_STATION_MUSIC = config['mqtt']['topic1']
    MQTT_TOPIC_PERIMETER = config['mqtt']['topic2']
    MQTT_TOPIC_ALARMS = config['mqtt']['topic3']
    MQTT_TOPIC_MOTUS = config['mqtt']['topic4']
    MQTT_TOPIC_STATION_CAMERA = config['mqtt']['topic5']
    DISTRESS_SIGNAL = config['mqtt']['distress_signal']
    CANCEL_SIGNAL = config['mqtt']['cancel_signal']
    UID = config['mqtt']['uid']
    PWD = config['mqtt']['pwd']
    DOORBELL_WAV = config['wav']['doorbell']
    SPEAKER_VOL = config['speaker']['volume']

tls_params = aiomqtt.TLSParameters(
    ca_certs=None,
    certfile=None,
    keyfile=None,
    cert_reqs=ssl.CERT_REQUIRED,
    tls_version=ssl.PROTOCOL_TLS,
    ciphers=None,
)

logger.info("setup leds and button completed...")

router = AiomqttRouter()
pygame.init()
pygame.mixer.init()
picam2 = Picamera2()
config = picam2.create_still_configuration(
                    main={"size":(640,480)},
                    transform=Transform(hflip=True,vflip=True))
picam2.configure(config)
picam2.start()

async def play(soundtrack, loops=0):
    logger.info(f"file:{soundtrack}")
    try:
        if pygame.mixer.music.get_busy() == True:
            pygame.mixer.music.stop
        fname = f"soundtracks/{soundtrack}"
        pygame.mixer.music.load(fname)
        pygame.mixer.music.set_volume(SPEAKER_VOL)
        pygame.mixer.music.play(loops=loops)
        logger.info(f"finnished {soundtrack}") 
    except Exception as e:
        logger.warning(f"An unexpected error : {e}")

async def broadcast(topic, msg):
    try:
        async with aiomqtt.Client(MQTT_BROKER, MQTT_PORT, \
                                  username=UID, password=PWD, \
                                  tls_params=tls_params) as client:
            # qos -> exactly once, no repeat.
            await client.publish(topic, payload=msg, qos=2)
            logging.info(f"Proximity {msg}.")

    except aiomqtt.MqttError:
        logging.info("MqttError")
    except Exception as e:
        logging.info(f"An unexpected error : {e}")
    
@router.subscribe(MQTT_TOPIC_PERIMETER)
async def handle_perimeter(message):
    msg = message.payload.decode('utf-8')
    logging.debug(f"{MQTT_TOPIC_PERIMETER}:{msg}.")
    try:
        jmsg = json.loads(msg)
        if "station" in jmsg and "status" in jmsg:
            if jmsg['station']=='frontdoor' and jmsg['status']=='doorbell':
                await play(DOORBELL_WAV)
    except json.JSONDecodeError:
        logging.debug(f"not valid json")
    except Exception as e:
        logging.info(f"An unexpected error : {e}")
    
@router.subscribe(MQTT_TOPIC_MOTUS)
async def handle_motus(message):
    msg = message.payload.decode('utf-8')
    # this feed is always non json format.
    logging.debug(f"{MQTT_TOPIC_MOTUS}:{msg}.")
    if msg.lower()=='snapshot': 
        try:
            logging.info(f"{STATION}: snapshot ")

            picam2.capture_file("test.jpg", format="jpeg")
            image = Image.open('test.jpg', 'r')
            optim_stream = io.BytesIO()
            image.save(optim_stream, format='jpeg', optimize=True)
            optim_stream.seek(0)
            value = base64.b64encode(optim_stream.read())
            await broadcast(MQTT_TOPIC_STATION_CAMERA, value )
                
        except Exception as e:
           logging.info(f"An unexpected error : {e}")

@router.subscribe(MQTT_TOPIC_STATION_MUSIC)
async def handle_kitchen(message):
    msg = message.payload.decode('utf-8')
    # this feed is always non json format.
    logging.debug(f"{MQTT_TOPIC_STATION_MUSIC}:{msg}.")
    try:
        if msg.lower()=="terminate":
            pygame.mixer.quit()
            logging.info(f"{STATION}: terminating")
            sys.exit()
        elif msg.lower()=="stop":
            logging.info(f"{STATION}: stopping")
            pygame.mixer.music.stop()
        else:
            logging.info(f"{STATION}: playing {msg}")
            await play(msg)
    except Exception as e:
        logging.info(f"An unexpected error : {e}")

@router.subscribe(MQTT_TOPIC_ALARMS)
async def handle_alarms(message):
    msg = message.payload.decode('utf-8')
    # this feed is always non json format.
    logging.debug(f"{MQTT_TOPIC_ALARMS}:{msg}.")
    if msg.lower() == DISTRESS_SIGNAL:
        await play('sos_hydrargyrum.ogg')
        await asyncio.sleep(0.5)
        await play('sos_hydrargyrum.ogg')
        await asyncio.sleep(2)
        await play('alarm_detected.wav')
        await asyncio.sleep(2)
        await play('alarm_detected.wav')
        await asyncio.sleep(2)
        await play('sos_hydrargyrum.ogg')
        await asyncio.sleep(0.5)
        await play('sos_hydrargyrum.ogg')
    elif msg.lower() == CANCEL_SIGNAL:
        await play('chimes.mp3')
        await asyncio.sleep(5)
        await play('alarm_cancelled.wav')
        await asyncio.sleep(5)
        await play('chimes.mp3')
        await asyncio.sleep(5)
        await play('alarm_cancelled.wav')
    elif msg.lower() == 'eta60':
        await play('tritone-chime.mp3')
        await asyncio.sleep(3)
        await play('alarm_ack.wav')
        await asyncio.sleep(2)
        await play('eta60.wav')
    elif msg.lower() == 'eta30':
        await play('tritone-chime.mp3')
        await asyncio.sleep(3)
        await play('alarm_ack.wav')
        await asyncio.sleep(2)
        await play('eta30.wav')
    elif msg.lower() == 'eta15':
        await play('tritone-chime.mp3')
        await asyncio.sleep(3)
        await play('alarm_ack.wav')
        await asyncio.sleep(2)
        await play('eta15.wav')

async def waiter():
    async with aiomqtt.Client(MQTT_BROKER,port=MQTT_PORT, \
                              username=UID, password=PWD, \
                              tls_params=tls_params ) as client:
        logging.info(f"mqtt connected...")
        while True:
            try:
                await router(client)
                await asyncio.sleep(0.5)

            except aiomqtt.MqttError:
                logging.info(f"mqtt error, reconnecting in 2 secs...")
                await asyncio.sleep(2)
            except Exception as e:
                logging.info(f"An unexpected error : {e}")

async def main():
    """
    Main function to run the asyncio event loop and serial reading.
    """
    logging.basicConfig(
        #level=logging.INFO,
        level=logging.DEBUG,
        format="%(asctime)s-%(relativeCreated)6d %(threadName)s %(message)s",
        filename='kitchen.log'
    )

    try:
        logger.info("started speaker service...")
        async with asyncio.TaskGroup() as tg:
                tg.create_task(waiter())

    #except* TerminateTaskGroup:
        #logging.info( f"task group terminated.")
    except KeyboardInterrupt:
        pygame.mixer.quit()
        logging.info( f"keyboard interrupt")
    except Exception as e:
        logging.debug( f"An unexpected error occurred: {e}")
    finally:
        pygame.quit()
        picam2.stop()

if __name__ == "__main__":
    asyncio.run(main())

