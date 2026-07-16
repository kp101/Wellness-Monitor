# Wellness Monitor Services for a Raspberry Pi

![pizero2w](https://github.com/kp101/Wellness-Monitor/blob/939f9f752db4c561c6f8b7b60292c3298390b5f6/images/P7151850.jpg)


This collection of services for a generic Raspberry Pi are divided into subfolders. Each can be independently implemented. Although there are duplicated modules, the projects are not mutually dependent in the sense of compiling.

- ![bme68x](https://github.com/kp101/Wellness-Monitor/tree/a29afb64b6e6dd7e5c559bf10b444d1349f239b6/RaspberryPi/bme68x) is an enviornmental monitor, it is for collecting temperature/pressure/humidity and voc and post them into an MQTT broker;
- ![listener](https://github.com/kp101/Wellness-Monitor/tree/6c04aa0b26d657ff89895f3c0a0416408bf7a29a/RaspberryPi/listener) is subscriber service. It will take action based on the arriving topic/payload. e.g. Take a picture, sound an alarm, display data or flash a message like "DOORBELL" for the hearing impaired. ;
- ![panicbutton](https://github.com/kp101/Wellness-Monitor/tree/4b4e6bd011d1f751786c2286c66832148589f3ab/RaspberryPi/panicbutton) is alarm trigger. Sending a message to the online MQTT broker when a remote button is pressed;
- ![proximity](https://github.com/kp101/Wellness-Monitor/tree/120a0db81271f26085dfffee569b2e1cec66d0f6/RaspberryPi/proximity) is a activity sensor. It will send a message to the MQTT broker if someone is present in the room. Yes, it can be fooled by a sleeping person or pets.
   
>[!NOTE]
>These projects have been tested with Raspberry Pi ZeroW/2W,2B/3B/3B+.
