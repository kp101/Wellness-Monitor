# Wellness Monitor Services for a Raspberry Pi

This collection of services are divided into subprojects, each can be independently implemented. Although there are duplicated modules, the subprojects are not mutually dependent. These has been tested with Pi Zero W/2W,2B,3B/3B+. 

1. bme68x is an enviornmental monitor, it is for collecting temperature/pressure/humidity and voc and post them into an online MQTT broker;
2. listener is subscriber service. It will take corresponding action based on the arriving payload from a particular topic. e.g. Take a picture, sound an alarm, display environmental data or flash a message of "DOORBELL" (for the hearing impaired);
3. panicbutton is alarm trigger. Sending a message to the online MQTT broker when a remote button is pressed;
4. ![proximity](https://github.com/kp101/Wellness-Monitor/tree/120a0db81271f26085dfffee569b2e1cec66d0f6/RaspberryPi/proximity) is a activity sensor. It will send a message to the MQTT broker if someone is present in the room. It can be fooled by a sleeping person or pets.
