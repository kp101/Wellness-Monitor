# PANIC BUTTON

This project is to be used with an online MQTT broker, a Raspberry Pi, a level shifter, a remote button and a remote receiver. 
>[!CAUTION]
>This alert is not commercial grade, It is only for non-critical alerts. Medical emergencies such as heart attacks and stokes should be directed to local emergency departments.

## Purpose
Pressing a remote button will send a message to a designated mqtt online feed. This online feed will be further directed to a recipient via SMS, email. An external listener maybe able to audibily confirm the message has been received by the online MQTT broker. For futher detail, see listener project. 
This is only part of a remote monitoring scheme. An alternative solution where commercial service is not widely available, too expensive, lacking privacy or unreliable . The only thing this solution depends on is electrical power, internet service and a MQTT broker with message forwarding capabilities.

## Why a remote button? 
A remote button is small, light, does not require regular charging, can be carry around the neck, in the pocket or attached to a belt with a carabiner. No dialing, no fiddling with a phone.
   
