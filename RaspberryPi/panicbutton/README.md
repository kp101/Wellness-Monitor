# RASPBERRY PI 2B + PANIC BUTTON

This project is to be used with an online MQTT broker, a Raspberry Pi, a level shifter, a remote button and a remote receiver. 
>[!CAUTION]
>This alarm is not commercial grade, It is only for non-critical alerts. Medical emergencies such as heart attacks and stokes should be directed to local emergency services without delay.

## Purpose
Pressing a remote button will send a message to a designated mqtt online feed. This online feed will automatically trigger a SMS, email to designated receipient(s). An external listener maybe added to audibily confirm the message was received by the online MQTT broker. For futher detail, see listener project. 

This is a part of a remote monitoring scheme. It's an alternative to commercial service where not available, too expensive, lacking privacy, not customizable or unreliable. The only thing this solution depends on is relaible electrical power, internet service and a MQTT broker with message forwarding capabilities.

Possible candidates: seniors without heart or stoke risks, falling and unable to get up but no fracture risks. In some remote areas, attentive neighours can be put on alert and reach the candidate faster than emergency response teams.

## Why a remote button? 
A remote button is small, light, does not require regular charging, can be carry around the neck, in the pocket or attached to a belt with a carabiner. No dialing, no fiddling with a phone.
   
## Required Hardware
- A remote button capable of transmitting with a matching receiver capable of receiving signal within a household boundary. This is a Simple RF T4 Receiver - 315MHz Toggle Type.
  
![remote](images/1095-small.jpg)
![remote](images/1097-small.jpg)

>[!IMPORTANT]
>To ensure reliable response, it is important to make sure sufficient voltage is fed to the receiver and the level shifter.

- The following level shifter was used for interfacing with the Pi's 3.3v pins to the 5v pins from the button receiver. Other solutions are possible. e.g. This particular level shifter is a bit slower, other faster products are available.
  
![levelshifter](images/757-small.jpg)

