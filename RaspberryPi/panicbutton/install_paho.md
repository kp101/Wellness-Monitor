# INSTALL paho.mqtt.c

references : [paho.mqtt.c](https://github.com/eclipse-paho/paho.mqtt.c)

Add dependencies:
```
sudo apt update
sudo apt install cmake libssl-dev
```
Clone library:
```
git clone https://github.com/eclipse/paho.mqtt.c.git
```
build and install library
```
cd paho.mqtt.c
cmake -Bbuild -H. -DPAHO_WITH_SSL=ON
sudo cmake --build build/ --target install
```
update loader cache:
```
sudo ldconfig
```

> [!NOTE]
> for synchronous communication: add -l paho-mqtt3c in your Makefile\
> for ssl/tls support: add -l paho-mqtt3cs in your Makefile\
> include the library in your code.\
> #include "MQTTClient.h 
