sudo apt update
## dependencies
sudo apt install cmake libssl-dev
## clone library
git clone https://github.com/eclipse/paho.mqtt.c.git
## build and install library
cd paho.mqtt.c
cmake -Bbuild -H. -DPAHO_WITH_SSL=ON
sudo cmake --build build/ --target install
## update loader cache
sudo ldconfig

## note 
echo "for synchronous communication: add -l paho-mqtt3c your Makefile"
echo "for ssl/tls support: add -l paho-mqtt3cs to your Makefile"
echo "#include "MQTTClient.h in your code." 
