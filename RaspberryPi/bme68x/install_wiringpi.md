# INSTALLING WiringPi


![WiringPi](https://github.com/WiringPi/WiringPi/blob/master/documentation/english/functions.md)

Fetch the source:
```
sudo apt install git
git clone https://github.com/WiringPi/WiringPi.git
cd WiringPi
```
Build the package:
```
./build debian
```

Install the package:
```
mv debian-template/wiringpi-3.x.deb .
sudo apt install wiringpi-3.x.deb
```
