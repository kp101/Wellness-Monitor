# INSTALLING WiringPi


Reference prefix: JIRA-
Target URL: https://jira.example.com/issue?query=<num>
Preview: JIRA-123 is converted to https://jira.example.com/issue?query=123

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
