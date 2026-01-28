#!/bin/bash
##
## This is the client side data/status collection shell script. Tested in Raspberry Pi 2B/3B+/4B.
##
## schedule this with crontab -e the following for hourly update:
## 59 * * * * /home/pi/iotstats.sh
## 
## get temperature/throttled/volts/
##
temp=$(vcgencmd measure_temp)
throttled=$(vcgencmd get_throttled)
volts=$(vcgencmd measure_volts)
##
## set the device name url encoded. e.g. %20 is space.
dev="device=RaspberryPi%203B%2B"
station="station=kitchen"
##
## get memory used/free/cache
##
memuse="memuse=$(free --mega -L |  awk -F' ' '{print $6}')"
cacheuse="cacheuse=$(free --mega -L |  awk -F' ' '{print $4}')"
memfree="memfree=$(free --mega -L |  awk -F' ' '{print $8}')"
##
## find /dev/mmcblk0p1 (boot drive) and /dev/mmcblk0p2 (system drive)
## alternatively, /dev/sda1 (boot drive) and /dev/sda2 (system drive)
##
diskspace="diskspace=$(df -H | awk -F' ' '/dev\/mmcblk0p2/ {print $2}')"
diskused="diskused=$(df -H | awk -F' ' '/dev\/mmcblk0p2/ {print $3}')"
diskavail="diskavail=$(df -H | awk -F' ' '/dev\/mmcblk0p2/ {print $4}')"
bootspace="bootspace=$(df -H | awk -F' ' '/dev\/mmcblk0p1/ {print $2}')"
bootused="bootused=$(df -H | awk -F' ' '/dev\/mmcblk0p1/ {print $3}')"
bootavail="bootavail=$(df -H | awk -F' ' '/dev\/mmcblk0p1/ {print $4}')"
##
## The following echos are for debugging purposes and can be removed.
echo "$temp"
echo "$throttled"
echo "$volts"
echo "$memuse"
echo "$cacheuse"
echo "$memfree"
echo "$diskspace"
echo "$diskused"
echo "$diskavail"
echo "$bootspace"
echo "$bootused"
echo "$bootavail"
## 
## substitute your server ip address or domain name for data point collected.
##
curl -X GET "http://192.168.1.11:1880/iot?$temp&$throttled&$volts&$memuse&$cacheuse&$memfree&$diskspace&$diskused&$diskavail&$bootspace&$bootused&$bootavail&$dev&$station"
