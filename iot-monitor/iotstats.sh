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
kernel="kernel=$(uname -r)"
##
## get memory used/free/cache
##
memuse="memuse=$(free --mega -L |  awk -F' ' '{print $6}')"
cacheuse="cacheuse=$(free --mega -L |  awk -F' ' '{print $4}')"
memfree="memfree=$(free --mega -L |  awk -F' ' '{print $8}')"
##
## sd card /dev/mmcblk0p1 (boot drive) and /dev/mmcblk0p2 (system drive)
## alternatively, /dev/sda1 (boot drive) and /dev/sda2 (system drive)
##
diskspace="diskspace=$(df -H | awk -F' ' '/dev\/mmcblk0p2/ {print $2}')"
diskused="diskused=$(df -H | awk -F' ' '/dev\/mmcblk0p2/ {print $3}')"
diskavail="diskavail=$(df -H | awk -F' ' '/dev\/mmcblk0p2/ {print $4}')"
bootspace="bootspace=$(df -H | awk -F' ' '/dev\/mmcblk0p1/ {print $2}')"
bootused="bootused=$(df -H | awk -F' ' '/dev\/mmcblk0p1/ {print $3}')"
bootavail="bootavail=$(df -H | awk -F' ' '/dev\/mmcblk0p1/ {print $4}')"
##
## use mpstat -P ALL to get general stats on cpu
##
cpuusr="cpuusr=$(mpstat -P ALL | awk -F' ' '/all/ {print $3}')"
cpunice="cpunice=$(mpstat -P ALL | awk -F' ' '/all/ {print $4}')"
cpusys="cpusys=$(mpstat -P ALL | awk -F' ' '/all/ {print $5}')"
cpuiowait="cpuiowait=$(mpstat -P ALL | awk -F' ' '/all/ {print $6}')"
cpuirq="cpuirq=$(mpstat -P ALL | awk -F' ' '/all/ {print $7}')"
cpusoft="cpusoft=$(mpstat -P ALL | awk -F' ' '/all/ {print $8}')"
cpusteal="cpusteal=$(mpstat -P ALL | awk -F' ' '/all/ {print $9}')"
cpuguest="cpuguest=$(mpstat -P ALL | awk -F' ' '/all/ {print $10}')"
cpugnice="cpugnice=$(mpstat -P ALL | awk -F' ' '/all/ {print $11}')"
cpuidle="cpuidle=$(mpstat -P ALL | awk -F' ' '/all/ {print $12}')"
##
## more can be added below using awk to extract data points.
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
echo "$cpuusr"
echo "$cpunice"
echo "$cpusys"
echo "$cpuiowait"
echo "$cpuirq"
echo "$cpusoft"
echo "$cpusteal"
echo "$cpugnice"
echo "$cpuidle"
echo "$kernel"
## 
## substitute your server ip address or domain name for data point collected.
##
curl -X GET "http://192.168.1.11:1880/iot?$temp&$throttled&$volts&$memuse&$cacheuse&$memfree&$diskspace&$diskused&$diskavail&$bootspace&$bootused&$bootavail&$cpuusr&$cpunice&$cpusys&$cpuiowait&$cpuirq&$cpusoft&$cpusteal&$cpugnice&$cpuidle&$dev&$station"
