This is a set of routines (client/server) to collect iot status as data point for monitoring over time. Skill level required is intermediate. Examination and modification of script is necessary to fit local configuration.

The IoT clients are tested for Raspberry Pi 2B/3B+/4B/Zero 2W. Others should work but not tested. Note:
  - dependencies: sudo apt install sysstat to get mpstat. awk, df, free should be already installed;
  - permission/privileages: the client shell script runs in user mode, is in BASH syntax and can be place anywhere in user home directory (modify location required);
  - this script assumes a boot and system partition as created by the RaspberryPi Foundation Partition Application. For other setup such as usb drive or ssd memory, modifications are required;
  - scheduling: crontab -e on a periodic basis for execution. No root privileages are required. Be sure to enter the final location of your script;
  - The script will gather data points and do a 'http get' to report on the server of your choice. Please edit the script to replace the server domain name/ip/port address;

On the server side, the server stack is node-red/influxdb3/grafana.  Import iot-flows.json into node-red for receiving the client data. You will have to supply the influxdb3 config.
It will map into the sensors table with fields/tags. Change this to your own preference.

In Node-Red, it should look like the following:
![node-red flow](images/nodered-iot_2026-01-28_15-38-30.png)

In Grafana, the exported json is iot-dashboard, the individual panels in the dashboard are shown in the following:

Memory usage over time. 

![grafana memory](images/grafana-memory_2026-01-30_11-09-14.png)

CPU status snapshot. 

![grafana cpu](images/grafana-cpu_2026-01-30_11-07-09.png)

Diskusage snapshot. 

![grafana cpu](images/grafana-diskusage_2026-01-30_11-07-48.png)

CPU throttling over time.

![grafana thorttled](images/grafana-throttled_2026-01-30_11-10-48.png)

CPU Temperature over time.

![grafana cpu temp](images/grafana-cpu-temp_2026-01-30_11-11-24.png)

CPU Voltage over time.

![grafana voltages](images/grafana-cpu-voltage_2026-01-30_11-12-57.png)

Import the following Grafana dashboard to generate the graphics above:
  iot-dashboard.json

The memory and throttled history were presented along a timeline so that maintainers can go back to see if there are major memory leaks/issues or intermittant throttled due to insufficient voltages.
The memory partition is what's used, free and occupied by cache.

