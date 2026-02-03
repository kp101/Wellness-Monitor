This part of documentation regards the role and programing of Adafruitio 'Actions'.

<b>Creating SMS Text/Email Notification</b>

When the 'alarms' feed received any data, a SMS text message and Email will be send.

![sending text](images/adafruitio-alarms_2026-02-02_23-26-33.png)

<b>Redirecting a json payload based on a value pair</b>
When the a json payload is send, this Action will examine the range value. if the range is > 0, it then extracts the station name and put it into the 'motus' feed. 

![sending text](images/adafruitio-selective-redirect_2026-02-02_23-13-46.png)
