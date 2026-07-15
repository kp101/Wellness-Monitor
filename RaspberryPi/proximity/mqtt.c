/*
 * references: https://github.com/eclipse-paho/paho.mqtt.c
 *
 */
#include <ctype.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include "mqtt.h"

#include "MQTTClient.h"
#include "pubsub_opts.h"

int mqtt_publish( char *topic, char *payload, int len ) {
   int rc = 1;
   int qos = 1; 
   int retained = 0;
   MQTTClient client;

    // server url. i.e. "ssl://name:8883". For adafruitio, the MQTT_UID should be left blank or 
    // provide an uuid. otherwise conflicts with other devices on your account will result in 
    // unpredictable disconnects. Don't ask me how I know.
    // MQTTClient_create(&client, MQTT_BROKER, MQTT_UID, MQTTCLIENT_PERSISTENCE_NONE, NULL);
    MQTTClient_create(&client, MQTT_BROKER, "", MQTTCLIENT_PERSISTENCE_NONE, NULL);

    MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;
    MQTTClient_SSLOptions ssl_opts = MQTTClient_SSLOptions_initializer;
    ssl_opts.enableServerCertAuth = 0;

    // declare values for ssl options, here we use only the ones necessary for TLS, but you can optionally define a lot more
    // look here for an example: https://github.com/eclipse/paho.mqtt.c/blob/master/src/samples/paho_c_sub.c
    ssl_opts.verify = 1;
    ssl_opts.CApath = NULL;
    ssl_opts.keyStore = NULL;
    ssl_opts.trustStore = NULL;
    ssl_opts.privateKey = NULL;
    ssl_opts.privateKeyPassword = NULL;
    ssl_opts.enabledCipherSuites = NULL;

    // use TLS for a secure connection, "ssl_opts" includes TLS
    conn_opts.ssl = &ssl_opts;
    conn_opts.keepAliveInterval = 10;
    conn_opts.cleansession = 1;
    // use your credentials that you created with the cluster
    conn_opts.username = MQTT_UID;
    conn_opts.password = MQTT_PWD;
    
    int j = MQTTClient_connect(client, &conn_opts);
    //printf("is connected %d \n", j);

   // payload the content of your message, send and forget. non-critical data.
   MQTTClient_deliveryToken dt;
   MQTTClient_publish(client, topic, len, payload, qos, retained, &dt);
   MQTTClient_disconnect(client, timeout);
   MQTTClient_destroy(&client);

   return rc;
}
