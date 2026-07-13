sensor: sensor.c sensor.h mqtt.o toolbox.o bme68x.o common.o bme68x_wrapper.o
	gcc -o sensor sensor.c mqtt.o toolbox.o bme68x.o common.o bme68x_wrapper.o -lwiringPi -lpaho-mqtt3cs -lpthread -lm -latomic

bme68x_wrapper.o: bme68x_wrapper.c bme68x_wrapper.h
	gcc -c bme68x_wrapper.c -o bme68x_wrapper.o

toolbox.o: toolbox.c toolbox.h
	gcc -c toolbox.c -o toolbox.o

common.o: common.c common.h bme68x.h
	gcc -c common.c -o common.o

bme68x.o: bme68x.h bme68x.c bme68x_defs.h
	gcc -c bme68x.c -o bme68x.o

mqtt.o: mqtt.c mqtt.h
	gcc -c mqtt.c -o mqtt.o
