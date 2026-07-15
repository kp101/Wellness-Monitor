proximity: proximity.c proximity.h mmwave.o mqtt.o toolbox.o
	gcc -o proximity proximity.c mqtt.o mmwave.o toolbox.o -lwiringPi -lpaho-mqtt3cs -lpthread -lm -latomic

toolbox.o: toolbox.c toolbox.h
	gcc -c toolbox.c -o toolbox.o

mqtt.o: mqtt.c mqtt.h
	gcc -c mqtt.c -o mqtt.o

mmwave.o: mmwave.c mmwave.h
	gcc -c mmwave.c -o mmwave.o
