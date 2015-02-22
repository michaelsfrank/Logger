CC = arm-linux-gnueabihf-gcc
#CC	= gcc
CFLAGS	= -g -gdwarf-2 -Ofast -mfpu=vfp -mfloat-abi=hard -march=armv6zk -mtune=arm1176jzf-s
#CFLAGS	= -g -gdwarf-2 -Ofast -mfpu=vfp -mfloat-abi=hard -march=armv6zk -mtune=arm1176jzf-s -Wall -pedantic
CPPFLAGS = -Iinclude
LDFLAGS  = -L.
LDLIBS   = -lnrf24

NAME     = libnrf24
TESTNAME = test

#all: lib examples
all: lib dump Logger

lib: src/gpio.o src/spi.o src/rf24.o
	$(CC) $(CPPFLAGS) -o $(NAME).so -shared -fPIC $(CFLAGS) src/gpio.o src/spi.o src/rf24.o

#examples: pong_irq pong_curl dump

#pong_irq: examples/pong_irq.o
#	$(CC) $(CPPFLAGS) $(LDFLAGS) $(LDLIBS) -o pong_irq $(CFLAGS) -lnrf24 examples/pong_irq.o

#pong_curl: examples/pong_curl.o
#	$(CC) $(CPPFLAGS) $(LDFLAGS) $(LDLIBS) -o pong_curl $(CFLAGS) -lnrf24 -lcurl examples/pong_curl.o

dump: examples/dump.o
	$(CC) $(CPPFLAGS) $(LDFLAGS) $(LDLIBS) -o dump $(CFLAGS) -lnrf24 examples/dump.o

Logger: examples/Logger.o
	$(CC) $(CPPFLAGS) -L. -lnrf24 -L/usr/local/lib -lwiringPi -lwiringPiDev -lpthread -lm -o Logger $(CFLAGS) examples/Logger.o
#	$(CC) $(CPPFLAGS) -L/usr/local/lib -lcgeneric_0.4.2 -lwiringPi -lwiringPiDev -lpthread -lm -L. -lnrf24 -o Logger $(CFLAGS) examples/Logger.o
	#$(CC) $(CPPFLAGS) -L. -lnrf24 -L/usr/local/lib -lcgeneric_0.4.2 -lwiringPi -lwiringPiDev -lpthread -lm -o Logger $(CFLAGS) examples/Logger.o
	#$(CC) $(CPPFLAGS) -L/usr/local/lib -lcgeneric_0.4.2 -lwiringPi -lwiringPiDev -lpthread -lm -o Logger $(CFLAGS) examples/Logger.o
	#$(CC) $(CPPFLAGS) /home/pi/c-generic-library/libs/libcgeneric_0.4.2.a -L/usr/local/lib -lcgeneric_0.4.2 -lwiringPi -lwiringPiDev -lpthread -lm -lnrf24 -o Logger $(CFLAGS) examples/Logger.o
	 
#ow_temp: examples/ow_temp.o
#	gcc -Wall -std=gnu99 ow_temp.c -o ow_temp
# need to add to library section above	

clean:
	rm -f *.so examples/*.o src/*.o pong_irq pong_curl dump w1

	
.PHONY: clean
