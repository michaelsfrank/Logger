CC = arm-linux-gnueabihf-gcc
#CC	= gcc
CFLAGS	= -g -gdwarf-2 -Ofast -mfpu=vfp -mfloat-abi=hard -march=armv6zk -mtune=arm1176jzf-s
#CFLAGS	= -g -gdwarf-2 -Ofast -mfpu=vfp -mfloat-abi=hard -march=armv6zk -mtune=arm1176jzf-s -Wall -pedantic
CPPFLAGS = -Iinclude
LDFLAGS  = -L.
LDLIBS   = -lnrf24

LIBNRF24 = libnrf24

VERSION=$(shell cat VERSION)
DESTDIR=/usr
PREFIX=/local
STATIC=$(LIBNRF24).a
DYNAMIC=$(LIBNRF24).so.$(VERSION)
#DYNAMIC=libnrf24.so.1.0
SRC	=	/src/gpio.c	/src/spi.c /src/rf24.c src/bmp180.c
HEADERS =	include/gpio.h	include/spi.h include/rf24.h include/bmp180.h
OBJ	=	$(SRC:.c=.o)

#all: lib examples
#all: $(LIBNRF24) dump Logger
all: $(LIBNRF24) bmp180 dump Logger
#all: $(LIBNRF24) dump Logger bmp180

#make shared library
bmp180:	src/bmp180.o
	$(CC) $(CPPFLAGS) -o libbmp180.so -shared -fPIC $(CFLAGS) src/bmp180.o

$(LIBNRF24): src/gpio.o src/spi.o src/rf24.o
	$(CC) $(CPPFLAGS) -o $(LIBNRF24).so.$(VERSION) -shared -fPIC $(CFLAGS) src/gpio.o src/spi.o src/rf24.o
# using this OBJ variable doesnt work
#$(LIBNRF24): $(OBJ)
#	$(CC) $(CPPFLAGS) -o $(LIBNRF24).so -shared -fPIC $(CFLAGS) $(OBJ)

#install:	$(LIBNRF24).so.$(VERSION)
#install:	$(DYNAMIC)
#install:	libnrf24.so.1.0


.PHONY:	install
install:	$(DYNAMIC)
	@echo "[Install Headers]"
	@install -m 0755 -d						$(DESTDIR)$(PREFIX)/include
	@install -m 0644 $(HEADERS)					$(DESTDIR)$(PREFIX)/include
	@echo "[Install Dynamic Lib]"
	@install -m 0755 -d						$(DESTDIR)$(PREFIX)/lib
	@install -m 0755 $(LIBNRF24).so.$(VERSION)			$(DESTDIR)$(PREFIX)/lib/$(LIBNRF24).so.$(VERSION)
	@ln -sf $(DESTDIR)$(PREFIX)/lib/$(LIBNRF24).so.$(VERSION)	$(DESTDIR)/lib/$(LIBNRF24).so
	@install -m 0755 libbmp180.so			$(DESTDIR)$(PREFIX)/lib/libbmp180.so
	@ln -sf $(DESTDIR)$(PREFIX)/lib/libbmp180.so	$(DESTDIR)/lib/libbmp180.so
	@ldconfig


	
#examples: pong_irq pong_curl dump

#pong_irq: examples/pong_irq.o
#	$(CC) $(CPPFLAGS) $(LDFLAGS) $(LDLIBS) -o pong_irq $(CFLAGS) -lnrf24 examples/pong_irq.o

#pong_curl: examples/pong_curl.o
#	$(CC) $(CPPFLAGS) $(LDFLAGS) $(LDLIBS) -o pong_curl $(CFLAGS) -lnrf24 -lcurl examples/pong_curl.o



# make according to comments in bmp180dev3.c
#bmp180: src/bmp180dev3.o
#	gcc $(CPPFLAGS) -Wall -o bmp180dev3 src/bmp180dev3.c -lm

# make according to Logger and dump
#bmp180: src/bmp180dev3.o
#	$(CC) $(CPPFLAGS) $(LDFLAGS) $(LDLIBS) -o bmp180dev3 $(CFLAGS) src/bmp180dev3.c -lm

dump: examples/dump.o
	$(CC) $(CPPFLAGS) $(LDFLAGS) $(LDLIBS) -o dump $(CFLAGS) -lnrf24 examples/dump.o
#	$(CC) $(CPPFLAGS) -L. -lnrf24 -L/usr/local/lib -o dump $(CFLAGS) examples/dump.o

# still being made with local nrf24 library... move nrf after -L/usr/local/lib
#Logger: examples/Logger.o
#	$(CC) $(CPPFLAGS) -L. -lnrf24 -L/usr/local/lib -lpaho-mqtt3c -lwiringPi -lwiringPiDev -lpthread -lm -o Logger $(CFLAGS) examples/Logger.o

# Logger with shared library
Logger: examples/Logger.o
	$(CC) $(CPPFLAGS) -L/usr/local/lib -lbmp180 -lnrf24 -lpaho-mqtt3c -lwiringPi -lwiringPiDev -lpthread -lm -o Logger $(CFLAGS) examples/Logger.o

	
# Logger previous
#	$(CC) $(CPPFLAGS) -L/usr/local/lib -lnrf24 -lwiringPi -lwiringPiDev -lpthread -lm -o Logger $(CFLAGS) examples/Logger.o
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
