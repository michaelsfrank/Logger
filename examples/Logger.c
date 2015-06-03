/*
 * Logger.c:
 *	Gather sensor data, forward
 *
 *
 * Copyright (c) 2015 Michael Frank
 ***********************************************************************
 * This file is part of wiringPi:
 *	https://projects.drogon.net/raspberry-pi/wiringpi/
 *
 *    wiringPi is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    wiringPi is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public License
 *    along with wiringPi.  If not, see <http://www.gnu.org/licenses/>.
 ***********************************************************************

*/

#if HOST == Pi1
	#define WIFI
	#define ADDRESS     "tcp://192.168.1.214:1883"		// MQTT
	#define CLIENTID    "Pi1"							// MQTT
	//#define ONE_WIRE

#elif HOST == Pi2
	#define WIFI
	#define NRF
	#define ADDRESS     "tcp://192.168.1.214:1883"		// MQTT
	#define CLIENTID    "Pi2"							// MQTT
	#define ONE_WIRE
	

#elif HOST == Pi3
	#define WIFI
	#define NRF
	#define ADDRESS     "tcp://192.168.1.214:1883"		// MQTT
	#define CLIENTID    "Pi3"							// MQTT
	#define ONE_WIRE
	#define BMP180
	#define CC2
	#define RELAYS
	#define CONTACTS
	
#elif HOST == Pi4
	#define WIFI
	#define NRF
	#define ADDRESS     "tcp://localhost:1883"			// MQTT
	#define CLIENTID    "Pi4"							// MQTT
	#define ONE_WIRE
	#define BMP180
	#define CC2
	#define RELAYS
	#define CONTACTS
	
#else
	#define WIFI
	#define NRF	
	#define ADDRESS     "tcp://192.168.1.214:1883"		// MQTT
	#define CLIENTID    "Default"							// MQTT
	//#define ONE_WIRE
	
#endif


#ifdef BMP180
	#define I2C
#elif defined CC2
	#define I2C
#endif

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <wiringPi.h> // wiringPi
#include <wiringPiI2C.h> // wiringPi
#include <MQTTClient.h> // MQTT
//#include <MQTTClientPersistence.h> // MQTT (future)

#ifdef ONE_WIRE
#include <dirent.h>		// ds18b20 command line read
#include <fcntl.h>		// ds18b20 command line read????
#endif

#include <inttypes.h>	// MF copied from dump.c 7-Feb-2015
#include <unistd.h>		// MF copied from dump.c 7-Feb-2015

#include <time.h>		// timing
#include <sys/types.h> 

#ifdef WIFI
	#include <sys/ioctl.h>		// WIFI SIGNAL INFO
	#include <sys/stat.h>
	#include <sys/socket.h>		// WIFI SIGNAL INFO
	#include <linux/if.h>		// WIFI SIGNAL INFO
	//#include <linux/if_tun.h>	// WIFI SIGNAL INFO
	#include <linux/wireless.h>	// WIFI SIGNAL INFO
#endif


#ifdef NRF
	#include <rf24.h>		// nRF
	#include <nRF24L01.h>	// nRF
	#include <gpio.h>		// nRF
	#define PAYLOAD_SIZE 8	// nRF
#endif

#ifdef BMP180
	#include <bmp180.h>
#endif

//#include "queue.h"					// c-generic-library
//#include "error_macros.h"				// c-generic-library
//#include "libcgeneric/libcgeneric.h"	// c-generic-library


#define LOCAL_DATA_INTERVAL 2	// Logger - main loop

#ifdef RELAYS
	#define STATUS1 26
	#define STATUS2 27
	#define STATUS3 28
	#define STATUS4 29
	#define RELAY1 21
	#define RELAY2 22
	#define RELAY3 23
	#define RELAY4 24
	#define RELAY_ENABLE 25
#endif	

#define TOPIC       "MQTT Examples"					// MQTT
#define PAYLOAD     "Hello World!"					// MQTT
#define QOS         1								// MQTT
#define TIMEOUT     10000L							// MQTT

// MF macros
#define _BV(x) (1 << (x))	
#define _BN(x, n) ( ( (unsigned char *)(&(x)) )[(n)] )


#ifdef WIFI
/*
// WIFI SIGNAL INFO
// http://blog.ajhodges.com/2011/10/using-ioctl-to-gather-wifi-information.html
struct signalInfo {
    char mac[18];
    char ssid[33];
    int bitrate;
    int level;
};
typedef struct signalInfo signalInfo_t;  // MF queue
signalInfo_t signalInfoData;	  // MF queue
*/


// WIFI SIGNAL INFO - TRY #2
#define IW_INTERFACE "wlan0"
extern int errno;
//struct iwreq wreq;

#endif

/*
int32_t intcmp(const void *, const void *,size_t);	// c-generic-library
void  print(const void *);							// c-generic-library
void  ckfree(void *);								// c-generic-library
void *ckalloc(size_t);								// c-generic-library
*/

// globalCounter:
//	Global variable to count interrupts
//	Should be declared volatile to make sure the compiler doesn't cache it.
static volatile int globalCounter [8] ;

// MQTT
volatile MQTTClient_deliveryToken deliveredtoken; 

// RELAYS
//volatile struct timeval relay_command_time; 


// MF queue - REQUIRED FOR ONE WIRE AND NRF TEMP READING
struct data_queue {
  uint8_t size;
  uint8_t d[32];
  uint8_t next;
};					
typedef struct data_queue data_queue_t;  // MF queue
data_queue_t data;	  // MF queue


#ifdef NRF
rf24_t radio_global;	// nRF
#endif

/*
 * myInterrupt:
 *********************************************************************************
 */

void myInterrupt0 (void) { ++globalCounter [0] ; }
void myInterrupt1 (void) { ++globalCounter [1] ; }
void myInterrupt2 (void) { ++globalCounter [2] ; }	  // nRF
void myInterrupt3 (void) { ++globalCounter [3] ; }
void myInterrupt4 (void) { ++globalCounter [4] ; }
void myInterrupt5 (void) { ++globalCounter [5] ; }
void myInterrupt6 (void) { ++globalCounter [6] ; }
//void myInterrupt7 (void) { ++globalCounter [7] ; }

#ifdef CC2
// ChipCap2 global variables
uint8_t  status;
float humidity;
float temperatureC;
float temperatureF;
#endif

#ifdef ONE_WIRE
// struct to hold ds18b20 data for linked list
// 1-Wire driver stores info in file for device as text
struct ds18b20 {
	char devPath[128];
	char devID[16];
	double tempC;
	//char tempData[6];
	struct ds18b20 *next;
};



// Find connected 1-wire devices. 1-wire driver creates entries for each device
// in /sys/bus/w1/devices on the Raspberry Pi. Create linked list.
int8_t findDevices(struct ds18b20 *d) {
	DIR *dir;
	struct dirent *dirent;
	struct ds18b20 *newDev;
	char path[] = "/sys/bus/w1/devices";
	int8_t i = 0;

	dir = opendir(path);

	if (dir != NULL)
	{
		while ((dirent = readdir(dir))) {
			printf("[1-wire] FindDevices() dirent=readdir(dir)=%s\n",readdir);
			//printf("[1-wire] FindDevices()   dirent->d_type=%d   dirent->d_name=%s\n",dirent->d_type, dirent->d_name);
			// 1-wire devices are links beginning with 28-
			if (dirent->d_type == DT_LNK && strstr(dirent->d_name, "28-") != NULL) {
				//printf("[1-wire] dirent->d_type=DT_LINK   dirent->d_name contains 28-\n",dirent->d_type, dirent->d_name);
				newDev = malloc( sizeof(struct ds18b20) );
				//printf("[1-wire] newDev Address = %d\n",&newDev);
				strcpy(newDev->devID, dirent->d_name);
				//printf("[1-wire] newDev->devID = %s\n",&newDev->devID);
				// Assemble path to OneWire device
				sprintf(newDev->devPath, "%s/%s/w1_slave", path, newDev->devID);
				i++;
				newDev->next = 0;
				
				//printf("[1wire] FindDevice(*d=devNode)  newDev: %s    Temp: %.3f C    Add: 0x %x    NextAdd: 0x %x    \n", newDev->devID, newDev->tempC, newDev, newDev->next);
								
				d->next = newDev; // don't see the need for this line, the next line overwrites it?
				//printf("[1wire] FindDevice(*d=devNode)      *d: %s    Temp: %.3f C    Add: 0x %x    NextAdd: 0x %x    \n", d->devID, d->tempC, d, d->next);
				
				d = d->next;
				//printf("[1wire] FindDevice(*d=devNode)      *d: %s    Temp: %.3f C    Add: 0x %x    NextAdd: 0x %x    \n", d->devID, d->tempC, d, d->next);
			}
		}
		
		(void) closedir(dir);
	}
	else {
		perror ("Couldn't open the w1 devices directory");
		return 1;
	}
	return i;
}


// Cycle through linked list of devices & take readings.
// Print out results & store readings in DB.
int8_t readTemp(struct ds18b20 *d) {
	char buffer_char1[256], buffer_char2[256];
	ssize_t numRead;
	int fd;
	
	while(d->next != NULL){
		//printf("while loop\n");
		//printf("[1wire] readTemp(*d=rootNode)      *d: %s    Temp: %.3f C    Add: 0x %x    NextAdd: 0x %x    \n", d->devID, d->tempC, d, d->next);
		d = d->next;
		//printf("[1wire] readTemp(*d=rootNode)      *d: %s    Temp: %.3f C    Add: 0x %x    NextAdd: 0x %x    \n", d->devID, d->tempC, d, d->next);
		
		fd = open(d->devPath, O_RDONLY);
		
		if(fd == -1) {
			perror ("Couldn't open the w1 device.\n");
			return 1;
		}
		// 1-wire driver stores data in file as long block of text
		// Store file contents in buf & look for t= that marks start of temp.
		
		while((numRead = read(fd, buffer_char1, 256)) > 0) {
			//strncpy(d->tempData, strstr(buf, "t=") + 2, 5);
			strncpy(buffer_char2, strstr(buffer_char1, "t=") + 2, 5);
			d->tempC = strtod(buffer_char2, NULL) / 1000;
			// Driver stores temperature in units of .001 degree C
			//tempC /= 1000;
			
			//printf("[1wire] readTemp(*d=rootNode)      *d: %s    Temp: %.3f C    Add: 0x %x    NextAdd: 0x %x    \n", d->devID, d->tempC, d, d->next);
			
			//printf("%.3f F   ", d->tempC * 9 / 5 + 32);
			//recordTemp(d->devID, tempC);
		}
		close(fd);
	}
	printf("returning from readTemp\n");
	return 0;
}
#endif

#ifdef CC2
// ChipCap2 Read Humidity and Temperature Sensor Data
float ChipCap2(const int fd_chipcap2)
{
    uint8_t rht[4];
	
    rht[0] = wiringPiI2CRead(fd_chipcap2);
    rht[1] = wiringPiI2CRead(fd_chipcap2);
    rht[2] = wiringPiI2CRead(fd_chipcap2);
    rht[3] = wiringPiI2CRead(fd_chipcap2);	
    status = rht[0] >> 6;
    humidity = (((rht[0] & 63) << 8) + rht[1]) / 163.84;
    temperatureC = (((rht[2] << 6) + (rht[3] / 4)) / 99.29) - 40; 
    temperatureF = temperatureC * 1.8 + 32;
	
	return humidity;
}
#endif

#ifdef BMP180
// BMP180 getPres() - https://github.com/ic11b025/wetterstation/blob/master/getPres.c
#define OSS 1 /* Oversampling_setting = 1 - > conversion time 7,5 ms */

double getPres(const int fd_bosch) {
	const char FNAME[]= "getPres()";
	double pressure = 0;
	double temperature = 0;
	double reduced_pressure = 0;
	double Tm = 0;

	//const double altitude = 180.0; /* Altitude of Floridsdorf in meters above mean sea level */
	const double altitude = 213.0; /* Altitude of Home in meters above mean sea level */
	//const double altitude = 201.0; /* Altitude of Cottage in meters above mean sea level */
	//const double altitude = 198.0; /* Altitude of Blackstone lake in meters above mean sea level */
	
	short AC1 = 0;
	short AC2 = 0;
	short AC3 = 0;
	unsigned short AC4 = 0;
	unsigned short AC5= 0;
	unsigned short AC6= 0;
	short B1= 0;
	short B2= 0;

	short MB= 0;
	short MC= 0;
	short MD= 0;

	long raw_pressure = 0;
	long raw_temperature = 0;

	long X1= 0;
	long X2= 0;
	long X3= 0;
	long B3= 0;
	unsigned long B4= 0;
	long B5 = 0;
	long B6= 0;
	unsigned long B7= 0;

	unsigned short MSB= 0;
	unsigned short LSB= 0;
	unsigned short XLSB= 0;

	long p = 0;
	long t = 0;

	/* Reading Calibration Data */
	AC1 = wiringPiI2CReadReg8(fd_bosch, 0xAA);
	AC1 = AC1 << 8;
	AC1 |= wiringPiI2CReadReg8(fd_bosch, 0xAB);
	if ((AC1 == 0) || (AC1 == 0xFFF)) fprintf(stderr, "ERROR: %s: Error with data communication on AC1!\n", FNAME);

	AC2 = wiringPiI2CReadReg8(fd_bosch, 0xAC);
	AC2 = AC2 << 8;
	AC2 |= wiringPiI2CReadReg8(fd_bosch, 0xAD);
	if ((AC2 == 0) || (AC2 == 0xFFF)) fprintf(stderr, "ERROR: %s: Error with data communication on AC2!\n", FNAME);

	AC3 = wiringPiI2CReadReg8(fd_bosch, 0xAE);
	AC3 = AC3 << 8;
	AC3 |= wiringPiI2CReadReg8(fd_bosch, 0xAF);
	if ((AC3 == 0) || (AC3 == 0xFFF)) fprintf(stderr, "ERROR: %s: Error with data communication on AC3!\n", FNAME);

	AC4 = wiringPiI2CReadReg8(fd_bosch, 0xB0);
	AC4 = AC4 << 8;
	AC4 |= wiringPiI2CReadReg8(fd_bosch, 0xB1);
	if ((AC4 == 0) || (AC4 == 0xFFF)) fprintf(stderr, "ERROR: %s: Error with data communication on AC4!\n", FNAME);

	AC5 = wiringPiI2CReadReg8(fd_bosch, 0xB2);
	AC5 = AC5 << 8;
	AC5 |= wiringPiI2CReadReg8(fd_bosch, 0xB3);
	if ((AC5 == 0) || (AC5 == 0xFFF)) fprintf(stderr, "ERROR: %s: Error with data communication on AC5!\n", FNAME);

	AC6 = wiringPiI2CReadReg8(fd_bosch, 0xB4);
	AC6 = AC6 << 8;
	AC6 |= wiringPiI2CReadReg8(fd_bosch, 0xB5);
	if ((AC6 == 0) || (AC6 == 0xFFF)) fprintf(stderr, "ERROR: %s: Error with data communication on AC6!\n", FNAME);

	B1 = wiringPiI2CReadReg8(fd_bosch, 0xB6);
	B1 = B1 << 8;
	B1 |= wiringPiI2CReadReg8(fd_bosch, 0xB7);
	if ((B1 == 0) || (B1 == 0xFFF)) fprintf(stderr, "ERROR: %s: Error with data communication on B1!\n", FNAME);

	B2 = wiringPiI2CReadReg8(fd_bosch, 0xB8);
	B2 = B2 << 8;
	B2 |= wiringPiI2CReadReg8(fd_bosch, 0xB9);
	if ((B2 == 0) || (B2 == 0xFFF)) fprintf(stderr, "ERROR: %s: Error with data communication on B2!\n", FNAME);

	MB = wiringPiI2CReadReg8(fd_bosch, 0xBA);
	MB = MB << 8;
	MB |= wiringPiI2CReadReg8(fd_bosch, 0xBB);
	if ((MB == 0) || (MB == 0xFFF)) fprintf(stderr, "ERROR: %s: Error with data communication on MB!\n", FNAME);

	MC = wiringPiI2CReadReg8(fd_bosch, 0xBC);
	MC = MC << 8;
	MC |= wiringPiI2CReadReg8(fd_bosch, 0xBD);
	if ((MC == 0) || (MC == 0xFFF)) fprintf(stderr, "ERROR: %s: Error with data communication on MC!\n", FNAME);

	MD = wiringPiI2CReadReg8(fd_bosch, 0xBE);
	MD = MD << 8;
	MD |= wiringPiI2CReadReg8(fd_bosch, 0xBF);
	if ((MD == 0) || (MD == 0xFFF)) fprintf(stderr, "ERROR: %s: Error with data communication on MD!\n", FNAME);

	/*Start Conversion for raw_temperature*/
	wiringPiI2CWriteReg8(fd_bosch, 0xF4, 0x2E);
	delay(5); /* conversion should be finished after 4,5ms */

	MSB = wiringPiI2CReadReg8(fd_bosch, 0xF6);
	LSB = wiringPiI2CReadReg8(fd_bosch, 0xF7);

	raw_temperature = (MSB << 8) + LSB;

	/*Start Conversion for raw_pressure*/
	wiringPiI2CWriteReg8(fd_bosch, 0xF4, (0x34 + (OSS<<6))); 

	switch (OSS) {
		case 0:
			delay(5); /* conversion should be finished after 4,5 ms with OSS=0 */
			break;
	
		case 1:
			delay(8); /* conversion should be finished after 7,5 ms with OSS=1 */
			break;

		case 2:
			delay(14); /* conversion should be finished after 13,5 ms with OSS=2 */
			break;

		case 3:
			delay(26); /* conversion should be finished after 25,5 ms with OSS=3 */
			break;

		default:
			delay(26); /* Let´s be on the safe side .... */
	}

	MSB = wiringPiI2CReadReg8(fd_bosch, 0xF6);
	LSB = wiringPiI2CReadReg8(fd_bosch, 0xF7);
	XLSB = wiringPiI2CReadReg8(fd_bosch, 0xF8);
	raw_pressure = ((MSB<<16) + (LSB<<8) + XLSB) >> (8-OSS);

	/* Calculate True Temperature */
	X1 = ((raw_temperature - AC6) * AC5) >> 15;
	X2 = (MC << 11) / (X1 + MD);
	B5 = X1 + X2;
	t = (B5 + 8) >> 4;
	temperature = t/10.00;

	/*Calculate True Pressure*/
	B6 = B5 - 4000;
	X1 = (B2 * ((B6 * B6) >> 12))>>11;
	X2 = (AC2 * B6)>>11;
	X3 = X1 + X2;
	B3 = (((((AC1) * 4) + X3)<< OSS) + 2)/4; 
	X1 = (AC3 * B6)>>13;
	X2 = (B1 * ((B6*B6)>>12))>>16;
	X3 = ((X1 + X2) + 2) >> 2;
	B4 = (AC4 * (unsigned long)(X3 + 32768))>>15;
	B7 = ((unsigned long)raw_pressure - B3) * (50000 >> OSS);
	if (B7 <  0x80000000) {
		p = (B7*2)/B4;
	}
	else {
		p = (B7 / B4 ) * 2;
	}

	X1 = (p>>8)*(p>>8);
	X1 = (X1 * 3038) >> 16;
	X2 = (-7357 * p) >> 16;
	p += (X1 + X2 + 3791) >> 4;
	pressure = p/100.0; /* Pascal -> hPa */

	/* calculate reduced pressure at mean sea level */
	Tm = (temperature + 273.15) + (0.00325 * altitude);
	reduced_pressure = pressure * exp((9.811 * altitude) / (287.05 * Tm));

	//fprintf(stdout, "DEBUG: %s: temperature = %2.1f Celcius\n", FNAME, temperature);
	//fprintf(stdout, "DEBUG: %s: real pressure = %4.0f HPa\n", FNAME, pressure);
	//fprintf(stdout, "DEBUG: %s: real pressure at sea level = %4.0f HPa\n", FNAME, reduced_pressure);

	/* close the I2C device */
/*
	errno = 0;
	if (close(fd_bosch) == -1) {
		fprintf(stderr, "ERROR: %s: Could not close I2C device! errno: %s\n", FNAME, strerror(errno));
	}
*/
	return reduced_pressure;
}
#endif

#ifdef RELAYS
void relays(int relay)
{
	printf("[RELAY] Relay %d activated\n", relay);
	digitalWrite (RELAY_ENABLE, 1);
	digitalWrite (relay, 0);
	delay (100) ;
	digitalWrite (relay, 1);
	digitalWrite (RELAY_ENABLE, 0);
	printf("[RELAY] Relay %d deactivated\n", relay);
}
#endif



/*
 *********************************************************************************
 * MQTT - Message Queue Telemetry Transport
 *********************************************************************************
 */

void delivered(void *context, MQTTClient_deliveryToken dt)
{
    //printf("[MQTT] Message with token value %d delivery confirmed\n", dt);
    deliveredtoken = dt;
}

int msgarrvd(void *context, char *topicName, int topicLen, MQTTClient_message *message)
{
    int i;
    char* ptr;
	char buffer_top[255], buffer_msg[255];

    //printf("[MQTT] Message arrived, topic: %s, message: ", topicName);
	
	/*
	ptr = message->payload;
    for(i=0; i<message->payloadlen; i++)
    {
        buffer_msg[i]=*ptr;
		putchar(*ptr++);
    }
    buffer_msg[i]='\0';
	putchar('\n');
	
	ptr = &topicName;
    for(i=0; i<topicLen; i++)
    {
        buffer_top[i]=*ptr;
		putchar(*ptr++);
    }
    buffer_top[i]='\0';
	
	*/
	strcpy(buffer_msg, message->payload);
	buffer_msg[message->payloadlen]='\0';
	strcpy(buffer_top, topicName);
	//printf("[MQTT]   topic %s   message: %s\n",buffer_top,buffer_msg);

	#ifdef RELAYS	
	if(strcmp(topicName,"Home/Garage/Command")==0)
	{	
		if (strcmp(buffer_msg,"NearDoor")==0)
			relays(RELAY1);
		if (strcmp(buffer_msg,"MidDoor")==0)
			relays(RELAY2);
		if (strcmp(buffer_msg,"NearLight")==0)
			relays(RELAY3);
		if (strcmp(buffer_msg,"MidLight")==0)
			relays(RELAY4);
	}
	#endif

    MQTTClient_freeMessage(&message);
    MQTTClient_free(topicName);
    return 1;
}

void connlost(void *context, char *cause)
{
    printf("[MQTT] Connection lost, cause: %s\n", cause);
}



#ifdef NRF
/*
 *********************************************************************************
 * nrf
 *********************************************************************************
 */

 void rf24_configure(rf24_t *radio)
{
	rf24_write_register(radio, 	CONFIG, 	0);
	rf24_write_register(radio, 	EN_AA, 		0);					//    ENABLE AUTO ACK
	rf24_write_register(radio, 	EN_RXADDR, 	3);					//    ENABLE ADDRESSES
	rf24_write_register(radio, 	SETUP_AW, 	3); // 3 = 5 bytes	//    ADDRESS WIDTH
	rf24_write_register(radio, 	SETUP_RETR, 0);					//    TRANSMIT RETRYS
	rf24_write_register(radio, 	RF_CH, 		76);				//    RF CHANNEL
	rf24_write_register(radio, 	RF_SETUP, 	_BV(RF_DR_LOW));	//    CLEAR INTERRUPTS
	rf24_write_register(radio, 	DYNPD, 		0);					//    DYNAMIC PAYLOAD LENGTH
	rf24_write_register(radio, 	FEATURE, 	0);					//    FEATURES

	rf24_write_register(radio, 	RX_PW_P0, 	8);
	rf24_write_register(radio, 	RX_PW_P1, 	8);
	rf24_write_register(radio, 	RX_PW_P2, 	8);
	rf24_write_register(radio, 	RX_PW_P3, 	8);
	rf24_write_register(radio, 	RX_PW_P4, 	8);
	rf24_write_register(radio, 	RX_PW_P5, 	8);

	rf24_write_address(radio, 	TX_ADDR,    0x77EE33FFFF);	// warning: integer constant is too large for ‘long’ type [-Wlong-long]
	rf24_write_address(radio, 	RX_ADDR_P0, 0x77EE33FF11);	// warning: integer constant is too large for ‘long’ type [-Wlong-long]
	rf24_write_address(radio, 	RX_ADDR_P1, 0x11FF33EE77);	// warning: integer constant is too large for ‘long’ type [-Wlong-long]
	rf24_write_address(radio, 	RX_ADDR_P2, 0xc3c3c3c3c3);	// warning: integer constant is too large for ‘long’ type [-Wlong-long]
	rf24_write_address(radio, 	RX_ADDR_P3, 0x4242424242);	// warning: integer constant is too large for ‘long’ type [-Wlong-long]
	rf24_write_address(radio,	RX_ADDR_P4, 0x5656565656);	// warning: integer constant is too large for ‘long’ type [-Wlong-long]
	rf24_write_address(radio, 	RX_ADDR_P5, 0x112233FF44);	// warning: integer constant is too large for ‘long’ type [-Wlong-long]
	// the problem was MSWord / LSWord

	rf24_write_register(radio, 	STATUS, 	_BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );		//    CLEAR INTERRUPTS
}
#endif



#ifdef WIFI
/*

// WIFI SIGNAL INFO
// http://blog.ajhodges.com/2011/10/using-ioctl-to-gather-wifi-information.html
int getSignalInfo(signalInfo_t *sigInfo, char *iwname){
    iwreq req;
    strcpy(req.ifr_name, iwname);
    struct iw_statistics *stats;
 
    //have to use a socket for ioctl
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
 
    //make room for the iw_statistics object
	req.u.data.pointer = (struct iw_statistics *) malloc(sizeof(* stats));
	req.u.data.length = sizeof(* stats);
	req.u.data.flags = 1;

    //this will gather the signal strength
    if(ioctl(sockfd, SIOCGIWSTATS, &req) == -1){
        //die with error, invalid interface
        fprintf(stderr, "Invalid interface.\n");
        return(-1);
    }
    else if(((iw_statistics *)req.u.data.pointer)->qual.updated & IW_QUAL_DBM){
        //signal is measured in dBm and is valid for us to use
        sigInfo->level=((iw_statistics *)req.u.data.pointer)->qual.level - 256;
    }
 
    //SIOCGIWESSID for ssid
    char buffer[32];
    memset(buffer, 0, 32);
    req.u.essid.pointer = buffer;
    req.u.essid.length = 32;
    //this will gather the SSID of the connected network
    if(ioctl(sockfd, SIOCGIWESSID, &req) == -1){
        //die with error, invalid interface
        return(-1);
    }
    else{
        memcpy(&sigInfo->ssid, req.u.essid.pointer, req.u.essid.length);
        memset(&sigInfo->ssid[req.u.essid.length],0,1);
    }
 
    //SIOCGIWRATE for bits/sec (convert to mbit)
    int bitrate=-1;
    //this will get the bitrate of the link
    if(ioctl(sockfd, SIOCGIWRATE, &req) == -1){
        fprintf(stderr, "bitratefail");
        return(-1);
    }else{
        memcpy(&bitrate, &req.u.bitrate, sizeof(int));
        sigInfo->bitrate=bitrate/1000000;
    }
 
 
    //SIOCGIFHWADDR for mac addr
    ifreq req2;
    strcpy(req2.ifr_name, iwname);
    //this will get the mac address of the interface
    if(ioctl(sockfd, SIOCGIFHWADDR, &req2) == -1){
        fprintf(stderr, "mac error");
        return(-1);
    }
    else{
        sprintf(sigInfo->mac, "%.2X", (unsigned char)req2.ifr_hwaddr.sa_data[0]);
        for(int s=1; s<6; s++){
            sprintf(sigInfo->mac+strlen(sigInfo->mac), ":%.2X", (unsigned char)req2.ifr_hwaddr.sa_data[s]);
        }
    }
    close(sockfd);
}


*/
#endif

/*
 *********************************************************************************
 * main
 *********************************************************************************
 */

int main (void)
{
	char hostname[20]={0};
	int gotOne, pin ;	// isr
	int myCounter [8] ;	// isr
	int i;	// address counter	// isr
	struct timeval mark_time, now;//time
	long int seconds;			// time
	//time_t mark_time, now;	// time
	//double seconds;			// time
	//unsigned int t=0;			// time
	//QueueList  object;	// c-generic-library
	//data_queue_t data;	// c-generic-library

    
	char 	buffer_char[256];
	float 	buffer_f=0;
	double 	buffer_lf=0;
	uint8_t buffer_u8=0;
	FILE *fp;
	
	unsigned int d[32]={0};
	int rc;
	
	#ifdef BMP180
	int fd_bmp180 = -1;	// BMP180
	#endif
	
	#ifdef CC2
	int fd_chipcap2 = -1;	// chipcap2
	#endif

	#ifdef ONE_WIRE
	struct ds18b20 *rootNode;
	struct ds18b20 *devNode;
	int8_t devCnt=0;
	#endif
	
	#ifdef WIFI
	// WIFI SIGNAL INFO - TRY #2
	int sockfd;
    struct iwreq wreq;				//	http://w1.fi/hostapd/devel/structiwreq.html
	struct iw_statistics wstats;	//	http://w1.fi/hostapd/devel/structiw__statistics.html
	#endif
    
	// HOST
	#if HOST == Pi1
		strcpy(hostname,"Pi1\0");
	#elif HOST == Pi2
		strcpy(hostname,"Pi2\0");
	#elif HOST == Pi3
		strcpy(hostname,"Pi3\0");
	#elif HOST == Pi4
		strcpy(hostname,"Pi4\0");
	#else
		strcpy(hostname,"unknown\0");
	#endif
	printf("HOST = %s\n ",hostname);

	//MQTT
    char MQTT_topic[256]={0};
	MQTTClient client;
    MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;
    MQTTClient_message pubmsg = MQTTClient_message_initializer;
    MQTTClient_deliveryToken token;
	pubmsg.qos = QOS;
	pubmsg.retained = 0;
	deliveredtoken = 0;

	// MQTT Create client and connect to broker
    MQTTClient_create(&client, ADDRESS, CLIENTID, MQTTCLIENT_PERSISTENCE_NONE, NULL);
    conn_opts.keepAliveInterval = 20;
    conn_opts.cleansession = 1;
    MQTTClient_setCallbacks(client, NULL, connlost, msgarrvd, delivered);
    if ((rc = MQTTClient_connect(client, &conn_opts)) != MQTTCLIENT_SUCCESS)
    {
        printf("[MQTT] Failed to connect, return code %d\n", rc);
        // exit(-1);
    }
	
	// MQTT Subscriptions
	//MQTTClient_subscribe(client, TOPIC, QOS);
	MQTTClient_subscribe(client, "Home/Garage/Command", QOS);
	MQTTClient_subscribe(client, "Home/Garage/Status", QOS);
	MQTTClient_subscribe(client, "Home/Systems/Pi1/Temp/CPUTemp", QOS);

	
	for (pin = 0 ; pin < 8 ; ++pin) 
		globalCounter [pin] = myCounter [pin] = 0 ;

	wiringPiSetup();
	

	#ifdef BMP180
	fd_bmp180 = wiringPiI2CSetup(0x77);
	if (fd_bmp180 == -1)
	{
		printf("[wiringPi] I2C setup failed for address 0x77 expected bmp180.\n");
	}
	#endif
	
	#ifdef CC2
	fd_chipcap2 = wiringPiI2CSetup(0x28);
	if (fd_chipcap2 == -1)
	{
		printf("[wiringPi] I2C setup failed for address 0x28 expected chipcap2.\n");
	}
	#endif
	
	/*
	wiringPiISR (0, INT_EDGE_FALLING, &myInterrupt0) ; // home: open
	wiringPiISR (1, INT_EDGE_FALLING, &myInterrupt1) ; // home: open
	wiringPiISR (2, INT_EDGE_FALLING, &myInterrupt2) ; // home: nRF hardware interrupt
	//wiringPiISR (2, INT_EDGE_FALLING, &(void)rf24_receive(&radio_global, &d, 8)) ; // home: nRF hardware interrupt
	wiringPiISR (3, INT_EDGE_FALLING, &myInterrupt3) ; // home: open
	wiringPiISR (4, INT_EDGE_FALLING, &myInterrupt4) ; // home: open
	wiringPiISR (5, INT_EDGE_FALLING, &myInterrupt5) ; // home: open
	wiringPiISR (6, INT_EDGE_FALLING, &myInterrupt6) ; // home: open
	//wiringPiISR (7, INT_EDGE_FALLING, &myInterrupt7) ; // has one wire at home
*/

	#ifdef RELAYS
	// wiringPi IO initial states
	digitalWrite (RELAY1, 1);
	digitalWrite (RELAY2, 1);
	digitalWrite (RELAY3, 1);
	digitalWrite (RELAY4, 1);
	digitalWrite (RELAY4, 1);
	digitalWrite (RELAY_ENABLE, 0);
	
	// wiringPi GPIO init
	pinMode (RELAY_ENABLE, OUTPUT);
	digitalWrite (RELAY_ENABLE, 0);
	
	pinMode (STATUS1, INPUT);
	pinMode (STATUS2, INPUT);
	pinMode (STATUS3, INPUT);
	pinMode (STATUS4, INPUT);
	pinMode (RELAY1, OUTPUT);
	pinMode (RELAY2, OUTPUT);
	pinMode (RELAY3, OUTPUT);
	pinMode (RELAY4, OUTPUT);
	
	//pullUpDnControl (int pin, PUD_OFF) 
	//pullUpDnControl (int pin, PUD_DOWN) 
	//pullUpDnControl (int pin, PUD_UP) 
	pullUpDnControl (STATUS1, PUD_UP);
	pullUpDnControl (STATUS2, PUD_UP);
	pullUpDnControl (STATUS3, PUD_UP);
	pullUpDnControl (STATUS4, PUD_UP);


	// wiringPi IO initial states
	digitalWrite (RELAY1, 1);
	digitalWrite (RELAY2, 1);
	digitalWrite (RELAY3, 1);
	digitalWrite (RELAY4, 1);
	
	#endif

	#ifdef NRF
	// ********* nRF *********
	//rf24_t radio_global; // replaced with radio_global

	rf24_initialize(&radio_global, RF24_SPI_DEV_0, 25, 7);
	rf24_reset_status(&radio_global);
	rf24_configure(&radio_global);

	// POWERUP & START LISTENING
	rf24_write_register(&radio_global, CONFIG, rf24_read_register(&radio_global, CONFIG) | _BV(PWR_UP) | _BV(PRIM_RX));
	rf24_write_register(&radio_global, STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );
	usleep(4500);
	gpio_write(radio_global.ce_pin, GPIO_PIN_HIGH);
	#endif
	
	
	
	
	for (;;)
	{
		gotOne = 0 ;

		//printf("[Logger] Service local data collection\n");

		gettimeofday(&now,NULL);
		gettimeofday(&mark_time,NULL);
		seconds=now.tv_sec-mark_time.tv_sec;
		//time(&now);
		//time(&mark_time);

		// ****************************
		//      MAGNETIC CONTACTS
		// ****************************
		
		#ifdef CONTACTS
		
		if (!digitalRead(STATUS1))
			sprintf(buffer_char,"%s","CLOSED");
		else
			sprintf(buffer_char,"%s","OPEN");
		pubmsg.payload = buffer_char;
		pubmsg.payloadlen = strlen(buffer_char);
		MQTTClient_publishMessage(client, "Home/Garage/Status/Door/Side", &pubmsg, &token);
		
		if (!digitalRead(STATUS2))
			sprintf(buffer_char,"%s","CLOSED");
		else
			sprintf(buffer_char,"%s","OPEN");
		pubmsg.payload = buffer_char;
		pubmsg.payloadlen = strlen(buffer_char);
		MQTTClient_publishMessage(client, "Home/Garage/Status/Door/Near", &pubmsg, &token);
		
		if (!digitalRead(STATUS3))
			sprintf(buffer_char,"%s","CLOSED");
		else
			sprintf(buffer_char,"%s","OPEN");
		pubmsg.payload = buffer_char;
		pubmsg.payloadlen = strlen(buffer_char);
		MQTTClient_publishMessage(client, "Home/Garage/Status/Door/Mid", &pubmsg, &token);
		
		if (!digitalRead(STATUS4))
			sprintf(buffer_char,"%s","CLOSED");
		else
			sprintf(buffer_char,"%s","OPEN");
		pubmsg.payload = buffer_char;
		pubmsg.payloadlen = strlen(buffer_char);
		MQTTClient_publishMessage(client, "Home/Garage/Status/Door/Far", &pubmsg, &token);
		/*
		IF(digitalRead(STATUS2)) {
			pubmsg.payload = buffer_char;
			pubmsg.payloadlen = strlen(buffer_char);
			MQTTClient_publishMessage(client, "Home/Garage/Status/Door/Near", &pubmsg, &token);
		}
		IF(digitalRead(STATUS3)) {
			pubmsg.payload = buffer_char;
			pubmsg.payloadlen = strlen(buffer_char);
			MQTTClient_publishMessage(client, "Home/Garage/Status/Door/Mid", &pubmsg, &token);
		}
		
		IF(digitalRead(STATUS4)) {
			pubmsg.payload = buffer_char;
			pubmsg.payloadlen = strlen(buffer_char);
			MQTTClient_publishMessage(client, "Home/Garage/Status/Door/Far", &pubmsg, &token);
		}
		*/
		
		#endif
		
		
		/*
		// MQTT Test Publish
		pubmsg.payload = PAYLOAD;
		pubmsg.payloadlen = strlen(PAYLOAD);
		pubmsg.qos = QOS;
		pubmsg.retained = 0;
		deliveredtoken = 0;
		MQTTClient_publishMessage(client, TOPIC, &pubmsg, &token);
		printf("[MQTT] Waiting for publication of %s on topic %s for client with ClientID %s and token %d\n", PAYLOAD, TOPIC, CLIENTID, token);
		//if(deliveredtoken != token)
		//	printf("[MQTT] Still not delivered\n");
		*/


		// ****************************
		//          CPU Temp
		// ****************************
		buffer_f=0;
		// Read CPU Temp
		fp = fopen ("/sys/class/thermal/thermal_zone0/temp", "r");
		if (fp == NULL)
			printf("[Logger] Unable to open file.\n");
		fscanf (fp, "%lf", &buffer_lf);
		buffer_lf /= 1000;
		//printf ("[Logger] local Pi CPU temp is %.3f C.\n", buffer_lf);
		fclose (fp);
		sprintf(buffer_char, "%.3f\0", buffer_lf);

		// MQTT local Pi CPU temp
		sprintf(MQTT_topic,"Home/Systems/%s/Temp/CPUTemp\0",hostname);
		pubmsg.payload = buffer_char;
		pubmsg.payloadlen = strlen(buffer_char);
		MQTTClient_publishMessage(client, MQTT_topic, &pubmsg, &token);
		//printf("[MQTT] %s on topic %s\n", pubmsg.payload, "Home/Systems/Pi1/Temp/CPUTemp");

		#ifdef WIFI
		// ****************************
		//          WIFI
		// ****************************
		memset(&wreq, 0, sizeof(struct iwreq));
		wreq.u.essid.length = IW_ESSID_MAX_SIZE+1;
		sprintf(wreq.ifr_name, IW_INTERFACE);
		if((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
			fprintf(stderr, "Cannot open socket \n");
			fprintf(stderr, "errno = %d \n", errno);
			fprintf(stderr, "Error description is : %s\n",strerror(errno));
		} //else printf("\nSocket opened successfully \n");
		memset(buffer_char, 0, sizeof(buffer_char));
		wreq.u.essid.pointer = buffer_char;
		wreq.u.essid.length = sizeof(buffer_char);
		
		/*
		if (ioctl(sockfd,SIOCGIWESSID, &wreq) == -1) {
			fprintf(stderr, "[IOCTL] Get ESSID ioctl failed \n");
			fprintf(stderr, "[IOCTL] errno = %d \n", errno);
			fprintf(stderr, "[IOCTL] Error description : %s\n",strerror(errno));
    
		} else {
			printf("[IOCTL] IOCTL Successfull\n");
			printf("[IOCTL] ESSID is %s\n", (char *)wreq.u.essid.pointer);
		}

		//make room for the iw_statistics object
		//wreq.u.data.pointer = (struct iw_statistics *) malloc(sizeof(* wstats));
		*/
		wreq.u.data.pointer = &wstats;
		wreq.u.data.length = sizeof(wstats);
		wreq.u.data.flags = 1;

		//this will gather the signal strength
		//if(ioctl(sockfd, SIOCGIWSTATS, &wreq) == -1){
		if(ioctl(sockfd, SIOCGIWSTATS, &wreq) == -1){
			printf("Invalid interface.\n");
		}
		else {
		//else if(((iw_statistics *)wreq.u.data.pointer)->qual.updated & IW_QUAL_DBM){
			
			sprintf(MQTT_topic,"Home/Systems/%s/Wifi/Level\0",hostname);
						
			//buffer_u8=((iw_statistics *)wreq.u.data.pointer)->qual.level - 256;

			//printf("wstats.qual.qual : %d\n",wstats.qual.qual);
			//printf("wstats.qual.level : %d\n",wstats.qual.level);
			//printf("wstats.qual.noise : %d\n",wstats.qual.noise);

			sprintf(MQTT_topic,"Home/Systems/%s/Wifi/Qual\0",hostname);
			sprintf(buffer_char, "%d\0", wstats.qual.qual);
			pubmsg.payload = buffer_char;
			pubmsg.payloadlen = strlen(buffer_char);
			MQTTClient_publishMessage(client, MQTT_topic, &pubmsg, &token);
			
			sprintf(MQTT_topic,"Home/Systems/%s/Wifi/Level",hostname);
			sprintf(buffer_char, "%d\0", wstats.qual.level);
			pubmsg.payload = buffer_char;
			pubmsg.payloadlen = strlen(buffer_char);
			MQTTClient_publishMessage(client, MQTT_topic, &pubmsg, &token);
			
			sprintf(MQTT_topic,"Home/Systems/%s/Wifi/Noise\0",hostname);
			sprintf(buffer_char, "%d\0", wstats.qual.noise);
			pubmsg.payload = buffer_char;
			pubmsg.payloadlen = strlen(buffer_char);
			MQTTClient_publishMessage(client, MQTT_topic, &pubmsg, &token);
		}
 
		//SIOCGIWRATE for bits/sec (convert to mbit)
		int bitrate=-1;

		if(ioctl(sockfd, SIOCGIWRATE, &wreq) == -1){
			printf("[IOCTL] bitratefail\n");
		}else{
			memcpy(&bitrate, &wreq.u.bitrate, sizeof(int));
			buffer_u8=bitrate/1000000;
		}
		
		close(sockfd);
	
		#endif
		
		
		
		#ifdef BMP180
		
		// ****************************
		//      BAROMETRIC PRESSURE
		// ****************************
		
		buffer_lf=0;
		// Read BMP180		
		buffer_lf=getPres(fd_bmp180);
		//printf ("[BMP180] local Pi external pressure sensor %.3lf\n", buffer_lf);
		sprintf(buffer_char, "%.3lf\0", buffer_lf);		
		
		// MQTT local bmp180 pressure sensor
		pubmsg.payload = buffer_char;
		pubmsg.payloadlen = strlen(buffer_char);
		MQTTClient_publishMessage(client, "Home/Garage/BP", &pubmsg, &token);
		
		#endif
		
		
		#ifdef CC2
		
		humidity=0;
		temperatureC=0;
		temperatureF=0;
		
		buffer_f=0;
		// Read ChipCap2
		buffer_f=ChipCap2(fd_chipcap2);
		//printf ("[ChipCap2] local Pi humidity sensor pressure sensor %.3f\n", buffer_f);
		sprintf(buffer_char, "%.3f\0", buffer_f);		
		
		// MQTT local bmp180 pressure sensor
		pubmsg.payload = buffer_char;
		pubmsg.payloadlen = strlen(buffer_char);
		MQTTClient_publishMessage(client, "Home/Garage/Hum", &pubmsg, &token);
		
//		humidity;
//		temperatureC;
//		temperatureF;

		#endif

		// ****************************
		//          ONE WIRE
		// ****************************

		#ifdef ONE_WIRE
		
		rootNode = malloc( sizeof(struct ds18b20) );
		devNode = rootNode;
		
		//printf("[1wire] init - RootNode: %s    Temp: %.3f C    Add: 0x %x    NextAdd: 0x %x    \n", rootNode->devID, rootNode->tempC, rootNode, rootNode->next);
		//printf("[1wire] init - DevNode: %s    Temp: %.3f C    Add: 0x %x    NextAdd: 0x %x    \n", devNode->devID, devNode->tempC, devNode, devNode->next);
		
		devCnt = findDevices(devNode);
		//printf("[1wire] Found %d devices\n", devCnt);
		
		//printf("[1wire] found - RootNode: %s    Temp: %.3f C    Add: 0x %x    NextAdd: 0x %x    \n", rootNode->devID, rootNode->tempC, rootNode, rootNode->next);
		//printf("[1wire] found - DevNode: %s    Temp: %.3f C    Add: 0x %x    NextAdd: 0x %x    \n", devNode->devID, devNode->tempC, devNode, devNode->next);
		
		readTemp(rootNode);
		
		//printf("[1wire] read - RootNode: %s    Temp: %.3f C    Add: 0x %x    NextAdd: 0x %x    \n", rootNode->devID, rootNode->tempC, rootNode, rootNode->next);
		//printf("[1wire] read - DevNode: %s    Temp: %.3f C    Add: 0x %x    NextAdd: 0x %x    \n", devNode->devID, devNode->tempC, devNode, devNode->next);
		
		//printf("returned from readTemp\n");

		
		// Send MQTT
		devNode = rootNode;
		while(devNode->next) {
			//printf("while(devNode->next)\n");
			
			//printf("[1wire] Send - RootNode: %s    Temp: %.3f C    Add: 0x %x    NextAdd: 0x %x    \n", rootNode->devID, rootNode->tempC, &rootNode, rootNode->next);
			//printf("[1wire] Send - DevNode: %s    Temp: %.3f C    Add: 0x %x    NextAdd: 0x %x    \n", devNode->devID, devNode->tempC, &devNode, devNode->next);
			
			// Start with current value of root node
			devNode = devNode->next;
			//printf("devNode = devNode->next;");
			
			//printf("[1wire] Send - RootNode: %s    Temp: %.3f C    Add: 0x %x    NextAdd: 0x %x    \n", rootNode->devID, rootNode->tempC, &rootNode, rootNode->next);
			//printf("[1wire] Send - DevNode: %s    Temp: %.3f C    Add: 0x %x    NextAdd: 0x %x    \n", devNode->devID, devNode->tempC, &devNode, devNode->next);
			
			//printf ("[1wire] local Pi d18b20 is %s temp sense %.3f\n", devNode->devID, devNode->tempC);
			sprintf(buffer_char, "%.3f\0", devNode->tempC);
			
			// MQTT local bmp180 pressure sensor
			pubmsg.payload = buffer_char;
			pubmsg.payloadlen = strlen(buffer_char);

#if HOST == Pi1
			if (!strcmp(devNode->devID, "28-000005eaf6c1"))
				MQTTClient_publishMessage(client, "Cottage/MikesRoom/Temp/Ceiling1", &pubmsg, &token);
			else if (!strcmp(devNode->devID, "28-000005e99b06"))
				MQTTClient_publishMessage(client, "Cottage/MikesRoom/Temp/Ceiling2", &pubmsg, &token);
			else if (!strcmp(devNode->devID, "28-000005eb03de"))
				MQTTClient_publishMessage(client, "Cottage/MikesRoom/Temp/Mid1", &pubmsg, &token);
			else if (!strcmp(devNode->devID, "28-000005eb5b13"))
				MQTTClient_publishMessage(client, "Cottage/MikesRoom/Temp/Floor1", &pubmsg, &token);
			else if (!strcmp(devNode->devID, "28-000005ea416a"))
				MQTTClient_publishMessage(client, "Cottage/MikesRoom/Temp/Floor2", &pubmsg, &token);
#endif
#if HOST == Pi2
			if (!strcmp(devNode->devID, "28-000005eaf6c1"))
				MQTTClient_publishMessage(client, "Cottage/MikesRoom/Temp/Ceiling1", &pubmsg, &token);
			else if (!strcmp(devNode->devID, "28-000005e99b06"))
				MQTTClient_publishMessage(client, "Cottage/MikesRoom/Temp/Ceiling2", &pubmsg, &token);
			else if (!strcmp(devNode->devID, "28-000005eb03de"))
				MQTTClient_publishMessage(client, "Cottage/MikesRoom/Temp/Mid1", &pubmsg, &token);
			else if (!strcmp(devNode->devID, "28-000005eb5b13"))
				MQTTClient_publishMessage(client, "Cottage/MikesRoom/Temp/Floor1", &pubmsg, &token);
			else if (!strcmp(devNode->devID, "28-000005ea416a"))
				MQTTClient_publishMessage(client, "Cottage/MikesRoom/Temp/Floor2", &pubmsg, &token);
#endif
#if HOST == Pi3
			if (!strcmp(devNode->devID, "28-04146d2647ff"))
				MQTTClient_publishMessage(client, "Home/Systems/Pi3/Temp/Case", &pubmsg, &token);
			else if (!strcmp(devNode->devID, "28-031467a707ff"))
				MQTTClient_publishMessage(client, "Home/Systems/Pi3/Temp/Wifi", &pubmsg, &token);
			else if (!strcmp(devNode->devID, "28-04146cd4e5ff"))
				MQTTClient_publishMessage(client, "Home/Garage/Temp", &pubmsg, &token);
			else if (!strcmp(devNode->devID, "28-04146cf597ff"))
				MQTTClient_publishMessage(client, "Home/Outside/Temp", &pubmsg, &token);
#endif
#if HOST == Pi4
			if (!strcmp(devNode->devID, "28-04146c93f6ff"))
				MQTTClient_publishMessage(client, "Home/Systems/Pi4/Temp1", &pubmsg, &token);
			else if (!strcmp(devNode->devID, "28-04146d1220ff"))
				MQTTClient_publishMessage(client, "Home/Systems/Pi4/Temp2", &pubmsg, &token);
#endif

			//printf("[MQTT] %s on topic %s\n", pubmsg.payload, "...");
			//printf("[1wire] MQTT publish %s from sensor %s.\n",buffer_char, devNode->devID);

		}

		/*
		// Free linked list memory
		while(rootNode) {
			printf("while(rootNode)\n");
			
			printf("[1wire] Free Linked List - RootNode: %s    Temp: %.3f C    Add: 0x %x    NextAdd: 0x %x    \n", rootNode->devID, rootNode->tempC, &rootNode, rootNode->next);
			printf("[1wire] Free Linked List - DevNode: %s    Temp: %.3f C    Add: 0x %x    NextAdd: 0x %x    \n", devNode->devID, devNode->tempC, &devNode, devNode->next);
			
			// Start with current value of root node
			devNode = rootNode;
			printf("devNode = rootNode;");
			
			// Save address of next devNode to rootNode before 
            // deleting current devNode
			rootNode = devNode->next;
			printf("rootNode = devNode->next;");
			
			printf("[1wire] Free Linked List - RootNode: %s    Temp: %.3f C    Add: 0x %x    NextAdd: 0x %x    \n", rootNode->devID, rootNode->tempC, &rootNode, rootNode->next);
			printf("[1wire] Free Linked List - DevNode: %s    Temp: %.3f C    Add: 0x %x    NextAdd: 0x %x    \n", devNode->devID, devNode->tempC, &devNode, devNode->next);
			
			//printf ("[1wire] local Pi d18b20 is %s temp sense %.3f\n", devNode->devID, devNode->tempC);
			sprintf(buffer_char, "%.3f\0", devNode->tempC);
			
			// MQTT local bmp180 pressure sensor
			pubmsg.payload = buffer_char;
			pubmsg.payloadlen = strlen(buffer_char);
			
			if (strcmp(devNode->devID, "28-?"))
				MQTTClient_publishMessage(client, "Home/Garage/Temperature/Pi4Enclosure", &pubmsg, &token);
			else if (strcmp(devNode->devID, "28-000005eaf6c1"))
				MQTTClient_publishMessage(client, "Cottage/MikesRoom/Temperature/Ceiling1", &pubmsg, &token);
			else if (strcmp(devNode->devID, "28-000005e99b06"))
				MQTTClient_publishMessage(client, "Cottage/MikesRoom/Temperature/Ceiling2", &pubmsg, &token);
			else if (strcmp(devNode->devID, "28-000005eb03de"))
				MQTTClient_publishMessage(client, "Cottage/MikesRoom/Temperature/Mid1", &pubmsg, &token);
			else if (strcmp(devNode->devID, "28-000005eb5b13"))
				MQTTClient_publishMessage(client, "Cottage/MikesRoom/Temperature/Floor1", &pubmsg, &token);
			else if (strcmp(devNode->devID, "28-000005ea416a"))
				MQTTClient_publishMessage(client, "Cottage/MikesRoom/Temperature/Floor2", &pubmsg, &token);
			else
				MQTTClient_publishMessage(client, "Home/Systems/Temperature/Pi4Unknown", &pubmsg, &token);
			
			printf("[1wire] MQTT publish complete.\n");
			
			// Free current devNode.
			free(devNode);

			printf("[1wire] devNode freed.\n");

		}
		
		
		// Now free rootNode
		free(rootNode);

		

		*/
		
		while(rootNode) {
			// Start with current value of root node
			devNode = rootNode;
			// Save address of next devNode to rootNode before 
            // deleting current devNode
			rootNode = devNode->next;
			// Free current devNode.
			free(devNode);
		}
		// Now free rootNode
		free(rootNode);
		
		#endif
		
		
		//printf("[Logger] entering loop, mark_time = %d, now = %d, seconds = %d\n", mark_time.tv_sec, now.tv_sec, seconds) ; fflush (stdout) ;
		////printf("[Logger] entering loop, mark_time = %lf, now = %lf, seconds = %lf\n", mark_time, now, seconds) ; fflush (stdout) ;
		
		for (; seconds<1;) {
			gettimeofday(&now,NULL);
			//time(&now);
			seconds=now.tv_sec-mark_time.tv_sec;

			
			
			//printf("[Logger] looping, mark_time = %d, now = %d, seconds = %d\n", mark_time.tv_sec, now.tv_sec, seconds) ; fflush (stdout) ;
			
//			if(rf24_isrFlag)
//				for (;;)
//					print("[Logger] rf24_isrFlag \n");

/*
			if((rf24_get_status(&radio_global) & _BV(RX_DR)))
				printf("[Logger] nrf24 RX_DR bit set\n");
*/
				
				
			for (pin = 0 ; pin < 8 ; ++pin) {
				if (globalCounter [pin] != myCounter [pin]) {
					printf("[Logger] Int on pin %d: Counter: %5d\n", pin, globalCounter [pin]) ;
					myCounter [pin] = globalCounter [pin] ;
					++gotOne ;
					switch (pin) {
						#ifdef NRF
						case '2':	rf24_receive(&radio_global, &data.d, 32);
									rf24_write_register(&radio_global, 	STATUS, 	_BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );
									for (i==0;i<32;i++)
										fprintf(stderr, " %5d ",data.d[i]);
									fprintf(stderr, "\n");
									
									break;
						#endif
						default:	
									
									break;
					}
				}
			}
	
			
			if (gotOne != 0)
			break ;
		}
		//printf("[Logger] exiting loop, mark_time = %d, now = %d, seconds = %d\n", mark_time.tv_sec, now.tv_sec, seconds) ; fflush (stdout) ;
		
	}

	return 0 ;
}
