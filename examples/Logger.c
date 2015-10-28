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
//	#define ADDRESS     "tcp://192.168.1.214:1883"		// MQTT
	#define ADDRESS     "tcp://frafle.ddns.net:22883"	// MQTT
	#define CLIENTID    "Pi1"				// MQTT
	#define BMP180
 	#define CPUTEMP
	//#define ONE_WIRE
	#define MQTT
	#define CONTACTS

#elif HOST == Pi2
	#define WIFI
	#define NRF
	#define ADDRESS     "tcp://frafle.ddns.net:22883"		// MQTT
	#define CLIENTID    "Pi2"							// MQTT
 	#define CPUTEMP
	#define ONE_WIRE
	#define MQTT
 	#define CONTACTS

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
 	#define CPUTEMP
	#define MQTT

#elif HOST == Pi4
	#define WIFI
	#define NRF
	#define ADDRESS     "tcp://localhost:1883"			// MQTT
	#define CLIENTID    "Pi4"							// MQTT
	#define ONE_WIRE
	#define BMP180
	#define CC2
//	#define RELAYS
//	#define CONTACTS
	#define SQLITE
	#define HANDLECTLC
  	#define CPUTEMP
	#define MQTT

#elif HOST == Pi5
        #define WIFI
        #define NRF
        #define ADDRESS     "tcp://localhost:1883"                      // MQTT
        #define CLIENTID    "Pi5"                                                       // MQTT
//        #define ONE_WIRE
//        #define BMP180
//        #define CC2
//      #define RELAYS
//      #define CONTACTS
        #define SQLITE
        #define HANDLECTLC
        #define CPUTEMP
        #define MQTT

#else
	#define WIFI
	#define NRF
	#define ADDRESS     "tcp://192.168.1.214:1883"		// MQTT
	#define CLIENTID    "Default"							// MQTT
	//#define ONE_WIRE
	#define MQTT
	#define CPUTEMP

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
    rf24_t radio_global;    // nRF
#endif

#ifdef BMP180
	#include <bmp180.h>

#endif

#ifdef SQLITE
	#include <sqlite3.h>
#endif

//#include "queue.h"					// c-generic-library
//#include "error_macros.h"				// c-generic-library
//#include "libcgeneric/libcgeneric.h"	// c-generic-library


#define LOCAL_DATA_INTERVAL 2	// Logger - main loop


#ifdef CONTACTS
	#define STATUS1 26
	#define STATUS2 27
	#define STATUS3 28
	#define STATUS4 29
#endif

#ifdef RELAYS
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

// Flag used by handler - changed to 0 when user presses Ctrl-C
// Loop that reads & records temperatures keeps running when
// loop_active_main = 1
int8_t volatile loop_active_main = 1;
int8_t volatile loop_active_msg_thread = 1;

#ifdef HANDLECTLC
	#include <signal.h> 
	// Called when user presses Ctrl-C
	void intHandler() {
		printf("\nStopping...\n");
		loop_active_main = 0;
	}
#endif
	
#ifdef SQLITE
	int8_t rc_global=0;

	// Pointer to Sqlite3 DB - used to access DB when open
	sqlite3 *db = NULL;

	// Path to DB file - same dir as this program's executable
	char *dbPath = "/var/www/Logger.db";

	char sql[255];
	char sql_real[255];
	char sql_int[255];
	char sql_text[255];

	// DB Statement handle - used to run SQL statements
	sqlite3_stmt *sql_stmt = NULL;
//	sqlite3_stmt *sql_stmt_ptr = NULL;
//	sqlite3_stmt *sql_stmt_real = NULL;
//	sqlite3_stmt *sql_stmt_int = NULL;
//	sqlite3_stmt *sql_stmt_text = NULL;
	
	// transaction counter
	int sqlite_transaction_ctr=0;
	int sqlite_transaction_tmr=0;
	int sqlite_transaction_flag=0;
	#define SQLITE_TRANSACTION_INTERVAL 1
	#define SQLITE_TRANSACTION_COUNT 1000
/*
	sprintf(sql_real, "INSERT INTO Real(Location, Area, Type, ID, DateTime, Value) VALUES(?, ?, ?, ?, ?, ?)");
	//sprintf(sql_int, "INSERT INTO Int(Location, Area, Type, ID, DateTime, Value) VALUES(?, ?, ?, ?, ?, ?)");
	//sprintf(sql_text, "INSERT INTO Text(Location, Area, Type, ID, DateTime, Value) VALUES(?, ?, ?, ?, ?, ?)");
	//sprintf(sql, "INSERT INTO Real(Location, Area, Type, ID, DateTime, Value) VALUES(?, ?, ?, ?, ?, ?)");
	test=sqlite3_prepare_v2(db, sql_real, strlen(sql_real), &sql_stmt_real, NULL);
	if (rc_global)
		printf("[SQLite] prepare return code %d: %s\n\n", rc_global, sqlite3_errmsg(db));
	
	//rc_global=sqlite3_prepare_v2(db, sql_int, strlen(sql_int), &sql_stmt_int, NULL);
	//if (rc_global)
	//	printf("[SQLite] prepare return code %d: %s\n\n", rc_global, sqlite3_errmsg(db));
	
	//rc_global=sqlite3_prepare_v2(db, sql_text, strlen(sql_text), &sql_stmt_text, NULL);
	//if (rc_global)
	//	printf("[SQLite] prepare return code %d: %s\n\n", rc_global, sqlite3_errmsg(db));
*/
#endif

#ifdef CC2
	// ChipCap2 global variables
	uint8_t  status;
	float humidity;
	float temperatureC;
	float temperatureF;
#endif

#ifdef ONE_WIRE
	// struct to hold ds18b20 data for linked list	// 1-Wire driver stores info in file for device as text
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
			while ((dirent = readdir(dir)))
			{
				//printf("[1-wire] FindDevices() dirent=readdir(dir)=%s\n",readdir);
				//printf("[1-wire] FindDevices()   dirent->d_type=%d   dirent->d_name=%s\n",dirent->d_type, dirent->d_name);
				// 1-wire devices are links beginning with 28-
				if (dirent->d_type == DT_LNK && strstr(dirent->d_name, "28-") != NULL)
				{
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
			perror ("Couldn't open the w1 devices directory\n");
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
		//printf("returning from readTemp\n");
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
	loop_active_msg_thread=1;
	clock_t start, end;
	double cpu_time_used;
	start=clock();

//	printf("loop_active_msg_thread=%d\n", loop_active_msg_thread);

	int i;

	double pythondatetime;	// double precision float is 8 bytes = IEEE float used in sqlite real datatype
	time_t unixtime;
	struct tm *loctime; 
	int years2000;
	int leapdays2000;

	int rc;
	int rc1=1,delay_time1=10;
	int rc2=1,delay_time2=10;

	char *ptr;
	char buffer_top[255], buffer_msg[255];

	//char sql[255];
	char *sqlErrMsg = 0;

	const char topic_search[2] = "/";
	char *topic_location;
	char *topic_area;
	char *topic_type;
	char *topic_id;
	char tableName[255];

	FILE *fp;
   
	double python_datetime_leapdays_since_2000;
	double python_datetime_2000;
	double python_datetime_years_since_2000;
	double python_datetime_days;
	double python_datetime_hour;
	double python_datetime_min;
	double python_datetime_sec;
   
   
	//printf("[MQTT] Message arrived, topic: %s, message: ", topicName);
	
	strcpy(buffer_msg, message->payload);
	buffer_msg[message->payloadlen]='\0';
	strcpy(buffer_top, topicName);
	
	//printf("[MQTT]   topic %s   message: %s\n",buffer_top,buffer_msg);
	
	topic_location	= strtok(buffer_top, topic_search);
	topic_area		= strtok(NULL, topic_search);
	topic_type		= strtok(NULL, topic_search);
	topic_id 		= strtok(NULL, topic_search);

	printf("%15s  %15s  %15s  %15s  %15s\n", topic_location, topic_area, topic_type, topic_id, buffer_msg);
	//printf("%s_%s_%s_%s", topic_location, topic_area, topic_type, topic_id);
	sprintf(tableName,"%s_%s_%s_%s", topic_location, topic_area, topic_type, topic_id);
	//printf("tableName = %s\n",tableName);

	//printf("string compares %d %d %d %d %d\n",strcmp(token, "Temp"),strcmp(token, "BP"),strcmp(token, "Hum"),strcmp(token, "Wifi"),strcmp(token, "Contact"));


/*
	if(strcmp(topic_type,"Temp")==0 || strcmp(topic_type,"BP")==0 || strcmp(topic_type,"Hum")==0)
		sprintf(sql, "INSERT INTO Real(Location, Area, Type, ID, DateTime, Value) VALUES(?, ?, ?, ?, ?, ?)");
	if(strcmp(topic_type,"Wifi")==0 || strcmp(topic_type,"Contact")==0)
		sprintf(sql, "INSERT INTO Int(Location, Area, Type, ID, DateTime, Value) VALUES(?, ?, ?, ?, ?, ?)");
	if(strcmp(topic_type,"Command")==0)
		sprintf(sql, "INSERT INTO Text(Location, Area, Type, ID, DateTime, Value) VALUES(?, ?, ?, ?, ?, ?)");	

	//sprintf(sql, "INSERT INTO '%s'(Date, Val) VALUES(?, ?)", buffer_top);	
	
	printf("SQL statement: %s \n", sql);	
*/

#ifdef RELAYS
		if(strcmp(topic_type,"Command")==0 && strcmp(topic_id,"Relays")==0)
		{
			if(strcmp(topic_location,"Home")==0 && strcmp(topic_area,"Garage")==0)
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
			else if(strcmp(topic_location,"Home")==0 && strcmp(topic_area,"Mike")==0)
			{
				if (strcmp(buffer_msg,"RELAY1")==0)
					relays(RELAY1);
				if (strcmp(buffer_msg,"RELAY2")==0)
					relays(RELAY2);
				if (strcmp(buffer_msg,"RELAY3")==0)
					relays(RELAY3);
				if (strcmp(buffer_msg,"RELAY4")==0)
					relays(RELAY4);
			}
		}
#endif

#ifdef SQLITE

		// IN PYTHON print datetime.date(1900, 1, 1).toordinal() RETURNS 693596
		// IN PYTHON print datetime.date(1970, 1, 1).toordinal() RETURNS 719163
		// IN PYTHON print datetime.date(2000, 1, 1).toordinal() RETURNS 730120
		// http://en.wikipedia.org/wiki/Unix_time
		// Unix time number is zero at the Unix epoch, and increases by exactly 86400 per day since the epoch
		// http://www.epochconverter.com/
		//pythondatetime=(double)(time(NULL))/86400+719163; // conversion from Unixtime
		//printf("pythondatetime = %lf \n", pythondatetime);

		time(&unixtime);
		//unixtime=time(NULL);
		//printf("unixtime = %lf \n", (double)unixtime);

		loctime=localtime(&unixtime);
		//loctime=localtime(time(NULL));
//		printf("localtime = y %d d %d h %d m %d s %d dst %d \n", loctime->tm_year, loctime->tm_yday, loctime->tm_hour, loctime->tm_min, loctime->tm_sec, loctime->tm_isdst);
//		printf("ASCII Current local time and date: %s\n", asctime(loctime));



		python_datetime_2000=(double)(730120);
		python_datetime_years_since_2000=(double)(loctime->tm_year-100)*365;
		python_datetime_days=(double)(loctime->tm_yday);
		python_datetime_leapdays_since_2000=(float)((int)((loctime->tm_year-100)/4)); // saving to int effectivly rounds down!!!!!!!!!!!!!!!!!!!!!
		python_datetime_hour=((double)(loctime->tm_hour))/24;
		python_datetime_min=((double)(loctime->tm_min))/1440;
		python_datetime_sec=((double)(loctime->tm_sec))/86400;
/*		
		printf("python_datetime_2000 : %lf\n", python_datetime_2000);
		printf("python_datetime_years_since_2000 : %lf\n", python_datetime_years_since_2000);
		printf("python_datetime_days : %lf\n", python_datetime_days);
		printf("python_datetime_leapdays_since_2000 : %lf\n", python_datetime_leapdays_since_2000);
		printf("python_datetime_hour : %lf\n", python_datetime_hour);
		printf("python_datetime_min : %lf\n", python_datetime_min);
		printf("python_datetime_sec : %lf\n", python_datetime_sec);
*/	
		// python datetime year 2000 = 730120, (localtime years since 1900)-100, leapdays since 2000 are int of years/4,
		pythondatetime=1+python_datetime_2000
						+python_datetime_years_since_2000
						+python_datetime_days
						+python_datetime_leapdays_since_2000
						+python_datetime_hour
						+python_datetime_min
						+python_datetime_sec;
						
		// printf("pythondatetime = %lf \n", pythondatetime);

		



		//rc1 = sqlite3_open(dbPath, &db);
		//rc1 = sqlite3_open_v2(dbPath, &db, SQLITE_OPEN_FULLMUTEX | SQLITE_OPEN_READWRITE, NULL);
		//if(rc1){
		//	printf("Can't open database: %s. Missed value from %s\n", sqlite3_errmsg(db),topicName);
		//}else{
			// printf("Database opened: %s. Writing value from %s...\n", sqlite3_errmsg(db),topicName);

			//rc2=sqlite3_prepare_v2(db, sql, strlen(sql), &stmt, NULL);
			//if (rc2)
			//	printf("prepare return code %d: %s\n\n", rc2, sqlite3_errmsg(db));			

//if(strcmp(topic_type,"Temp")==0 || strcmp(topic_type,"BP")==0 || strcmp(topic_type,"Hum")==0)
//{
			printf("[SQLite] BEGIN  sqlite_transaction_flag=%d   sqlite_transaction_ctr=%d   sqlite_transaction_tmr=%d\n",sqlite_transaction_flag,sqlite_transaction_ctr,sqlite_transaction_tmr);



			if (sqlite_transaction_flag==0)
			{
				sqlite3_exec(db, "BEGIN TRANSACTION", NULL, NULL, &sqlErrMsg);
				if(sqlErrMsg)
					printf("[SQLite] Begin Transaction Error: %s\n", sqlErrMsg);
//				else
//					printf("[SQLite] Begin Transaction\n");
			
				sqlite_transaction_flag=1;
				sqlite_transaction_tmr=loctime->tm_min;
			}

			if (sqlite_transaction_flag==1)
			{
				sqlite_transaction_ctr++;
			}
			
//			sql_stmt_ptr=sql_stmt;




			// 2015 AUG
		        sprintf(sql, "INSERT INTO %s(Time, Value) VALUES(?, ?)", tableName);
//			printf("sql=%s\n",sql);
		        rc=sqlite3_prepare_v2(db, sql, strlen(sql), &sql_stmt, NULL);
		        if (rc)
		                printf("[SQLite] prepare return code %d: %s\n\n", rc, sqlite3_errmsg(db));
//			printf("sql_stmt=%s\n",sql_stmt);


			/*** BIND DATE ***/
                        rc2=sqlite3_bind_double(sql_stmt, 1, pythondatetime);
                        if (rc2)
                                printf("[SQLite] bind double return code %d: %s\n\n", rc2, sqlite3_errmsg(db));


                        /*** BIND VALUE ***/
                        if((strcmp(topic_type, "Temp")==0)||(strcmp(topic_type, "BP")==0)||(strcmp(topic_type, "Hum")==0))
                        {
                                rc2=sqlite3_bind_double(sql_stmt, 2, atof(buffer_msg));
                                if (rc2)
                                        printf("[SQLite] bind double return code %d: %s\n\n", rc2, sqlite3_errmsg(db)); 
                        }

                        if( (strcmp(topic_type, "Wifi")==0) || (strcmp(topic_type, "Contact")==0) )
                        {
                                rc2=sqlite3_bind_int(sql_stmt, 2, atoi(buffer_msg));
                                if (rc2)
                                        printf("[SQLite] bind int return code %d: %s\n\n", rc2, sqlite3_errmsg(db));
                        }
                        if((strcmp(topic_type, "Command")==0))
                        {
                                rc2=sqlite3_bind_text(sql_stmt, 2, buffer_msg, strlen(buffer_msg), 0);
                                if (rc2)
                                        printf("[SQLite] bind text return code %d: %s\n\n", rc2, sqlite3_errmsg(db));
                        }



/*
                        /*** BIND VALUE ***
                        if((strcmp(topic_type, "Temp")==0)||(strcmp(topic_type, "BP")==0)||(strcmp(topic_type, "Hum")==0))
                        {
				sql_stmt=sql_stmt_real;

                                rc2=sqlite3_bind_double(sql_stmt, 6, atof(buffer_msg));
                                if (rc2)
                                        printf("[SQLite] bind double return code %d: %s\n\n", rc2, sqlite3_errmsg(db)); 
                        }

                        if( (strcmp(topic_type, "Wifi")==0) || (strcmp(topic_type, "Contact")==0) )
                        {
				sql_stmt=sql_stmt_int;

                                rc2=sqlite3_bind_int(sql_stmt, 6, atoi(buffer_msg));
                                if (rc2)
                                        printf("[SQLite] bind int return code %d: %s\n\n", rc2, sqlite3_errmsg(db));
                        }
                        if((strcmp(topic_type, "Command")==0))
                        {
				sql_stmt=sql_stmt_text;

                                rc2=sqlite3_bind_text(sql_stmt, 6, buffer_msg, strlen(buffer_msg), 0);
                                if (rc2)
                                        printf("[SQLite] bind text return code %d: %s\n\n", rc2, sqlite3_errmsg(db));
			}

			*/

			/*** BIND TOPIC ***
			rc2=sqlite3_bind_text(sql_stmt, 1, topic_location, strlen(topic_location), 0);
				if (rc2)
					printf("[SQLite] bind text return code %d: %s\n\n", rc2, sqlite3_errmsg(db));

			rc2=sqlite3_bind_text(sql_stmt, 2, topic_area, strlen(topic_area), 0);
				if (rc2)
					printf("[SQLite] bind text return code %d: %s\n\n", rc2, sqlite3_errmsg(db));

			rc2=sqlite3_bind_text(sql_stmt, 3, topic_type, strlen(topic_type), 0);
				if (rc2)
					printf("[SQLite] bind text return code %d: %s\n\n", rc2, sqlite3_errmsg(db));

			rc2=sqlite3_bind_text(sql_stmt, 4, topic_id, strlen(topic_id), 0);
				if (rc2)
					printf("bind text return code %d: %s\n\n", rc2, sqlite3_errmsg(db));
			*/

			/*** BIND DATE **
			rc2=sqlite3_bind_double(sql_stmt, 5, pythondatetime);
			if (rc2)
				printf("[SQLite] bind double return code %d: %s\n\n", rc2, sqlite3_errmsg(db));

			*/

			/*** SQL COMMANDS ***/
			rc2=sqlite3_step(sql_stmt);  // Run SQL INSERT
			if (rc2 && (rc2 != 101))
				printf("[SQLite] step return code %d: %s\n\n", rc2, sqlite3_errmsg(db));
//			printf("[SQLite]  Step completed\n");

			rc2=sqlite3_clear_bindings(sql_stmt);
			if (rc2)
				printf("[SQLite] reset return code %d: %s\n\n", rc2, sqlite3_errmsg(db));
//			printf("[SQLite]  Clear bindings completed\n");
			
			rc2=sqlite3_reset(sql_stmt); // Clear statement handle for next use
			if (rc2)
				printf("[SQLite] reset return code %d: %s\n\n", rc2, sqlite3_errmsg(db));
//			printf("[SQLite]  Reset completed\n");

			if (sqlite_transaction_flag==2)
			{
				sqlite3_exec(db, "END TRANSACTION", NULL, NULL, &sqlErrMsg);
				if(sqlErrMsg)
					printf("[SQLite] End Transaction Error: %s\n\n", sqlErrMsg);
				sqlite_transaction_flag=0;
				sqlite_transaction_ctr=0;
				sqlite_transaction_tmr=loctime->tm_min;
			}
			else
			{
//				printf("[SQLite]  Transaction skipped.\n");

				// SQLITE TRANSACTION COUNTER HOOK - comment out to disconnect counter
				if ((sqlite_transaction_ctr==SQLITE_TRANSACTION_COUNT)||(loctime->tm_min-sqlite_transaction_tmr>=SQLITE_TRANSACTION_INTERVAL))
					sqlite_transaction_flag=2;
			}
			//printf("[SQLite] END   sqlite_transaction_flag=%d   _ctr=%d   _tmr=%d   loctime->tm_min=%d   diff=%d\n",sqlite_transaction_flag,sqlite_transaction_ctr,sqlite_transaction_tmr,loctime->tm_min,loctime->tm_min-sqlite_transaction_tmr);

//}				

#endif

	
    MQTTClient_freeMessage(&message);
    MQTTClient_free(topicName);
    loop_active_msg_thread=0;
//	printf("loop_active_msg_thread=%d\n", loop_active_msg_thread);
	end=clock();
//	cpu_time_used = ((double) (end - start)) / CLOCKS_PER_SEC;
//	printf("start %d, end %d, CPU time %f\n", start, end, cpu_time_used);
//	fp = fopen ("Logger_msgarrvd.txt", "a");
//	fprintf(fp, "%f\n",cpu_time_used);
//	fclose(fp);
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

int nrf_int=0;

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
	rf24_write_address(radio, 	RX_ADDR_P1, 0x77EE33FF11);	// warning: integer constant is too large for ‘long’ type [-Wlong-long]
	rf24_write_address(radio, 	RX_ADDR_P0, 0x11FF33EE77);	// warning: integer constant is too large for ‘long’ type [-Wlong-long]
	rf24_write_address(radio, 	RX_ADDR_P2, 0xc3c3c3c3c3);	// warning: integer constant is too large for ‘long’ type [-Wlong-long]
	rf24_write_address(radio, 	RX_ADDR_P3, 0x4242424242);	// warning: integer constant is too large for ‘long’ type [-Wlong-long]
	rf24_write_address(radio,	RX_ADDR_P4, 0x5656565656);	// warning: integer constant is too large for ‘long’ type [-Wlong-long]
	rf24_write_address(radio, 	RX_ADDR_P5, 0x112233FF44);	// warning: integer constant is too large for ‘long’ type [-Wlong-long]
	// the problem was MSWord / LSWord

	rf24_write_register(radio, 	STATUS, 	_BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );		//    CLEAR INTERRUPTS
}



void rx(rf24_t *radio, MQTTClient *client)
{
	int16_t temps[16];
	uint8_t tctr,dctr=0,i, len, done, reg, pipe, data[32]={1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
	char buffer_char[256];
	char smscmd[500];
 	
	MQTTClient_message pubmsg = MQTTClient_message_initializer;
 	MQTTClient_deliveryToken token;
	pubmsg.qos = QOS;
	pubmsg.retained = 0;
	deliveredtoken = 0;

	FILE *fp;

/*
	//MQTT
    char MQTT_topic[256]={0};
	MQTTClient client;
    MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;
    MQTTClient_message pubmsg = MQTTClient_message_initializer;
    MQTTClient_deliveryToken token;
	pubmsg.qos = QOS;
	pubmsg.retained = 0;
	deliveredtoken = 0;
*/

	// POWERUP & START LISTENING
//	rf24_write_register(radio, CONFIG, rf24_read_register(radio, CONFIG) | _BV(PWR_UP) | _BV(PRIM_RX));
//	rf24_write_register(radio, STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );
//	usleep(4500);

//	while (1)
//	{
	
		// LISTENING
//		gpio_write(radio->ce_pin, GPIO_PIN_HIGH);

		//fprintf(stderr, "\r[RX] Waiting for interrupt...  ");
		//while (!(rf24_get_status(radio) & _BV(RX_DR)));	
		
		// GOT INTERRUPT
		
		//rf24_dump_ext(radio,'r');
	
	 
		// PROCESS DATA PIPES
		pipe=(uint8_t)((reg >> RX_P_NO) & 0b111);
	/*
		if (pipe<6)	{
			fprintf(stderr, "reading pipe %d...\n",pipe);
			rf24_receive(radio, &buf, rf24_read_register(radio, pipe_payload_size_registers[i]));
		} else {
			fprintf(stderr, "reading fifo...\n");
			rf24_receive(radio, &data, PAYLOAD_SIZE);
			
		}
	*/
		//  while( !( rf24_receive(radio, &data, PAYLOAD_SIZE) ) );	//  RETURNS STATUS OF RX_EMPTY BIT -  but it doesnt go low when data is received
		
		rf24_receive(radio, &data, PAYLOAD_SIZE); 	// RECEIVE

/* 		
		gpio_write(radio->csn_pin, GPIO_PIN_LOW);
		data[0]=spi_transfer(radio->spi, R_RX_PAYLOAD);
		
		for (i=1;i<PAYLOAD_SIZE;i++)
			data[i]=spi_transfer(radio->spi, 0xFF);
		pipe=(uint8_t)((reg >> RX_P_NO) & 0b111);	// GET PIPE
		
		gpio_write(radio->csn_pin, GPIO_PIN_HIGH);
		gpio_write(radio->ce_pin, GPIO_PIN_LOW);
		
 */


		// PRINT DATA
//		fprintf(stderr, "\r                                                                                                                  ");
//		fprintf(stderr, "\r[RECEIEVED]   P%2d S%2d   data", pipe, PAYLOAD_SIZE);
//		for (i=0;i<PAYLOAD_SIZE+2;i++)
//			fprintf(stderr, " %5d ",data[i]);
//		fprintf(stderr, "\n");
		
		// *** CONVERT ***************
		for (dctr=0,tctr=0;dctr<PAYLOAD_SIZE;dctr+=2,tctr++)
		{
			temps[tctr] = (int16_t)(0xFF*data[dctr]) + (int16_t)data[dctr+1];
//			fprintf(stderr, "\r[converting]      dctr %d   tctr %d   data[dctr] %d   data[dctr+1] %2d   temps[tctr] %d           \n", dctr, tctr, data[dctr], data[dctr+1], temps[tctr]);
		}
		
		for (;tctr<16;tctr++);
			temps[tctr] = 0;

		if (data[7]==70 || data[7]==86) {
			fprintf(stderr, "FISH ON!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
			sprintf(smscmd, "curl -H %cAuthorization: Token 90b9c796a85fa2c936ad5f7f6eb1f2869e712ca7%c -H %cContent-Type: application/json%c -X POST -d %c{ %c%cbody%c%c : %c%cFISH ON and water temp is %.1dC%c%c, %c%cmessage_type%c%c : %c%ctext/plain%c%c }%c http:%c%capi.pushetta.com%capi%cpushes%cFraTol%c",
			0x22,0x22,0x22,0x22,0x22,
			0x5C,0x22,0x5C,0x22,0x5C,0x22,
			temps[0]/10,
			0x5C,0x22,0x5C,0x22,0x5C,0x22,0x5C,0x22,0x5C,0x22,
			0x22,
			0x2F,0x2F,0x2F,0x2F,0x2F,0x2F);
			system(smscmd);
		}	
			
		// PRINT CONVERSION
//		fprintf(stderr, "\r                                                                                                                  ");
//		fprintf(stderr, "\r[CONVERTED]   P%2d S%2d   temps ", pipe, PAYLOAD_SIZE/2);
//		for (tctr=0;tctr<PAYLOAD_SIZE/2;tctr++)
//			fprintf(stderr, " %5d ",temps[tctr]);
//		fprintf(stderr, "\n");

		// PRINT DATA TO FILE
//		fp=fopen("tempdata.txt","a");
//		fprintf(fp,     "P%d S%d ", pipe, PAYLOAD_SIZE/2);
//		for (tctr=0;tctr<PAYLOAD_SIZE/2;tctr++)
//			fprintf(fp,     "%d ",temps[tctr]);
//		fprintf(fp,     "\n");
		//usleep(10000);
//		fclose(fp);		// CLOSE FILE

        sprintf(buffer_char,"%f",((float)temps[0])/10);
        pubmsg.payload = buffer_char;
        pubmsg.payloadlen = strlen(buffer_char);
        MQTTClient_publishMessage(client, "Cottage/Lake/Temp/FrankDock1ft", &pubmsg, &token);

	//}
	
	return;
}

void nrf_int_handler (void)
{
	printf("[nrf int handler] nrf_int=%d -> ", nrf_int);
	nrf_int++;
	printf("%d \n", nrf_int);
}	  // nRF

#endif

/*
 *********************************************************************************
 * main
 *********************************************************************************
 */

int main(void)
{
	char hostname[20]={0};
	//int gotOne, pin ;	// isr
	//int myCounter [8] ;	// isr
	int i;	// address counter	// isr
	struct timeval mark_time, now;//time
	long int seconds;			// time
	//time_t mark_time, now;	// time
	//double seconds;			// time
	//unsigned int t=0;			// time
	//QueueList  object;	// c-generic-library
	//data_queue_t data;	// c-generic-library
	int delay_time2;

	#ifdef HANDLECTLC
		signal(SIGINT, intHandler);
	#endif

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
        #elif HOST == Pi5
                strcpy(hostname,"Pi5\0");
	#else
		strcpy(hostname,"unknown\0");
	#endif
	printf("HOST = %s\n ",hostname);

#ifdef SQLITE

        //rc = sqlite3_open(dbPath, &db);
        rc = sqlite3_open_v2(dbPath, &db, SQLITE_OPEN_FULLMUTEX | SQLITE_OPEN_READWRITE, NULL);
        if(rc)
        	printf("Can't open database: %s.", sqlite3_errmsg(db));

/*
        sprintf(sql_real, "INSERT INTO Real(Location, Area, Type, ID, DateTime, Value) VALUES(?, ?, ?, ?, ?, ?)");
        sprintf(sql_int, "INSERT INTO Int(Location, Area, Type, ID, DateTime, Value) VALUES(?, ?, ?, ?, ?, ?)");
        sprintf(sql_text, "INSERT INTO Text(Location, Area, Type, ID, DateTime, Value) VALUES(?, ?, ?, ?, ?, ?)");
        //sprintf(sql, "INSERT INTO Real(Location, Area, Type, ID, DateTime, Value) VALUES(?, ?, ?, ?, ?, ?)");

        rc=sqlite3_prepare_v2(db, sql_real, strlen(sql_real), &sql_stmt_real, NULL);
        if (rc)
                printf("[SQLite] prepare return code %d: %s\n\n", rc, sqlite3_errmsg(db));

        rc=sqlite3_prepare_v2(db, sql_int, strlen(sql_int), &sql_stmt_int, NULL);
        if (rc)
              printf("[SQLite] prepare return code %d: %s\n\n", rc, sqlite3_errmsg(db));

        rc=sqlite3_prepare_v2(db, sql_text, strlen(sql_text), &sql_stmt_text, NULL);
        if (rc)
	      printf("[SQLite] prepare return code %d: %s\n\n", rc, sqlite3_errmsg(db));

        rc=sqlite3_prepare_v2(db, sql, strlen(sql), &sql_stmt, NULL);
        if (rc)
                printf("[SQLite] prepare return code %d: %s\n\n", rc, sqlite3_errmsg(db));
*/
#endif	// SQLITE

#ifdef MQTT
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
	
	//MQTTClient_subscribe(client, "+", QOS);

	// MQTT Subscriptions - this is a crashing MQTT client!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	//  [MQTT] Connection lost, cause: (null)

	#ifdef SQLITE
		
		MQTTClient_subscribe(client, "Home/#", QOS);
		MQTTClient_subscribe(client, "Cottage/#", QOS);
		MQTTClient_subscribe(client, "Systems/#", QOS);

/*
		MQTTClient_subscribe(client, "Home/Garage/Temp/001", QOS);
		MQTTClient_subscribe(client, "Home/Garage/BP/001", QOS);
		MQTTClient_subscribe(client, "Home/Garage/Hum/001", QOS);
		MQTTClient_subscribe(client, "Home/Garage/Contact/DoorSide", QOS);
		MQTTClient_subscribe(client, "Home/Garage/Contact/DoorNear", QOS);
		MQTTClient_subscribe(client, "Home/Garage/Contact/DoorMid", QOS);
		MQTTClient_subscribe(client, "Home/Garage/Contact/DoorFar", QOS);
		MQTTClient_subscribe(client, "Home/Outside/Temp/001", QOS);
		MQTTClient_subscribe(client, "Systems/Pi1/Temp/CPUTemp", QOS);
		MQTTClient_subscribe(client, "Systems/Pi1/Wifi/Qual", QOS);
		MQTTClient_subscribe(client, "Systems/Pi1/Wifi/Level", QOS);
		MQTTClient_subscribe(client, "Systems/Pi1/Wifi/Noise", QOS);
		MQTTClient_subscribe(client, "Systems/Pi2/Temp/CPUTemp", QOS);
		MQTTClient_subscribe(client, "Systems/Pi2/Wifi/Qual", QOS);
		MQTTClient_subscribe(client, "Systems/Pi2/Wifi/Level", QOS);
		MQTTClient_subscribe(client, "Systems/Pi2/Wifi/Noise", QOS);
		MQTTClient_subscribe(client, "Systems/Pi3/Temp/CPUTemp", QOS);
		MQTTClient_subscribe(client, "Systems/Pi3/Temp/Case", QOS);
		MQTTClient_subscribe(client, "Systems/Pi3/Temp/Wifi", QOS);
		MQTTClient_subscribe(client, "Systems/Pi3/Wifi/Qual", QOS);
		MQTTClient_subscribe(client, "Systems/Pi3/Wifi/Level", QOS);
		MQTTClient_subscribe(client, "Systems/Pi3/Wifi/Noise", QOS);
		MQTTClient_subscribe(client, "Systems/Pi4/Temp/CPUTemp", QOS);
		MQTTClient_subscribe(client, "Systems/Pi4/Temp/Temp1", QOS);
		MQTTClient_subscribe(client, "Systems/Pi4/Temp/Temp2", QOS);
		MQTTClient_subscribe(client, "Systems/Pi4/Wifi/Qual", QOS);
		MQTTClient_subscribe(client, "Systems/Pi4/Wifi/Level", QOS);
		MQTTClient_subscribe(client, "Systems/Pi4/Wifi/Noise", QOS);
		MQTTClient_subscribe(client, "Home/Garage/Command/Relays", QOS);
*/	
	
	#endif 	// SQLITE
#endif	// MQTT

	


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

//	for (pin = 0 ; pin < 8 ; ++pin) 
//		globalCounter [pin] = myCounter [pin] = 0 ;

	wiringPiSetup();

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
		//uint8_t ctr=0, rx_buffer[32]={0}, reg, input;
		//uint64_t pipe_addresses[2] = { 0x1122334455LL,  0x1122334400LL };
		rf24_t radio;

        // HOST
        #if HOST == Pi1
                rf24_initialize(&radio, RF24_SPI_DEV_0, 25, 8);
        #elif HOST == Pi2
                rf24_initialize(&radio, RF24_SPI_DEV_0, 25, 8);
        #elif HOST == Pi3
                rf24_initialize(&radio, RF24_SPI_DEV_0, 25, 7);
        #elif HOST == Pi4
                rf24_initialize(&radio, RF24_SPI_DEV_0, 25, 7);
        #elif HOST == Pi5
                rf24_initialize(&radio, RF24_SPI_DEV_0, 25, 7);
        #else
                rf24_initialize(&radio, RF24_SPI_DEV_0, 25, 7);
        #endif

		rf24_reset_status(&radio);
		rf24_configure(&radio);

		// POWERUP & START LISTENING
		rf24_write_register(&radio, CONFIG, rf24_read_register(&radio, CONFIG) | _BV(PWR_UP) | _BV(PRIM_RX));
		rf24_write_register(&radio, STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );
		usleep(4500);
		gpio_write(radio.ce_pin, GPIO_PIN_HIGH);

		wiringPiISR (11, INT_EDGE_FALLING, &nrf_int_handler); // nRF hardware interrupt - Pi1/2/3/4 

//		wiringPiISR (11, INT_EDGE_FALLING, &nrf_int_handler(radio) ); // nRF hardware interrupt - Pi1/2/3/4 
//			examples/Logger.c:1362:3: error: incompatible type for argument 1 of ‘nrf_int_handler’
//			examples/Logger.c:1070:6: note: expected ‘struct rf24_t *’ but argument is of type ‘rf24_t’


//		wiringPiISR (11, INT_EDGE_FALLING, &nrf_int_handler(&(radio)) ); // nRF hardware interrupt - Pi1/2/3/4 

	#endif


	while (loop_active_main)
	{
//		gotOne = 0 ;

		printf("[Logger] Service local data collection\n");

		gettimeofday(&now,NULL);
		gettimeofday(&mark_time,NULL);
		seconds=now.tv_sec-mark_time.tv_sec;
		//time(&now);
		//time(&mark_time);

		// ****************************
		//      MAGNETIC CONTACTS
		// ****************************

#ifdef CONTACTS

#if HOST == Pi1
        sprintf(buffer_char,"%d",!digitalRead(STATUS1));
        pubmsg.payload = buffer_char;
        pubmsg.payloadlen = strlen(buffer_char);
        MQTTClient_publishMessage(client, "Cottage/Mike/Contact/WindowBack", &pubmsg, &token);

        sprintf(buffer_char,"%d",!digitalRead(STATUS2));
        pubmsg.payload = buffer_char;
        pubmsg.payloadlen = strlen(buffer_char);
        MQTTClient_publishMessage(client, "Cottage/Mike/Contact/WindowSide", &pubmsg, &token);


#elif HOST == Pi2
        sprintf(buffer_char,"%d",!digitalRead(STATUS1));
        pubmsg.payload = buffer_char;
        pubmsg.payloadlen = strlen(buffer_char);
        MQTTClient_publishMessage(client, "Cottage/Mike/Contact/WindowFront", &pubmsg, &token);

        sprintf(buffer_char,"%d",!digitalRead(STATUS2));
        pubmsg.payload = buffer_char;
        pubmsg.payloadlen = strlen(buffer_char);
        MQTTClient_publishMessage(client, "Cottage/Mike/Contact/Door", &pubmsg, &token);


#elif HOST == Pi3
		sprintf(buffer_char,"%d",!digitalRead(STATUS1));
		pubmsg.payload = buffer_char;
		pubmsg.payloadlen = strlen(buffer_char);
		MQTTClient_publishMessage(client, "Home/Garage/Contact/DoorSide", &pubmsg, &token);

		sprintf(buffer_char,"%d",!digitalRead(STATUS2));
		pubmsg.payload = buffer_char;
		pubmsg.payloadlen = strlen(buffer_char);
		MQTTClient_publishMessage(client, "Home/Garage/Contact/DoorNear", &pubmsg, &token);

		sprintf(buffer_char,"%d",!digitalRead(STATUS3));
		pubmsg.payload = buffer_char;
		pubmsg.payloadlen = strlen(buffer_char);
		MQTTClient_publishMessage(client, "Home/Garage/Contact/DoorMid", &pubmsg, &token);

		sprintf(buffer_char,"%d",!digitalRead(STATUS4));
		pubmsg.payload = buffer_char;
		pubmsg.payloadlen = strlen(buffer_char);
		MQTTClient_publishMessage(client, "Home/Garage/Contact/DoorFar", &pubmsg, &token);

#elif HOST == Pi5
		sprintf(buffer_char,"%d",!digitalRead(STATUS1));
		pubmsg.payload = buffer_char;
		pubmsg.payloadlen = strlen(buffer_char);
		MQTTClient_publishMessage(client, "Home/Mike/Contact/001", &pubmsg, &token);

		sprintf(buffer_char,"%d",!digitalRead(STATUS2));
		pubmsg.payload = buffer_char;
		pubmsg.payloadlen = strlen(buffer_char);
		MQTTClient_publishMessage(client, "Home/Mike/Contact/002", &pubmsg, &token);

		sprintf(buffer_char,"%d",!digitalRead(STATUS3));
		pubmsg.payload = buffer_char;
		pubmsg.payloadlen = strlen(buffer_char);
		MQTTClient_publishMessage(client, "Home/Mike/Contact/003", &pubmsg, &token);

		sprintf(buffer_char,"%d",!digitalRead(STATUS4));
		pubmsg.payload = buffer_char;
		pubmsg.payloadlen = strlen(buffer_char);
		MQTTClient_publishMessage(client, "Home/Mike/Contact/004", &pubmsg, &token);

#endif

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

#ifdef CPUTEMP
		// ****************************
		//          CPU Temp
		// ****************************
		// Read CPU Temp
		buffer_f=0;
		fp = fopen ("/sys/class/thermal/thermal_zone0/temp", "r");
		if (fp == NULL)
			printf("[Logger] Unable to open file.\n");
		fscanf (fp, "%lf", &buffer_lf);
		buffer_lf /= 1000;
		//printf ("[Logger] local Pi CPU temp is %.3f C.\n", buffer_lf);
		fclose (fp);
		sprintf(buffer_char, "%.3f\0", buffer_lf);

		// MQTT local Pi CPU temp
		sprintf(MQTT_topic,"Systems/%s/Temp/CPUTemp\0",hostname);
		pubmsg.payload = buffer_char;
		pubmsg.payloadlen = strlen(buffer_char);
		MQTTClient_publishMessage(client, MQTT_topic, &pubmsg, &token);
		//printf("[MQTT] %s on topic %s\n", pubmsg.payload, "Systems/Pi1/Temp/CPUTemp");
#endif	// CPUTEMP

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

			sprintf(MQTT_topic,"Systems/%s/Wifi/Level\0",hostname);

			//buffer_u8=((iw_statistics *)wreq.u.data.pointer)->qual.level - 256;

			//printf("wstats.qual.qual : %d\n",wstats.qual.qual);
			//printf("wstats.qual.level : %d\n",wstats.qual.level);
			//printf("wstats.qual.noise : %d\n",wstats.qual.noise);

			sprintf(MQTT_topic,"Systems/%s/Wifi/Qual\0",hostname);
			sprintf(buffer_char, "%d\0", wstats.qual.qual);
			pubmsg.payload = buffer_char;
			pubmsg.payloadlen = strlen(buffer_char);
			MQTTClient_publishMessage(client, MQTT_topic, &pubmsg, &token);

			sprintf(MQTT_topic,"Systems/%s/Wifi/Level",hostname);
			sprintf(buffer_char, "%d\0", wstats.qual.level);
			pubmsg.payload = buffer_char;
			pubmsg.payloadlen = strlen(buffer_char);
			MQTTClient_publishMessage(client, MQTT_topic, &pubmsg, &token);

			sprintf(MQTT_topic,"Systems/%s/Wifi/Noise\0",hostname);
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

#endif	// WIFI

#ifdef BMP180

		// ****************************
		//      BAROMETRIC PRESSURE
		// ****************************

		buffer_lf=0;
		buffer_lf=getPres(fd_bmp180);
		sprintf(buffer_char, "%.3lf\0", buffer_lf);
		//printf ("[BMP180] local Pi external pressure sensor %.3lf\n", buffer_lf);

		// MQTT local bmp180 pressure sensor
		pubmsg.payload = buffer_char;
		pubmsg.payloadlen = strlen(buffer_char);

#if HOST == Pi1
		MQTTClient_publishMessage(client, "Cottage/Mike/BP/001", &pubmsg, &token);
#elif HOST == Pi2
		MQTTClient_publishMessage(client, "Cottage/Mike/BP/002", &pubmsg, &token);
#elif HOST == Pi3
		MQTTClient_publishMessage(client, "Home/Garage/BP/001", &pubmsg, &token);
#elif HOST == Pi5
		MQTTClient_publishMessage(client, "Home/Mike/BP/001", &pubmsg, &token);
#endif
#endif	// BMP180

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

#if HOST == Pi1
		MQTTClient_publishMessage(client, "Cottage/Mike/Hum/001", &pubmsg, &token);
#elif HOST == Pi2
		MQTTClient_publishMessage(client, "Home/Garage/Hum/001", &pubmsg, &token);
#elif HOST == Pi3
		MQTTClient_publishMessage(client, "Home/Garage/Hum/001", &pubmsg, &token);
#elif HOST == Pi5
		MQTTClient_publishMessage(client, "Home/Mike/Hum/001", &pubmsg, &token);
#endif

#endif




#ifdef ONE_WIRE
		// ****************************
		//          ONE WIRE
		// ****************************

		
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
// Pi2 cd /sys/bus/w1/devices/
// 28-0000007c750d  28-0000015dd661  28-000005e50d0a  28-000005e9a47f  28-000005e9e919  28-000005ea9920  28-000005eac141  w1_bus_master1

			if (!strcmp(devNode->devID, "28-0000007c750d"))
                                MQTTClient_publishMessage(client, "Cottage/Mike/Temp/UnderCabin1", &pubmsg, &token);	// Cottage/Mike/Temp/001 - close to splice
                        else if (!strcmp(devNode->devID, "28-0000015dd661"))
				MQTTClient_publishMessage(client, "Cottage/Mike/Temp/UnderCabin2", &pubmsg, &token);	// Cottage/Mike/Temp/002 - dangling
			else if (!strcmp(devNode->devID, "28-000005e50d0a"))
				MQTTClient_publishMessage(client, "Cottage/Mike/Temp/Earth", &pubmsg, &token);	// Cottage/Mike/Temp/003 - waterproof buried in sand
			else if (!strcmp(devNode->devID, "28-000005e9a47f"))
				MQTTClient_publishMessage(client, "Cottage/Mike/Temp/Floor1", &pubmsg, &token);	// Cottage/Mike/Temp/004
			else if (!strcmp(devNode->devID, "28-000005e9e919"))
				MQTTClient_publishMessage(client, "Cottage/Mike/Temp/Ceiling1", &pubmsg, &token);	// Cottage/Mike/Temp/005 - ceiling
			else if (!strcmp(devNode->devID, "28-000005ea9920"))
                                MQTTClient_publishMessage(client, "Cottage/Mike/Temp/Ceiling2", &pubmsg, &token);	// Cottage/Mike/Temp/006 - ceiling
			else if (!strcmp(devNode->devID, "28-000005eac141"))
                                MQTTClient_publishMessage(client, "Cottage/Mike/Temp/Floor2", &pubmsg, &token);	// Cottage/Mike/Temp/007 - floor
#endif
#if HOST == Pi3
			if (!strcmp(devNode->devID, "28-04146d2647ff"))
				MQTTClient_publishMessage(client, "Systems/Pi3/Temp/Case", &pubmsg, &token);
			else if (!strcmp(devNode->devID, "28-031467a707ff"))
				MQTTClient_publishMessage(client, "Systems/Pi3/Temp/Wifi", &pubmsg, &token);
			else if (!strcmp(devNode->devID, "28-04146cd4e5ff"))
				MQTTClient_publishMessage(client, "Home/Garage/Temp/001", &pubmsg, &token);
			else if (!strcmp(devNode->devID, "28-04146cf597ff"))
				MQTTClient_publishMessage(client, "Home/Outside/Temp/001", &pubmsg, &token);
#endif
#if HOST == Pi5
			if (!strcmp(devNode->devID, "28-04146c93f6ff"))
				MQTTClient_publishMessage(client, "Home/Mike/Temp/001", &pubmsg, &token);
			else if (!strcmp(devNode->devID, "28-04146d1220ff"))
				MQTTClient_publishMessage(client, "Home/Mike/Temp/002", &pubmsg, &token);
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
				MQTTClient_publishMessage(client, "Systems/Temperature/Pi4Unknown", &pubmsg, &token);
			
			printf("[1wire] MQTT publish complete.\n");
			
			// Free current devNode.
			free(devNode);

			printf("[1wire] devNode freed.\n");

		}
		
		
		// Now free rootNode
		free(rootNode);

		

		*/
		
		while(rootNode)
		{
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
		
#endif	// ONEWIRE
		
		
		//printf("[Logger] entering loop, mark_time = %d, now = %d, seconds = %d\n", mark_time.tv_sec, now.tv_sec, seconds) ; fflush (stdout) ;
		////printf("[Logger] entering loop, mark_time = %lf, now = %lf, seconds = %lf\n", mark_time, now, seconds) ; fflush (stdout) ;
		
		for (; seconds<10;)
		{
			gettimeofday(&now,NULL);
			//time(&now);
			seconds=now.tv_sec-mark_time.tv_sec;
/*
			if ( nrf_int_rx_dr )
			{
				nrf_int_rx_dr--;
				printf("RX_DR processing...");
				rx(&radio);
			}		
			if ( nrf_int_tx_ds )	
			{
				nrf_int_tx_ds--;
				printf("TX_DS processing...");
			}
			if ( nrf_int_max_rt )
			{
				nrf_int_max_rt--;
				printf("MAX_RT processing...");
			}

*/
#ifdef NRF
			if(nrf_int)
			{
				printf("[nRF] Interrupt... nrf_int=%d, ",nrf_int);
				if ( rf24_get_status(&radio) & _BV(RX_DR) )
				{	
					printf("RX_DR\n");
//					rx(&radio, &client, &pubmsg, &token);
					rx(&radio, client);
					rf24_write_register(&radio,	STATUS,	_BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );
					nrf_int--;
				}
				else if ( rf24_get_status(&radio) & _BV(TX_DS) )	
				{
					printf("TX_DS\n");
					nrf_int--;
				}
				else if ( rf24_get_status(&radio) & _BV(MAX_RT) )
				{
					printf("MAX_RT\n");
					nrf_int--;
				}
				else
				{
					printf("no flag set in nRF24 module... resetting nrf_int=0\n");
					nrf_int=0;
				}
			}

#endif	// NRF
			//printf("[Logger] looping, mark_time = %d, now = %d, seconds = %d\n", mark_time.tv_sec, now.tv_sec, seconds) ; fflush (stdout) ;
			
//			if(rf24_isrFlag)
//				for (;;)
//					print("[Logger] rf24_isrFlag \n");

/*
			if((rf24_get_status(&radio_global) & _BV(RX_DR)))
				printf("[Logger] nrf24 RX_DR bit set\n");
*/
				
/*				
			for (pin = 0 ; pin < 8 ; ++pin)
			{
				if (globalCounter [pin] != myCounter [pin])
				{
					printf("[Logger] Int on pin %d: Counter: %5d\n", pin, globalCounter [pin]) ;
					myCounter [pin] = globalCounter [pin] ;
					++gotOne ;
					switch (pin) {
						#ifdef NRF
						case 2:	printf("[Logger] nRF receiving data...\n");
								
								rx(&radio);
							
							rf24_receive(&radio_global, &data.d, 32);
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
*/
			
			//if (gotOne != 0)
			//break ;
		}	// FOR
		//printf("[Logger] exiting loop, mark_time = %d, now = %d, seconds = %d\n", mark_time.tv_sec, now.tv_sec, seconds) ; fflush (stdout) ;
		
	}	//



	while(loop_active_msg_thread) {}
	
#ifdef SQLITE
	for(rc=1,delay_time2=100;rc=1&&delay_time2<1000;delay_time2+=100)
	{
		rc = sqlite3_close(db);
		// If rc is not 0, there was an error
		if(rc)
		{
			printf("Can't close database, return code $d: %s. Delaying %dms...\n", rc, sqlite3_errmsg(db),delay_time2);
			// Can't close database: unable to close due to unfinalised statements. -> SOLVED by sqlite3_finalize(stmt);
			delay(delay_time2); // 100ms more delay each loop
		}
		else
		{
			printf("Database closed, return code %d: %s\n", rc, sqlite3_errmsg(db));
		} // if(rc)
	
	} // for(rc...)
#endif	// 	while (loop_active_main)
	
	return 0 ;
}	// MAIN
