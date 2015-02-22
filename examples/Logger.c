/*
 * isr.c:
 *	Wait for Interrupt test program - ISR method
 *
 *	How to test:
 *	  Use the SoC's pull-up and pull down resistors that are avalable
 *	on input pins. So compile & run this program (via sudo), then
 *	in another terminal:
 *		gpio mode 0 up
 *		gpio mode 0 down
 *	at which point it should trigger an interrupt. Toggle the pin
 *	up/down to generate more interrupts to test.
 *
 * Copyright (c) 2013 Gordon Henderson.
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

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <wiringPi.h>


#include <inttypes.h>	// MF copied from dump.c 7-Feb-2015
#include <unistd.h>		// MF copied from dump.c 7-Feb-2015

#include <time.h>		// timing

//#include "queue.h"				// c-generic-library
//#include "error_macros.h"		// c-generic-library
//#include "libcgeneric/libcgeneric.h"	// c-generic-library

#include "rf24.h"		// nRF
#include "nRF24L01.h"	// nRF
#include "gpio.h"		// nRF

#define PAYLOAD_SIZE 8	// nRF

// MF main loop
#define LOCAL_DATA_INTERVAL 2




// MF macros
#define _BV(x) (1 << (x))	
#define _BN(x, n) ( ( (unsigned char *)(&(x)) )[(n)] )


int32_t intcmp(const void *, const void *,size_t);	// c-generic-library
void  print(const void *);							// c-generic-library
void  ckfree(void *);								// c-generic-library
void *ckalloc(size_t);								// c-generic-library

// globalCounter:
//	Global variable to count interrupts
//	Should be declared volatile to make sure the compiler doesn't cache it.

static volatile int globalCounter [8] ;

// MF queue//
struct data_queue {
  uint8_t size;
  uint8_t d[32];
  uint8_t next;
};					

typedef struct data_queue data_queue_t;  // MF queue

data_queue_t data;	  // MF queue

rf24_t radio_global;	// nRF
	
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



 



/*
 *********************************************************************************
 * main
 *********************************************************************************
 */

int main (void)
{
	int gotOne, pin ;	// isr
	int myCounter [8] ;	// isr
	int i;	// address counter
	struct timeval mark_time, now;
	long int seconds;	// time
	//time_t mark_time, now;	// time
	//double seconds;	// time
	//unsigned int t=0;	// time
	unsigned int d[32]={0};
	//QueueList  object;	// c-generic-library
	//data_queue_t data;
	
		
	// ********* nRF *********
	//rf24_t radio_global; // replaced with radio_global
 
	rf24_initialize(&radio_global, RF24_SPI_DEV_0, 25, 4);
	rf24_reset_status(&radio_global);
	rf24_configure(&radio_global);
  
	
  
	for (pin = 0 ; pin < 8 ; ++pin) 
		globalCounter [pin] = myCounter [pin] = 0 ;

  
	wiringPiSetup();

	wiringPiISR (0, INT_EDGE_FALLING, &myInterrupt0) ; // home: open
	wiringPiISR (1, INT_EDGE_FALLING, &myInterrupt1) ; // home: open
	wiringPiISR (2, INT_EDGE_FALLING, &myInterrupt2) ; // home: nRF hardware interrupt
	//wiringPiISR (2, INT_EDGE_FALLING, &(void)rf24_receive(&radio_global, &d, 8)) ; // home: nRF hardware interrupt
	wiringPiISR (3, INT_EDGE_FALLING, &myInterrupt3) ; // home: open
	wiringPiISR (4, INT_EDGE_FALLING, &myInterrupt4) ; // home: open
	wiringPiISR (5, INT_EDGE_FALLING, &myInterrupt5) ; // home: open
	wiringPiISR (6, INT_EDGE_FALLING, &myInterrupt6) ; // home: open
	//wiringPiISR (7, INT_EDGE_FALLING, &myInterrupt7) ; // has one wire at home

	// POWERUP & START LISTENING
	rf24_write_register(&radio_global, CONFIG, rf24_read_register(&radio_global, CONFIG) | _BV(PWR_UP) | _BV(PRIM_RX));
	rf24_write_register(&radio_global, STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );
	usleep(4500);
	gpio_write(radio_global.ce_pin, GPIO_PIN_HIGH);
	
	
	for (;;)
	{
		gotOne = 0 ;

		printf("[Logger] Service local data collection\n");

		gettimeofday(&now,NULL);
		gettimeofday(&mark_time,NULL);
		seconds=now.tv_sec-mark_time.tv_sec;
		//time(&now);
		//time(&mark_time);
		printf("[Logger] entering loop, mark_time = %d, now = %d, seconds = %d\n", mark_time.tv_sec, now.tv_sec, seconds) ; fflush (stdout) ;
		//printf("[Logger] entering loop, mark_time = %lf, now = %lf, seconds = %lf\n", mark_time, now, seconds) ; fflush (stdout) ;

		for (; seconds<10;) {
			gettimeofday(&now,NULL);
			//time(&now);
			seconds=now.tv_sec-mark_time.tv_sec;

			//printf("[Logger] looping, mark_time = %d, now = %d, seconds = %d\n", mark_time.tv_sec, now.tv_sec, seconds) ; fflush (stdout) ;
			
//			if(rf24_isrFlag)
//				for (;;)
//					print("[Logger] rf24_isrFlag \n");

			if((rf24_get_status(&radio_global) & _BV(RX_DR)))
				printf("[Logger] nrf24 RX_DR bit set\n");

				
				
			for (pin = 0 ; pin < 8 ; ++pin) {
				if (globalCounter [pin] != myCounter [pin]) {
					printf("[Logger] Int on pin %d: Counter: %5d\n", pin, globalCounter [pin]) ;
					myCounter [pin] = globalCounter [pin] ;
					++gotOne ;
					switch (pin) {
						case '2':	rf24_receive(&radio_global, &data.d, 32);
									rf24_write_register(&radio_global, 	STATUS, 	_BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );
									for (i==0;i<32;i++)
										fprintf(stderr, " %5d ",data.d[i]);
									fprintf(stderr, "\n");
									
									break;
						default:	
									
									break;
					}
				}
			}
	
			
			if (gotOne != 0)
			break ;
		}
		printf("[Logger] exiting loop, mark_time = %d, now = %d, seconds = %d\n", mark_time.tv_sec, now.tv_sec, seconds) ; fflush (stdout) ;
		//printf("[Logger] exiting loop, mark_time = %lf, now = %lf, seconds = %lf\n", mark_time, now, seconds) ; fflush (stdout) ;
	}

	return 0 ;
}
