// Set the register EN_AA = 0x00 and ARC = 0 to disable Enhanced ShockBurst™.
// In addition, the nRF24L01+ air data rate must be set to 1Mbps or 250kbps.

// RX_P_NO - Data pipe number for the payload available for reading from RX_FIFO    111: RX FIFO Empty

#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

#include <dirent.h> // added for ow_temp
#include <fcntl.h>  // added for ow_temp 

#include "rf24.h"
#include "nRF24L01.h"
#include "gpio.h"
//#include "ow_temp.h"

#define _BV(x) (1 << (x))
#define _BN(x, n) ( ( (unsigned char *)(&(x)) )[(n)] )

#define PAYLOAD_SIZE 8

static const uint8_t pipe_address_registers[]      = { RX_ADDR_P0, RX_ADDR_P1, RX_ADDR_P2, RX_ADDR_P3, RX_ADDR_P4, RX_ADDR_P5 };
static const uint8_t pipe_payload_size_registers[] = { RX_PW_P0, RX_PW_P1, RX_PW_P2, RX_PW_P3, RX_PW_P4, RX_PW_P5 };
static const uint8_t pipe_enable_registers[]       = { ERX_P0, ERX_P1, ERX_P2, ERX_P3, ERX_P4, ERX_P5 };

int ow_temp(void) {
 DIR *dir;
 struct dirent *dirent;
 char dev[16];      // Dev ID
 char devPath[128]; // Path to device
 char buf[256];     // Data from device
 char tmpData[6];   // Temp C * 1000 reported by device 
 char path[] = "/sys/bus/w1/devices"; 
 ssize_t numRead;
 
 dir = opendir (path);
 if (dir != NULL)
 {
  while ((dirent = readdir (dir)))
   // 1-wire devices are links beginning with 28-
   if (dirent->d_type == DT_LNK && 
     strstr(dirent->d_name, "28-") != NULL) { 
    strcpy(dev, dirent->d_name);
    printf("\nDevice: %s\n", dev);
   }
        (void) closedir (dir);
        }
 else
 {
  perror ("Couldn't open the w1 devices directory");
  return 1;
 }

 // Assemble path to OneWire device
 sprintf(devPath, "%s/%s/w1_slave", path, dev);
 // Read temp continuously
 // Opening the device's file triggers new reading
 while(1) {
  int fd = open(devPath, O_RDONLY);
  if(fd == -1)
  {
   perror ("Couldn't open the w1 device.");
   return 1;   
  }
  while((numRead = read(fd, buf, 256)) > 0) 
  {
   strncpy(tmpData, strstr(buf, "t=") + 2, 5); 
   float tempC = strtof(tmpData, NULL);
   printf("Device: %s  - ", dev); 
   printf("Temp: %.3f C  ", tempC / 1000);
   printf("%.3f F\n\n", (tempC / 1000) * 9 / 5 + 32);
  }
  close(fd);
 } 
        /* return 0; --never called due to loop */
}

void send_pong(void * data)
{
  char buf[32];
  uint8_t i, len, done;
  rf24_t * radio = (rf24_t *) data;
  
  rf24_sync_status(radio);

  fprintf(stderr, "[rf24 pong callback] Got IRQ on pin %d. TX ok: %d, TX fail: %d RX ready: %d RX_LEN: %d PIPE: %d\n",
      radio->irq_pin,
      radio->status.tx_ok,
      radio->status.tx_fail_retries,
      radio->status.rx_data_available,
      radio->status.rx_data_len,
      radio->status.rx_data_pipe
      );

  if (radio->status.rx_data_available) {
    len = radio->status.rx_data_len;

    rf24_receive(radio, &buf, len);
    usleep(20);
    rf24_reset_status(radio);

    fprintf(stderr, "[rf24 pong callback] Data len: %d, data: %s\n", len, buf);

    rf24_stop_listening(radio);
    rf24_send(radio, &buf, 8 * sizeof(uint8_t));
    rf24_start_listening(radio);
  } else {
	fprintf(stderr, "[rf24 pong callback] no rx data\n");
	rf24_reset_status(radio);
	fprintf(stderr, "[rf24 pong callback] status reset\n");
  }
  return;
}





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

	rf24_write_address(radio, 	TX_ADDR,    0x77EE33FFFF);
	rf24_write_address(radio, 	RX_ADDR_P0, 0x77EE33FF11);
	rf24_write_address(radio, 	RX_ADDR_P1, 0x11FF33EE77);
	rf24_write_address(radio, 	RX_ADDR_P2, 0xc3c3c3c3c3);
	rf24_write_address(radio, 	RX_ADDR_P3, 0x4242424242);
	rf24_write_address(radio,	RX_ADDR_P4, 0x5656565656);
	rf24_write_address(radio, 	RX_ADDR_P5, 0x112233FF44);
	// the problem was MSWord / LSWord

	rf24_write_register(radio, 	STATUS, 	_BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );		//    CLEAR INTERRUPTS
}


/*
void data2temps(void * d, void * t)
{
	uint8_t i=0;
	uint8_t * dpos, * tpos;
	
	dpos = (uint8_t *) d;
	tpos = (uint8_t *) t;
	
	while (len--) {
		*tpos++ = spi_transfer(this->spi, 0xFF);
	}
	while (blanks--) {
		spi_transfer(this->spi, 0xFF);
	}
	
	for (i=0;i<PAYLOAD_SIZE/2;i++);
	{
		fprintf(stderr, "*dpos %d    *dpos+1 %d     *(buf+1)  %d\n", *dpos, *(dpos+1),);
		*tpos++ = 16*(*dpos++) + (*dpos++);
		
		fprintf(stderr, "*tpos %d \n", *(tpos-1));
		
		buf++;
	}
}
*/

	
void rx(rf24_t *radio)
{
	int16_t temps[16];
	uint8_t tctr,dctr=0,i, len, done, reg, pipe, data[32]={1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
	char smscmd[500];

	FILE *fp;

	
	//printf("%s\n\n", smscmd);
	
	//sprintf(smscmd, "curl http:%c%ctextbelt.com%cCanada -d number=9059754781 -d \"message=FISH ON!!!!!!!!\"",0x2F,0x2F,0x2F);
	//printf("curl http:%c%ctextbelt.com%cCanada -d number=9059754781 -d \"message=FISH ON!!!!!!!!\"",0x2F,0x2F,0x2F);

	//rf24_start_listening(radio);
	
	
	
	// POWERUP & START LISTENING
	rf24_write_register(radio, CONFIG, rf24_read_register(radio, CONFIG) | _BV(PWR_UP) | _BV(PRIM_RX));
	rf24_write_register(radio, STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );
	usleep(4500);

	
	
	rf24_dump_ext(radio,'a');
	rf24_dump_ext(radio,'r');
	
	while (1)
	{
	
		// LISTENING
		gpio_write(radio->ce_pin, GPIO_PIN_HIGH);

		fprintf(stderr, "\r[RX] Waiting for interrupt...  ");
		while (!(rf24_get_status(radio) & _BV(RX_DR)));	
		
		// GOT INTERRUPT
		
		//rf24_dump_ext(radio,'r');
		
		
		
	/* 
		// PROCESS DATA PIPES
		pipe=(uint8_t)((reg >> RX_P_NO) & 0b111);
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
		fprintf(stderr, "\r                                                                                                                  ");
		fprintf(stderr, "\r[RECEIEVED]   P%2d S%2d   data", pipe, PAYLOAD_SIZE);
		for (i=0;i<PAYLOAD_SIZE+2;i++)
			fprintf(stderr, " %5d ",data[i]);
		fprintf(stderr, "\n");
		
		// *** CONVERT ***************
		for (dctr=0,tctr=0;dctr<PAYLOAD_SIZE;dctr+=2,tctr++)
		{
			temps[tctr] = (int16_t)(0xFF*data[dctr]) + (int16_t)data[dctr+1];
			fprintf(stderr, "\r[converting]      dctr %d   tctr %d   data[dctr] %d   data[dctr+1] %2d   temps[tctr] %d           \n", dctr, tctr, data[dctr], data[dctr+1], temps[tctr]);
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
		fprintf(stderr, "\r                                                                                                                  ");
		fprintf(stderr, "\r[CONVERTED]   P%2d S%2d   temps ", pipe, PAYLOAD_SIZE/2);
		for (tctr=0;tctr<PAYLOAD_SIZE/2;tctr++)
			fprintf(stderr, " %5d ",temps[tctr]);
		fprintf(stderr, "\n");

		// PRINT DATA TO FILE
		fp=fopen("tempdata.txt","a");
		fprintf(fp,     "P%d S%d ", pipe, PAYLOAD_SIZE/2);
		for (tctr=0;tctr<PAYLOAD_SIZE/2;tctr++)
			fprintf(fp,     "%d ",temps[tctr]);
		fprintf(fp,     "\n");
		//usleep(10000);
		fclose(fp);		// CLOSE FILE

		rf24_write_register(radio, 	STATUS, 	_BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );
	}
	
	return;
}

			/*
			switch(pipe) {
				case 0:
					rf24_receive(&radio, &buf, rf24_read_register(&radio, RX_PW_P0));
					break;
				case 1:
					rf24_receive(&radio, &buf, rf24_read_register(&radio, RX_PW_P1));
					break;
				case 2:
					rf24_receive(&radio, &buf, rf24_read_register(&radio, RX_PW_P2));
					break;
				case 3:
					rf24_receive(&radio, &buf, rf24_read_register(&radio, RX_PW_P3));
					break;
				case 4:
					rf24_receive(&radio, &buf, rf24_read_register(&radio, RX_PW_P4));
					break;
				case 5:
					rf24_receive(&radio, &buf, rf24_read_register(&radio, RX_PW_P5));
					break;
				case 7:
					// FIFO Empty
					rf24_receive(&radio, &buf, rf24_read_register(&radio, RX_PW_P0));
					break;
			}
			*/


void tx(rf24_t *radio)
{
	char buf[32];
	uint8_t i, len, done, reg, pipe;
	
	// POWERUP
//	rf24_write_register(radio, CONFIG, rf24_read_register(radio, CONFIG) | _BV(PWR_UP) | _BV(PRIM_RX));
//	usleep(4500);	/* wait for the radio to come up (130us actually only needed) */
//	gpio_write(radio->ce_pin, GPIO_PIN_HIGH);	
  
	rf24_write_register(radio, CONFIG, ( rf24_read_register(radio, CONFIG) | _BV(PWR_UP) ) & ~_BV(PRIM_RX) );
	usleep(4500);	/* wait for the radio to come up (130us actually only needed) */

	rf24_dump_ext(radio,'t');
	// TRANSMITTING
	while(1)
	{
		rf24_write_register(radio, 	STATUS, 	_BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );	// RESET INTS
		
		rf24_write_payload(radio, W_TX_PAYLOAD, buf, len);	// TRANSMIT
  
		gpio_write(radio->ce_pin, GPIO_PIN_HIGH);	// CE HIGH
	
		rf24_dump_ext(radio,'t');
		fprintf(stderr, "[TX] Waiting for int...\r");
		
		while (!(rf24_get_status(radio) & _BV(TX_DS)))
				fprintf(stderr, ".");
		
		gpio_write(radio->ce_pin, GPIO_PIN_LOW);  
		fprintf(stderr, "[TX] transmitting                        \r");
		
		usleep(1000);

	}
	return;
}




int main(void)
{
	uint8_t ctr=0, rx_buffer[32]={0}, reg, input;
	uint64_t pipe_addresses[2] = { 0x1122334455LL,  0x1122334400LL };
	rf24_t radio;
	
	fprintf(stderr, "[nrf] init...\n\r");
	// INIT SPI AND RADIO
	rf24_initialize(&radio, RF24_SPI_DEV_0, 25, 4); //CE=25, IRQ=27, CSN=8   .........IRQ was 4
	fprintf(stderr, "[nrf] initialzed\n\r");
	rf24_reset_status(&radio);
	fprintf(stderr, "[nrf] status reset\n\r");
	rf24_configure(&radio);
	fprintf(stderr, "[nrf] configured\n\r");

	// DISPLAY
	rf24_dump_ext(&radio,'a');
	rf24_dump_ext(&radio,'h');
	
	while (1)
	{
		input=getchar();
		if (input=='l'||input=='L')
		{
				rx(&radio);
		}
		else if (input=='k'||input=='K')
		{
				tx(&radio);
		}
		else if (input=='g'||input=='G')
		{
				rf24_configure(&radio);
		}
		else if (input=='1'||input=='1')
		{
				ow_temp();
		}
		else
		{
			rf24_dump_ext(&radio,input);
			usleep(10000);
		}
	}
	return 0;
}
