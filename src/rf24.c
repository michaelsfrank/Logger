/*
Modified by Michael Frank
7-Feb-2015

Added functions:
	void rf24_dump_ext(rf24_t * this, uint8_t cmd)
	void rf24_OnInterrupt(void)

Added variables:
	extern volatile uint8_t rf24_isrFlag=0;
	
Externalized functions by removing prototype statement and static from implicit definition:
	static uint8_t rf24_write_payload(rf24_t * this, uint8_t reg, void * buf, uint8_t len);
	static uint8_t rf24_read_register(rf24_t * this, uint8_t reg);
	static uint8_t rf24_write_register(rf24_t * this, uint8_t reg, uint8_t value);
	static uint8_t rf24_write_address(rf24_t * this, uint8_t pipe_reg, uint64_t address);
	static uint8_t rf24_get_status(rf24_t * this);

ORIGINAL COPYRIGHT:
	Copyright (C) 2011 J. Coliz <maniacbug@ymail.com>
	Portions Copyright (C) 2011 Greg Copeland
	Cleanup + C version 2014 by Balazs Kutil

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
version 2 as published by the Free Software Foundation.
*/

#include <assert.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "rf24.h"
#include "gpio.h"
#include "spi.h"
#include "nRF24L01.h"
#include "ow_temp.h"

#define _BV(x) (1 << (x))
#define _BN(x, n) ( ( (unsigned char *)(&(x)) )[(n)] )

// **************** MF ******************
extern volatile uint8_t rf24_isrFlag=0; /* flag set by ISR */
// **************** MF ******************

static const uint8_t pipe_address_registers[]      = { RX_ADDR_P0, RX_ADDR_P1, RX_ADDR_P2, RX_ADDR_P3, RX_ADDR_P4, RX_ADDR_P5 };
static const uint8_t pipe_payload_size_registers[] = { RX_PW_P0, RX_PW_P1, RX_PW_P2, RX_PW_P3, RX_PW_P4, RX_PW_P5 };
static const uint8_t pipe_enable_registers[]       = { ERX_P0, ERX_P1, ERX_P2, ERX_P3, ERX_P4, ERX_P5 };

static uint64_t now(void);
//static uint8_t rf24_write_payload(rf24_t * this, uint8_t reg, void * buf, uint8_t len);
//static uint8_t rf24_read_register(rf24_t * this, uint8_t reg);
//static uint8_t rf24_write_register(rf24_t * this, uint8_t reg, uint8_t value);
static uint64_t rf24_read_address(rf24_t * this, uint8_t pipe_reg);
//static uint8_t rf24_write_address(rf24_t * this, uint8_t pipe_reg, uint64_t address);
static void rf24_unmask_irqs(rf24_t * this);
//static uint8_t rf24_get_status(rf24_t * this);
static uint8_t rf24_flush_rx(rf24_t * this);
static uint8_t rf24_flush_tx(rf24_t * this);
static uint8_t rf24_get_data_rate(rf24_t * this);
static void rf24_enable_features(rf24_t * this);

static uint64_t now(void) {
  struct timeval now;
  gettimeofday(&now, NULL);
  return (now.tv_sec * 1000 + now.tv_usec/1000.0);
}

void rf24_open_reading_pipe(rf24_t * this, uint8_t pipe, uint64_t address)
{
  assert(pipe >= 0 && pipe <= 5);

  if (pipe == 0) {
    this->pipe0_address = address;
  }

  /* all pipes have 5 bytes configurable address, pipe 0 has unique 5 byte address, other pipes share first 4 bytes. */
  if (pipe == 0 || pipe == 1) {
    rf24_write_address(this, pipe_address_registers[pipe], address);
  } else {
    rf24_write_register(this, pipe_address_registers[pipe], (uint8_t) address);
  }
  rf24_write_register(this, pipe_payload_size_registers[pipe], this->payload_size);
  rf24_write_register(this, EN_RXADDR, rf24_read_register(this, EN_RXADDR) | _BV(pipe_enable_registers[pipe]));
}

void rf24_open_writing_pipe(rf24_t * this, uint64_t address)
{
  rf24_write_address(this, pipe_address_registers[0], address);
  rf24_write_address(this, TX_ADDR, address);
  rf24_write_register(this, pipe_payload_size_registers[0], this->payload_size);
}

void rf24_start_listening(rf24_t * this)
{
  rf24_write_register(this, CONFIG, rf24_read_register(this, CONFIG) | _BV(PWR_UP) | _BV(PRIM_RX));
  rf24_write_register(this, STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );

  if (this->pipe0_address) {
    rf24_write_address(this, RX_ADDR_P0, this->pipe0_address);
  }

  gpio_write(this->ce_pin, GPIO_PIN_HIGH);

  /* wait for the radio to come up (130us actually only needed) */
  usleep(130);
}

void rf24_stop_listening(rf24_t * this)
{
  gpio_write(this->ce_pin, GPIO_PIN_LOW);
  rf24_flush_rx(this);
  rf24_flush_tx(this);
}

void rf24_dump(rf24_t * this)
{
  uint64_t addr, txaddr;
  uint8_t status = rf24_get_status(this);
  uint8_t i, reg;

  fprintf(stderr, "[rf24] Device configuration\n");
  fprintf(stderr, "[rf24] CE: %d CS: %d\n", this->ce_pin, this->csn_pin);
  fprintf(stderr, "[rf24] P variant: %s\n", this->p_variant ? "yes" : "no");
  fprintf(stderr, "[rf24] STATUS register: 0x%02x RX_DR=%x TX_DS=%x MAX_RT=%x RX_P_NO=%x TX_FULL=%x\n",
      status,
      (status & _BV(RX_DR))?  1:0,
      (status & _BV(TX_DS))?  1:0,
      (status & _BV(MAX_RT))? 1:0,
      ((status >> RX_P_NO) & 0b111),
      (status & _BV(TX_FULL))? 1:0
      );

  for (i = 0; i < 6; i++) {
    addr = rf24_read_address(this, pipe_address_registers[i]);
    fprintf(stderr, "[rf24] RX_ADDR_P%d: %x %x %x %x %x\n", i, _BN(addr, 4), _BN(addr, 3), _BN(addr, 2), _BN(addr, 1), _BN(addr, 0));
  }

  for (i = 0; i < 6; i++) {
    reg = rf24_read_register(this, pipe_payload_size_registers[i]);
    fprintf(stderr, "[rf24] RX_PW_P%d: 0x%02x\n", i, reg);
  }

  reg = rf24_read_register(this, EN_RXADDR);
  for (i = 0; i < 6; i++) {
    fprintf(stderr, "[rf24] ERX_P%d: %d\n", i, (reg & _BV(pipe_enable_registers[i])) >> pipe_enable_registers[i]);
  }

  txaddr = rf24_read_address(this, TX_ADDR);
  fprintf(stderr, "[rf24] TX_ADDR: %x %x %x %x %x\n", _BN(txaddr, 4), _BN(txaddr, 3), _BN(txaddr, 2), _BN(txaddr, 1), _BN(txaddr, 0));

  fprintf(stderr, "[rf24] EN_RXADDR: 0x%02x\n", rf24_read_register(this, EN_RXADDR));
  fprintf(stderr, "[rf24] RF_CH: 0x%02x\n", rf24_read_register(this, RF_CH));
  fprintf(stderr, "[rf24] EN_AA: 0x%02x\n", rf24_read_register(this, EN_AA));
  fprintf(stderr, "[rf24] RF_SETUP: 0x%02x\n", rf24_read_register(this, RF_SETUP));
  fprintf(stderr, "[rf24] CONFIG: 0x%02x\n", rf24_read_register(this, CONFIG));
  fprintf(stderr, "[rf24] DYNPD, FEATURE: 0x%02x 0x%02x\n", rf24_read_register(this, DYNPD), rf24_read_register(this, FEATURE));

  reg = rf24_get_data_rate(this);
  fprintf(stderr, "[rf24] Data rate: %s\n", reg == RF24_250KBPS ? "250KBPS" : ((reg == RF24_1MBPS) ? "1MBPS" : "2MBPS"));

  reg = rf24_get_crc_length(this);
  fprintf(stderr, "[rf24] CRC: %s\n", reg == RF24_CRC_DISABLED ? "Disabled" : ((reg == RF24_CRC_8) ? "8bit" : "16bit"));
}


void rf24_dump_ext(rf24_t * this, uint8_t cmd)
{
  uint64_t addr;
  uint8_t i, reg, val8, enreg;

	if (cmd=='a'||cmd=='A')
	{
		fprintf(stderr, "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
		fprintf(stderr, "==============================================================================================================================\n");
		fprintf(stderr, "==============================================       RF24 DUMP      ==========================================================\n");
		fprintf(stderr, "==============================================================================================================================\n\n");
		fprintf(stderr, "[SPI] CE %d   CS %d", this->ce_pin, this->csn_pin);
		fprintf(stderr, "   \tP variant: %s\n", this->p_variant ? "yes" : "no");
	}
	
	if (cmd=='s'||cmd=='S'||cmd=='a'||cmd=='A')
	{
		reg = rf24_get_status(this);
		fprintf(stderr, "[STATUS] \t0x%02x   \tRX_DR %x   \tTX_DS %x   \tMAX_RT %x  \tRX_P_NO %x   \tTX_FULL %x\n",
			reg,
			(reg & _BV(RX_DR))?  1:0,
			(reg & _BV(TX_DS))?  1:0,
			(reg & _BV(MAX_RT))? 1:0,
			((reg >> RX_P_NO) & 0b111),
			(reg & _BV(TX_FULL))? 1:0
		);
	}
 
	if (cmd=='c'||cmd=='C'||cmd=='a'||cmd=='A')
	{
		reg=rf24_read_register(this, CONFIG);	  
		fprintf(stderr, "[CONFIG] \t0x%02x   \tMASK_RX_DR %x   \tMASK_TX_DS %x   \tMASK_MAX_RT %x   \tEN_CRC %x   \tCRCO %x    \tPWR_UP %x    \tPRIM_RX %x\n",
			reg,
			(reg & _BV(MASK_RX_DR))?  1:0,
			(reg & _BV(MASK_TX_DS))?  1:0,
			(reg & _BV(MASK_MAX_RT))? 1:0,
			(reg & _BV(EN_CRC))? 1:0,
			(reg & _BV(CRCO))? 1:0,
			(reg & _BV(PWR_UP))? 1:0,
			(reg & _BV(PRIM_RX))? 1:0
		);  
 	}
	
	if (cmd=='f'||cmd=='F'||cmd=='a'||cmd=='A')
	{
		reg=rf24_read_register(this, RF_SETUP);
		fprintf(stderr, "[RF SETUP] \t0x%02x   \tCW %x  \tRF_DR_L %x  \tPLL_LOCK %x  \tRF_DR_H %x  \tRF_PWR_H %x   \tRF_PWR_L %x\n",
			reg,
			(reg & _BV(CW))?  1:0,
			(reg & _BV(RF_DR_LOW))?  1:0,
			(reg & _BV(PLL_LOCK))? 1:0,
			(reg & _BV(RF_DR_HIGH))? 1:0,
			(reg & _BV(RF_PWR_HIGH))? 1:0,
			(reg & _BV(RF_PWR_LOW))? 1:0
		);  
	}
	
	if (cmd=='w'||cmd=='W'||cmd=='a'||cmd=='A')
	{
		// *****************   CONST WAVE   *****************
		fprintf(stderr, "[CONTWAVE]   \tRPD %d", rf24_read_register(this, RPD));
		reg=rf24_read_register(this, RF_CH);
		fprintf(stderr, "   \tChannel 0x%02x %4dMHz", reg, reg+2400);
		reg=rf24_read_register(this, RF_SETUP);
		fprintf(stderr, "   \tCW %x   \tPLL_LOCK %x   \tRF_PWR_H %x   \tRF_PWR_L %x",
			(reg & _BV(CW))?  1:0,
			(reg & _BV(PLL_LOCK))? 1:0,
			(reg & _BV(RF_PWR_HIGH))? 1:0,
			(reg & _BV(RF_PWR_LOW))? 1:0,
			(reg & _BV(PRIM_RX))? 1:0
		);  
		reg=rf24_read_register(this, CONFIG);	  
		fprintf(stderr, "    \tPWR_UP %x    \tPRIM_RX %x\n",
			reg,
			(reg & _BV(PWR_UP))? 1:0,
			(reg & _BV(PRIM_RX))? 1:0
		);  
	}
	
	if (cmd=='n'||cmd=='N'||cmd=='a'||cmd=='A')
	{
		// *****************   OTHER   *****************
		fprintf(stderr, "[OTHER] \tEN_AA 0x%02x", rf24_read_register(this, EN_AA));
		fprintf(stderr, "\tSETUP_RETR 0x%02x", rf24_read_register(this, SETUP_RETR));
		fprintf(stderr, "\tSETUP_AW 0x%02x", rf24_read_register(this, SETUP_AW));
		fprintf(stderr, "\tDYNPD 0x%02x", rf24_read_register(this, DYNPD));
		fprintf(stderr, "\tFEATURE 0x%02x", rf24_read_register(this, FEATURE));
		reg = rf24_get_crc_length(this);
		fprintf(stderr, "   \tCRC %s\n", reg == RF24_CRC_DISABLED ? "Disabled" : ((reg == RF24_CRC_8) ? "8bit" : "16bit"));
	}
	
	if (cmd=='r'||cmd=='R'||cmd=='d'||cmd=='D'||cmd=='a'||cmd=='A')
	{
		// *****************   RX RECEIVE   *****************
		reg = rf24_read_register(this, EN_RXADDR);
		fprintf(stderr, "[RX] \tEN_RXADDR 0x%02x", reg);
		fprintf(stderr, "\tFIFO STATUS 0x%02x", rf24_read_register(this, FIFO_STATUS));
		reg=rf24_read_register(this, RF_CH);
		fprintf(stderr, "\tRFCH 0x%02x %4dMHz", reg, reg+2400); 
		reg=rf24_read_register(this, RF_SETUP);
		fprintf(stderr, "\tRF_DR 0b%x%x",
			(reg & _BV(RF_DR_HIGH))? 1:0,
			(reg & _BV(RF_DR_LOW))?  1:0
			
		);
		
		if((reg & _BV(RF_DR_HIGH)) && (reg & _BV(RF_DR_LOW)))
			fprintf(stderr, " ERROR");
		else if(!(reg & _BV(RF_DR_HIGH)) && (reg & _BV(RF_DR_LOW)))
			fprintf(stderr, " 250KBPS");
		else if((reg & _BV(RF_DR_HIGH)) && !(reg & _BV(RF_DR_LOW)))
			fprintf(stderr, " 2MBPS");
		else if(!(reg & _BV(RF_DR_HIGH)) && !(reg & _BV(RF_DR_LOW)))
			fprintf(stderr, " 1MBPS"); 
			
		reg=rf24_read_register(this, CONFIG);	  
		fprintf(stderr, "   PWR_UP %x   PRIM_RX %x",
			(reg & _BV(PWR_UP))? 1:0,
			(reg & _BV(PRIM_RX))? 1:0
		);
		
		reg=rf24_read_register(this, STATUS);
		fprintf(stderr, "   \tRX_DR %x   \tRX_PIPE %x   \tRPD %d\n\r",
			(reg & _BV(RX_DR))?  1:0,
			((reg >> RX_P_NO) & 0b111),
			rf24_read_register(this, RPD)
		);
	
	}

	if (cmd=='d'||cmd=='D'||cmd=='a'||cmd=='A')
	{
		// *****************   RX RECEIVE ADDRESSES  *****************
		for (i = 0; i < 6; i++) {
			addr = rf24_read_address(this, pipe_address_registers[i]);
			reg = rf24_read_register(this, pipe_payload_size_registers[i]);
			fprintf(stderr, "[RX]\t\tERX_P%d %d\tRX_ADDR_P%d  %02x %02x %02x %02x %02x \tRX_PW_P%d 0x%02x\n",
				i,
				(enreg & _BV(pipe_enable_registers[i]))>>pipe_enable_registers[i],
				i,
				_BN(addr, 4),
				_BN(addr, 3),
				_BN(addr, 2),
				_BN(addr, 1),
				_BN(addr, 0),
				i,
				reg);
			/*	
			gpio_write(this->csn_pin, GPIO_PIN_LOW);
			fprintf(stderr, "[RX]\t\t         \tRX_ADDR     %02d %02d %02d %02d %02d %02d %02d\n\r",
			spi_transfer(this->spi, R_REGISTER | pipe_address_registers[i]),
			spi_transfer(this->spi, 0xFF),
			spi_transfer(this->spi, 0xFF),
			spi_transfer(this->spi, 0xFF),
			spi_transfer(this->spi, 0xFF),
			spi_transfer(this->spi, 0xFF),
			spi_transfer(this->spi, 0xFF)
			);
			gpio_write(this->csn_pin, GPIO_PIN_HIGH);
			
			gpio_write(this->csn_pin, GPIO_PIN_LOW);
			fprintf(stderr, "[RX]\t\t         \tRX_ADDR     %02d %02d %02d %02d %02d %02d %02d\n\r",
			spi_transfer(this->spi, R_REGISTER | RX_ADDR_P0+i),
			spi_transfer(this->spi, 0xFF),
			spi_transfer(this->spi, 0xFF),
			spi_transfer(this->spi, 0xFF),
			spi_transfer(this->spi, 0xFF),
			spi_transfer(this->spi, 0xFF),
			spi_transfer(this->spi, 0xFF)
			);
			gpio_write(this->csn_pin, GPIO_PIN_HIGH);
			*/
		}
	}

	if (cmd=='t'||cmd=='T'||cmd=='a'||cmd=='A')
	{
		// *****************   TX TRANSMIT   *****************
		fprintf(stderr, "[TX]   FIFO STATUS 0x%02x", rf24_read_register(this, FIFO_STATUS));
		
		reg=rf24_get_status(this);
		if(reg & _BV(RX_DR))
			fprintf(stderr, "\tRX_DR");
		if(reg & _BV(TX_DS))
			fprintf(stderr, "\tTX_DS");
		if(reg & _BV(MAX_RT))
			fprintf(stderr, "\tMAX_RT");
		if(reg & _BV(TX_FULL))
			fprintf(stderr, "\tTX_FULL");
		
		reg=rf24_read_register(this, RF_CH);
		fprintf(stderr, "\tRFCH 0x%02x %4dMHz", reg, reg+2400); 
		
		reg=rf24_read_register(this, RF_SETUP);
		fprintf(stderr, "\tRF_DR 0b%x%x",
			(reg & _BV(RF_DR_HIGH))? 1:0,
			(reg & _BV(RF_DR_LOW))?  1:0
			
		);
		
		if((reg & _BV(RF_DR_HIGH)) && (reg & _BV(RF_DR_LOW)))
			fprintf(stderr, " 1MBPS");
		else if(!(reg & _BV(RF_DR_HIGH)) && (reg & _BV(RF_DR_LOW)))
			fprintf(stderr, " 250KBPS");
		else if((reg & _BV(RF_DR_HIGH)) && !(reg & _BV(RF_DR_LOW)))
			fprintf(stderr, " 2MBPS");
		else if(!(reg & _BV(RF_DR_HIGH)) && !(reg & _BV(RF_DR_LOW)))
			fprintf(stderr, " ERROR"); 
		
		reg=rf24_read_register(this, CONFIG);	  
		fprintf(stderr, "\tPWR_UP %x\tPRIM_RX %x",
			(reg & _BV(PWR_UP))? 1:0,
			(reg & _BV(PRIM_RX))? 1:0
		);
		
		fprintf(stderr, "\tFIFO STATUS 0x%02x\n\r", rf24_read_register(this, FIFO_STATUS));
		
	}
	
	if (cmd=='d'||cmd=='D'||cmd=='a'||cmd=='A')
	{
		// *****************   TX RECEIVE ADDRESSES  *****************
		addr = rf24_read_address(this, TX_ADDR);
		fprintf(stderr, "[TX]\t\t         \tTX_ADDR     %02x %02x %02x %02x %02x \tSETUP_AW 0x%02x\n\r",
			_BN(addr, 4),
			_BN(addr, 3),
			_BN(addr, 2),
			_BN(addr, 1),
			_BN(addr, 0),
			rf24_read_register(this, SETUP_AW)
		);
		/*
		gpio_write(this->csn_pin, GPIO_PIN_LOW);
		fprintf(stderr, "[TX]\t\t         \tTX_ADDR     %02d %02d %02d %02d %02d %02d %02d\n\r",
			spi_transfer(this->spi, R_REGISTER | (REGISTER_MASK & TX_ADDR)),
			spi_transfer(this->spi, 0xFF),
			spi_transfer(this->spi, 0xFF),
			spi_transfer(this->spi, 0xFF),
			spi_transfer(this->spi, 0xFF),
			spi_transfer(this->spi, 0xFF),
			spi_transfer(this->spi, 0xFF)
		);
		gpio_write(this->csn_pin, GPIO_PIN_HIGH);
		*/
  
	}

	
//	int8_t spi_transfer(uint32_t fd, uint8_t payload)
	
/*
static uint64_t rf24_read_address(rf24_t * this, uint8_t pipe_reg)
{
  uint64_t address = 0;
  uint8_t * cur    = (uint8_t *) &address;
  uint8_t len      = 5;

  gpio_write(this->csn_pin, GPIO_PIN_LOW);
  spi_transfer(this->spi, R_REGISTER | (REGISTER_MASK & pipe_reg));
  while (len--) {
    *cur++ = spi_transfer(this->spi, 0xFF);
  }
  gpio_write(this->csn_pin, GPIO_PIN_HIGH);

  return address;
}
 */	
	
	if (cmd=='h'||cmd=='H')
	{
		fprintf(stderr, "[HELP]\n");
		fprintf(stderr, "    REGISTERS\n");
		fprintf(stderr, "        a   all\n");
		fprintf(stderr, "        s   status\n");
		fprintf(stderr, "        c   config\n");
		fprintf(stderr, "        f   rf setup\n");
		fprintf(stderr, "        r   receive\n");
		fprintf(stderr, "        t   transmit\n");
		fprintf(stderr, "        d   tx/rx addresses\n");
		fprintf(stderr, "    PROGRAMS\n");
		fprintf(stderr, "        g   configure\n");
		fprintf(stderr, "        w   continuous wave / carrier detect receive mode\n");
		fprintf(stderr, "        l   receive mode with active listen\n");
		fprintf(stderr, "        k   receive mode with active listen\n");
		fprintf(stderr, "        1   1-wire local temps\n");
		fprintf(stderr, "        o   other\n");
		fprintf(stderr, "        h   help\n");
	}
}

void rf24_OnInterrupt(void) {
	rf24_isrFlag=1; /* flag set by ISR */
}


uint8_t rf24_write_payload(rf24_t * this, uint8_t reg, void * buf, uint8_t len)
{
  uint8_t status, blanks;
  uint8_t * pos;

  if (!this->dynamic_payloads_enabled) { assert(len <= this->payload_size); }
  assert(reg == W_TX_PAYLOAD || this->ack_payload_enabled);

  blanks = this->dynamic_payloads_enabled ? 0 : this->payload_size - len;

  /* send the data out */
  gpio_write(this->csn_pin, GPIO_PIN_LOW);
  status = spi_transfer(this->spi, reg);

  pos = (uint8_t *) buf;
  while (len--) {
    spi_transfer(this->spi, *pos++);
  }
  if (reg == W_TX_PAYLOAD) {
    while (blanks--) {
      spi_transfer(this->spi, 0x0);
    }
  }
  gpio_write(this->csn_pin, GPIO_PIN_HIGH);
  return status;
}

void rf24_send_ack_on_pipe(rf24_t * this, uint8_t pipe_no, void * buf, uint8_t len)
{
  rf24_write_payload(this, (W_ACK_PAYLOAD | (pipe_no & 0b111)), buf, len);
}

void rf24_irq_poll(rf24_t * this, void(* callback)(void * radio))
{
  assert(this->irq_pin);
  fprintf(stderr, "[rf24] polling IRQ %d", this->irq_pin);
  if (gpio_poll(this->irq_pin, GPIO_EDGE_FALLING, callback, (void *) this) == -1) {
    fprintf(stderr, "[rf24] Error registering irq callback on pin %d\n", this->irq_pin);
  }
  fprintf(stderr, "[rf24] gpio complete");
  }

uint8_t rf24_send(rf24_t * this, void * buf, uint8_t len)
{
  uint64_t sent_at;
  uint32_t timeout;
  uint8_t  status;

  /* time to write */
  rf24_write_register(this, CONFIG, ( rf24_read_register(this, CONFIG) | _BV(PWR_UP) ) & ~_BV(PRIM_RX) );
  rf24_write_payload(this, W_TX_PAYLOAD, buf, len);

  /* Activate the TX mode for at least 10us (nRF24L01P_Product_spec, page 43 - Fig. 16) */
  gpio_write(this->ce_pin, GPIO_PIN_HIGH);
  usleep(10);
  gpio_write(this->ce_pin, GPIO_PIN_LOW);

  /* FIXME: looks like according section 7.7, there is Tstdby 130us before Time on air and TX_DS irq, so we could sleep */
  /* now poll for finished tx */
  timeout = this->tx_timeout;
  sent_at = now();

  do {
    status = rf24_read_register(this, OBSERVE_TX);
  } while (! (status & ( _BV(TX_DS) | _BV(MAX_RT) ) ) && ( (now() - sent_at) < timeout ) );

  rf24_sync_status(this);
  return this->status.tx_ok;
}

void rf24_sync_status(rf24_t * this)
{
  uint8_t status  = rf24_read_register(this, STATUS) & (_BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT));
  uint8_t pipe_no = (status >> RX_P_NO) & 0b111;

  this->status.tx_ok                 = status & _BV(TX_DS);
  this->status.tx_fail_retries       = status & _BV(MAX_RT);
  this->status.rx_data_available     = status & _BV(RX_DR);
  this->status.rx_dyn_data_len       = this->dynamic_payloads_enabled == 1 ? rf24_get_dynamic_payload_size(this) : 0;
  this->status.rx_data_len           = this->dynamic_payloads_enabled == 1 ? 0 : this->payload_size;
  this->status.rx_data_pipe          = pipe_no;
}

void rf24_reset_status(rf24_t * this)
{
  rf24_write_register(this, STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT));
  memset(&(this->status), 0, sizeof(this->status));
}

uint8_t rf24_receive(rf24_t * this, void * buf, uint8_t len)
{
  uint8_t blanks;
  uint8_t * pos;

  if (!this->dynamic_payloads_enabled) { assert(len <= this->payload_size); }
  blanks = this->dynamic_payloads_enabled == 1 ? 0 : this->payload_size - len;

  gpio_write(this->csn_pin, GPIO_PIN_LOW);
  spi_transfer(this->spi, R_RX_PAYLOAD);

  pos = (uint8_t *) buf;
  while (len--) {
    *pos++ = spi_transfer(this->spi, 0xFF);
  }
  while (blanks--) {
    spi_transfer(this->spi, 0xFF);
  }
  gpio_write(this->csn_pin, GPIO_PIN_HIGH);

  return rf24_read_register(this, FIFO_STATUS) & _BV(RX_EMPTY);
}

uint8_t rf24_data_available(rf24_t * this)
{
  return rf24_data_available_on_pipe(this, NULL);
}

uint8_t rf24_data_available_on_pipe(rf24_t * this, uint8_t * pipe_number)
{
  uint8_t status, result;

  status = rf24_get_status(this);
  result = status & _BV(RX_DR);

  /*
  fprintf(stderr, "[rf24] status %d, RX_DR: %d TX_DS: %d MAX_RT: %d, RX_P_NO: %d TX_FULL: %d\n",
      status,
      result,
      status & _BV(TX_DS),
      status & _BV(MAX_RT),
      (status >> RX_P_NO) & 0b111,
      status & _BV(TX_FULL)
  );
  */

  if (pipe_number != NULL) {
    *pipe_number = (status >> RX_P_NO) & 0b111;

    rf24_write_register(this, STATUS, _BV(RX_DR));
    if (status & _BV(TX_DS)) {
      rf24_write_register(this, STATUS, _BV(TX_DS));
    }
  }

  return result;
}



rf24_t * rf24_new(char * spi_dev, uint8_t ce_pin, uint8_t irq_pin)
{
  rf24_t * this = NULL;
  if ((this = malloc(sizeof(rf24_t))) == NULL) {
    return NULL;
  }
  if (rf24_initialize(this, spi_dev, ce_pin, irq_pin) == -1) {
    rf24_delete(this);
    return NULL;
  }

  return this;
}

//      rf24_initialize(&radio,        RF24_SPI_DEV_0, 25,             4);
uint8_t rf24_initialize(rf24_t * this, char * spi_dev, uint8_t ce_pin, uint8_t irq_pin)
{
  memset(this, 0, sizeof(rf24_t));

  this->ce_pin = ce_pin;
  this->irq_pin = irq_pin;
  this->tx_timeout = 500;
  this->payload_size = 32;

  if (strcmp(spi_dev, RF24_SPI_DEV_0) == 0) {
    this->csn_pin = 8;
  } else {
    this->csn_pin = 7;
  }

  if ((this->spi = spi_open(spi_dev)) == -1) {
    return -1;
  }

  gpio_export_wait(this->ce_pin);
  gpio_export_wait(this->csn_pin);
  gpio_set_direction(this->ce_pin, GPIO_PIN_OUTPUT);
  gpio_set_direction(this->csn_pin, GPIO_PIN_OUTPUT);

  /* Minimum ideal SPI bus speed is 2x data rate
   * If we assume 2Mbs data rate and 16Mhz clock, a
   * divider of 4 is the minimum we want.
   * CLK:BUS 8Mhz:2Mhz, 16Mhz:4Mhz, or 20Mhz:5Mhz
   */
  spi_config(this->spi, 8, 8000000, 0);

  gpio_write(this->ce_pin, GPIO_PIN_LOW);
  gpio_write(this->csn_pin, GPIO_PIN_HIGH);

  /* Must allow the radio time to settle else configuration bits will not necessarily stick.
   * This is actually only required following power up but some settling time also appears to
   * be required after resets too. For full coverage, we'll always assume the worst.
   * Enabling 16b CRC is by far the most obvious case if the wrong timing is used - or skipped.
   * Technically we require 4.5ms + 14us as a worst case. We'll just call it 5ms for good measure.
   * WARNING: Delay is based on P-variant whereby non-P *may* require different timing.
   */

  /* enforce chip reset */
  rf24_reset(this);

  usleep(5000);

  /* Set 1500uS (minimum for 32B payload in ESB@250KBPS) timeouts, to make testing a little easier
   * WARNING: If this is ever lowered, either 250KBS mode with AA is broken or maximum packet
   * sizes must never be used. See documentation for a more complete explanation.
   */

  rf24_write_register(this, SETUP_RETR, (0b0101 << ARD) | (0b1111 << ARC));

  rf24_set_pa_level(this, RF24_PA_MAX);

  /* Determine if this is a p or non-p RF24 module and then
   * reset our data rate back to default value. This works
   * because a non-P variant won't allow the data rate to
   * be set to 250Kbps.
   */
  rf24_set_data_rate(this, RF24_250KBPS);
  this->p_variant = (rf24_get_data_rate(this) == RF24_250KBPS);

  /* Then set the data rate to the slowest (and most reliable) speed supported by all hardware */
  rf24_set_data_rate(this, RF24_1MBPS);

  rf24_set_crc_length(this, RF24_CRC_16);

  /* Disable dynamic payloads, to match dynamic_payloads_enabled setting */
  rf24_write_register(this, DYNPD, 0);

  /* Notice reset and flush is the last thing we do */
  rf24_write_register(this, STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );

  /* This channel should be universally safe and not bleed over into adjacent spectrum. */
  rf24_set_channel(this, 76);

  /* unmask irqs */
  rf24_unmask_irqs(this);

  rf24_flush_rx(this);
  rf24_flush_tx(this);

  return 0;
}

uint8_t rf24_delete(rf24_t * this)
{
  assert(this != NULL);

  if (this->spi != -1) { spi_close(this->spi); }

  free(this);
  this = NULL;

  return 0;
}

uint8_t rf24_read_register(rf24_t * this, uint8_t reg)
{
  uint8_t value;

  gpio_write(this->csn_pin, GPIO_PIN_LOW);
  spi_transfer(this->spi, R_REGISTER | ( REGISTER_MASK & reg ) );
  value = spi_transfer(this->spi, 0xFF);
  gpio_write(this->csn_pin, GPIO_PIN_HIGH);

  return value;
}

static uint64_t rf24_read_address(rf24_t * this, uint8_t pipe_reg)
{
  uint64_t address = 0;
  uint8_t * cur    = (uint8_t *) &address;
  uint8_t len      = 5;

  gpio_write(this->csn_pin, GPIO_PIN_LOW);
  spi_transfer(this->spi, R_REGISTER | (REGISTER_MASK & pipe_reg));
  while (len--) {
    *cur++ = spi_transfer(this->spi, 0xFF);
  }
  gpio_write(this->csn_pin, GPIO_PIN_HIGH);

  return address;
}

uint8_t rf24_write_address(rf24_t * this, uint8_t pipe_reg, uint64_t address)
{
  uint8_t * cur      = (uint8_t *) &address;
  uint8_t   addr_len = 5;
  uint8_t   status;

  gpio_write(this->csn_pin, GPIO_PIN_LOW);
  status = spi_transfer(this->spi, W_REGISTER | (REGISTER_MASK & pipe_reg));
  while (addr_len--) {
    spi_transfer(this->spi, *cur++);
  }
  gpio_write(this->csn_pin, GPIO_PIN_HIGH);

  return status;
}

static void rf24_unmask_irqs(rf24_t * this)
{
  rf24_write_register(this, CONFIG, rf24_read_register(this, CONFIG) & ~( _BV(MASK_RX_DR) | _BV(MASK_TX_DS) | _BV(MASK_MAX_RT) ));
}

uint8_t rf24_write_register(rf24_t * this, uint8_t reg, uint8_t value)
{
  uint8_t status;

  gpio_write(this->csn_pin, GPIO_PIN_LOW);
  status = spi_transfer(this->spi, W_REGISTER | ( REGISTER_MASK & reg ) );
  spi_transfer(this->spi, value);
  gpio_write(this->csn_pin, GPIO_PIN_HIGH);

  return status;
}

uint8_t rf24_get_dynamic_payload_size(rf24_t * this)
{
  uint8_t result = 0;
  gpio_write(this->csn_pin, GPIO_PIN_LOW);
  spi_transfer(this->spi, R_RX_PL_WID);
  result = spi_transfer(this->spi, 0xff);
  gpio_write(this->csn_pin, GPIO_PIN_HIGH);
  return result;
}

uint8_t rf24_get_status(rf24_t * this)
{
  uint8_t status;
  gpio_write(this->csn_pin, GPIO_PIN_LOW);
  status = spi_transfer(this->spi, NOP);
  gpio_write(this->csn_pin, GPIO_PIN_HIGH);
  return status;
}

void rf24_enable_ack_payload(rf24_t * this)
{
  uint8_t ack_payloads = rf24_read_register(this, FEATURE) | _BV(EN_ACK_PAY) | _BV(EN_DPL);

  rf24_write_register(this, FEATURE, ack_payloads);
  if (!rf24_read_register(this, FEATURE)) {
    rf24_enable_features(this);
    rf24_write_register(this, FEATURE, ack_payloads);
  }

  /* Spec states (page 63, table 28, note d) that for ack payloads, we should
   * enable dynamic payload length on pipe 0.
   * FIXME: find out why it is enabled on pipe 1 as well in the original lib
   */
  rf24_write_register(this, DYNPD, rf24_read_register(this, DYNPD) | _BV(DPL_P0));

  this->ack_payload_enabled = 1;
  this->dynamic_payloads_enabled |= _BV(DPL_P0);
}

void rf24_enable_dynamic_payloads(rf24_t * this)
{
  uint8_t dynamic_payloads = rf24_read_register(this, FEATURE) | _BV(EN_DPL);
  uint8_t dynamic_payloads_pipes = _BV(DPL_P5) | _BV(DPL_P4) | _BV(DPL_P3) | _BV(DPL_P2) | _BV(DPL_P1) | _BV(DPL_P0);

  /* enable the feature */
  rf24_write_register(this, FEATURE, dynamic_payloads);

  if (!rf24_read_register(this, FEATURE)) {
    rf24_enable_features(this);
    rf24_write_register(this, FEATURE, dynamic_payloads);
  }

  /* set dynamic payloads for all pipes */
  rf24_write_register(this, DYNPD, rf24_read_register(this, DYNPD) | dynamic_payloads_pipes);

  this->dynamic_payloads_enabled = dynamic_payloads_pipes;
}

static void rf24_enable_features(rf24_t * this)
{
  gpio_write(this->csn_pin, GPIO_PIN_LOW);
  spi_transfer(this->spi, ACTIVATE);
  spi_transfer(this->spi, 0x73);
  gpio_write(this->csn_pin, GPIO_PIN_HIGH);
}

static uint8_t rf24_flush_rx(rf24_t * this)
{
  uint8_t status;
  gpio_write(this->csn_pin, GPIO_PIN_LOW);
  status = spi_transfer(this->spi, FLUSH_RX);
  gpio_write(this->csn_pin, GPIO_PIN_HIGH);
  return status;
}


static uint8_t rf24_flush_tx(rf24_t * this)
{
  uint8_t status;
  gpio_write(this->csn_pin, GPIO_PIN_LOW);
  status = spi_transfer(this->spi, FLUSH_TX);
  gpio_write(this->csn_pin, GPIO_PIN_HIGH);
  return status;
}

void rf24_disable_crc(rf24_t * this)
{
  rf24_write_register(this, CONFIG, rf24_read_register(this, CONFIG) & ~_BV(EN_CRC) );
}

void rf24_set_crc_length(rf24_t * this, uint8_t crc_length)
{
  assert(crc_length == RF24_CRC_DISABLED || crc_length == RF24_CRC_8 || crc_length == RF24_CRC_16);

  uint8_t config = rf24_read_register(this, CONFIG) & ~( _BV(CRCO) | _BV(EN_CRC) );
  switch (crc_length) {
    case RF24_CRC_DISABLED:
      /* NOOP */
      break;
    case RF24_CRC_8:
      config |= _BV(EN_CRC);
      break;
    case RF24_CRC_16:
      config |= _BV(EN_CRC);
      config |= _BV(CRCO);
      break;
  }
  rf24_write_register(this, CONFIG, config);
}

uint8_t rf24_get_crc_length(rf24_t * this)
{
  uint8_t crc_config = rf24_read_register(this, CONFIG) & (_BV(CRCO) | _BV(EN_CRC));

  if (crc_config & _BV(EN_CRC)) {
    if (crc_config & _BV(CRCO)) {
      return RF24_CRC_16;
    } else {
      return RF24_CRC_8;
    }
  } else {
    return RF24_CRC_DISABLED;
  }
}

void rf24_set_pa_level(rf24_t * this, uint8_t pa_level)
{
  assert(pa_level == RF24_PA_MIN || pa_level == RF24_PA_LOW || pa_level == RF24_PA_HIGH || pa_level == RF24_PA_MAX);

  uint8_t setup = rf24_read_register(this, RF_SETUP) & ~(_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH));
  switch(pa_level) {
    case RF24_PA_MAX:
      setup |= (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH));
      break;
    case RF24_PA_HIGH:
      setup |= _BV(RF_PWR_HIGH);
      break;
    case RF24_PA_LOW:
      setup |= _BV(RF_PWR_LOW);
    case RF24_PA_MIN:
      /* NOOP */
      break;
  }
  rf24_write_register(this, RF_SETUP, setup);
}

void rf24_set_retries(rf24_t * this, uint8_t delay, uint8_t count)
{
  assert(delay >= 0 && delay <= 15 && count >= 0 && count <= 15);
  rf24_write_register(this, SETUP_RETR, (delay & 0xF << ARD) | (count & 0xf) << ARC);
}

void rf24_set_autoack(rf24_t * this, uint8_t autoack)
{
  if (autoack) {
    rf24_write_register(this, EN_AA, 0x3F);
  } else {
    rf24_write_register(this, EN_AA, 0);
  }
}

void rf24_set_autoack_for_pipe(rf24_t * this, uint8_t pipe, uint8_t autoack)
{
  assert(pipe >= 0 && pipe <= 5);

  uint8_t autoack_enable = rf24_read_register(this, EN_AA);
  if (autoack) {
    rf24_write_register(this, EN_AA, autoack_enable | _BV(pipe));
  } else {
    rf24_write_register(this, EN_AA, autoack_enable & ~_BV(pipe));
  }
}

void rf24_power_up(rf24_t * this)
{
  rf24_write_register(this, CONFIG, rf24_read_register(this, CONFIG) | _BV(PWR_UP));
  usleep(150);
}

void rf24_power_down(rf24_t * this)
{
  rf24_write_register(this, CONFIG, rf24_read_register(this, CONFIG) & ~_BV(PWR_UP));
  usleep(150);
}

void rf24_reset(rf24_t * this)
{
  rf24_write_register(this, CONFIG, 0xF);
}

void rf24_set_data_rate(rf24_t * this, uint8_t speed)
{
  assert(speed == RF24_250KBPS || speed == RF24_1MBPS || speed == RF24_2MBPS);

  uint8_t setup = rf24_read_register(this, RF_SETUP) & ~(_BV(RF_DR_LOW) | _BV(RF_DR_HIGH));
  switch (speed) {
    case RF24_250KBPS:
      setup |= _BV(RF_DR_LOW);
      break;
    case RF24_2MBPS:
      setup |= _BV(RF_DR_HIGH);
      break;
    case RF24_1MBPS:
      /* NOOP */
      break;
  }

  rf24_write_register(this, RF_SETUP, setup);
}

static uint8_t rf24_get_data_rate(rf24_t * this)
{
  uint8_t data_rate = rf24_read_register(this, RF_SETUP) & (_BV(RF_DR_LOW) | _BV(RF_DR_HIGH));
  switch (data_rate) {
    case _BV(RF_DR_LOW):  return RF24_250KBPS;
    case _BV(RF_DR_HIGH): return RF24_2MBPS;
    default:              return RF24_1MBPS;
  }
}

void rf24_set_channel(rf24_t * this, uint8_t channel)
{
  assert(channel >= 0 && channel <= 127);
  rf24_write_register(this, RF_CH, channel);
}

uint8_t rf24_get_payload_size(rf24_t * this)
{
  return this->payload_size;
}

void rf24_set_payload_size(rf24_t * this, uint8_t size)
{
  assert(size > 0 && size <= 32);
  this->payload_size = size;
}
// vim:ai:cin:et:sts=2 sw=2 ft=c
