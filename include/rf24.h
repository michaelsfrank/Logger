/*
Modified by Michael Frank
7-Feb-2015

Externalized functions by removing prototype statement and static from implicit definition:
	static uint8_t rf24_write_payload(rf24_t * this, uint8_t reg, void * buf, uint8_t len);
	static uint8_t rf24_read_register(rf24_t * this, uint8_t reg);
	static uint8_t rf24_write_register(rf24_t * this, uint8_t reg, uint8_t value);
	static uint8_t rf24_write_address(rf24_t * this, uint8_t pipe_reg, uint64_t address);
	static uint8_t rf24_get_status(rf24_t * this);

Added functions:
	void rf24_dump_ext(rf24_t * this, uint8_t cmd)
	void rf24_isr(void)
*/

#ifndef __RF24_H__
#define __RF24_H__

#include <inttypes.h>

/* CRC settings */
#define RF24_CRC_DISABLED 0
#define RF24_CRC_8        1
#define RF24_CRC_16       2

/* power level settings */
#define RF24_PA_MIN   0
#define RF24_PA_LOW   1
#define RF24_PA_HIGH  2
#define RF24_PA_MAX   3

/* data rates */
#define RF24_250KBPS  0
#define RF24_1MBPS    1
#define RF24_2MBPS    2

/* SPI device names */
#define RF24_SPI_DEV_0 "/dev/spidev0.0"
#define RF24_SPI_DEV_1 "/dev/spidev0.1"

struct rf24 {
  struct {
    uint8_t tx_ok, tx_fail_retries;
    uint8_t rx_data_available, rx_dyn_data_len, rx_data_len, rx_data_pipe;
  } status;
  uint64_t pipe0_address;
  uint32_t spi, tx_timeout;
  uint8_t csn_pin, ce_pin, irq_pin;
  uint8_t ack_payload_enabled, p_variant, dynamic_payloads_enabled, payload_size;
};

typedef struct rf24 rf24_t;

// ************************************************************************************
// ***************               MF added 7-Feb-2015                    ***************

/* Added functions */
void rf24ext_dump(rf24_t * this, uint8_t cmd);
void rf24_isr(void);

/* Externalized functions */
uint8_t rf24_get_status(rf24_t * this);
uint8_t rf24_read_register(rf24_t * this, uint8_t reg);
uint8_t rf24_write_register(rf24_t * this, uint8_t reg, uint8_t value);
uint8_t rf24_write_address(rf24_t * this, uint8_t pipe_reg, uint64_t address);
uint8_t rf24_write_payload(rf24_t * this, uint8_t reg, void * buf, uint8_t len);

// ************************************************************************************

rf24_t * rf24_new(char * spi_dev, uint8_t ce_pin, uint8_t irq_pin);
uint8_t  rf24_delete(rf24_t * this);

uint8_t rf24_initialize(rf24_t * this, char * spi_dev, uint8_t ce_pin, uint8_t irq_pin);
void rf24_dump(rf24_t * this);

void rf24_power_up(rf24_t * this);
void rf24_power_down(rf24_t * this);
void rf24_reset(rf24_t * this);

void rf24_start_listening(rf24_t * this);
void rf24_stop_listening(rf24_t * this);

uint8_t rf24_send(rf24_t * this, void * buf, uint8_t len);
uint8_t rf24_receive(rf24_t * this, void * buf, uint8_t len);

void rf24_open_reading_pipe(rf24_t * this, uint8_t pipe, uint64_t address);
void rf24_open_writing_pipe(rf24_t * this, uint64_t address);

void rf24_send_ack_on_pipe(rf24_t * this, uint8_t pipe_no, void * buf, uint8_t len);

uint8_t rf24_data_available(rf24_t * this);
uint8_t rf24_data_available_on_pipe(rf24_t * this, uint8_t * pipe_number);

void rf24_enable_dynamic_payloads(rf24_t * this);
void rf24_enable_ack_payload(rf24_t * this);
void rf24_disable_crc(rf24_t * this);

void rf24_set_crc_length(rf24_t * this, uint8_t crc_length);
void rf24_set_pa_level(rf24_t * this, uint8_t pa_level);
void rf24_set_retries(rf24_t * this, uint8_t delay, uint8_t count);
void rf24_set_autoack(rf24_t * this, uint8_t autoack);
void rf24_set_autoack_for_pipe(rf24_t * this, uint8_t pipe, uint8_t autoack);
void rf24_set_data_rate(rf24_t * this, uint8_t speed);
void rf24_set_channel(rf24_t * this, uint8_t channel);

void rf24_set_payload_size(rf24_t * this, uint8_t size);
uint8_t rf24_get_payload_size(rf24_t * this);
uint8_t rf24_get_dynamic_payload_size(rf24_t * this);
uint8_t rf24_get_crc_length(rf24_t * this);

void rf24_poll(rf24_t * this, void(* callback)(rf24_t * radio));
void rf24_sync_status(rf24_t * this);
void rf24_reset_status(rf24_t * this);
#endif
// vim:ai:cin:et:sts=2 sw=2 ft=c
