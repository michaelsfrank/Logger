#ifndef __SPI_H__
#define __SPI_H__

#include <inttypes.h>

int32_t spi_open(char * dev);
int32_t spi_close(uint32_t fd);

int8_t spi_config(uint32_t fd, uint8_t bits, uint32_t speed, uint8_t mode);
int8_t spi_transfer(uint32_t fd, uint8_t payload);

#endif
// vim:ai:cin:et:sts=2 sw=2 ft=c
