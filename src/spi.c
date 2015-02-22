#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#include "spi.h"

int32_t spi_open(char * dev)
{
  int fd;
  fd = open(dev, O_RDWR);
  if (fd == -1) {
     fprintf(stderr, "[spi] Error opening %s\n", dev);
  }
  return fd;
}

int32_t spi_close(uint32_t fd)
{
  if (close(fd) == -1) {
     fprintf(stderr, "[spi] Error closing spi device\n");
     return -1;
  }
  return 0;
}

int8_t spi_config(uint32_t fd, uint8_t bits, uint32_t speed, uint8_t mode)
{
  if (ioctl(fd, SPI_IOC_WR_MODE, &mode) == -1) {
    fprintf(stderr, "[spi] Error setting SPI mode to %d\n", mode);
    return -1;
  }

  if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits) == -1) {
    fprintf(stderr, "[spi] Error setting SPI bits per word to %d\n", bits);
    return -1;
  }

  if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) == -1) {
    fprintf(stderr, "[spi] Error setting SPI speed to %d\n", speed);
    return -1;
  }

  return 0;
}

int8_t spi_transfer(uint32_t fd, uint8_t payload)
{
  uint32_t speed = 0;
  uint8_t bits   = 0;

  uint8_t tx[] = {payload};
  uint8_t rx[] = {0};

  if (ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed) == -1) {
    fprintf(stderr, "[spi] Error getting SPI device speed.\n");
    return -1;
  }

  if (ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits) == -1) {
    fprintf(stderr, "[spi] Error getting SPI device bits per word.\n");
    return -1;
  }

  struct spi_ioc_transfer tr = {
	  .tx_buf        = (uint32_t) tx,
	  .rx_buf        = (uint32_t) rx,
	  .len           = sizeof(tx) / sizeof(tx[0]),
	  .delay_usecs   = 0,
	  .speed_hz      = speed,
	  .bits_per_word = bits,
  };

  if (ioctl(fd, SPI_IOC_MESSAGE(1), &tr) == -1) {
    fprintf(stderr, "[spi] Error sending SPI message.\n");
    return -1;
  } else {
    //fprintf(stderr, "[spi] Sent %d, received %d\n", tx[0], rx[0]);
  }

  return rx[0];
}
// vim:ai:cin:et:sts=2 sw=2 ft=c
