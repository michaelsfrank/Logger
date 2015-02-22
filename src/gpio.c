#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <assert.h>
#include "gpio.h"

uint8_t gpio_export(uint8_t gpio_pin)
{
  FILE * file;

  if ((file = fopen("/sys/class/gpio/export", "w")) == NULL) {
    fprintf(stderr, "[gpio] Error opening /sys/class/gpio/export for writing.\n");
    return -1;
  }
  fprintf(file, "%d", gpio_pin);
  fclose(file);
  fprintf(stderr, "[gpio] pin %d exported\n", gpio_pin);

  return 0;
}

uint8_t gpio_export_wait(uint8_t gpio_pin) {
  char gpio_file[32];
  uint32_t  timeout, sleep;

  snprintf(gpio_file, sizeof(gpio_file), "/sys/class/gpio/gpio%d/value", gpio_pin);

  timeout = 5e6;
  sleep   = 1e5;

  if (gpio_export(gpio_pin) == -1) {
    return -1;
  }

  while (access(gpio_file, R_OK | W_OK) != 0 && timeout >= 0) {
    usleep(sleep);
    timeout -= sleep;
  }

  if (timeout < 0) {
    fprintf(stderr, "[gpio] Timeout on exporting GPIO pin %d.\n", gpio_pin);
    return -1;
  } else {
    return 0;
  }
}

uint8_t gpio_unexport(uint8_t gpio_pin)
{
  FILE * file;

  if ((file = fopen("/sys/class/gpio/unexport", "w")) == NULL) {
    fprintf(stderr, "[gpio] Error opening /sys/class/gpio/unexport for writing.\n");
    return -1;
  }
  fprintf(file, "%d", gpio_pin);
  fclose(file);
  fprintf(stderr, "[gpio] pin %d unexported\n", gpio_pin);

  return 0;
}

uint8_t gpio_set_active_low(uint8_t gpio_pin, uint8_t gpio_active_low)
{
  char gpio_file[34];
  FILE * file;

  assert (gpio_active_low == GPIO_ACTIVE_LOW || gpio_active_low == GPIO_ACTIVE_HIGH );

  snprintf(gpio_file, sizeof(gpio_file), "/sys/class/gpio/gpio%d/active_low", gpio_pin);
  if ((file = fopen(gpio_file, "w")) == NULL) {
    fprintf(stderr, "[gpio] Error opening /sys/class/gpio/gpio%d/active_low for writing.\n", gpio_pin);
    return -1;
  }

  fprintf(file, "%d", gpio_active_low);
  fclose(file);
  fprintf(stderr, "[gpio] pin %d active set to %s\n", gpio_pin, gpio_active_low == GPIO_ACTIVE_LOW ? "low" : "high");

  return 0;
}

uint8_t gpio_set_direction(uint8_t gpio_pin, uint8_t gpio_direction)
{
  char gpio_file[34];
  FILE * file;

  assert (gpio_direction == GPIO_PIN_INPUT || gpio_direction == GPIO_PIN_OUTPUT);

  snprintf(gpio_file, sizeof(gpio_file), "/sys/class/gpio/gpio%d/direction", gpio_pin);
  if ((file = fopen(gpio_file, "w")) == NULL) {
    fprintf(stderr, "[gpio] Error opening /sys/class/gpio/gpio%d/direction for writing.\n", gpio_pin);
    return -1;
  }

  fprintf(file, "%s", gpio_direction == GPIO_PIN_INPUT ? "in" : "out");
  fclose(file);
  fprintf(stderr, "[gpio] pin %d direction set to %s\n", gpio_pin, gpio_direction == GPIO_PIN_INPUT ? "input" : "output");

  return 0;
}

uint8_t gpio_set_edge(uint8_t gpio_pin, uint8_t gpio_edge)
{
  char gpio_file[32];
  FILE * file;

  assert (gpio_edge == GPIO_EDGE_NONE || gpio_edge == GPIO_EDGE_FALLING || gpio_edge == GPIO_EDGE_RISING || gpio_edge == GPIO_EDGE_BOTH);

  snprintf(gpio_file, sizeof(gpio_file), "/sys/class/gpio/gpio%d/edge", gpio_pin);
  if ((file = fopen(gpio_file, "w")) == NULL) {
    fprintf(stderr, "[gpio] Error opening /sys/class/gpio/gpio%d/edge for writing.\n", gpio_pin);
    return -1;
  }

  switch(gpio_edge) {
    case GPIO_EDGE_NONE    : fprintf(file, "none");    break;
    case GPIO_EDGE_RISING  : fprintf(file, "rising");  break;
    case GPIO_EDGE_FALLING : fprintf(file, "falling"); break;
    case GPIO_EDGE_BOTH    : fprintf(file, "both");    break;
  }

  fprintf(stderr, "[gpio] pin %d edge set to %d\n", gpio_pin, gpio_edge);

  fclose(file);

  return 0;
}

uint8_t gpio_write(uint8_t gpio_pin, uint8_t gpio_value)
{
  char gpio_file[32];
  FILE * file;
  uint8_t ret;

  assert(gpio_value == GPIO_PIN_LOW || gpio_value == GPIO_PIN_HIGH);

  snprintf(gpio_file, sizeof(gpio_file), "/sys/class/gpio/gpio%d/value", gpio_pin);
  if ((file = fopen(gpio_file, "w")) == NULL) {
    fprintf(stderr, "[gpio] Error opening /sys/class/gpio/gpio%d/value for writing.\n", gpio_pin);
    return -1;
  }

  fprintf(file, "%d", gpio_value > 0 ? GPIO_PIN_HIGH : GPIO_PIN_LOW);
  fclose(file);

  return 0;
}

uint8_t gpio_read(uint8_t gpio_pin)
{
  char gpio_file[32];
  FILE * file;
  uint8_t ret;

  snprintf(gpio_file, sizeof(gpio_file), "/sys/class/gpio/gpio%d/value", gpio_pin);
  if ((file = fopen(gpio_file, "r")) == NULL) {
    fprintf(stderr, "[gpio] Error opening /sys/class/gpio/gpio%d/value for reading.\n", gpio_pin);
    return -1;
  }

  if (fscanf(file, "%d", &ret) < 0) {
    return -1;
  }

  fclose(file);

  return ret > 0 ? GPIO_PIN_HIGH : GPIO_PIN_LOW;
}

uint8_t gpio_poll(uint8_t gpio_pin, uint8_t gpio_edge, void(* callback)(void * arg), void * arg)
{
  char gpio_file[32];
  struct pollfd polls;

  int32_t fd;
  uint32_t count;
  uint8_t i, ret, val;

  if (gpio_export_wait(gpio_pin) == -1) { return -1; }
  if (gpio_set_direction(gpio_pin, GPIO_PIN_INPUT) == -1) { return -1; }
  if (gpio_set_edge(gpio_pin, gpio_edge)) { return -1; }
  if (callback == NULL) { return -1; }

  snprintf(gpio_file, sizeof(gpio_file), "/sys/class/gpio/gpio%d/value", gpio_pin);
  if ((fd = open(gpio_file, O_RDWR)) == -1) {
    fprintf(stderr, "[gpio] Error opening /sys/class/gpio/gpio%d/value for reading.\n", gpio_pin);
    return -1;
  }

  polls.fd = fd;
  polls.events = POLLPRI;

  ret = 0;
  
  fprintf(stderr, "[gpio] entering loop\n");
  while(1) {
    fprintf(stderr, "[gpio] starting while loop\n");
	ret = poll(&polls, 1, -1) ;
	
    if (ret == -1) {
      fprintf(stderr, "[gpio] Error polling /sys/class/gpio/gpio%d/value.\n", gpio_pin);
      break;
    } else
    if (ret == 0) {
      fprintf(stderr, "[gpio] ret = 0, continuuing\n");
	  continue;
    } else {
      fprintf(stderr, "[gpio] reading fd\n");
	  read(fd, &val, 1);
      fprintf(stderr, "[gpio] calling back\n");
	  callback(arg);
	  fprintf(stderr, "[gpio] ret = 0, continuuing\n");
    }
  }
  fprintf(stderr, "[gpio] exiting loop\n");
  close(fd);
  fprintf(stderr, "[gpio] returning\n");
  return ret;
}
// vim:ai:cin:et:sts=2 sw=2 ft=c
