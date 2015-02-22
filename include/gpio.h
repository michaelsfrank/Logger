#ifndef __GPIO_H__
#define __GPIO_H__

#define GPIO_PIN_LOW  0
#define GPIO_PIN_HIGH 1

#define GPIO_PIN_INPUT  0
#define GPIO_PIN_OUTPUT 1

#define GPIO_EDGE_NONE    0
#define GPIO_EDGE_RISING  1
#define GPIO_EDGE_FALLING 2
#define GPIO_EDGE_BOTH    3

#define GPIO_ACTIVE_HIGH  0
#define GPIO_ACTIVE_LOW   1

#include <inttypes.h>

uint8_t gpio_export(uint8_t gpio_pin);
uint8_t gpio_export_wait(uint8_t gpio_pin);
uint8_t gpio_unexport(uint8_t gpio_pin);

uint8_t gpio_set_direction(uint8_t gpio_pin, uint8_t gpio_direction);
uint8_t gpio_set_active_low(uint8_t gpio_pin, uint8_t gpio_active_low);
uint8_t gpio_set_edge(uint8_t gpio_pin, uint8_t gpio_edge);

uint8_t gpio_read(uint8_t gpio_pin);
uint8_t gpio_write(uint8_t gpio_ping, uint8_t gpio_value);

uint8_t gpio_poll(uint8_t gpio_pin, uint8_t gpio_edge, void(* callback)(void * arg), void * arg);
#endif
// vim:ai:cin:et:sts=2 sw=2 ft=c
