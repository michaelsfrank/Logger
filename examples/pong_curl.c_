#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <curl/curl.h>
#include "rf24.h"

#define URL       "http://localhost:9292/"
#define HTTP_AUTH "api:b7273d35bb1926e56a9671fca815c99b"

static uint32_t now(void) {
  struct timeval now;
  gettimeofday(&now, NULL);
  return now.tv_sec;
}

void post_data(int node, float temp, float voltage)
{
  char url[1024];
  uint32_t ts = now();

  CURL * curl = curl_easy_init();

  curl_easy_setopt(curl, CURLOPT_VERBOSE, 0);
  curl_easy_setopt(curl, CURLOPT_NOPROGRESS, 1);
  curl_easy_setopt(curl, CURLOPT_POST, 1);
  curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, 0);
  curl_easy_setopt(curl, CURLOPT_HTTPAUTH, CURLAUTH_BASIC);
  curl_easy_setopt(curl, CURLOPT_USERPWD, HTTP_AUTH);

  snprintf(url, 1024, "%sdata/node%02d/voltage/%d000/%f", URL, node, ts, voltage);
  curl_easy_setopt(curl, CURLOPT_URL, url);
  curl_easy_perform(curl);

  snprintf(url, 1024, "%sdata/node%02d/temperature/%d000/%f", URL, node, ts, temp);
  curl_easy_setopt(curl, CURLOPT_URL, url);
  curl_easy_perform(curl);

  curl_easy_cleanup(curl);
}

void send_pong(void * data)
{
  char buf[32];
  int node, volt, temp;
  uint8_t i, len;

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
    sscanf(buf, "N=%d;V=%d;T=%d",&node,&volt,&temp);
    fprintf(stderr, "[rf24 pong callback] Node %d, payload %s, temperature %f, voltage: %f\n", node, buf, (float) (temp / 100.0), (float) (volt / 100.0));
    post_data(node, (float) (temp / 100.0), (float) (volt / 100.0));
  }
  rf24_reset_status(radio);
}

int main(void)
{
  rf24_t radio;
  curl_global_init(CURL_GLOBAL_DEFAULT);

  uint64_t pipe_addresses[3] = { 0xF0F0F0F0D2LL, 0xF0F0F0F0E1LL, 0xF0F0F0F0A2LL };

  rf24_initialize(&radio, RF24_SPI_DEV_0, 25, 4);
  rf24_set_retries(&radio, 15, 15);
  rf24_set_autoack(&radio, 1);

  rf24_open_writing_pipe(&radio, pipe_addresses[0]);
  rf24_open_reading_pipe(&radio, 1, pipe_addresses[1]);
  rf24_open_reading_pipe(&radio, 2, pipe_addresses[2]);

  rf24_start_listening(&radio);
  rf24_dump(&radio);
  rf24_irq_poll(&radio, &send_pong);

  return 0;
}
// vim:ai:cin:et:sts=2 sw=2 ft=c
