# libnrf24 - C library for nRF24L01 on Raspberry Pi

## About

A plain C port and de-arduinization of
[gnulnulf's](https://github.com/gnulnulf/RF24) and
[stanleyseow's](https://github.com/stanleyseow/RF24) C++ library for running
nRF24L01 2.4GHz card on a Raspberry Pi.

Supports ```poll()```ing on an IRQ pin and calling user defined callback on
events sent by the card. It does not use threads, as only a single pin needs to
be polled.

See ```examples/pong_irq.c``` for an example receiver. The
```examples/pong_curl.c``` can be used to send data to a [picasso
dashboard](http://balazs.kutilovi.cz/2014/03/26/picasso-a-sinatra-dashboard-app/).

## License

The original code for Arduino comes from
[Maniacbug](https://github.com/maniacbug/RF24). gnulnulf and stanleyseow ported
the library. Huge thanks to all of them.

The code retains the same license as the original: GNU GPL v2.

## TODO

  * Add ping example.
  * Find out possible causes for packet loss. The Pi receives and sends the
    pong, but the node does not receive it.
  * Add debugging #ifdefs
  * Add profiling + check for timing discrepancies vs. the datasheet
  * Clean up the interface
  * Add docs

## Docs

  * [nRF24L01+ datasheet](http://www.nordicsemi.com/eng/nordic/download_resource/8765/2/27999719)
    Super useful, especially the registers part.
