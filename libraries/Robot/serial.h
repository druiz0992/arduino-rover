#ifndef __SERIAL_H__
#define __SERIAL_H__

#include <Arduino.h>
#include "channel.h"
#include "serial_debug.h"

#define SERIAL_MAX_MSG_BYTES 100

#define SERIAL_DEFAULT_SPEED 115200

#define CHANNEL_MAX_BYTES 30
#define CHANNEL_MAX_N_CHANNELS 20
#define MAX_SERIAL_READ_COMMANDS 10

class SerialComms
{
public:
  SerialComms(uint8_t controller_channel_offset, bool debug_enable);
  void send(char *msg, uint8_t chan_idx);
  uint8_t receive(char msg[MAX_SERIAL_READ_COMMANDS][SERIAL_MAX_MSG_BYTES], uint8_t *handler_index);
  void set_channel(uint8_t idx, char *channel);
  void stripQuotes();

private:
  uint8_t find_handler_index(char msg[SERIAL_MAX_MSG_BYTES]);

  char _tx_msg[SERIAL_MAX_MSG_BYTES];
  char _rx_msg[SERIAL_MAX_MSG_BYTES];
  uint8_t _read_bytes;
  uint16_t _speed;
  Channel _channels;
  uint8_t _n_sensors;
  SerialDebug _debug;
};

#endif