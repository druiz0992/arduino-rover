#ifndef __CHANNEL_H__
#define __CHANNEL_H__

#include <Arduino.h>

#define CHANNEL_MAX_BYTES 30
#define CHANNEL_MAX_N_CHANNELS 20

class Channel
{
public:
  void set_channel(uint8_t idx, char *channel);
  char *get_channel(uint8_t idx);
  uint8_t get_channel_idx(char *channel);
  size_t extract_channel_name(const char *input, char *channel);

private:
  char _channel[CHANNEL_MAX_N_CHANNELS][CHANNEL_MAX_BYTES];
};

#endif