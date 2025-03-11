#ifndef __SERIAL_DEBUG_H__
#define __SERIAL_DEBUG_H__

#include <Arduino.h>
#include "serial_debug.h"

#define SERIAL_DEBUG_MAX_MSG_BYTES 1024

class SerialDebug
{
public:
  SerialDebug(bool enabled);
  void write(char *msg);
  void print();

private:
  char _log[SERIAL_DEBUG_MAX_MSG_BYTES] = {'\0'};
  uint8_t _read_bytes;
  bool _enable;
};

#endif