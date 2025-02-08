#ifndef __SERIAL_H__
#define __SERIAL_H__

#include <Arduino.h>

#define SERIAL_MAX_MSG_BYTES 100
#define SERIAL_MSG_TIMESTAMP_LEN 10
#define SERIAL_MSG_TYPE_LEN 1
#define SERIAL_MSG_MESSAGE_OFFSET (SERIAL_MSG_TIMESTAMP_LEN + SERIAL_MSG_TYPE_LEN)

#define SERIAL_DEFAULT_SPEED 115200

class SerialComms {
    public:
    
      SerialComms();
      void send(char *msg, uint8_t chan_idx);
      void receive(uint64_t *ts, char *msg, uint8_t *chan_idx);

    private:
      void dispatch(uint64_t *ts, char *msg, uint8_t *chan_idx);

      char _tx_msg[SERIAL_MAX_MSG_BYTES];
      char _rx_msg[SERIAL_MAX_MSG_BYTES];
      uint8_t _read_bytes;
      uint16_t _speed;
};


#endif