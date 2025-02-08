
#include <TimeLib.h>
#include "serial.h"

SerialComms::SerialComms() {
    Serial.begin(SERIAL_DEFAULT_SPEED);
}

void SerialComms::send(char *msg, uint8_t chan_idx) {
  time_t ts = now();
  sprintf(_tx_msg, "%08lx:%02X:%s",ts,chan_idx,msg);
  Serial.println(_tx_msg);
}

void SerialComms::receive(uint64_t *ts, char *msg, uint8_t *chan_idx){
    while (Serial.available()) {
        char inByte = Serial.read();

        if (inByte != '\n' && _read_bytes < SERIAL_MAX_MSG_BYTES) {
            _rx_msg[_read_bytes++] = inByte;
        } else {
            _rx_msg[_read_bytes] = '\0';
            dispatch(ts, msg, chan_idx);
        }
    }
}

void SerialComms::dispatch(uint64_t *ts, char *msg, uint8_t *chan_idx) {
    char ts_char[SERIAL_MSG_TIMESTAMP_LEN];
    char type_char[SERIAL_MSG_TYPE_LEN];
    strncpy(ts_char, _rx_msg, SERIAL_MSG_TIMESTAMP_LEN);
    strncpy(type_char, &_rx_msg[SERIAL_MAX_MSG_BYTES], SERIAL_MSG_TYPE_LEN);
    strcpy(msg, &_rx_msg[SERIAL_MSG_MESSAGE_OFFSET]);

    *ts = strtoul(ts_char, NULL, 16);
    unsigned long tmp = strtoul(type_char, NULL, 16);
    *chan_idx = (uint8_t) (tmp & 0xFF);
}
