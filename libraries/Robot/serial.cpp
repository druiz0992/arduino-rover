
#include <TimeLib.h>
#include "serial.h"

SerialComms::SerialComms(uint8_t n_sensors)
{
    _n_sensors = n_sensors;
    Serial.begin(SERIAL_DEFAULT_SPEED);
}

void SerialComms::send(char *msg, uint8_t chan_idx)
{
    char *channel;
    // time_t ts = now();
    channel = _channels.get_channel(chan_idx);
    sprintf(_tx_msg, "%s %s", channel, msg);
    Serial.println(_tx_msg);
}

uint8_t SerialComms::receive(char *msg, uint8_t *handler_index)
{
    uint8_t current_idx = 0;
    while (Serial.available())
    {
        char inByte = Serial.read();

        if (inByte != '\n' && _read_bytes < SERIAL_MAX_MSG_BYTES)
        {
            _rx_msg[_read_bytes++] = inByte;
        }
        else
        {
            _rx_msg[_read_bytes] = '\0';
            handler_index[current_idx] = find_handler_index(msg);
            _read_bytes = 0;
            if (++current_idx >= MAX_SERIAL_READ_COMMANDS)
            {
                break;
            }
        }
    }
    return current_idx;
}

uint8_t SerialComms::find_handler_index(char *msg)
{
    char channel[CHANNEL_MAX_BYTES];
    size_t len;
    uint8_t idx;

    if ((len = _channels.extract_channel_name(_rx_msg, channel)) == 0)
    {
        return;
    }

    strcpy(msg, &_rx_msg[len]);
    idx = _channels.get_channel_idx(channel);
    if (idx != 0xFF && idx >= _n_sensors)
    {
        idx -= _n_sensors;
        return idx;
        // Serial.println(channel);
        // Serial.println(msg);
    }
    return 0xFF;
}

void SerialComms::set_channel(uint8_t idx, char *channel)
{
    _channels.set_channel(idx, channel);
}

char *SerialComms::get_channel(uint8_t idx)
{
    return _channels.get_channel(idx);
}

uint8_t SerialComms::get_channel_idx(char *channel)
{
    return _channels.get_channel_idx(channel);
}
