
#include <TimeLib.h>
#include "serial.h"

SerialComms::SerialComms(uint8_t controller_channel_offset, bool debug_enable) : _n_sensors(controller_channel_offset), _debug(debug_enable)
{
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

uint8_t SerialComms::receive(char msg[MAX_SERIAL_READ_COMMANDS][SERIAL_MAX_MSG_BYTES], uint8_t *handler_index)
{
    uint8_t n_commands = 0;
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
            _read_bytes = 0;
            char handler_msg[SERIAL_MAX_MSG_BYTES] = {'\0'};
            uint8_t current_idx = find_handler_index(handler_msg);
            _debug.write(_rx_msg);
            if (current_idx < CHANNEL_MAX_N_CHANNELS)
            {
                handler_index[current_idx] = current_idx;
                strcpy(msg[current_idx], handler_msg);
                n_commands++;
            }
            else
            {
                break;
            }
        }
    }
    _debug.print();
    return n_commands;
}

void SerialComms::stripQuotes()
{
    size_t len = strlen(_rx_msg);
    if (len > 1 && _rx_msg[0] == '"' && _rx_msg[len - 1] == '"')
    {
        memmove(_rx_msg, _rx_msg + 1, len - 2);
        _rx_msg[len - 2] = '\0'; // Null-terminate after removing the last quote
    }
}
uint8_t SerialComms::find_handler_index(char msg[SERIAL_MAX_MSG_BYTES])
{
    char channel[CHANNEL_MAX_BYTES] = {'\0'};
    size_t len;
    uint8_t idx;

    stripQuotes();

    if ((len = _channels.extract_channel_name(_rx_msg, channel)) == 0)
    {
        return 0xFF;
    }

    strcpy(msg, &_rx_msg[len]);
    idx = _channels.get_channel_idx(channel) - _n_sensors;
    if (idx < CHANNEL_MAX_N_CHANNELS)
    {
        return idx;
    }
    return 0xFF;
}

void SerialComms::set_channel(uint8_t idx, char *channel)
{
    _channels.set_channel(idx, channel);
}
