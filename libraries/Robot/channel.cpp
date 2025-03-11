
#include <TimeLib.h>
#include "channel.h"

void Channel::set_channel(uint8_t idx, char *channel)
{
    snprintf(_channel[idx], CHANNEL_MAX_BYTES, "##%s##", channel);
}

char *Channel::get_channel(uint8_t idx)
{
    return _channel[idx];
}

uint8_t Channel::get_channel_idx(char *channel)
{
    for (uint8_t i = 0; i < CHANNEL_MAX_N_CHANNELS; i++)
    {

        if (strcmp(channel, _channel[i]) == 0)
        {
            return i;
        }
    }

    return 0xFF;
}

size_t Channel::extract_channel_name(const char *input, char *channel)
{
    const char *start = strchr(input, '#'); // Find first #
    if (!start || *(start + 1) != '#')
        return 0;

    const char *end = strstr(start + 2, "##"); // Find closing "##"
    if (!end)
        return 0;

    end += 2;
    size_t len = end - start;
    if (len >= CHANNEL_MAX_BYTES)
        return 0;

    strncpy(channel, start, len);
    channel[len] = '\0'; // Null-terminate the string
    return len;
}