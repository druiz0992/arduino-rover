
#include <TimeLib.h>
#include "serial_debug.h"

SerialDebug::SerialDebug(bool enabled) : _read_bytes(0), _enable(enabled) {}
void SerialDebug::write(char *msg)
{
    if (!_enable)
        return;
    size_t len = strlen(msg);
    if (len + _read_bytes < SERIAL_DEBUG_MAX_MSG_BYTES)
    {
        memcpy(_log + _read_bytes, msg, len);
        _read_bytes += len;
        _log[_read_bytes] = '\0';
    }
};
void SerialDebug::print()
{
    if (!_enable)
        return;
    Serial.print("DEBUG ");
    Serial.println(_log);
}