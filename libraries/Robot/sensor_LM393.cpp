
#include <Arduino.h>
#include "sensor_LM393.h"

SensorLm393::SensorLm393(t_pin d0_pin) : _d0_pin(d0_pin)
{
  _ticks = 0;
}

void SensorLm393::initialize()
{

  pinMode(_d0_pin, INPUT_PULLUP);
}

void SensorLm393::isr()
{
  if (digitalRead(_d0_pin) == HIGH)
  {
    _ticks++;
  }
  else
  {
    _ticks--;
  }
}

void SensorLm393::read(MeasurementBase *sample)
{
  sample->setValue(_ticks);
}

void SensorLm393::calibrate() {}
