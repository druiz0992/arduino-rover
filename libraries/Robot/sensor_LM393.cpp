
#include <Arduino.h>
#include "sensor_LM393.h"

SensorLm393::SensorLm393(t_pin d0_pin) : _d0_pin(d0_pin), 
                                      _ticks(0),
                                      _debounce(0)
{
}

void SensorLm393::initialize()
{

  pinMode(_d0_pin, INPUT);
}

void SensorLm393::isr()
{
  if (digitalRead(_d0_pin) && micros() - _debounce > LM393_MIN_SIGNAL_TIME_USEC && digitalRead(_d0_pin))
  {
    _debounce = micros();
    _ticks++;
  }
}

void SensorLm393::read(MeasurementBase *sample)
{
  sample->setValue(_ticks);
}

void SensorLm393::calibrate() {}

void SensorLm393Combined::read(MeasurementBase *sample)
{
  noInterrupts();
  _ticks.left = _left.getTicks();
  _ticks.right = _right.getTicks();
  interrupts();

  sample->setValue(_ticks);
}