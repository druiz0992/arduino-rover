
#ifndef __SENSOR_LM393_H__
#define __SENSOR_LM393_H__

#include "sensor.h"

#define LM393_DEFAULT_D0_PIN 2
#define LM393_MAX_MEASUREMENT_LEN 10

#define LM393_MIN_SIGNAL_TIME_USEC 500

class MeasurementLm393 : public Measurement<int16_t>
{
public:
  void toString(char *str) const override
  {
    sprintf(str, "%d", _value);
  }
};

typedef struct {
  int16_t left;
  int16_t right;
}t_measurement_lm393;

class MeasurementLm393Combined : public Measurement<t_measurement_lm393>
{
public:
  void toString(char *str) const override
  {
    char _value_left_str[LM393_MAX_MEASUREMENT_LEN];
    char _value_right_str[LM393_MAX_MEASUREMENT_LEN];

    sprintf(_value_left_str, "%d", _value.left);
    sprintf(_value_right_str, "%d", _value.right);
    sprintf(str, "%s,%s", _value_left_str, _value_right_str);
  }
};

class SensorLm393 : public Sensor
{
public:
  SensorLm393(t_pin);

  void initialize() override;
  void read(MeasurementBase *sample) override;
  void isr() override;
  void calibrate() override;
  int16_t getTicks() const { return _ticks; }

private:
  int _d0_pin;
  volatile int16_t _ticks;
  volatile unsigned long _debounce; 
};

class SensorLm393Combined: public Sensor 
{
public:
  SensorLm393Combined(SensorLm393 &left, SensorLm393 &right) :
    _left(left),
    _right(right)
  {};

  void read(MeasurementBase *sample) override;
  void initialize() override {};
  void isr() override {};
  void calibrate() override {};

private:
  t_measurement_lm393 _ticks;

  SensorLm393 &_left;
  SensorLm393 &_right;
};

#endif /* Sensor_LM393 */