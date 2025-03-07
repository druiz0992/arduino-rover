
#ifndef __SENSOR_HCSR04_H__
#define __SENSOR_HCSR04_H__

#include "sensor.h"

#define HCSR04_DEFAULT_TRIG_PIN 12
#define HCSR04_DEFAULT_ECHO_PIN 11

#define HCSR04_TRIG_PULSE_WIDTH_USEC 10
#define HCSR04_SPEED_OF_SOUND_CM_USEC 0.034
#define HCSR04_MAX_DISTANCE_CM 250

#define HCSR04_MAX_MEASUREMENT_LEN 10
#define HCSR04_MEASUREMENT_MIN_WIDTH 4
#define HCSR04_MEASUREMENT_PRECISION 2

class MeasurementHcsr04 : public Measurement<float>
{
public:
  void toString(char *str) const override
  {
    char _value_str[HCSR04_MAX_MEASUREMENT_LEN];
    dtostrf(_value, HCSR04_MEASUREMENT_MIN_WIDTH, HCSR04_MEASUREMENT_PRECISION, _value_str);
    sprintf(str, "%s", _value_str);
  }
};

class SensorHcsr04 : public Sensor
{
public:
  SensorHcsr04(t_pin, t_pin);

  void initialize() override;
  void read(MeasurementBase *sample) override;
  void isr() override;
  void calibrate() override;

private:
  int _trig_pin;
  int _echo_pin;
};

#endif /* Sensor_HCSR04 */