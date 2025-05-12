#ifndef __CONTROLLER_L298_H__
#define __CONTROLLER_L298_H__

#include "controller.h"

#define L298N_DEFAULT_EN_PIN 9
#define L298N_DEFAULT_IN1_PIN 5
#define L298N_DEFAULT_IN2_PIN 6

#define L298N_MAX_DUTY_CYCLE 0.75
#define L298N_MIN_DUTY_CYCLE 0.4
#define L298N_MAX_COUNTER 255

#define L298N_MAX_N_BYTES 50
#define L298N_MEASUREMENT_MIN_WIDTH 4
#define L298N_MEASUREMENT_PRECISION 2

class ControllerL298N
{
public:
  ControllerL298N(t_pin *, bool);

  void initialize();
  void forward(float);
  void backward(float);
  void stop();

private:
  t_pin _en_pin;
  t_pin _in1_pin;
  t_pin _in2_pin;

  uint8_t _high;
  uint8_t _low;
};

typedef struct
{
  float left;
  float right;
} t_measurement_l298n;

class MeasurementL298N : public Measurement<t_measurement_l298n>
{
  public:
  void toString(char *str) const override
  {
      char _value_left_str[L298N_MAX_N_BYTES];
      char _value_right_str[L298N_MAX_N_BYTES];
      dtostrf(_value.left, L298N_MEASUREMENT_MIN_WIDTH, L298N_MEASUREMENT_PRECISION, _value_left_str);
      dtostrf(_value.right, L298N_MEASUREMENT_MIN_WIDTH, L298N_MEASUREMENT_PRECISION, _value_right_str);
      sprintf(str, "%s,%s", _value_left_str, _value_right_str);
  }
};

class CombinedControllerL298N: public  Controller
{
public:
  ControllerL298N *left;
  ControllerL298N *right;
  CombinedControllerL298N(ControllerL298N *left, ControllerL298N *right) : left(left), right(right), _left_speed(0), _right_speed(0) {}
  void initialize() override;
  void handler(char *msg, MeasurementBase *sample) override;
  void controlMotor(ControllerL298N *motor, float speed);
  void getSpeed(int *left_speed, int *right_speed);
private:
  int _left_speed;
  int _right_speed;
};
#endif