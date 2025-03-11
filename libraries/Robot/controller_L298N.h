#ifndef __CONTROLLER_L298_H__
#define __CONTROLLER_L298_H__

#include "types.h"

#define L298N_DEFAULT_EN_PIN 9
#define L298N_DEFAULT_IN1_PIN 5
#define L298N_DEFAULT_IN2_PIN 6

#define L928N_MAX_DUTY_CYCLE 0.8
#define L928N_MIN_DUTY_CYCLE 0.4
#define L928N_MAX_COUNTER 255

#define LN298_MAX_N_BYTES 50

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

class CombinedControllerL298N
{
public:
  ControllerL298N *left;
  ControllerL298N *right;
  CombinedControllerL298N(ControllerL298N *left, ControllerL298N *right) : left(left), right(right) {}
  void initialize();
  void handler(char *msg);
  void controlMotor(ControllerL298N *motor, float speed);
};
#endif