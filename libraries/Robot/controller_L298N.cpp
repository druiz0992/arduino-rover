
#include <Arduino.h>
#include "controller_L298N.h"

ControllerL298N::ControllerL298N(t_pin *pins, bool invert) : _en_pin(pins[0]),
                                                             _in1_pin(pins[1]),
                                                             _in2_pin(pins[2]),
                                                             _high(invert ? LOW : HIGH),
                                                             _low(invert ? HIGH : LOW) {}

void ControllerL298N::initialize()
{
  pinMode(_en_pin, OUTPUT);
  pinMode(_in1_pin, OUTPUT);
  pinMode(_in2_pin, OUTPUT);

  digitalWrite(_in1_pin, LOW);
  digitalWrite(_in2_pin, LOW);
}

void ControllerL298N::forward(float duty_cycle)
{
  digitalWrite(_in1_pin, _low);
  digitalWrite(_in2_pin, _high);
  int speed = duty_cycle * L298N_MAX_COUNTER;

  analogWrite(_en_pin, speed);
}

void ControllerL298N::backward(float duty_cycle)
{
  digitalWrite(_in1_pin, _high);
  digitalWrite(_in2_pin, _low);
  int speed = duty_cycle * L298N_MAX_COUNTER;

  analogWrite(_en_pin, speed);
}
void ControllerL298N::stop()
{
  digitalWrite(_in1_pin, LOW);
  digitalWrite(_in2_pin, LOW);

  analogWrite(_en_pin, 0);
}

void CombinedControllerL298N::initialize()
{
  left->initialize();
  right->initialize();
}

void CombinedControllerL298N::handler(char *msg, MeasurementBase *sample)
{
  char buffer[L298N_MAX_N_BYTES];
  float left_speed = 0.0;
  float right_speed = 0.0;

  strcpy(buffer, msg);
  buffer[sizeof(buffer) - 1] = '\0';

  // Get left and right speeds from the message
  char *token = strtok(buffer, ",");
  if (token != NULL)
  {
    left_speed = atof(token);
    if (left_speed < -0.01) left_speed = -0.5; 
    else if (left_speed > 0.01) left_speed = 0.5;
    else left_speed = 0.0;
  } else return;

  token = strtok(NULL, ",");
  if (token != NULL)
  {
    right_speed = atof(token);
    if (right_speed < -0.01) right_speed = -0.5; 
    else if (right_speed > 0.01) right_speed = 0.5;
    else right_speed = 0.0;
  } else return;


  if (left_speed != _left_speed || left_speed == 0.0) {
    controlMotor(left, left_speed);
    _left_speed = left_speed; // Update with new speed
  }

  if (right_speed != _right_speed || right_speed == 0.) {
    controlMotor(right, right_speed);
    _right_speed = right_speed; // Update with new speed
  }

  t_measurement_l298n measurement;
  measurement.left = _left_speed;
  measurement.right = _right_speed;
  sample->setValue(measurement);
}


void CombinedControllerL298N::controlMotor(ControllerL298N *motor, float speed)
{
  if (speed < 0)
  {
    motor->backward(-speed);
  }
  else if (speed > 0)
  {
    motor->forward(speed);
  }
  else
  {
    motor->stop();
  }
}

void CombinedControllerL298N::getSpeed(int *left_speed, int *right_speed)
{
  *left_speed = _left_speed;
  *right_speed = _right_speed;
}
