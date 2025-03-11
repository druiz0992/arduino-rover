
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
  int speed = duty_cycle * L928N_MAX_COUNTER;

  analogWrite(_en_pin, speed);
}

void ControllerL298N::backward(float duty_cycle)
{
  digitalWrite(_in1_pin, _high);
  digitalWrite(_in2_pin, _low);
  int speed = duty_cycle * L928N_MAX_COUNTER;

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

void CombinedControllerL298N::handler(char *msg)
{
  char buffer[LN298_MAX_N_BYTES]; // Create a copy since strtok modifies the string
  float left_speed = 0.0;
  float right_speed = 0.0;

  strcpy(buffer, msg);
  buffer[sizeof(buffer) - 1] = '\0';

  char *token = strtok(buffer, ","); // First number
  if (token != NULL)
  {
    left_speed = atof(token);
  }

  token = strtok(NULL, ",");
  if (token != NULL)
  {
    right_speed = atof(token);
  }
  controlMotor(left, left_speed);
  controlMotor(right, right_speed);
}

void CombinedControllerL298N::controlMotor(ControllerL298N *motor, float speed)
{
  if (speed < 0)
  {
    if (speed > -L928N_MIN_DUTY_CYCLE)
    {
      speed = 0;
      motor->stop();
    }
    else if (speed < -L928N_MAX_DUTY_CYCLE)
    {
      speed = -L928N_MAX_DUTY_CYCLE;
      motor->backward(-speed);
    }
    else
    {
      motor->backward(-speed);
    }
  }
  else if (speed > 0)
  {
    if (speed < L928N_MIN_DUTY_CYCLE)
    {
      speed = 0;
      motor->stop();
    }
    else if (speed > L928N_MAX_DUTY_CYCLE)
    {
      speed = L928N_MAX_DUTY_CYCLE;
      motor->forward(speed);
    }
    else
    {
      motor->forward(speed);
    }
  }
  else
  {
    motor->stop();
  }
}
