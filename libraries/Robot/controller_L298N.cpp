
#include <Arduino.h>
#include "controller_L298N.h"

ControllerL298N::ControllerL298N(t_pin *pins, bool invert) : 
  _en_pin(pins[0]),
  _in1_pin(pins[1]),
  _in2_pin(pins[2]),
  _high(invert ? LOW : HIGH),
  _low(invert ? HIGH: LOW) {}


void ControllerL298N::initialize() {
    pinMode(_en_pin, OUTPUT);
    pinMode(_in1_pin, OUTPUT);
    pinMode(_in2_pin, OUTPUT);

    digitalWrite(_in1_pin, LOW);
    digitalWrite(_in2_pin, LOW);

}

void ControllerL298N::forward(float duty_cycle) {
  digitalWrite(_in1_pin, _low);
  digitalWrite(_in2_pin, _high);
  int speed = duty_cycle * L928N_MAX_COUNTER;

  analogWrite(_en_pin, speed);
}

void ControllerL298N::backward(float duty_cycle) {
  digitalWrite(_in1_pin, _high);
  digitalWrite(_in2_pin, _low);
  int speed = duty_cycle * L928N_MAX_COUNTER;

  analogWrite(_en_pin, speed);

}
void ControllerL298N::stop() {
  digitalWrite(_in1_pin, LOW);
  digitalWrite(_in2_pin, LOW);

  analogWrite(_en_pin, 0);

}



  