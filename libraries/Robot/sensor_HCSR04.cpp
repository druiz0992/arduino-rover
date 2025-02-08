
#include <Arduino.h>
#include "sensor_HCSR04.h"

SensorHcsr04::SensorHcsr04(t_pin trig_pin, t_pin echo_pin) :
      _trig_pin(trig_pin),
      _echo_pin(echo_pin) {}

void SensorHcsr04::initialize() {

  pinMode(this->_trig_pin, OUTPUT);
  pinMode(this->_echo_pin, INPUT);

  digitalWrite(this->_trig_pin, LOW);

}

void SensorHcsr04::read(MeasurementBase *sample) {
    digitalWrite(this->_trig_pin, HIGH);
    delayMicroseconds(HCSR04_TRIG_PULSE_WIDTH_USEC);
    digitalWrite(this->_trig_pin, LOW);


    float duration_us = pulseIn(this->_echo_pin, HIGH);
    sample->setValue(duration_us * HCSR04_SPEED_OF_SOUND_CM_USEC / 2.0);
}


void SensorHcsr04::isr() {}
void SensorHcsr04::calibrate() {}
