#include <Arduino.h>
#include "wheel_odometry.h"

WheelOdometry::WheelOdometry(
   float diameter_cm,
   float separation_cm,
   float n_sections,
   MeasurementL298N *l298_sample, MeasurementLm393Combined *lm393_sample) :
    _diameter_cm(diameter_cm),
    _separation_cm(separation_cm),
    _n_sections(n_sections),
    _l298_sample(l298_sample),
    _lm393_sample(lm393_sample) {

      _pose.x = 0.0;
      _pose.y = 0.0;
      _pose.phi = 0.0;

      _prev_ticks_left = 0;
      _prev_ticks_right = 0;
      
      _distance_per_tick = PI * _diameter_cm / _n_sections;
}

void WheelOdometry::processMeasurement(MeasurementBase *sample) {
  int16_t ticks_left, ticks_right;
  float left_speed, right_speed;

  if (_lm393_sample) {
    ticks_left = ((MeasurementLm393Combined *)_lm393_sample)->getValue().left;
    ticks_right = ((MeasurementLm393Combined *)_lm393_sample)->getValue().right;
  } else {
    ticks_left = 0;
    ticks_right = 0;
  }

  if (_l298_sample) {
    left_speed = ((MeasurementL298N *)_l298_sample)->getValue().left;
    right_speed = ((MeasurementL298N *)_l298_sample)->getValue().right;
  } else {
    left_speed = 0.0;
    right_speed = 0.0;
  }
  int16_t incr_ticks_right = TICKS_INCREMENT_16BIT(_prev_ticks_right, ticks_right);
  int16_t incr_ticks_left = TICKS_INCREMENT_16BIT(_prev_ticks_left, ticks_left);

  if (left_speed < 0.0f) {
    incr_ticks_left = -incr_ticks_left;
  }
  if (right_speed < 0.0f) {
    incr_ticks_right = -incr_ticks_right;
  }

  _prev_ticks_right = ticks_right;
  _prev_ticks_left = ticks_left;

  float distance_right = incr_ticks_right * _distance_per_tick;
  float distance_left = incr_ticks_left * _distance_per_tick; 

  float total_distance = (distance_left + distance_right) / 2.0;

  _pose.x += total_distance * cos(_pose.phi);
  _pose.y += total_distance * sin(_pose.phi);
  _pose.phi += (distance_right - distance_left) / _separation_cm;
  _pose.phi = atan2(sin(_pose.phi),   cos(_pose.phi));

  sample->setValue(_pose);

}

void WheelOdometry::read(MeasurementBase *sample) {
  sample->setValue(_pose);
}
