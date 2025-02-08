#include <Arduino.h>
#include "wheel_odometry.h"

WheelOdometry::WheelOdometry(
   double diamter_cm,
   double separation_cm,
   double n_sections):
    _diameter_cm(diamter_cm),
    _separation_cm(separation_cm),
    _n_sections(n_sections) {
      _pose.x = 0.0;
      _pose.y = 0.0;
      _pose.phi = 0.0;

      _prev_ticks_left = 0;
      _prev_ticks_right = 0;
}

void WheelOdometry::update(int16_t ticks_right, int16_t ticks_left) {
  int16_t incr_ticks_right = TICKS_INCREMENT_16BIT(_prev_ticks_right, ticks_right);
  int16_t incr_ticks_left = TICKS_INCREMENT_16BIT(_prev_ticks_left, ticks_left);

  _prev_ticks_right = ticks_right;
  _prev_ticks_left = ticks_left;

  double distance_right = PI * _diameter_cm * incr_ticks_right / _n_sections;
  double distance_left = PI * _diameter_cm * incr_ticks_left / _n_sections;

  double total_distance = (distance_left + distance_right) / 2.0;

  _pose.x += total_distance * cos(_pose.phi);
  _pose.y += total_distance * sin(_pose.phi);
  _pose.phi += (distance_right - distance_left) / _separation_cm;
  _pose.phi = atan2(sin(_pose.phi),   cos(_pose.phi));

}

void WheelOdometry::read(t_pose *pose) {
  memcpy(pose, &_pose, sizeof(t_pose));
}
