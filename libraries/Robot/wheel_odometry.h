#ifndef __WHEEL_ODOMETRY_H__
#define __WHEEL_ODOMETRY_H__

#include "types.h"

#define WHEEL_ODOMETRY_DEFAULT_DIAMETER_CM 6.6
#define WHEEL_ODOMETRY_DEFAULT_DISTANCE_CM 11

#define WHEEL_ODOMETRY_DEFAULT_ALPHA_MOVING_AVERAGE 0.8

#define TICKS_INCREMENT_16BIT(prev, current) \
  ( ((current) >= (prev)) ? \
    (((current) - (prev) <= ((1<<15)-1)) ? ((current) - (prev)) : ((current) - (prev) - (1<<16))): \
    (((prev) - (current) <= ((1<<15)-1)) ? ((current) - (prev)) : ((current) - (prev) + (1<<16))) )


class WheelOdometry {
    public:
      WheelOdometry(double diamter_cm, double separation_cm, double n_sections);
      void update(int16_t right_sample, int16_t left_sample);
      void read(t_pose *);


    private:
      double _diameter_cm;
      double _separation_cm; 
      double _n_sections;
      int16_t _prev_ticks_right;
      int16_t _prev_ticks_left;

      t_pose _pose;
};

#endif