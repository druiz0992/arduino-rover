#ifndef __WHEEL_ODOMETRY_H__
#define __WHEEL_ODOMETRY_H__

#include "types.h"
#include "controller_L298N.h"
#include "sensor_LM393.h"
#include "processor.h"

#define WHEEL_ODOMETRY_DEFAULT_DIAMETER_CM 6.6
#define WHEEL_ODOMETRY_DEFAULT_DISTANCE_CM 11

#define WHEEL_ODOMETRY_DEFAULT_ALPHA_MOVING_AVERAGE 0.8

#define WHEEL_ODOMETRY_MAX_N_BYTES 32
#define WHEEL_ODOMETRY_MEASUREMENT_MIN_WIDTH 4
#define WHEEL_ODOMETRY_MEASUREMENT_PRECISION 2

#define TICKS_INCREMENT_16BIT(prev, current) \
  ( ((current) >= (prev)) ? \
    (((current) - (prev) <= ((1<<15)-1)) ? ((current) - (prev)) : ((current) - (prev) - (1<<16))): \
    (((prev) - (current) <= ((1<<15)-1)) ? ((current) - (prev)) : ((current) - (prev) + (1<<16))) )


typedef struct {
  float x;
  float y;
  float phi;
} t_pose;

class MeasurementPose : public Measurement<t_pose>
{
  public:
    void toString(char *str) const override
    {
      char _value_x_str[WHEEL_ODOMETRY_MAX_N_BYTES];
      char _value_y_str[WHEEL_ODOMETRY_MAX_N_BYTES];
      char _value_phi_str[WHEEL_ODOMETRY_MAX_N_BYTES];

      dtostrf(_value.x, WHEEL_ODOMETRY_MEASUREMENT_MIN_WIDTH, WHEEL_ODOMETRY_MEASUREMENT_PRECISION, _value_x_str);
      dtostrf(_value.y, WHEEL_ODOMETRY_MEASUREMENT_MIN_WIDTH, WHEEL_ODOMETRY_MEASUREMENT_PRECISION, _value_y_str);
      dtostrf(_value.phi, WHEEL_ODOMETRY_MEASUREMENT_MIN_WIDTH, WHEEL_ODOMETRY_MEASUREMENT_PRECISION, _value_phi_str);
      sprintf(str, "%s,%s,%s", _value_x_str, _value_y_str, _value_phi_str);
    }
};

class WheelOdometry: public Processor {
    public:
      WheelOdometry(float diamter_cm, float separation_cm, float n_sections, MeasurementL298N *l298_sample, MeasurementLm393Combined *lm393_sample);
      void processMeasurement(MeasurementBase *sample) override;
      void read(MeasurementBase *sample) override; 


    private:
      float _diameter_cm;
      float _separation_cm; 
      float _n_sections;
      float _distance_per_tick;
      int16_t _prev_ticks_right;
      int16_t _prev_ticks_left;
      MeasurementL298N *_l298_sample;
      MeasurementLm393Combined *_lm393_sample;

      t_pose _pose;
};

#endif