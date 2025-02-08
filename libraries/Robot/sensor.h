#ifndef __SENSOR_H__
#define __SENSOR_H__

#include "measurement.h"
#include "types.h"


class Sensor {
    public:
       virtual void initialize() = 0;
       virtual void isr() = 0;
       virtual void read(MeasurementBase* sample) = 0;
       virtual void calibrate() = 0;

};

#endif