#ifndef __PROCESSOR_H__
#define __PROCESSOR_H__

#include "measurement.h"
#include "types.h"


class Processor {
    public:
       virtual void processMeasurement(MeasurementBase *sample) = 0;
       virtual void read(MeasurementBase *sample) = 0;
};

#endif