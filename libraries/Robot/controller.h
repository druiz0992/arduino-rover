#ifndef __CONTROLLER_H__
#define __CONTROLLER_H__

#include "measurement.h"
#include "types.h"


class Controller {
    public:
       virtual void handler(char *msg, MeasurementBase *sample) = 0;
       virtual void initialize() = 0;
};

#endif