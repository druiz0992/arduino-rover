
#ifndef __SENSOR_LM393_H__
#define __SENSOR_LM393_H__

#include "sensor.h"

#define LM393_DEFAULT_D0_PIN 2

class MeasurementLm393: public Measurement<int16_t> {
    public:
      void toString(char *str) const override {
        sprintf(str,"%d", _value);
      }
};

class SensorLm393: public Sensor {
    public:

        SensorLm393(t_pin );

        void initialize() override;
        void read(MeasurementBase* sample) override;
        void isr() override;
        void calibrate() override;

    private:
        int _d0_pin;
        volatile int16_t _ticks;


};

#endif /* Sensor_LM393 */