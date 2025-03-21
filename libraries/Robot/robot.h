#ifndef __ROBOT_H__
#define __ROBOT_H__

#define ROBOT_MAX_N_SENSORS 10

#define ROBOT_STATUS_OK 0
#define ROBOT_STATUS_ERROR_MAX_NUMBER_OF_SENSORS 1
#define ROBOT_STATUS_ERROR_NONEXISTENT_SENSOR 2
#define ROBOT_STATUS_ERROR_BUSY_SENSOR_SLOT 3

#define NO_ISR 1234
#define ROBOT_N_CONTROLLERS 1

#include "sensor.h"
#include "controller_L298N.h"
#include "serial.h"

class Robot
{
public:
    Robot(bool debug_enable);

    uint8_t installSensor(Sensor *sensor, MeasurementBase *sample, t_pin, uint8_t, uint8_t, char *);
    uint8_t installIsr(uint8_t sensor_idx, t_pin pin_number, uint8_t mode);
    void initialize();
    void readSensors();
    void readAndDispatchSensors();
    void readSensor(uint8_t);
    void isrHandler(uint8_t sensor_idx);
    template <typename T>
    T getMeasurement(uint8_t sensor_idx);

    void installController(CombinedControllerL298N *controller, uint8_t idx, char *channel);
    void readCommand();

private:
    Sensor *_sensors[ROBOT_MAX_N_SENSORS];
    MeasurementBase *_samples[ROBOT_MAX_N_SENSORS];
    CombinedControllerL298N *_controller[ROBOT_N_CONTROLLERS];
    SerialComms *_serial;
};

#endif