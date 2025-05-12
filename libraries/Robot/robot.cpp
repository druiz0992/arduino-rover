
#include <Arduino.h>
#include <TimeLib.h>
#include "robot.h"

Robot *_robot = nullptr;
static void (*_isrTable[ROBOT_MAX_N_SENSORS])();

static void isr0()
{
  if (_robot)
    _robot->isrHandler(0);
}
static void isr1()
{
  if (_robot)
    _robot->isrHandler(1);
}
static void isr2()
{
  if (_robot)
    _robot->isrHandler(2);
}
static void isr3()
{
  if (_robot)
    _robot->isrHandler(3);
}
static void isr4()
{
  if (_robot)
    _robot->isrHandler(4);
}
static void isr5()
{
  if (_robot)
    _robot->isrHandler(5);
}
static void isr6()
{
  if (_robot)
    _robot->isrHandler(6);
}
static void isr7()
{
  if (_robot)
    _robot->isrHandler(7);
}
static void isr8()
{
  if (_robot)
    _robot->isrHandler(8);
}
static void isr9()
{
  if (_robot)
    _robot->isrHandler(9);
}

Robot::Robot(bool debug_enable)
{
  for (uint8_t i = 0; i < ROBOT_MAX_N_SENSORS; i++)
  {
    _sensors[i] = nullptr;
    _samples[i] = nullptr;
  }
  for (uint8_t i = 0; i < ROBOT_MAX_N_CONTROLLERS; i++)
  {
    _controller[i] = nullptr;
    _samples[i + ROBOT_MAX_N_SENSORS] = nullptr;
  }
  for (uint8_t i = 0; i < ROBOT_MAX_N_PROCESSORS; i++)
  {
    _processor[i] = nullptr;
    _samples[i + ROBOT_MAX_N_SENSORS + ROBOT_MAX_N_CONTROLLERS] = nullptr;
  }

  _serial = new SerialComms(ROBOT_MAX_N_SENSORS, debug_enable);

  _robot = this;
}

uint8_t Robot::installSensor(Sensor *sensor, MeasurementBase *sample, t_pin pin_irq, uint8_t mode, uint8_t idx, char *channel)
{
  if (idx < ROBOT_MAX_N_SENSORS)
  {
    if (_sensors[idx] != nullptr || _samples[idx] != nullptr)
      return ROBOT_STATUS_ERROR_BUSY_SENSOR_SLOT;
      _sensors[idx] = sensor;
      _samples[idx] = sample;

    if (channel != nullptr) {
      _serial->set_channel(idx, channel);
    }

    if (mode != NO_ISR)
    {
      installIsr(idx, pin_irq, mode);
    }

    return ROBOT_STATUS_OK;
  }

  return ROBOT_STATUS_ERROR_MAX_NUMBER_OF_SENSORS;
}

uint8_t Robot::installIsr(uint8_t sensor_idx, t_pin pin_number, uint8_t mode)
{
  if (_sensors[sensor_idx] == nullptr)
  {
    return ROBOT_STATUS_ERROR_NONEXISTENT_SENSOR;
  }

  switch (sensor_idx)
  {
  case 0:
    _isrTable[sensor_idx] = isr0;
    break;

  case 1:
    _isrTable[sensor_idx] = isr1;
    break;

  case 2:
    _isrTable[sensor_idx] = isr2;
    break;

  case 3:
    _isrTable[sensor_idx] = isr3;
    break;

  case 4:
    _isrTable[sensor_idx] = isr4;
    break;

  case 5:
    _isrTable[sensor_idx] = isr5;
    break;

  case 6:
    _isrTable[sensor_idx] = isr6;
    break;

  case 7:
    _isrTable[sensor_idx] = isr7;
    break;

  case 8:
    _isrTable[sensor_idx] = isr8;
    break;

  case 9:
    _isrTable[sensor_idx] = isr9;
    break;

  default:
    return ROBOT_STATUS_ERROR_NONEXISTENT_SENSOR;
  }

  attachInterrupt(digitalPinToInterrupt(pin_number), _isrTable[sensor_idx], mode);

  return ROBOT_STATUS_OK;
}

void Robot::initialize()
{
  for (uint8_t i = 0; i < ROBOT_MAX_N_SENSORS; i++)
  {
    if (_sensors[i] != nullptr)
      _sensors[i]->initialize();
  }
  for (uint8_t i = 0; i < ROBOT_MAX_N_CONTROLLERS; i++)
  {
    _controller[i]->initialize();
  }
}

void Robot::readSensors()
{
  for (uint8_t i = 0; i < ROBOT_MAX_N_SENSORS; i++)
  {
    if (_sensors[i] != nullptr)
      _sensors[i]->read(_samples[i]);
  }
}

void Robot::readSensor(uint8_t sensor_idx)
{
  if (_sensors[sensor_idx] != nullptr)
  {
    _sensors[sensor_idx]->read(_samples[sensor_idx]);
  }
}

void Robot::readAndDispatchSensors()
{
  char buffer[SERIAL_MAX_MSG_BYTES];
  for (uint8_t i = 0; i < ROBOT_MAX_N_SENSORS; i++)
  {
    if (_sensors[i] != nullptr)
    {
      _sensors[i]->read(_samples[i]);
      _samples[i]->toString(buffer);
      _serial->send(buffer, i);
    }
  }
}

void Robot::isrHandler(uint8_t sensor_idx)
{
  if (_robot)
  {

    if (_robot->_sensors[sensor_idx] != nullptr)
    {
      _robot->_sensors[sensor_idx]->isr();
    }
  }
}

void Robot::readCommand()
{
  char buffer[MAX_SERIAL_READ_COMMANDS][SERIAL_MAX_MSG_BYTES];
  uint8_t handler_index[MAX_SERIAL_READ_COMMANDS] = {0xFF};

  uint8_t n_commands = _serial->receive(buffer, handler_index);
  /*
  Serial.print("n_commands: ");
  Serial.println(n_commands);
  */
  for (uint8_t i = 0; i < n_commands; i++)
  {
    if (handler_index[i] != 0xFF)
    {
      /*
      Serial.print("handler_index: ");
      Serial.println(handler_index[i]);
      Serial.print("buffer: ");
      Serial.println(buffer[i]);
      */
      _controller[handler_index[i]]->handler(buffer[i], _samples[handler_index[i]+ROBOT_MAX_N_SENSORS]);
    }
  }
}

void Robot::processMeasurements() {
  char buffer[SERIAL_MAX_MSG_BYTES];
  for (uint8_t i = 0; i < ROBOT_MAX_N_PROCESSORS; i++)
  {
    if (_processor[i] != nullptr)
    {
      _processor[i]->processMeasurement(_samples[i + ROBOT_MAX_N_SENSORS + ROBOT_MAX_N_CONTROLLERS]);
      _samples[i + ROBOT_MAX_N_SENSORS + ROBOT_MAX_N_CONTROLLERS]->toString(buffer);
      _serial->send(buffer, i + ROBOT_MAX_N_SENSORS + ROBOT_MAX_N_CONTROLLERS);
    }
  }
}

template <typename T>
T Robot::getMeasurement(uint8_t sensor_idx)
{
  if (_samples[sensor_idx] != nullptr)
  {
    return _samples[sensor_idx]->getValue<T>();
  };

  return T();
}

template float Robot::getMeasurement(uint8_t sensor_idx);
template int16_t Robot::getMeasurement(uint8_t sensor_idx);

void Robot::installController(Controller *controller, MeasurementBase *sample, uint8_t idx, char *channel)
{
  if (idx < ROBOT_MAX_N_CONTROLLERS)
  {
    _controller[idx] = controller;
    _samples[idx + ROBOT_MAX_N_SENSORS] = sample;
  }

  if (channel != nullptr) {
    _serial->set_channel(idx + ROBOT_MAX_N_SENSORS, channel);
  }
}

void Robot::installProcessor(Processor *processor, MeasurementBase *sample, uint8_t idx, char *channel)
{
  if (idx < ROBOT_MAX_N_PROCESSORS)
  {
    _processor[idx] = processor;
    _samples[idx + ROBOT_MAX_N_SENSORS + ROBOT_MAX_N_CONTROLLERS] = sample;
  }

  if (channel != nullptr) {
    _serial->set_channel(idx + ROBOT_MAX_N_SENSORS + ROBOT_MAX_N_CONTROLLERS, channel);
  }
}