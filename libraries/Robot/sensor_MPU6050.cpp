
#include <Arduino.h>
#include "sensor_MPU6050.h"

// pointer to sensor
SensorMpu6050 *mpu6050[1];

SensorMpu6050::SensorMpu6050() :
      _sda_pin(DEFAULT_SDA_PIN),
      _scl_pin(DEFAULT_SCL_PIN),
      _status(STATUS_NOT_INITIALIZED) {}

void SensorMpu6050::initialize() {

  Wire.begin();
  Wire.setClock(I2C_CLOCK_FREQUENCY);

  Serial.begin(115200);

  mpu6050[0] = this; 

  // init MPU6050
  this->_mpu6050.initialize();

  if (!this->_mpu6050.testConnection()) {
    this->_status = STATUS_CONNECTION_FAILED;
    return;
  }

  // start DMP
  this->_status = this->_mpu6050.dmpInitialize();

  if (this->_status == STATUS_OK) {
    // calibrate
    if (ENABLE_CALIBRATION) {
      this->calibrate();
    } else {
      this->_mpu6050.setXAccelOffset(DEFAULT_X_ACCEL_OFFSET);
      this->_mpu6050.setYAccelOffset(DEFAULT_Y_ACCEL_OFFSET);
      this->_mpu6050.setZAccelOffset(DEFAULT_Z_ACCEL_OFFSET);
 
      this->_mpu6050.setXGyroOffset(DEFAULT_X_GYRO_OFFSET);
      this->_mpu6050.setYGyroOffset(DEFAULT_Y_GYRO_OFFSET);
      this->_mpu6050.setZGyroOffset(DEFAULT_Z_GYRO_OFFSET);
    }

    this->_mpu6050.setDMPEnabled(true);
  }
}

void SensorMpu6050::read(MeasurementBase *sample) {
  if (this->_status != STATUS_OK) return;

  uint8_t buffer[BUFFER_SIZE];
  Quaternion q;
  VectorInt16 aa;         // [x, y, z]            accel sensor measurements
  VectorInt16 aaReal;         // [x, y, z]            accel sensor measurements
  VectorFloat gravity;    // [x, y, z]            gravity vector

  MeasurementType type = sample->getType();
  if (type == MeasurementType::QUATERNION) {
      if (this->_mpu6050.dmpGetCurrentFIFOPacket(buffer)) {
          this->_mpu6050.dmpGetQuaternion(&q, buffer);
          this->_setGravity(&q, &gravity);
          this->_mpu6050.dmpGetAccel(&aa, buffer);
          this->_mpu6050.dmpGetLinearAccel(&aaReal, &aa, &gravity);
          _aaReal.x = (float)aaReal.x / ACCEL_SENSITIVITY_FACTOR * GRAVITY;
          _aaReal.y = (float)aaReal.y / ACCEL_SENSITIVITY_FACTOR * GRAVITY;
          _aaReal.z = (float)aaReal.z / ACCEL_SENSITIVITY_FACTOR * GRAVITY;
          //this->toYPR(&q, _ypr_sample, &gravity);
          sample->setValue(q);
      } 
  } else {
      sample->setValue(_aaReal);
  } 
}

void SensorMpu6050::_setGravity(Quaternion *from , VectorFloat *gravity) {
    this->_mpu6050.dmpGetGravity(gravity, from);
}

void SensorMpu6050::toYPR(Quaternion *from , float *to, VectorFloat *gravity) {
    //VectorFloat gravity;
    //this->_mpu6050.dmpGetGravity(&gravity, from);
    this->_mpu6050.dmpGetYawPitchRoll(to, from, gravity);
}

void SensorMpu6050::calibrate(){
  int retries = 0;

  t_calibration_data raw_data;
  t_calibration_data calibration;

  this->_mpu6050.setXAccelOffset(0);
  this->_mpu6050.setYAccelOffset(0);
  this->_mpu6050.setZAccelOffset(0);
 
  this->_mpu6050.setXGyroOffset(0);
  this->_mpu6050.setYGyroOffset(0);
  this->_mpu6050.setZGyroOffset(0);

  this->_averageReadings(&raw_data);

  calibration.ax = raw_data.ax;
  calibration.ay = raw_data.ay;
  calibration.az = raw_data.az;
 
  calibration.gx = raw_data.gx;
  calibration.gy = raw_data.gy;
  calibration.gz = raw_data.gz;

  while (retries <= CALIBRATION_RETRIES){
    retries++;
    int ready=0;
    this->_mpu6050.setXAccelOffset(calibration.ax);
    this->_mpu6050.setYAccelOffset(calibration.ay);
    this->_mpu6050.setZAccelOffset(calibration.az);
 
    this->_mpu6050.setXGyroOffset(calibration.gx);
    this->_mpu6050.setYGyroOffset(calibration.gy);
    this->_mpu6050.setZGyroOffset(calibration.gz);
 
    this->_averageReadings(&raw_data);
 
    if (abs(raw_data.ax) <= ACCEL_ERROR) ready++;
    else calibration.ax += raw_data.ax;
 
    if (abs(raw_data.ay) <= ACCEL_ERROR) ready++;
    else calibration.ay += raw_data.ay;
 
    if (abs(raw_data.az) <= ACCEL_ERROR) ready++;
    else calibration.az += raw_data.az;
 
    if (abs(raw_data.gx) <= GYRO_ERROR) ready++;
    else calibration.gx = raw_data.gx;
 
    if (abs(raw_data.gy) <= GYRO_ERROR) ready++;
    else calibration.gy = raw_data.gy;
 
    if (abs(raw_data.gz) <= GYRO_ERROR) ready++;
    else calibration.gz = raw_data.gz;
 
    if (ready==6) break;
  }
}


void SensorMpu6050::_averageReadings(t_calibration_data *raw_data){
  int16_t i=0,buff_ax=0,buff_ay=0,buff_az=0,buff_gx=0,buff_gy=0,buff_gz=0;
  int16_t ax=0,ay=0,az=0,gx=0,gy=0,gz=0;

  while (i<=(BUFFER_SIZE)) {
    // read raw accel/gyro measurements from device
    this->_mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    buff_ax += ax;
    buff_ay += ay;
    buff_az += (ACCEL_SENSITIVITY_FACTOR - az);
    buff_gx += gx;
    buff_gy += gy;
    buff_gz += gz;

    i++;
    delay(2); //Needed so we don't get repeated measures
  }
  raw_data->ax = buff_ax/(BUFFER_SIZE * ACCEL_CALIBRATION_FACTOR);
  raw_data->ay = buff_ay/(BUFFER_SIZE * ACCEL_CALIBRATION_FACTOR);
  raw_data->az = buff_az/(BUFFER_SIZE * ACCEL_CALIBRATION_FACTOR);
  raw_data->gx = buff_gx/(BUFFER_SIZE * GYRO_CALIBRATION_FACTOR);
  raw_data->gy = buff_gy/(BUFFER_SIZE * GYRO_CALIBRATION_FACTOR);
  raw_data->gz = buff_gz/(BUFFER_SIZE * GYRO_CALIBRATION_FACTOR);
}

void SensorMpu6050::isr() {}