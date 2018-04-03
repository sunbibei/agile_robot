/*
 * imu_sensor.cpp
 *
 *  Created on: Oct 11, 2017
 *      Author: bibei
 */

#include "repository/resource/imu_sensor.h"
#include "foundation/cfg_reader.h"

namespace middleware {

struct __PrivateImuData {
  ///< A pointer to the storage of the orientation value: a quaternion (x,y,z,w)
  double* orientation;
  ///< A pointer to the storage of the orientation covariance value: a row major 3x3 matrix about (x,y,z)
  double* orientation_covariance;
  ///< A pointer to the storage of the angular velocity value: a triplet (x,y,z)
  double* angular_velocity;
  ///< A pointer to the storage of the angular velocity covariance value: a row major 3x3 matrix about (x,y,z)
  double* angular_velocity_covariance;
  ///< A pointer to the storage of the linear acceleration value: a triplet (x,y,z)
  double* linear_acceleration;
  ///< A pointer to the storage of the linear acceleration covariance value: a row major 3x3 matrix about (x,y,z)
  double* linear_acceleration_covariance;
};

ImuSensor::ImuSensor(const std::string& l)
: Label(l), imu_data_(new __PrivateImuData) {

}

bool ImuSensor::auto_init() {
  imu_data_->orientation                    = new double[4];   // a quaternion (x,y,z,w)
  imu_data_->orientation_covariance         = new double[3*3]; // a row major 3x3 matrix about (x,y,z)
  imu_data_->angular_velocity               = new double[3];   // a triplet (x,y,z)
  imu_data_->angular_velocity_covariance    = new double[3*3]; // a row major 3x3 matrix about (x,y,z)
  imu_data_->linear_acceleration            = new double[3];   // a triplet (x,y,z)
  imu_data_->linear_acceleration_covariance = new double[3*3]; // a row major 3x3 matrix about (x,y,z)
  memset(imu_data_->orientation,                    0x00, 4 *   sizeof(double));
  memset(imu_data_->orientation_covariance,         0x00, 3*3 * sizeof(double));
  memset(imu_data_->angular_velocity,               0x00, 3 *   sizeof(double));
  memset(imu_data_->angular_velocity_covariance,    0x00, 3*3 * sizeof(double));
  memset(imu_data_->linear_acceleration,            0x00, 3 *   sizeof(double));
  memset(imu_data_->linear_acceleration_covariance, 0x00, 3*3 * sizeof(double));
  return true;
}

ImuSensor::~ImuSensor() {
  if (nullptr != imu_data_) {
    if (nullptr != imu_data_->orientation) {
      delete imu_data_->orientation;
      imu_data_->orientation = nullptr;
    }
    if (nullptr != imu_data_->orientation_covariance) {
      delete imu_data_->orientation_covariance;
      imu_data_->orientation_covariance = nullptr;
    }
    if (nullptr != imu_data_->angular_velocity) {
      delete imu_data_->angular_velocity;
      imu_data_->angular_velocity = nullptr;
    }
    if (nullptr != imu_data_->angular_velocity_covariance) {
      delete imu_data_->angular_velocity_covariance;
      imu_data_->angular_velocity_covariance = nullptr;
    }
    if (nullptr != imu_data_->linear_acceleration) {
      delete imu_data_->linear_acceleration;
      imu_data_->linear_acceleration = nullptr;
    }
    if (nullptr != imu_data_->linear_acceleration_covariance) {
      delete imu_data_->linear_acceleration_covariance;
      imu_data_->linear_acceleration_covariance = nullptr;
    }
    delete imu_data_;
    imu_data_ = nullptr;
  }
}

/*const */ double* ImuSensor::orientation_const_pointer() {
  return imu_data_->orientation;
}

/*const */ double* ImuSensor::orientation_covariance_const_pointer() {
  return imu_data_->orientation_covariance;
}

/*const */ double* ImuSensor::angular_velocity_const_pointer() {
  return imu_data_->angular_velocity;
}

/*const */ double* ImuSensor::angular_velocity_covariance_const_pointer() {
  return imu_data_->angular_velocity_covariance;
}

/*const */ double* ImuSensor::linear_acceleration_const_pointer() {
  return imu_data_->linear_acceleration;
}

/*const */ double* ImuSensor::linear_acceleration_covariance_const_pointer() {
  return imu_data_->linear_acceleration_covariance;
}

///! The interface for ImuNode
void ImuSensor::updateLinearAcc(double x, double y, double z) {
  // LOG_WARNING << "ACC: " << x << " " << y << " " << z << std::endl;
  imu_data_->linear_acceleration[0] = x;
  imu_data_->linear_acceleration[1] = y;
  imu_data_->linear_acceleration[2] = z;
}

void ImuSensor::updateOrientation(double x, double y, double z, double w) {
  // LOG_WARNING << "Orientation: " << x << " " << y << " " << z << " " << w << std::endl;
  imu_data_->orientation[0] = x;
  imu_data_->orientation[1] = y;
  imu_data_->orientation[2] = z;
  imu_data_->orientation[3] = w;
}

void ImuSensor::updateAngVel(double x, double y, double z) {
  // LOG_WARNING << "AngVel: " << x << " " << y << " " << z << std::endl;
  imu_data_->angular_velocity[0] = x;
  imu_data_->angular_velocity[1] = y;
  imu_data_->angular_velocity[2] = z;
}

} /* namespace middleware */


#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(middleware::ImuSensor, Label)
