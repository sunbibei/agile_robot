/*
 * imu_sensor.h
 *
 *  Created on: Oct 11, 2017
 *      Author: bibei
 */

#ifndef INCLUDE_REPOSITORY_RESOURCE_IMU_SENSOR_H_
#define INCLUDE_REPOSITORY_RESOURCE_IMU_SENSOR_H_

#include "foundation/label.h"

namespace middleware {

class ImuSensor: public Label {
public:
  friend class ImuNode;
  ImuSensor(const std::string& l = Label::null);
  // 妥协方案
  virtual bool auto_init() override;

  virtual ~ImuSensor();
public:
  /**
   * Interface for user layer.
   * TODO The hardware_interface::ImuSensorHandle::Data needs non-const pointer.
   */
  /*const */ double* orientation_const_pointer();
  /*const */ double* orientation_covariance_const_pointer();

  /*const */ double* angular_velocity_const_pointer();
  /*const */ double* angular_velocity_covariance_const_pointer();

  /*const */ double* linear_acceleration_const_pointer();
  /*const */ double* linear_acceleration_covariance_const_pointer();

protected:
  /**
   * Interface for system layer.
   */
  void updateLinearAcc(double x, double y, double z);
  void updateOrientation(double x, double y, double z, double w);
  void updateAngVel(double x, double y, double z);
  // The private IMU data structure define
  class __PrivateImuData* imu_data_;
};

} /* namespace middleware */

#endif /* INCLUDE_REPOSITORY_RESOURCE_IMU_SENSOR_H_ */
