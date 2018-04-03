/*
 * data_body.h
 *
 *  Created on: Dec 28, 2017
 *      Author: bibei
 */

#ifndef INCLUDE_ROBOT_BODY_DATA_BODY_H_
#define INCLUDE_ROBOT_BODY_DATA_BODY_H_

#include <foundation/label.h>
#include <Eigen/Dense>
// namespace middleware { class ImuSensor; }

namespace agile_control {

class DataBody: public Label {
public:
  DataBody(const std::string& _l = "data-body");
  virtual bool auto_init() override;

  virtual ~DataBody();

///! These are the data interfaces
public:
  Eigen::VectorXd        orientation()               const;
  const Eigen::VectorXd& orientation_const_ref()     const;
  const Eigen::VectorXd* orientation_const_pointer() const;

  Eigen::MatrixXd        orientation_covariance()               const;
  const Eigen::MatrixXd& orientation_covariance_const_ref()     const;
  const Eigen::MatrixXd* orientation_covariance_const_pointer() const;

  Eigen::VectorXd        angular_velocity()               const;
  const Eigen::VectorXd& angular_velocity_const_ref()     const;
  const Eigen::VectorXd* angular_velocity_const_pointer() const;

  Eigen::MatrixXd        angular_velocity_covariance()               const;
  const Eigen::MatrixXd& angular_velocity_covariance_const_ref()     const;
  const Eigen::MatrixXd* angular_velocity_covariance_const_pointer() const;

  Eigen::VectorXd        linear_acceleration()               const;
  const Eigen::VectorXd& linear_acceleration_const_ref()     const;
  const Eigen::VectorXd* linear_acceleration_const_pointer() const;

  Eigen::MatrixXd        linear_acceleration_covariance()               const;
  const Eigen::MatrixXd& linear_acceleration_covariance_const_ref()     const;
  const Eigen::MatrixXd* linear_acceleration_covariance_const_pointer() const;

  double body_length() const;
  double body_height() const;
  double body_width()  const;
public:
  class _ImuSensor* imu_sensor_;

  class _BodyTopology* body_size_;
//  const double* imu_ang_vel_;
//  const double* imu_ang_vel_cov_;
//  const double* imu_lin_acc_;
//  const double* imu_lin_acc_cov_;
//  const double* imu_quat_;
//  const double* imu_quat_cov_;
};

} /* namespace qr_control */

#endif /* INCLUDE_ROBOT_BODY_DATA_BODY_H_ */
