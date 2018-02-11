/*
 * hopping.h
 *
 *  Created on: Feb 8, 2018
 *      Author: bibei
 */

#ifndef INCLUDE_GAIT_SL_HOPPING_H_
#define INCLUDE_GAIT_SL_HOPPING_H_

#include <gait/gait_base.h>
#include <Eigen/Dense>

///! Forward declaration
class TimeControl;

namespace qr_control {

enum SLHPState {
  INVALID_SL_HP_STATE = -1,
  SL_HP_TRU_STATE,
  SL_HP_FGT_STATE,
  SL_HP_STP_STATE,
  SL_STC_TEST_STATE,
  N_SL_HP_STATE
};

class SLHopping: public GaitBase {
public:
  SLHopping();
  virtual bool auto_init() override;

  virtual ~SLHopping();

///! These methods are inherited from super class.
public:
  virtual void checkState() override;
  ///! call it after the state callback
  virtual void post_tick() override;

  virtual bool starting()   override;
  virtual void stopping()   override;

///! These methods are the callback methods for WalkState.
private:
  ///! callback for SL_TRU_STATE
  void sl_hp_thrust();
  ///! callback for SL_FGT_STATE
  void sl_hp_flight();
  ///! callback for SL_STP_STATE
  void sl_hp_stopping();
  ///! callback for SL_STC_TEST_STATE
  void sl_static_test();

protected:
  ///! The state enumeration for trot.
  SLHPState             current_state_;
  ///! The private parameters for trot.
  class __SLHPParams*   params_;
  ///! The interface for legs
  class RobotLeg*       leg_iface_;
  ///! The Control tick interval for send command(in ms)
  int64_t               post_tick_interval_;

private:
  ///! The position of leg
  Eigen::Vector3d       last_leg_eef_;
  ///! The last difference of the end-effector.
  Eigen::Vector3d       last_eef_delta_;
  ///! The timer for static test.
  TimeControl*          stc_test_timer_;
  ///! The output command of static test.
  class LegTarget*      leg_cmd_;
};

} /* namespace qr_control */

#endif /* INCLUDE_GAIT_TROT_TROT_H_ */
