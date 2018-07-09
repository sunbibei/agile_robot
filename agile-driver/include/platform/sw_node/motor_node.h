/*
 * motor_node.h
 *
 *  Created on: Apr 12, 2018
 *      Author: bibei
 */

#ifndef INCLUDE_SYSTEM_PLATFORM_SW_NODE_MOTOR_NODE_H_
#define INCLUDE_SYSTEM_PLATFORM_SW_NODE_MOTOR_NODE_H_

#include "sw_node.h"
#include "toolbox/time_control.h"

#include <cmath>

#define Joint_Num 3
#define DataMin 0
#define DataMax 1

namespace agile_robot {

class MotorNode: public SWNode {
typedef struct _PID_Struct{
	   float SetLocation;        //定义设定值
	   float ActualLocation;       //定义实际值
	   float IncrementLocation;    //定义位置增量值
	   float Locatioin_err;           //定义偏差值
	   float Location_err_last;         //定义上一个偏差值
	   float Location_err_next;         //定义下一个偏差值
	   float L_Kp,L_Ki,L_Kd;         //定义位置比例、积分、微分系数
	   float S_Kp,S_Ki,S_Kd;         //定义速度比例、积分、微分系数
	   float Output_Speed;         //定义输出速度（位置环输出值）
	   float Location_integral;         //定义位置环积分值
}PID_Struct;

public:
  MotorNode();
  virtual bool auto_init() override;

  virtual ~MotorNode();

  virtual void handleMsg(const Packet&)          override;
  virtual bool generateCmd(std::vector<Packet>&) override;

  void Initialize(float l_kp,float l_ki,float l_kd,float s_kp,float s_ki,float s_kd);

  void Initialize_increment(float l_kp,float l_ki,float l_kd,float s_kp,float s_ki,float s_kd);

  float PID_realize(float setlocation,float actuallocation);

  float PID_incremental_realize(float setlocation,float actuallocation);

//  float Location_datalimit(float data, float dataMax, float dataMin);
///! The helper methods
private:
  bool __fill_pos_cmd(std::vector<Packet>& pkts);
  // bool __fill_vel_cmd(std::vector<Packet>& pkts);
  // bool __fill_tor_cmd(std::vector<Packet>& pkts);
  // bool __fill_pos_vel_cmd(std::vector<Packet>& pkts);
  // bool __fill_motor_vel_cmd(std::vector<Packet>& pkts);
  void __parse_heart_beat_1(const unsigned char*);

  PID_Struct PID;
protected:
  ///! Whether is startup the motor
  bool                         is_startup_;
  // there are three joint in each leg
  LegType                      leg_;
  ///! The type of joint.
  JntType                      jnt_;
  ///! update each motor, this vector order by the type of joint.
  class Motor*                 motor_handle_;
  ///! the source of command, this vector order by the type of joint.
  class Joint*              joint_handle_;
  // The constant pointer of the joint command
  const JntCmdType&         jnt_mode_;
  const double*             jnt_cmd_;
  const short*              motor_cmd_;

  ///! For generateCmd delay
  TimeControl*              cmd_tick_time_ctrl_;
  uint64_t                  cmd_tick_interval_;
  uint64_t                  sum_tick_interval_;

};

} /* namespace agile_robot */

#endif /* INCLUDE_SYSTEM_PLATFORM_SW_NODE_MOTOR_NODE_H_ */
