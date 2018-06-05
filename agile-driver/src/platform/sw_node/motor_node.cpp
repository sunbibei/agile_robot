/*
 * motor_node.cpp
 *
 *  Created on: Apr 12, 2018
 *      Author: bibei
 */

#include "platform/sw_node/motor_node.h"
#include "platform/propagate/can_usb.h"

#include "foundation/utf.h"
#include "foundation/cfg_reader.h"

#include "repository/resource/joint_manager.h"
#include "repository/resource/joint.h"
#include "repository/resource/motor.h"
#include "foundation/thread/threadpool.h"

#include <boost/algorithm/string.hpp>
#include <stdio.h>
#include <iostream>
#include <cstring>

namespace agile_robot {

// angle = \frac{360 \pi \alpha}{180*4096} C - \frac{\pi}{18000}\alpha*\beta
// so, the ABS(scale) = \frac{360 \pi \alpha}{180*4096} = \frac{360\pi}{180*4096}
// offset = - \frac{\pi}{18000}\alpha*\beta = -0.000174528*\beta
struct __PrivateLinearParams {
  double scale;
  double offset;
};

MotorNode::MotorNode()
  : SWNode("motor_node"), is_startup_(false),
    leg_(LegType::UNKNOWN_LEG),  jnt_(JntType::UNKNOWN_JNT),
    motor_handle_(nullptr), jnt_param_(nullptr), joint_handle_(nullptr),
    jnt_mode_(JointManager::instance()->getJointCommandMode()),
    cmd_tick_time_ctrl_(nullptr), cmd_tick_interval_(2),
    sum_tick_interval_(0) {
  jnt_cmd_   = nullptr;
  motor_cmd_ = nullptr;
}

MotorNode::~MotorNode() {
  ; // Nothing to do here.
}

bool MotorNode::auto_init() {
  if (!SWNode::auto_init()) return false;
  auto cfg = MiiCfgReader::instance();
  cfg->get_value_fatal(getLabel(), "leg", leg_);
  cfg->get_value_fatal(getLabel(), "jnt", jnt_);

  jnt_param_    = nullptr;
  motor_handle_ = nullptr;
  joint_handle_ = nullptr;

  joint_handle_ = JointManager::instance()->getJointHandle(leg_, jnt_);
  if (!joint_handle_ || !joint_handle_->joint_motor())
    LOG_FATAL << "Can't get the joint '" << LEGTYPE2STR(leg_) << " - "
        << JNTTYPE2STR(jnt_) << "' pointer from JointManager, or the motor within joint.";

  jnt_param_  = new __PrivateLinearParams;
  double alpha = 0, beta = 0;
  cfg->get_value_fatal(getLabel(), "scale",  alpha);
  cfg->get_value_fatal(getLabel(), "offset", beta);
  jnt_param_->scale  = alpha;
  jnt_param_->offset = beta;
  // LOG_DEBUG << jnt->joint_name() << ": " << param->scale << ", " << param->offset;

  motor_handle_ = joint_handle_->joint_motor();

  jnt_cmd_       = joint_handle_->joint_command_const_pointer();
  motor_cmd_     = joint_handle_->joint_motor()->motor_command_const_pointer();

  ///! Got the parameters of PID
  std::vector<float> gains;
  std::string _pid_tag = Label::make_label(getLabel(), "pid");
  cfg->get_value_fatal(_pid_tag, "gains", gains);
  if (6 != gains.size()) {
    LOG_FATAL << "The input parameters of PID in " << LEGTYPE2STR(leg_)
      << " - " << JNTTYPE2STR(jnt_) << " error!";
  }

  Initialize(gains[0], gains[1], gains[2], gains[3], gains[4], gains[5]);

  cfg->get_value_fatal(getLabel(), "interval", cmd_tick_interval_);
  cmd_tick_time_ctrl_ = new TimeControl();
  cmd_tick_time_ctrl_->start();
  return true;
}

void MotorNode::handleMsg(const Packet& pkt) {
	// int pos_value,vel_value;
	// Comd[0].driver_id = pkt.node_id;         //驱动器ID号
	// Comd[0].msg_id = pkt.msg_id;             //位置 速度 力矩命令标识符
	// Comd[0].datasize = pkt.size;             //数据长度
  auto msg_id = MII_MSG_HEARTBEAT_1;
	if(pkt.data[0] == 0x49 && pkt.data[1] == 0x51)
	{
    msg_id = MII_MSG_HEARTBEAT_1;

	//驱动器反馈值为电流值
	// Comd[0].msg_id = MII_MSG_HEARTBEAT_1;
	// memcpy(&Comd[0].actualvalue, &pkt.data[4], 4);
 //        std::cout<<"Torque Value : "<<Comd[0].actualvalue<<std::endl;
	}else if(pkt.data[0] == 0x56 && pkt.data[1] == 0x58)
	{
    msg_id = MII_MSG_HEARTBEAT_2;
		//驱动器反馈值为速度值
	// Comd[0].msg_id = MII_MSG_HEARTBEAT_2;
	// memcpy(&vel_value, &pkt.data[4], 4);
 //        Comd[0].actualvalue = vel_value;
 //        std::cout<<"Velocity Value : "<<Comd[0].actualvalue<<std::endl;
 //        printf("  <- NODE_ID:0x%02X MSG_ID:0x%02X LEN:%1x DATA:0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
 //        (int)pkt.node_id,
 //        (int)pkt.msg_id,  (int)pkt.size,
 //        (int)pkt.data[0], (int)pkt.data[1],
 //        (int)pkt.data[2], (int)pkt.data[3],
 //        (int)pkt.data[4], (int)pkt.data[5],
 //        (int)pkt.data[6], (int)pkt.data[7]);
	}else if(pkt.data[0] == 0x50 && pkt.data[1] == 0x58)
	{
    msg_id = MII_MSG_HEARTBEAT_3;
		// //驱动器反馈值为位置值
		// Comd[0].msg_id = MII_MSG_HEARTBEAT_3;
		// memcpy(&pos_value, &pkt.data[4], 4);
  //               Comd[0].actualvalue = pos_value;
  //               std::cout<<"Position Value : "<<Comd[0].actualvalue<<std::endl;
	} else {
	  LOG_ERROR << "ERROR data of msg";
    LOG_HEADER;
    printf(" NODE_ID:0x%02X MSG_ID:0x%02X LEN:%1x DATA:0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
      (int)pkt.node_id, (int)pkt.msg_id,  (int)pkt.size,
      (int)pkt.data[0], (int)pkt.data[1], (int)pkt.data[2], (int)pkt.data[3],
      (int)pkt.data[4], (int)pkt.data[5], (int)pkt.data[6], (int)pkt.data[7]);
	}

  float _tor = 0.0;
  int   _vel = 0, _pos = 0;
	switch (msg_id) {
	  case MII_MSG_HEARTBEAT_1:
	    if (8 != pkt.size) {
	      LOG_ERROR << "The data size of MII_MSG_HEARTBEAT_MSG_1 message does not match!"
	          << ", the expect size is 8, but the real size is " << pkt.size;
	      return;
	    }
      
      memcpy(&_tor, pkt.data + 4, sizeof(_tor));
	    // parse the joint state and touchdown data
      motor_handle_->updateMotorTorque(_tor);
	    break;
	  case MII_MSG_HEARTBEAT_2:
	  if (8 != pkt.size) {
			  LOG_ERROR << "The data size of MII_MSG_HEARTBEAT_MSG_1 message does not match!"
				  << ", the expect size is 8, but the real size is " << pkt.size;
			  return;
			}
      memcpy(&_vel, pkt.data + 4, sizeof(_vel));
			// parse the joint state and touchdown data
      motor_handle_->updateMotorVelocity(_vel);
	   break;
	  case MII_MSG_HEARTBEAT_3:
      if (8 != pkt.size) {
        LOG_ERROR << "The data size of MII_MSG_HEARTBEAT_MSG_1 message does not match!"
          << ", the expect size is 8, but the real size is " << pkt.size;
        return;
      }
      memcpy(&_pos, pkt.data + 4, sizeof(_pos));
      // parse the joint state and touchdown data
      motor_handle_->updateMotorPosition(_pos);
      break;
	  default:
	    SWNode::handleMsg(pkt);
	}

}

bool MotorNode::generateCmd(std::vector<Packet>& pkts) {
  if (!is_startup_) {
  Packet cmd = {bus_id_, node_id_, MII_MSG_MOTOR_3, 8, {0}};

  cmd.data[0] = 0x01;
  cmd.data[1] = 0x00;
  pkts.push_back(cmd);
  
  cmd.data[0] = 0x55;
  cmd.data[1] = 0x4D;
  cmd.data[2] = 0x00;
  cmd.data[3] = 0x00;
  cmd.data[4] = 0x02; 
  cmd.data[5] = 0x00;
  cmd.data[6] = 0x00;
  cmd.data[7] = 0x00;
  pkts.push_back(cmd);

  cmd.data[0]  = 0x4D;
  cmd.data[1]  = 0x4F;
  cmd.data[2]  = 0x00;
  cmd.data[3]  = 0x00;
  cmd.data[4]  = 0x01; 
  cmd.data[5]  = 0x00;
  cmd.data[6]  = 0x00;
  cmd.data[7]  = 0x00;
  pkts.push_back(cmd);

  is_startup_ = true;
  return true;
  }

  bool is_any_valid = false;
  switch (jnt_mode_) {
  case JntCmdType::CMD_POS:
    is_any_valid = __fill_pos_cmd(pkts);
    break;
  // case JntCmdType::CMD_VEL:
  //   is_any_valid = __fill_vel_cmd(pkts);
  //   break;
  // case JntCmdType::CMD_TOR:
  //   is_any_valid = __fill_tor_cmd(pkts);
  //   break;
  // case JntCmdType::CMD_POS_VEL:
  //   is_any_valid = __fill_pos_vel_cmd(pkts);
  //   break;
  // case JntCmdType::CMD_MOTOR_VEL:
  //   is_any_valid = __fill_motor_vel_cmd(pkts);
  //   break;
  default:
    LOG_ERROR << "What a fucking the command mode of joint.";
  }

  return is_any_valid;
}

bool MotorNode::__fill_pos_cmd(std::vector<Packet>& pkts) {
  sum_tick_interval_ += cmd_tick_time_ctrl_->dt();
  if (sum_tick_interval_ < cmd_tick_interval_) return false;
  sum_tick_interval_ = 0;

  Packet cmd = {bus_id_, node_id_, MII_MSG_MOTOR_3, 8, {0}};
 int cmd_val = PID_realize(*jnt_cmd_, joint_handle_->joint_position());
 // int cmd_val = PID_realize(-1.5, joint_handle_->joint_position());
  std::cout<<"jnt_cmd Value : "<<*jnt_cmd_<<std::endl;
  std::cout<<"joint_position Value : "<<joint_handle_->joint_position()<<std::endl;
  std::cout<<"cmd_val Value : "<<cmd_val<<std::endl;
  cmd.data[0] = 0x4A;
  cmd.data[1] = 0x56;
  cmd.data[2] = 0x00;
  cmd.data[3] = 0x00;
  cmd.data[4] = cmd_val & 0xff;
  cmd.data[5] = (cmd_val >> 8) & 0xff;
  cmd.data[6] = (cmd_val >> 16) & 0xff;
  cmd.data[7] = (cmd_val >> 24) & 0xff;
  printf("  <- NODE_ID:0x%02X MSG_ID:0x%02X LEN:%1x DATA:0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
        (int)cmd.node_id,
        (int)cmd.msg_id,  (int)cmd.size,
        (int)cmd.data[0], (int)cmd.data[1],
        (int)cmd.data[2], (int)cmd.data[3],
        (int)cmd.data[4], (int)cmd.data[5],
        (int)cmd.data[6], (int)cmd.data[7]);
  joint_handle_->new_command_ = false;
  pkts.push_back(cmd);

  cmd.size = 4;
  cmd.data[0]  = 0x42;
  cmd.data[1]  = 0x47;
  cmd.data[2]  = 0x00;
  cmd.data[3]  = 0x00;
  pkts.push_back(cmd);
  return true;
}

float Location_datalimit(float data, float dataMax, float dataMin){
	if(data >= dataMax)
	{
		data = dataMax;
	}
	if(data <= dataMin)
	{
		data = dataMin;
	}
	return data;
}
void MotorNode::Initialize(float l_kp,float l_ki,float l_kd,float s_kp,float s_ki,float s_kd)
{
    //初始化控制器参数
   PID.L_Kp = l_kp;
   PID.L_Ki = l_ki;
   PID.L_Kd = l_kd;
   PID.S_Kp = s_kp;
   PID.S_Ki = s_ki;
   PID.S_Kd = s_kd;

}

void MotorNode::Initialize_increment(float l_kp,float l_ki,float l_kd,float s_kp,float s_ki,float s_kd)
{
    //初始化增量式控制器参数
	PID.L_Kp = l_kp;
	PID.L_Ki = l_ki;
	PID.L_Kd = l_kd;
	PID.S_Kp = s_kp;
	PID.S_Ki = s_ki;
	PID.S_Kd = s_kd;
	PID.Locatioin_err = 0;
	PID.Location_err_last = 0;
	PID.Location_err_next = 0;

}

float MotorNode :: PID_realize(float setlocation,float actuallocation)
{   
    float Outspeed;
    PID.SetLocation = setlocation;
    PID.ActualLocation = actuallocation;
    PID.Locatioin_err = PID.SetLocation-PID.ActualLocation;
    PID.Location_integral += PID.Locatioin_err;
    PID.Output_Speed = PID.L_Kp * PID.Locatioin_err + PID.L_Ki * PID.Location_integral + PID.L_Kd * (PID.Locatioin_err-PID.Location_err_last);
    PID.Location_err_last = PID.Locatioin_err;
    if(actuallocation >= 1.4)
    {
       Outspeed = 0;
    }else{
      Outspeed = PID.Output_Speed ;// /(std::abs(PID.Output_Speed)) * (std::abs(PID.Output_Speed));
    }
    return Outspeed;
//    return Location_datalimit(PID.Output_Speed, DataMax, DataMin);
}

float MotorNode ::PID_incremental_realize(float setlocation,float actuallocation)
{
	PID.SetLocation = setlocation;
	PID.ActualLocation = actuallocation;
	PID.Locatioin_err = PID.SetLocation-PID.ActualLocation;
	PID.IncrementLocation = PID.L_Kp * (PID.Locatioin_err - PID.Location_err_next) + PID.L_Ki * PID.Locatioin_err + PID.L_Kd * (PID.Locatioin_err - 2 * PID.Location_err_next + PID.Location_err_last);
	PID.Output_Speed += PID.IncrementLocation;
	PID.Location_err_last = PID.Location_err_next;
	PID.Location_err_next = PID.Locatioin_err;
	return PID.Output_Speed;
}

} /* namespace agile_robot */

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(agile_robot::MotorNode, Label)
