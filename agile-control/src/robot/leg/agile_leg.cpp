/*
 * qr_leg.cpp
 *
 *  Created on: Dec 7, 2017
 *      Author: bibei
 *  Implemented by Sampson on Dec 8, 2017
 */

#include <foundation/cfg_reader.h>
#include <foundation/auto_instor.h>
#include <robot/leg/agile_leg.h>

#include <chrono>
#include <thread>
#include <iostream>

namespace agile_control {
// params list and parser from cfg
class QrLegTopology
{
public:
  QrLegTopology(const std::string& _prefix) {
    auto cfg = CfgReader::instance();
    cfg->get_value_fatal(_prefix, "base",  BASE_LEN);
    cfg->get_value_fatal(_prefix, "thigh", THIGH_LEN);
    cfg->get_value_fatal(_prefix, "shank", SHANK_LEN);
    cfg->get_value_fatal(_prefix, "sign",  sign);
  }

  const double& L0()   const { return BASE_LEN;  }
  const double& L1()   const { return THIGH_LEN; }
  const double& L2()   const { return SHANK_LEN; }
  const int&    SIGN() const { return sign; }

private:
  int    sign;
  double BASE_LEN;
  double THIGH_LEN;
  double SHANK_LEN;
};

AgileLeg::AgileLeg()
  : td_thres_(0), topology_(nullptr)
{
  ; // Nothing to do here.
}

bool AgileLeg::auto_init() {
  if (!RobotLeg::auto_init()) return false;

  auto cfg = CfgReader::instance();
  cfg->get_value(getLabel(), "td_threshold", td_thres_);

  topology_ = new QrLegTopology(Label::make_label(getLabel(), "topology"));
  return true;
}

AgileLeg::~AgileLeg()  {
  delete topology_;
  topology_ = nullptr;
}

void AgileLeg::followJntTrajectory(JntType jnt, const Traj1dSp& _traj)  {
  const auto& _jnt_poss = joint_position_const_ref();
  // const auto& _jnt_vels = joint_velocity_const_ref();
  // const auto& _jnt_cmds = joint_command_ref();

  // Just for debug
  for (int i = 0; i < 10; ++i) {
    std::cout << i
     << ": " << _jnt_poss(JntType::HFE)
     << ", " << _jnt_poss(JntType::HFE)
     << ", " << _jnt_poss(JntType::KFE) << std::endl;

    std::this_thread::sleep_for(std::chrono::seconds(4));
  }
}

void AgileLeg::followJntTrajectory(const Traj3dSp&) {
  LOG_ERROR << "Call the 'followJntTrajectory' which has does not complemented.";
}

void AgileLeg::followEefTrajectory(const Traj3dSp&)
{
  LOG_ERROR << "Call the 'followJntTrajectory' which has does not complemented.";
}

void AgileLeg::setForceThreshold(double threshold) {
  td_thres_ = threshold;
}

LegState AgileLeg::leg_state()  {
  // TODO
  // Eigen::Vector3d _xyz;
  // forwardKinematics(_xyz);
  // return ((eef().z() <= -46) ? LegState::TD_STATE : LegState::AIR_STATE);
  return (foot_force() > td_thres_) ? LegState::TD_STATE : LegState::AIR_STATE;
}

void AgileLeg::printDH() {
  LOG_INFO << "Axis No. /"<<"a(i-1) /"<<"alpha(i-1) /"<<"di /"<<"Joint Vars";
  LOG_INFO << "1    "<<"0    "<<"0    "<<"0   "<<"Theta_0";
  LOG_INFO << "2    "<<"L0   "<<"PI/2 "<<"0   "<<"Theta_1";
  LOG_INFO << "3    "<<"L1   "<<"0    "<<"0   "<<"Theta_2";
  LOG_INFO << "4    "<<"L2   "<<"0    "<<"0   "<<"0";
}

EMX AgileLeg::getTransMatrixT01(const EV3& a)
{
  EMX result(4,4);
  result << cos(a(0)),-sin(a(0)),0,0,
            sin(a(0)), cos(a(0)),0,0, 
            0,         0,        1,0,
            0,         0,        0,1;
   return result;
}
EMX AgileLeg::getTransMatrixT12(const EV3& a)
{
  EMX result(4,4);
  result << cos(a(1)),-sin(a(1)),0, topology_->L0(),
            0,         0,       -1,0, 
            sin(a(1)), cos(a(1)),0,0,
            0,         0,        0,1;
  return result;
}
EMX AgileLeg::getTransMatrixT23(const EV3& a)
{
  EMX result(4,4);
  result << cos(a(2)),-sin(a(2)),0, topology_->L1(),
            sin(a(2)), cos(a(2)),0,0, 
            0,         0,        1,0,
            0,         0,        0,1;
  return result;
}
EMX AgileLeg::getTransMatrixT34(const EV3& a)
{
  EMX result(4,4);
  result << 1,0,0, topology_->L2(),
            0,1,0,0, 
            0,0,1,0,
            0,0,0,1;
  return result; 
}
//if ture ,the inverse is useful then
bool AgileLeg::getJacobMatrix(const EV3& a, EM3& JacobMatrix, EM3& inverseJacobMatrix)
{
  bool invertible;
  auto L0 = topology_->L0();
  auto L1 = topology_->L1();
  auto L2 = topology_->L2();

  JacobMatrix << 0, L1*cos(a(1)) + L2*cos(a(1)+a(2)), L2*cos(a(1)+a(2)),
     L0*cos(a(0)) + L1*cos(a(0))*cos(a(1)) + L2*cos(a(0))*cos(a(1)+a(2)), 
    -L1*sin(a(0))*sin(a(1)) - L2*sin(a(0))*sin(a(1)+a(2)),
    -L2*sin(a(0))*sin(a(1)+a(2)),
     L0*sin(a(0)) + L1*sin(a(0))*cos(a(1)) + L2*sin(a(0))*cos(a(1)+a(2)), 
     L1*cos(a(0))*sin(a(1)) + L2*cos(a(0))*sin(a(1)+a(2)),
     L2*cos(a(0))*sin(a(1)+a(2));           

  JacobMatrix.computeInverseWithCheck(inverseJacobMatrix, invertible);

  return invertible;
}

EV3 AgileLeg::jointVelToFoot(const EV3& joint_pos, const EV3& joint_vel)
{ 
  EM3 JacobMatrix, inverseJacobMatrix;
  /*bool invertible = */getJacobMatrix(joint_pos, JacobMatrix, inverseJacobMatrix);
  return JacobMatrix * joint_vel;
}

EV3 AgileLeg::footVelToJoint(const EV3& joint_pos, const EV3& foot_vel)
{ 
  EV3 result(-100000,-100000,-100000);
  EM3 JacobMatrix, inverseJacobMatrix;
  bool invertible = getJacobMatrix(joint_pos,JacobMatrix,inverseJacobMatrix);
  if(invertible)
  {
    result = inverseJacobMatrix * foot_vel;
  }  
  return result; 
}
EV3 AgileLeg::getHipPostion(const EV3& a)
{
  EV3 result;
  EMX T = getTransMatrixT01(a);
  T = T * getTransMatrixT12(a);
  result(0) = T(0,3);
  result(1) = T(1,3);
  result(2) = T(2,3);
  return result;
}
EV3 AgileLeg::getKneePostion(const EV3& a)
{
  EV3 result;
  EMX T = getTransMatrixT01(a);
  T = T * getTransMatrixT12(a);
  T = T * getTransMatrixT23(a);
  result(0) = T(0,3);
  result(1) = T(1,3);
  result(2) = T(2,3);
  return result;
}


/*  
Description: calculating forward kinematics for 3 DOF leg using D-H methods
Formula:
   Px = L1 * S1 + L2 * S12;
   Py = L0 * S0 + L1 * S0 * C1 + L2 * S0 * C12;
   Pz = - Lo * C0 - L1 * C0 * C1 - L2 * C0 * C12;
*/
//void QrLeg::fk(Eigen::Vector3d& xyz, Eigen::Quaterniond&) {
//  LOG_ERROR << "Call the 'forwardKinematics' which has does not complemented.";
//}

void AgileLeg::fk(Eigen::Vector3d& xyz) {
  // TODO WSR is as follow
  // xyz.x() = -topology_->L1() * sin(hip())
  //     - topology_->L2() * sin(hip() + knee());
  xyz.x() = -topology_->L1() * sin(hip())
      - topology_->L2() * sin(hip() + knee());

  xyz.y() = topology_->L0() * sin(yaw()) + topology_->L1() * sin(yaw()) * cos(hip())
      + topology_->L2() * sin(yaw()) * cos(hip() + knee());

  xyz.z() = - topology_->L0() * cos(yaw()) - topology_->L1() * cos(yaw()) * cos(hip())
      - topology_->L2() * cos(yaw()) * cos(hip() + knee());
}

//void QrLeg::fk(Eigen::Quaterniond&) {
//  LOG_ERROR << "Call the 'forwardKinematics' which has does not complemented.";
//}
/*
Description: calculating reverse kinematics for quadruped robot(3 DOF)
Formula:
   Theta_0 = atan(- Py / Pz );
   Theta_1 = 2 * atan((Epsilon + sqrt(Epsilon^2 - Phi * (L2 * S2 - Delte))) / Phi);
   Theta_2 = sgn * acos((Delte^2 + Epsilon^2 - L1^2 - L2^2) / 2 / L1 / L2);
   Meantime, Delte = Px ; Phi = Delte + L2 * S2; Epsilon = L0 + Pz * C0 - Py * S0;
*/
void AgileLeg::ik(const Eigen::Vector3d& xyz, Eigen::VectorXd& jnts) {
  if (JntType::N_JNTS != jnts.size())
    jnts.resize(JntType::N_JNTS);

  double Delte = -xyz.x();
  jnts(JntType::HAA) = atan(-xyz.y() / xyz.z());

  double Epsilon = topology_->L0() + xyz.z() * cos(jnts(JntType::HAA))
      - xyz.y() * sin(jnts(JntType::HAA));

  jnts(JntType::KFE) = topology_->SIGN() * acos((pow(Delte,2) + pow(Epsilon,2)
      - pow(topology_->L1(),2) - pow(topology_->L2(),2)) / 2.0 / topology_->L1() / topology_->L2());

  double Phi = Delte + topology_->L2() * sin(jnts(JntType::KFE));
  if(Phi == 0)
    Phi = Phi + 0.000001;

  jnts(JntType::HFE) = 2 * atan((Epsilon + sqrt(pow(Epsilon,2)
      - Phi * (topology_->L2() * sin(jnts(JntType::KFE)) - Delte))) / Phi);
}

//void QrLeg::ik(const Eigen::Vector3d&, const Eigen::Quaterniond&, EVX& angle) {
//  LOG_ERROR << "Call the 'inverseKinematics' which has does not complemented.";
//}
//
//void QrLeg::ik(const Eigen::Quaterniond&, EVX& angle) {
//  LOG_ERROR << "Call the 'inverseKinematics' which has does not complemented.";
//}

/*!
 * The forward static, given the torque of each joint, calculate the force of the foot.
 * as follow:
 *      F = (J^T)^{-1} \cdot \tau
 */
void AgileLeg::fs(Eigen::Vector3d& _f_eef) {
  Eigen::Matrix3Xd _jacob;
  jacobian(_jacob);
  _f_eef = _jacob.transpose().inverse() * joint_torque();
}

/*!
 * The inverse static, given the force of the foot, calculate the torque of each joint.
 * as follow:
 *      \tau = J^T \cdot F.
 */
void AgileLeg::is(const Eigen::Vector3d& _f_eef, Eigen::VectorXd& _tor_jnts) {
  Eigen::Matrix3Xd _jacob;
  jacobian(_jacob);
  _tor_jnts = _jacob.transpose() * _f_eef;
}

/*!
 * It calculate the robot jacobian matrix. as follow:
 * \begin{matrix}
 *   0                                 & -L_1C_1-L_2C_{12}       & -L_2C_{12}    \\
 *   L_0C_0 + L_1C_0C_1 + L_2C_0C_{12} & -L_1S_0S_1-L_2S_0S_{12} & -L_2S_0S_{12} \\
 *   L_0S_0 + L_1S_0C_1 + L_2S_0C_{12} & L_1C_0S_1+L_2C_0S_{12}  & L_2C_0S_{12}
 * \end{matrix}
 * =
 * \begin{matrix}
 *   0            & -L_1C_1-L_2C_{12} & -L_2C_{12}    \\
 *   C_0 * \alpha & -S_0 * \beta      & -S_0 * \gamma \\
 *   S_0 * \alpha &  S_0 * \beta      &  C_0 * \gamma
 * \end{matrix}
 * where, \alpha = L_0 + L_1C_1 + L_2C_{12};
 *        \beta  = L_1S_1 - L_2S_{12};
 *        \gamma = L_2S_{12};
 */
void AgileLeg::jacobian(Eigen::Matrix3Xd& _jacob) {
  _jacob.resize(Eigen::NoChange, (int)JntType::N_JNTS);
  double c12   = cos(hip() + knee());
  double s12   = sin(hip() + knee());
  double alpha = topology_->L0() + topology_->L1() * cos(hip()) + topology_->L2() * c12;
  double beta  = topology_->L1() * sin(hip()) + topology_->L2() * s12;
  double gamma = topology_->L2() * s12;
  _jacob << 0,                  -topology_->L1()*cos(yaw()) - topology_->L2()*c12, topology_->L2()*c12,
            cos(yaw()) * alpha,                                  -sin(yaw())*beta, -sin(yaw())*gamma,
            sin(yaw()) * alpha,                                   sin(yaw())*beta,  cos(yaw())*gamma;

//  _jacob.row(0) << 0, -topology_->L1()*cos(yaw()) - topology_->L2()*c12, topology_->L2()*c12;
//  _jacob.row(1) << cos(yaw()) * alpha, -sin(yaw())*beta, -sin(yaw())*gamma;
//  _jacob.row(2) << sin(yaw()) * alpha, sin(yaw())*beta,  cos(yaw())*gamma;
}

} /* namespace qr_control */

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(agile_control::AgileLeg, Label)
