/*
 * leg_robot.cpp
 *
 *  Created on: Jan 9, 2018
 *      Author: bibei
 */

#include <robot/leg_robot.h>

#ifdef DIS_JNT_LIMIT
#include <repository/resource/joint_manager.h>
#endif

namespace qr_control {

LegRobot* LegRobot::leg_robot_inst_ = nullptr;

LegRobot* LegRobot::instance() {
  if (nullptr == leg_robot_inst_)
    LOG_WARNING << "This method instance() should be called after "
        << "create_instance()";
  return leg_robot_inst_;
}

LegRobot::LegRobot()
  : body_iface_(nullptr) {
  for (auto& l : leg_ifaces_)
    l = nullptr;

  leg_robot_inst_ = this;
}

LegRobot::~LegRobot() {
	; // Nothing to do here.
}

/*!
 * @brief The interface for robot leg.
 */
RobotLeg* LegRobot::robot_leg(LegType _l) {
  return ((LegType::UNKNOWN_LEG == _l) || (LegType::N_LEGS == _l)) ? nullptr : leg_ifaces_[_l];
}

/*!
 * @brief The interface for robot body,
 */
RobotBody* LegRobot::robot_body() {
  return body_iface_;
}




inline void __print_color_helper(LegType l) {
  FOR_EACH_JNT(j) {
#ifdef DIS_JNT_LIMIT
    static auto _jnts  = middleware::JointManager::instance();
    const auto _jnt   = _jnts->getJointHandle(LegType(l), JntType(j));
    double _min = _jnt->joint_position_min();
    double _max = _jnt->joint_position_max();
    double _val = _jnt->joint_position();
    if (_val <= _min)
      printf("| \033[31;1m%+7.04f\033[0m", _val);
    else if (_val >= _max)
      printf("| \033[33;1m%+7.04f\033[0m", _val);
    else
#endif
      printf("| %+7.04f", _val);
  }
  printf("|");
}

inline void __print_color_helper(LegType l, const Eigen::VectorXd& jnt) {
  FOR_EACH_JNT(j) {
#ifdef DIS_JNT_LIMIT
    static auto _jnts  = middleware::JointManager::instance();
    const auto _jnt   = _jnts->getJointHandle(LegType(l), JntType(j));
    double _min = _jnt->joint_position_min();
    double _max = _jnt->joint_position_max();
    if (jnt(j) <= _min)
      printf("| \033[31;1m%+7.04f\033[0m", jnt(j));
    else if (jnt(j) >= _max)
      printf("| \033[33;1m%+7.04f\033[0m", jnt(j));
    else
#endif
      printf("| %+7.04f", jnt(j));
  }
  printf("|");
}

///! print joint position of the single leg
void print_jnt_pos(LegType l) {
  printf("___________________________________________\n");
  printf("|    -|   YAW  |   HIP  |  KNEE  |\n");
//printf("|LEG -| +0.0000| +0.0000| +0.0000|\n");
  printf("|    -");
  __print_color_helper(l);
  printf("\n-------------------------------------------\n");
}

///! print the v.s. result and different between joint position of the single leg
void print_jnt_pos(LegType l, const Eigen::VectorXd& _tjnt) {
  auto _jnt = LegRobot::instance()->robot_leg(l)->joint_position_const_ref();
  printf("___________________________________________\n");
  printf("|    -|   YAW  |   HIP  |  KNEE  |  ERROR |\n");
//printf("|LEG -| +0.0000| +0.0000| +0.0000| +0.0000|\n");
  printf("|    -"); __print_color_helper(l); printf("\n");
  printf("|    =| %+7.04f| %+7.04f| %+7.04f| %+7.04f|\n",
      _tjnt(JntType::YAW), _tjnt(JntType::HIP), _tjnt(JntType::KNEE), (_tjnt - _jnt).norm());
}
///! print the joint position of the all of leg.
void print_jnt_pos() {
  printf("___________________________________________________________________\n");
  printf("|LEG -|   YAW  |   HIP  |  KNEE  |LEG -|   YAW  |   HIP  |  KNEE  |\n");
//printf("LEG -| +0.0000| +0.0000| +0.0000|LEG -| +0.0000| +0.0000| +0.0000|\n");
  printf("| FL -"); __print_color_helper(LegType::FL);
  printf( " FR -"); __print_color_helper(LegType::FR); printf("\n");

  printf("| HL -"); __print_color_helper(LegType::HL);
  printf( " HR -"); __print_color_helper(LegType::HR); printf("\n");
  printf("-------------------------------------------------------------------\n");
}
///! print the v.s. result and different between joint position of the all of leg.
void print_jnt_pos(const Eigen::VectorXd& fl, const Eigen::VectorXd& fr,
    const Eigen::VectorXd& hl, const Eigen::VectorXd& hr) {
  printf("_____________________________________________________________________________________\n");
  printf("|LEG -|   YAW  |   HIP  |  KNEE  |  ERROR |LEG -|   YAW  |   HIP  |  KNEE  |  ERROR |\n");
//printf("|LEG  | +0.0000| +0.0000| +0.0000| +0.0000|LEG  | +0.0000| +0.0000| +0.0000| +0.0000|\n");
  printf("| FL -"); __print_color_helper(LegType::FL);     printf("    -   |");
  printf( " FR -"); __print_color_helper(LegType::FR);     printf("    -   |\n");

  printf("| FL ="); __print_color_helper(LegType::FL, fl);
  printf("%+8.04f|", (fl - LegRobot::instance()->robot_leg(LegType::FL)->joint_position_const_ref()).norm());
  printf( " FR ="); __print_color_helper(LegType::FR, fr);
  printf("%+8.04f|\n", (fr - LegRobot::instance()->robot_leg(LegType::FR)->joint_position_const_ref()).norm());

  printf("| HL -"); __print_color_helper(LegType::HL);     printf("    -   |");
  printf( " HR -"); __print_color_helper(LegType::HR);     printf("    -   |\n");

  printf("| HL ="); __print_color_helper(LegType::HL, hl);
  printf("%+8.04f|", (hl - LegRobot::instance()->robot_leg(LegType::HL)->joint_position_const_ref()).norm());
  printf( " HR ="); __print_color_helper(LegType::HR, hr);
  printf("%+8.04f|\n", (hr - LegRobot::instance()->robot_leg(LegType::HR)->joint_position_const_ref()).norm());
  printf("-------------------------------------------------------------------------------------\n");
}
///! print FPT(foot point) coordinate of the special leg.
void print_eef_pos(LegType l) {
  printf("____________________________________\n");
  printf("|LEG -|    X   |    Y    |    Z    |\n");
//printf("LEG -| +00.0000| +00.0000| +00.0000|\n");
  auto eef = LegRobot::instance()->robot_leg(l)->eef();
  printf("| %s -| %+8.04f| %+8.04f| %+8.04f|\n",
      LEGTYPE_TOSTRING(l), eef.x(), eef.y(), eef.z());
  printf("------------------------------------\n");
}
///! print the v.s. result and different between FPT or COG
void print_eef_pos(LegType l, const Eigen::Vector3d& teef) {
  printf("_______________________________________________\n");
  printf("|LEG  |    X    |    Y    |    Z    |  ERROR  |\n");
//printf("|LEG  | +00.0000| +00.0000| +00.0000|+00.0000 |\n");
  Eigen::Vector3d eef = LegRobot::instance()->robot_leg(l)->eef();
  printf("| %s -| %+8.04f| %+8.04f| %+8.04f|    -    |\n",
      LEGTYPE_TOSTRING(l), eef.x(), eef.y(), eef.z());
  printf("| %s -| %+8.04f| %+8.04f| %+8.04f|%+8.04f |\n",
      LEGTYPE_TOSTRING(l), teef.x(), teef.y(), teef.z(), (teef - eef).norm());
  printf("-----------------------------------------------\n");
}

///! print the PFT of the all of leg.
void print_eef_pos() {
  printf("________________________________________________________________________\n");
  printf("|LEG -|    X    |    Y    |    Z    |LEG  |    X    |    Y    |    Z    |\n");
//printf("|LEG -| +00.0000| +00.0000| +00.0000|LEG -| +00.0000| +00.0000| +00.0000|\n");
  Eigen::Vector3d eef(0.0, 0.0, 0.0);
  for (const auto& l : {LegType::FL, LegType::HL}) {
    LegRobot::instance()->robot_leg(l)->eef(eef);
    printf("| %s -| %+8.04f| %+8.04f| %+8.04f|",
        LEGTYPE_TOSTRING(l), eef.x(), eef.y(), eef.z());

    auto sl = LEGTYPE_SL(l);
    LegRobot::instance()->robot_leg(sl)->eef(eef);
    printf(" %s -| %+8.04f| %+8.04f| %+8.04f|\n",
        LEGTYPE_TOSTRING(sl), eef.x(), eef.y(), eef.z());
  }
  printf("------------------------------------------------------------------------\n");
}

///! print the v.s. result and different between FPT
void print_eef_pos(const Eigen::Vector3d& fl, const Eigen::Vector3d& fr,
    const Eigen::Vector3d& hl, const Eigen::Vector3d& hr) {
  printf("_____________________________________________________________________________________________\n");
  printf("|LEG  |    X    |    Y    |    Z    |  ERROR  |LEG  |    X    |    Y    |    Z    |  ERROR  |\n");
//printf("|LEG  | +00.0000| +00.0000| +00.0000|+00.0000 |LEG  | +00.0000| +00.0000| +00.0000|+00.0000 |\n");
  Eigen::Vector3d eef(0.0, 0.0, 0.0);
  Eigen::Vector3d teefs[LegType::N_LEGS];
  teefs[LegType::FL] = fl; teefs[LegType::FR] = fr;
  teefs[LegType::HL] = hl; teefs[LegType::HR] = hr;
  for (const auto& l : {LegType::FL, LegType::HL}) {
    auto sl = LEGTYPE_SL(l);
    LegRobot::instance()->robot_leg(l)->eef(eef);
    auto diff1 = (teefs[l] - eef).norm();

    printf("| %s -| %+8.04f| %+8.04f| %+8.04f|    -    |",
        LEGTYPE_TOSTRING(l), eef.x(), eef.y(), eef.z());

    LegRobot::instance()->robot_leg(sl)->eef(eef);
    auto diff2 = (teefs[sl] - eef).norm();

    printf(" %s -| %+8.04f| %+8.04f| %+8.04f|    -    |\n",
        LEGTYPE_TOSTRING(sl), eef.x(), eef.y(), eef.z());


    printf("| %s =| %+8.04f| %+8.04f| %+8.04f|%+8.04f |",
        LEGTYPE_TOSTRING(l), teefs[l].x(), teefs[l].y(), teefs[l].z(), diff1);
    LegRobot::instance()->robot_leg(sl)->eef(eef);
    printf(" %s =| %+8.04f| %+8.04f| %+8.04f|%+8.04f |\n",
        LEGTYPE_TOSTRING(sl), teefs[sl].x(), teefs[sl].y(), teefs[sl].z(), diff2);
  }
  printf("---------------------------------------------------------------------------------------------\n");
}

} /* namespace qr_control */
