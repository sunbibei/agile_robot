/*
 * joint_manager.cpp
 *
 *  Created on: Sep 1, 2017
 *      Author: silence
 */

#include "repository/resource/joint_manager.h"

namespace middleware {

SINGLETON_IMPL(JointManager)

JointManager::JointManager()
  : ResourceManager<Joint>(),
    jnt_mode_(JntCmdType::CMD_POS) {
  jnt_list_by_type_.resize(LegType::N_LEGS);
  for (auto& leg : jnt_list_by_type_) {
    leg.resize(JntType::N_JNTS);
    for (auto& jnt : leg)
      jnt = nullptr;
  }
}

JointManager::~JointManager() {
  // Nothing to do here
  for (auto& j : res_list_)
    j->stop();
}

void JointManager::add(Joint* _res) {
  ResourceManager<Joint>::add(_res);

  jnt_list_by_name_.insert(std::make_pair(_res->joint_name(), _res));

  if ((_res->leg_type() < 0) || (_res->joint_type() < 0)) {
    LOG_WARNING << "The joint '" << _res->getLabel()
        << "' must be configured inaccurately. LegType: "
        << _res->leg_type() << ", JntType: " << _res->joint_type();
    return;
  }
  jnt_list_by_type_[_res->leg_type()][_res->joint_type()] = _res;
  // LOG_DEBUG << "JointManager has received a joint -- " << _res->getLabel();
}

JointManager::iterator JointManager::find(const MiiString& _n) {
  for (JointManager::iterator itr = JointManager::res_list_.begin();
      itr != JointManager::res_list_.end(); ++itr) {

    if (0 == _n.compare((*itr)->joint_name())) return itr;
  }

  return end();
}

void JointManager::addJointCommand(LegType owner, JntType type, double val) {
  if (nullptr != jnt_list_by_type_[owner][type])
    jnt_list_by_type_[owner][type]->updateJointCommand(val);
}

void JointManager::addJointCommand(const MiiString& name, double val) {
  if (jnt_list_by_name_.end() != jnt_list_by_name_.find(name))
    jnt_list_by_name_[name]->updateJointCommand(val);
}

Joint* JointManager::getJointHandle(LegType owner, JntType type) {
  return jnt_list_by_type_[owner][type];
}

Joint* JointManager::getJointHandle(const MiiString& _jn) {
  auto itr = jnt_list_by_name_.find(_jn);
  if (jnt_list_by_name_.end() == itr)
    return nullptr;

  return itr->second;
}

void JointManager::setJointCommandMode(JntCmdType type) {
  jnt_mode_ = type;
}

const JntCmdType& JointManager::getJointCommandMode() {
  return jnt_mode_;
}

double JointManager::operator()(LegType owner, JntType type, JntDataType data) {
  switch (data) {
  case JntDataType::POS: return jnt_list_by_type_[owner][type]->joint_position();
  case JntDataType::VEL: return jnt_list_by_type_[owner][type]->joint_velocity();
  case JntDataType::TOR: return jnt_list_by_type_[owner][type]->joint_torque();
  default: return 0;
  }
}

void JointManager::joint_position_const_pointer(LegType _owner, JntType _type, const double* & _c_p) {
  _c_p = jnt_list_by_type_[_owner][_type]->joint_position_const_pointer();
}
void JointManager::joint_velocity_const_pointer(LegType _owner, JntType _type, const double* & _c_p) {
  _c_p = jnt_list_by_type_[_owner][_type]->joint_velocity_const_pointer();
}
void JointManager::joint_torque_const_pointer  (LegType _owner, JntType _type, const double* & _c_p) {
  _c_p = jnt_list_by_type_[_owner][_type]->joint_torque_const_pointer();
}
// override
void JointManager::joint_position_const_pointer(const MiiString& _n, const double* & _c_p) {
  _c_p = jnt_list_by_name_[_n]->joint_position_const_pointer();
}
void JointManager::joint_velocity_const_pointer(const MiiString& _n, const double* & _c_p) {
  _c_p = jnt_list_by_name_[_n]->joint_velocity_const_pointer();
}
void JointManager::joint_torque_const_pointer(const MiiString& _n, const double* & _c_p) {
  _c_p = jnt_list_by_name_[_n]->joint_torque_const_pointer();
}

void JointManager::joint_names(MiiVector<MiiString>& names) {
  names.clear();
  for (auto jnt : res_list_) {
    names.push_back(jnt->joint_name());
  }
}
// override
void JointManager::joint_position_const_pointer(MiiVector<const double*>& _c_ps) {
  _c_ps.clear();
  for (auto jnt : res_list_) {
    _c_ps.push_back(jnt->joint_position_const_pointer());
  }
}
void JointManager::joint_velocity_const_pointer(MiiVector<const double*>& _c_ps) {
  _c_ps.clear();
  for (auto jnt : res_list_) {
    _c_ps.push_back(jnt->joint_velocity_const_pointer());
  }
}
void JointManager::joint_torque_const_pointer(MiiVector<const double*>& _c_ps) {
  _c_ps.clear();
  for (auto jnt : res_list_) {
    _c_ps.push_back(jnt->joint_torque_const_pointer());
  }
}

/*JointManager::iterator JointManager::find(const MiiString& _n) {
  for (auto itr = res_list_.begin(); itr != res_list_.end(); ++itr) {
    if (itr->joint_name() == _n) return itr;
  }
  return end();
}*/

} /* namespace middleware */
