/*
 * registry.cpp
 *
 *  Created on: Nov 29, 2017
 *      Author: bibei
 */

#include "repository/registry.h"

#include <iostream>

namespace agile_robot {

SINGLETON_IMPL(Registry)


Registry::Registry() {
  ;
}

Registry::~Registry() {
  for (auto& cmd : cmd_origin_) {
    delete cmd.second.flag;
  }
}

bool Registry::registerResource(const std::string& _n, ResType _handle) {
  if (res_origin_.end() != res_origin_.find(_n)) {
    LOG_WARNING << "The named resource '" << _n << "' has registered in the resource table."
        << ", now it will be replaced.";
  }

  res_origin_[_n] = _handle;
  return true;
}

bool Registry::registerCommand(const std::string& _n, CmdType _handle, std::atomic_bool** flag) {
  if (cmd_origin_.end() != cmd_origin_.find(_n)) {
    LOG_WARNING << "The named command '" << _n << "' has registered in the command table."
        << ", now it will be replaced.";
  }

  CmdStruct  str;
  str.handle      = _handle;
  str.flag        = new std::atomic_bool;
  if (flag) *flag = str.flag;

  str.flag->store(false);
  cmd_origin_[_n] = str;
  return true;
}

inline std::string getTypeName(const ResType& t) {
  if (typeid(const short*) == t.type()) {
    return "short   ";
  } else if (typeid(const int*) == t.type()) {
    return "int     ";
  } else if (typeid(const double*) == t.type()) {
    return "double  ";
  } else if (typeid(const Eigen::VectorXi*) == t.type()) {
    return "vectorXi";
  } else if (typeid(const Eigen::MatrixXi*) == t.type()) {
    return "matrixXi";
  } else if (typeid(const Eigen::VectorXd*) == t.type()) {
    return "vectorXd";
  } else if (typeid(const Eigen::MatrixXd*) == t.type()) {
    return "matrixXd";
  } else {
    return t.type().name();
  }
}

inline std::string getTypeName(const CmdType& t) {
  if (typeid(const short*) == t.type()) {
    return "short   ";
  } else if (typeid(int*) == t.type()) {
    return "int     ";
  } else if (typeid(double*) == t.type()) {
    return "double  ";
  } else if (typeid(Eigen::VectorXi*) == t.type()) {
    return "vectorXi";
  } else if (typeid(Eigen::MatrixXi*) == t.type()) {
    return "matrixXi";
  } else if (typeid(Eigen::VectorXd*) == t.type()) {
    return "vectorXd";
  } else if (typeid(Eigen::MatrixXd*) == t.type()) {
    return "matrixXd";
  } else {
    return t.type().name();
  }
}

///! print the all of registry.
void Registry::print() {
  if (_DEBUG_INFO_FLAG) {
    LOG_WARNING << "+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+";
    LOG_WARNING << "The size of resource Registry: " << res_origin_.size();
    LOG_WARNING << "-------------------------------------------------------------";
    LOG_WARNING << "COUNT\tNAME\t\tTYPE\t\tADDR";
    int count = 0;
    for (const auto& l : res_origin_) {
      LOG_INFO << count++ << "\t" << l.first << "\t" << getTypeName(l.second)
          << "  " << l.second;
    }
    LOG_WARNING << "-------------------------------------------------------------";
    LOG_WARNING << "The size of command Registry: " << cmd_origin_.size();
    LOG_WARNING << "-------------------------------------------------------------";
    LOG_WARNING << "COUNT\tNAME\t\tTYPE\t\tADDR";
    count = 0;
    for (const auto& l : cmd_origin_) {
      LOG_INFO << count++ << "\t" << l.first << "\t" << getTypeName(l.second.handle)
          << "  " << l.second.handle;
    }
    LOG_WARNING << "+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+";
  }
  /*if (true) {
    std::cout << "+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+" << std::endl;
    std::cout << "The size of resource Registry: " << res_origin_.size() << std::endl;
    std::cout << "-------------------------------------------------------------" << std::endl;
    std::cout << "COUNT\tNAME\t\tTYPE\t\tADDR" << std::endl;
    int count = 0;
    for (const auto& l : res_origin_) {
      std::cout << count++ << "\t" << l.first << "\t" << getTypeName(l.second)
          << "  " << l.second << std::endl;
    }
    std::cout << "-------------------------------------------------------------" << std::endl;
    std::cout << "The size of command Registry: " << cmd_origin_.size() << std::endl;
    std::cout << "-------------------------------------------------------------" << std::endl;
    std::cout << "COUNT\tNAME\t\tTYPE\t\tADDR" << std::endl;
    count = 0;
    for (const auto& l : cmd_origin_) {
      std::cout << count++ << "\t" << l.first << "\t" << getTypeName(l.second)
          << "  " << l.second << std::endl;
    }
    std::cout << "+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+" << std::endl;
  }*/
}

} /* namespace middleware */
