/*
 * data_service.h
 *
 *  Created on: Dec 26, 2017
 *      Author: bibei
 */

#ifndef INCLUDE_REPOSITORY_DATA_SERVICE_H_
#define INCLUDE_REPOSITORY_DATA_SERVICE_H_

#include "foundation/label.h"

#include <chrono>
#include <fstream>

namespace agile_robot {

class LogRobot: public Label {
public:
  LogRobot();
  virtual bool auto_init() override;

  virtual ~LogRobot();

public:
  virtual void start();
  virtual void tick();

protected:
  bool            enable_;
  std::string     path_;
  std::string     ofn_;
  std::ofstream   ofd_;
  char*           buffer_;

  bool                       tick_alive_;
  std::chrono::milliseconds  tick_duration_;

  class DataSources* sources_;

};

} /* namespace middleware */

#endif /* INCLUDE_REPOSITORY_DATA_SERVICE_H_ */
