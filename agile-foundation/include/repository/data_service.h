/*
 * data_service.h
 *
 *  Created on: Dec 26, 2017
 *      Author: bibei
 */

#ifndef INCLUDE_REPOSITORY_DATA_SERVICE_H_
#define INCLUDE_REPOSITORY_DATA_SERVICE_H_

#include <foundation/label.h>
#include <toolbox/time_control.h>

#include <fstream>

namespace middleware {

class DataService: public Label {
public:
  DataService(const std::string& _l = "data-service");
  virtual bool auto_init() override;

  virtual ~DataService();

public:
  virtual void start();
  virtual void tick();

protected:
  bool          enable_;
  std::string     path_;
  std::string     ofn_;
  std::ofstream ofd_;
  char*         buffer_;

  bool                       tick_alive_;
  std::chrono::milliseconds  tick_duration_;

  TimeControl*       timer_;
  class DataSources* sources_;

};

} /* namespace middleware */

#endif /* INCLUDE_REPOSITORY_DATA_SERVICE_H_ */
