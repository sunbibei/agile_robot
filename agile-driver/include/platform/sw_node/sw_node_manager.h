/*
 * sw_node_manager.h
 *
 *  Created on: Sep 8, 2017
 *      Author: bibei
 */

#ifndef INCLUDE_SYSTEM_PLATFORM_SW_NODE_MANAGER_H_
#define INCLUDE_SYSTEM_PLATFORM_SW_NODE_MANAGER_H_

#include "foundation/internal/resource_manager.h"
#include "platform/sw_node/sw_node.h"

#include "foundation/utf.h"

namespace agile_robot {

class SWNodeManager: public internal::ResourceManager<SWNode> {
  SINGLETON_DECLARE(SWNodeManager)

public:
  /*!
   * @brief This method will initialize something.
   */
  bool init();

  void handleMsg(const std::vector<Packet>&);
  void generateCmd(std::vector<Packet>&);

  // FOR DEBUG
  void print();

protected:
  // store all of the hw_unit which order by id --> [bus_id][node_id]
  std::vector<std::vector<MiiPtr<SWNode>>> hw_list_by_id_;
  // store all of the hw_unit which require to send some command
  std::vector<MiiPtr<SWNode>>              hw_list_by_cmd_;
  // store all of the hw_unit which order by name
  // std::map<std::string, MiiPtr<SWNode>>    hw_list_by_name_;
};

} /* namespace middleware */

#endif /* INCLUDE_SYSTEM_PLATFORM_SW_NODE_MANAGER_H_ */
