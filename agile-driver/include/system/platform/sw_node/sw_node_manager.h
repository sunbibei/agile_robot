/*
 * sw_node_manager.h
 *
 *  Created on: Sep 8, 2017
 *      Author: bibei
 */

#ifndef INCLUDE_SYSTEM_PLATFORM_SW_NODE_MANAGER_H_
#define INCLUDE_SYSTEM_PLATFORM_SW_NODE_MANAGER_H_

#include "foundation/internal/resource_manager.h"
#include "system/platform/sw_node/sw_node.h"

#include "foundation/utf.h"

namespace middleware {

class SWNodeManager: public internal::ResourceManager<SWNode> {
  SINGLETON_DECLARE(SWNodeManager)

public:
  bool init();
  void handleMsg(const MiiVector<Packet>&);
  void generateCmd(MiiVector<Packet>&);

protected:
  // store all of the hw_unit which order by id
  std::vector<SWNode*>             hw_list_by_id_;
  // store all of the hw_unit which require to send some command
  std::vector<SWNode*>             hw_list_by_cmd_;
  // store all of the hw_unit which order by name
  std::map<std::string, SWNode*>   hw_list_by_name_;
};

} /* namespace middleware */

#endif /* INCLUDE_SYSTEM_PLATFORM_SW_NODE_MANAGER_H_ */
