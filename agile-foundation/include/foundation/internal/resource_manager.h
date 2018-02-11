/*
 * manager.h
 *
 *  Created on: Aug 26, 2017
 *      Author: silence
 */

#ifndef INCLUDE_SYSTEM_ROBOT_RESOURCE_MANAGER_H_
#define INCLUDE_SYSTEM_ROBOT_RESOURCE_MANAGER_H_

#include "foundation/utf.h"

///! cancel the namespace middleware
// namespace middleware {
namespace internal {


const unsigned int __RESERVE_SIZE = 16;

template<class _Resource>
class ResourceManager {
public:
  typedef typename MiiVector<_Resource*>::iterator  iterator;
  typedef typename MiiVector<_Resource*>::reference reference;
  /**
   * @brief Return the number of the registered resource.
   */
  unsigned int   size()  { return res_list_.size(); };
  reference operator[](int i) { return res_list_[i]; };
  bool empty() { return res_list_.empty(); };
  /**
   * @brief These methods offer the ability of the range-based loop for classes.
   */
  iterator begin() { return res_list_.begin(); };
  iterator end()   { return res_list_.end();   };

  virtual void add(_Resource* _res) {
    res_list_.push_back(_res);
  }

  virtual void remove(_Resource* _res) {
    for (auto iter = res_list_.begin(); iter != res_list_.end();) {
      if (_res == *iter) iter = res_list_.erase(iter);
      else  ++iter;
    }
  }

protected:
  ResourceManager() : res_list_(MiiVector<_Resource*>()) {
    res_list_.reserve(__RESERVE_SIZE);
  };
  virtual ~ResourceManager() { }

  // store all of the resource
  MiiVector<_Resource*> res_list_;
};

} /* namespace internal */
// } /* namespace middleware */

#endif /* INCLUDE_SYSTEM_ROBOT_RESOURCE_MANAGER_H_ */
