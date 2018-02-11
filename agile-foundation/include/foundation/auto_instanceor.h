/*
 * auto_instanceor.h
 *
 *  Created on: Aug 31, 2017
 *      Author: bibei
 */

#ifndef INCLUDE_SYSTEM_UTILS_AUTO_INSTANCEOR_H_
#define INCLUDE_SYSTEM_UTILS_AUTO_INSTANCEOR_H_

#include <map>
#include <boost/shared_ptr.hpp>
#include <class_loader/class_loader.h>

#include "utf.h"

// Cancel the namespace middleware
// namespace middleware {

class AutoInstanceor {
  SINGLETON_DECLARE(AutoInstanceor, const MiiString&)

public:
  /*template<class _Base>
  bool make_instance(MiiStringConstRef, _Base);*/

  /**
   * @brief Create an object about specific __type
   * @param __p    The tag which contains the type
   * @param __type The type of object to create
   */
  bool make_instance(const MiiString& __p, const MiiString& __type);
  /*!
   * @brief Add a library into the AutoInstanceor.
   */
  bool add_library(const MiiString& _l);

  // Just for debug.
  void printAvailableClass();

protected:
  // static MiiMap<MiiString, Label::LabelPtr> s_inst_table_;
  class_loader::ClassLoader** class_loader_;
  size_t                      n_library_;
  size_t                      N_loader_;
  MiiMap<MiiString, int>      type_map_;
};

//} /* namespace middleware */

#endif /* INCLUDE_SYSTEM_UTILS_AUTO_INSTANCEOR_H_ */
