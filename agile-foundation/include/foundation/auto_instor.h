/*
 * auto_instor.h
 *
 *  Created on: Aug 31, 2017
 *      Author: bibei
 */

#ifndef INCLUDE_FOUNDATION_AUTO_INSTOR_H_
#define INCLUDE_FOUNDATION_AUTO_INSTOR_H_

#include <map>
#include <boost/shared_ptr.hpp>
#include <class_loader/class_loader.hpp>
// #include <class_loader/class_loader.h>

#include "utf.h"

// Cancel the namespace middleware
// namespace middleware {

class AutoInstor final {
  // SINGLETON_DECLARE(AutoInstor, const std::string&)
  SINGLETON_DECLARE(AutoInstor)

public:
  /*!
   * @brief Add a new path into ENV $path, not a file name but a path.
   */
  static void add_path(const std::string&);

  /**
   * @brief Create an object about specific __type
   * @param __p    The tag which contains the type
   * @param __type The type of object to create
   */
  bool make_instance(const std::string& __p, const std::string& __type);
  /*!
   * @brief Add a library into the AutoInstor.
   */
  bool add_library(const std::string& _l);

  // Just for debug.
  void printAvailableClass();

protected:
  // static MiiMap<MiiString, Label::LabelPtr> s_inst_table_;
  class_loader::ClassLoader** class_loader_;
  size_t                      n_library_;
  size_t                      N_loader_;
  std::map<std::string, int>  type_map_;

  static std::vector<std::string>   s_cfg_paths_;
};

//} /* namespace middleware */

#endif /* INCLUDE_FOUNDATION_AUTO_INSTOR_H_ */
