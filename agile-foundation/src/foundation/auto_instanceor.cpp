/*
 * auto_instanceor.cpp
 *
 *  Created on: Aug 31, 2017
 *      Author: bibei
 */

#include "foundation/auto_instanceor.h"
#include "foundation/label.h"
// Cancel the namespace middleware
// namespace middleware {

// std::map<std::string, Label::LabelPtr> AutoInstanceor::s_inst_table_;
SINGLETON_IMPL_NO_CREATE(AutoInstanceor)

void __updateTypeMap(class_loader::ClassLoader* _new_loader, int idx, std::map<std::string, int>& _type_map) {
  auto new_class_list = _new_loader->getAvailableClasses<Label>();
  for (const auto& c : new_class_list)
    _type_map[c] = idx;
}

AutoInstanceor* AutoInstanceor::create_instance(const std::string& lib) {
  if (nullptr != instance_) {
    LOG_WARNING << "Create the AutoInstanceor instance twice!";
    // std::cout << "Create the AutoInstanceor instance twice!" << std::endl;
  } else
    instance_ = new AutoInstanceor(lib);
  return instance_;
}

AutoInstanceor::AutoInstanceor(const std::string& lib_path)
  : class_loader_(nullptr), n_library_(0), N_loader_(4) {
  // class_loader_ = new class_loader::MultiLibraryClassLoader(true);
  // class_loader_->loadLibrary(lib_path);
  class_loader_             = new class_loader::ClassLoader*[N_loader_];
  for (size_t i = 0; i < N_loader_; ++i) class_loader_[i] = nullptr;

  class_loader_[n_library_] = new class_loader::ClassLoader(lib_path);
  __updateTypeMap(class_loader_[n_library_], n_library_, type_map_);
  ++n_library_;
  // printAvailableClass();
}

bool AutoInstanceor::add_library(const std::string& _l) {
  if (n_library_ == N_loader_) {
    N_loader_ *= 2;
    auto tmp = new class_loader::ClassLoader*[N_loader_];
    for (size_t i = 0; i < N_loader_;  ++i) tmp[i] = nullptr;
    for (size_t i = 0; i < n_library_; ++i) tmp[i] = class_loader_[i];

    delete[] class_loader_;
    class_loader_ = tmp;
  }

  class_loader_[n_library_] = new class_loader::ClassLoader(_l);
  __updateTypeMap(class_loader_[n_library_], n_library_, type_map_);
  ++n_library_;
  return true;
}

// Just for debug
void AutoInstanceor::printAvailableClass() {
  if (_DEBUG_INFO_FLAG) {
    LOG_WARNING << "+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-";
    for (size_t i = 0; i < n_library_; ++i) {
      auto class_list = class_loader_[i]->getAvailableClasses<Label>();
      LOG_INFO << "Available Classes[" << i << "]: ";
      int count = 0;
      for (const auto& c : class_list)
        LOG_INFO << "    " << count++ << ")  " << c;
      LOG_WARNING << "----------------------------------------";
    }
    LOG_WARNING << "+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-";
  }
}

AutoInstanceor::~AutoInstanceor() {
  // s_inst_table_.clear();
  Label::s_label_table_.clear();
  // The s_inst_table_ will be automatic dealloc.
  if (nullptr != class_loader_) {
    for (size_t  i = 0; i < N_loader_; ++i)
      if (nullptr != class_loader_[i]) delete class_loader_[i];

    delete[] class_loader_;
    class_loader_ = nullptr;
  }
}

bool AutoInstanceor::make_instance(const std::string& __p, const std::string& __type) {
  if (type_map_.end() == type_map_.find(__type)) return false;

  int idx = type_map_[__type];
  Label::LabelPtr __inst = class_loader_[idx]->createInstance<Label>(__type);
  if (nullptr != __inst.get()) {
    // 情非得已， 不应该出现这样的代码。
    // 但当前自动实例使用class_loader的方式，仅能如此妥协处理。
    // 庆幸的是，所有问题都还控制在AutoInstance中
    __inst->label_ = __p;
    __inst->auto_init();

    // s_inst_table_.insert(std::make_pair(__inst->getLabel(), __inst));
    Label::registerClass(__inst);
    return true;
  } else {
    LOG_WARNING << "What FUNK! The '" << Label::make_label(__p, __type)
        << " instances fail.";
    return false;
  }
}

//} /* namespace middleware */
