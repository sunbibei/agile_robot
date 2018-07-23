/*
 * auto_instanceor.cpp
 *
 *  Created on: Aug 31, 2017
 *      Author: bibei
 */

#include "foundation/auto_instor.h"
#include "foundation/label.h"

#include <fstream>
// Cancel the namespace middleware
// namespace middleware {
void __updateTypeMap(class_loader::ClassLoader* _new_loader, int idx, std::map<std::string, int>& _type_map) {
  auto new_class_list = _new_loader->getAvailableClasses<Label>();
  for (const auto& c : new_class_list)
    _type_map[c] = idx;
}

bool __full_fn(const std::vector<std::string>& _ps, const std::string& _f, std::string& _fn) {
  std::ifstream _ifd;
  _fn = _f;
  _ifd.open(_fn);
  if (_ifd.is_open()) {
    _ifd.close();
    return true;
  }

  for (const auto& _p : _ps) {
    _fn = _p + "/" + _f;
    _ifd.open(_fn);
    if (_ifd.is_open()) {
      _ifd.close();
      return true;
    }
    _ifd.close();
  }
  _fn = "";
  return false;
}

// std::map<std::string, Label::LabelPtr> AutoInstor::s_inst_table_;
// SINGLETON_IMPL_NO_CREATE(AutoInstor)
SINGLETON_IMPL(AutoInstor)

std::vector<std::string> AutoInstor::s_cfg_paths_;
//AutoInstor* AutoInstor::create_instance(const std::string& lib) {
//  if (nullptr != s_inst_) {
//    LOG_WARNING << "Create the AutoInstor instance twice!";
//  } else
//    s_inst_ = new AutoInstor(lib);
//  return s_inst_;
//}

AutoInstor::AutoInstor(/*const std::string& lib_path*/)
  : class_loader_(nullptr), n_library_(0), N_loader_(4) {
  // class_loader_ = new class_loader::MultiLibraryClassLoader(true);
  // class_loader_->loadLibrary(lib_path);
  class_loader_             = new class_loader::ClassLoader*[N_loader_];
  for (size_t i = 0; i < N_loader_; ++i) class_loader_[i] = nullptr;

  // add_library(lib_path);
  // printAvailableClass();
}

void AutoInstor::add_path(const std::string& _p) {
  for (const auto& p : s_cfg_paths_)
    if (0 == p.compare(_p)) return;

  s_cfg_paths_.push_back(_p);
}

bool AutoInstor::add_library(const std::string& _l) {
  if (n_library_ == N_loader_) {
    N_loader_ *= 2;
    auto tmp = new class_loader::ClassLoader*[N_loader_];
    for (size_t i = 0; i < N_loader_;  ++i) tmp[i] = nullptr;
    for (size_t i = 0; i < n_library_; ++i) tmp[i] = class_loader_[i];

    delete[] class_loader_;
    class_loader_ = tmp;
  }

  std::string _fn;
  if (!__full_fn(s_cfg_paths_, _l, _fn)) {
    LOG_ERROR << "Could not found the " << _l
        << ", did you forget define the file?";
    return false;
  }

  class_loader_[n_library_] = new class_loader::ClassLoader(_fn);
  __updateTypeMap(class_loader_[n_library_], n_library_, type_map_);
  ++n_library_;
  return true;
}

// Just for debug
void AutoInstor::printAvailableClass() {
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

AutoInstor::~AutoInstor() {
  // s_inst_table_.clear();
  for (auto& inst : Label::s_label_table_) {
    inst.second.reset();
  }

  Label::s_label_table_.clear();
  // The s_inst_table_ will be automatic dealloc.
  if (nullptr != class_loader_) {
    for (size_t  i = 0; i < N_loader_; ++i)
      if (nullptr != class_loader_[i]) delete class_loader_[i];

    delete[] class_loader_;
    class_loader_ = nullptr;
  }
}

bool AutoInstor::make_instance(const std::string& __p, const std::string& __type) {
  if (type_map_.end() == type_map_.find(__type)) return false;

  int idx = type_map_[__type];
  Label::LabelPtr __inst = class_loader_[idx]->createInstance<Label>(__type);
  if (nullptr != __inst.get()) {
    // 情非得已， 不应该出现这样的代码。
    // 但当前自动实例使用class_loader的方式，仅能如此妥协处理。
    // 庆幸的是，所有问题都还控制在AutoInstance中
    __inst->label_ = __p;
    // s_inst_table_.insert(std::make_pair(__inst->getLabel(), __inst));
    Label::registerClass(__inst);

    __inst->auto_init();
    return true;
  } else {
    LOG_WARNING << "What FUNK! The '" << Label::make_label(__p, __type)
        << " instances fail.";
    return false;
  }
}

//} /* namespace middleware */
