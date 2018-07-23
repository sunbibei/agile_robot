/*
 * label.cpp
 *
 *  Created on: Aug 28, 2017
 *      Author: silence
 */

#include "foundation/label.h"

// Cancel the namespace middleware
// namespace middleware {

#define COMMA (".")

const std::string                      Label::null = "";
std::map<std::string, Label::LabelPtr> Label::s_label_table_;

Label::Label(const std::string& __l, const std::string& __p)
: label_(make_label(__p, __l)) {
  /*std::cout << "table's address: " << &s_label_table_ << std::endl;
  std::cout << "Insert? address: " << this << std::endl;
  label_table().insert(std::make_pair(label_, this));
  std::cout << "After insert, size: " << s_label_table_.size() << std::endl;*/
}

Label::Label(const std::string& _l, const Label& _obj)
: Label(_l, _obj.getLabel()) {
  ;
}

Label::Label(const std::string& _l, LabelPtr _obj)
: Label(_l, _obj->getLabel()) {
  ;
}

Label::Label(const std::string& _l, Label* _obj)
: Label(_l, _obj->getLabel()) {
  ;
}

Label::~Label() {
  // auto itr = label_table().find(label_);
  // if (label_table().end() != itr) label_table().erase(itr);
  // LOG_DEBUG << "The instance named\t" << label_ << "\thas destroyed.";
}

std::string Label::make_label(const std::string& p, const std::string& l) {
  return std::string(p + COMMA + l);
}

std::string Label::parent_label(const std::string& l) {
  size_t p = l.rfind(COMMA);
  if (std::string::npos == p)
    return null;
  else
    return l.substr(0, p);
}

void Label::split_label(std::string l, std::string& p, std::string& v) {
  size_t pos = l.rfind(COMMA);
  if (std::string::npos == pos) {
    p = null;
    v = l;
  } else {
    p = l.substr(0, pos);
    v = l.substr(pos+1, l.size());
  }
}

void Label::foreachHardware(std::function<void (LabelPtr)> cb) {
  for (auto& obj : s_label_table_) cb(obj.second);
}

bool Label::auto_init() { return true; }

// } /* namespace middleware */
