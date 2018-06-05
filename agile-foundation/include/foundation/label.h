/*
 * label.h
 *
 *  Created on: Aug 28, 2017
 *      Author: silence
 */

#ifndef INCLUDE_SYSTEM_RESOURCES_LABEL_H_
#define INCLUDE_SYSTEM_RESOURCES_LABEL_H_

#include <map>
#include <string>
#include <boost/shared_ptr.hpp>

#include "foundation/utf.h"


// Cancel the namespace middleware
// namespace middleware {

class Label {
  friend class AutoInstanceor;
public:
  typedef boost::shared_ptr<Label> LabelPtr;
  const static std::string null;

  Label(const std::string& l, const std::string& p = Label::null);
  Label(const std::string& l, const Label& p);
  Label(const std::string& l, LabelPtr p);
  Label(const std::string& l, Label* p);

  virtual ~Label();

  const std::string& getLabel() { return label_; }
  const std::string& getLabel() const { return label_; }

  static std::string make_label  (const std::string& _p, const std::string& _l);
  static std::string parent_label(const std::string&);
  static void      split_label (std::string, std::string&, std::string&);

  template<class _Hardware>
  static _Hardware* getHardwareByName(const std::string&);

  // For Debug
  static void printfEveryInstance() {
    if (_DEBUG_INFO_FLAG) {
      LOG_WARNING << "+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+";
      LOG_WARNING << "Label's table, address " << &s_label_table_
          << ", size " << s_label_table_.size();
      LOG_WARNING << "-------------------------------------------------------------";
      LOG_WARNING << "COUNT\t\tLABEL\t\t\tREF\t\tADDR";
      int count = 0;
      for (auto l : s_label_table_) {
        LOG_INFO << count++ << "\t" << l.second->getLabel()
            << "\t\t" << l.second.use_count() << "\t" << l.second;
      }
      LOG_WARNING << "+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+";
    }
  }

protected:
  std::string label_;

protected:
  /**
   * @brief Initialization will be completed here.
   *        The default implementation is null, All of subclass
   *        judge whether or not it need to override.
   *        Note that you must be fill the label_ before call this method.
   */
  virtual bool auto_init();

  static std::map<std::string, LabelPtr>& label_table() { return s_label_table_; }

  /**
   * TODO
   * 不应该出现的函数，并且还使用了友元类的方式才解决了注册问题。
   * 妥协方案，下一步应该处理这个问题
   */
  static void registerClass(LabelPtr& l_sp) {
    label_table().insert(std::make_pair(l_sp->getLabel(), l_sp));
  }

private:
  static std::map<std::string, LabelPtr> s_label_table_;
};



///////////////////////////////////////////////////////////////////////////////
////////////        The implementation of template methods         ////////////
///////////////////////////////////////////////////////////////////////////////
template<class _Hardware>
_Hardware* Label::getHardwareByName(const std::string& l) {
  auto hw = label_table().find(l);
  if ((label_table().end() == hw)
      || (nullptr == boost::dynamic_pointer_cast<_Hardware>(hw->second))) {
    return nullptr;
  }

  return static_cast<_Hardware*>(hw->second.get());
}

// } /* namespace middleware */

#endif /* INCLUDE_SYSTEM_RESOURCES_LABEL_H_ */
