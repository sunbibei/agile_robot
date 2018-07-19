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

///! The alias for boost::shared_ptr
template <typename _T>
using MiiPtr = boost::shared_ptr<_T>;

class Label {
  friend class AutoInstor;
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
  static void        split_label (std::string, std::string&, std::string&);

  /*!
   * @brief Change the return value to boost::shared_ptr, it's safety for the
   *        operating pointer, we can using the boost::shared_ptr::use_count
   *        to track the information who using this pointer.
   *        MODIFY: 2018-07-19, 17:42:00
   */
  template<class _Hardware>
  static MiiPtr<_Hardware> getHardwareByName(const std::string&);

  // For Debug
  static void printfEveryInstance() {
    size_t N_max_label = 0;
    for (const auto& l : s_label_table_) {
      if (N_max_label < l.first.size())
        N_max_label = l.first.size();
    }

    char format[128] = {0};
//    printf("COUNT LABEL REF ADDR\n");
//    printf("%5d %Xs %3d %p\n");
    printf("\n");
    LOG_WARNING << "\nLabel's table, address " << &s_label_table_
        << ", size " << s_label_table_.size()
        << "\n-------------------------------------------------------------";
    sprintf(format, "COUNT %%-%lds  REF   ADDR\n", N_max_label);
    printf(format, "LABEL");

    memset(format, 0x00, 128);
    sprintf(format, "%%-5d %%-%lds %%3d  %%p\n", N_max_label);
    int count = 0;
    for (const auto& l : s_label_table_) {
      printf(format, count++, l.first.c_str(), l.second.use_count(), l.second);
    }
    LOG_WARNING;
    printf("\n");
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
MiiPtr<_Hardware> Label::getHardwareByName(const std::string& l) {
  if (label_table().end() == label_table().find(l)) return nullptr;

  return boost::dynamic_pointer_cast<_Hardware>(label_table()[l]);
}

// } /* namespace middleware */

#endif /* INCLUDE_SYSTEM_RESOURCES_LABEL_H_ */
