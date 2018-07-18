/*
 * cfg_reader.h
 *
 *  Created on: Aug 28, 2017
 *      Author: silence
 */

#ifndef INCLUDE_FOUNDATION_CFG_READER_H_
#define INCLUDE_FOUNDATION_CFG_READER_H_

// #include <foundation/label.h>
// #include <foundation/utf.h>

#include "label.h"
#include "utf.h"

#include <vector>
#include <tinyxml.h>

// Cancel the namespace middleware
// namespace middleware {

class CfgReader final {
  SINGLETON_DECLARE(CfgReader)

public:

  typedef bool (*Callback)   (const std::string&, const std::string&);
  typedef std::function<bool (const std::string&, const std::string&)>  Callback1;
  /**
   * @brief The Callback function for specific @__attr
   * @param __p        The parent of tag which contains the specific attribute.
   * @param __attr_val The value of __attr under the __p tag.
   */
  void regAttrCb(const std::string& __attr, Callback,  const std::string& __prefix = "");
  void regAttrCb(const std::string& __attr, Callback1, const std::string& __prefix = "");
  /**
   * @brief The Callback function for specific @__tag
   * @param __p        The parent of tag which contains the specific attribute.
   * @param __attr_val The value of __attr under the __p tag.
   */
  // typedef void (*Callback1)(const std::string& __p, TiXmlElement*);
  // void regTagCb(const std::string& _tag, Callback1, const std::string& _prefix = "");

  /*!
   * @brief The iterator for the all attributes under a given tag @_p, FOR EXAMPLE:
   *        auto f = [](const std::string& attr, const std::string& val) {
   *            printf("key: %s -> value: %s\n", attr.c_str(), val.c_str());
   *            };
   *        CfgReader::instance()->foreachAttr("leg.res.fl", f);
   *        NOTE: The prototype of callback: void (*cb)(attr, value);
   */
  void foreachAttr(const std::string& _p,
      std::function<void(const std::string&, const std::string&)> _cb);

  /*!
   * @brief The iterator for the all sub-tag under a given tag @_p, FOR EXAMPLE:
   *        auto f = [](const std::string& tag) {
   *            printf("tag: %s\n", tag.c_str());
   *            };
   *        CfgReader::instance()->foreachTag("leg.res.fl", f);
   *        NOTE: The prototype of callback: void (*cb)(tag);
   */
  void foreachTag(const std::string&, std::function<void(const std::string&)>);
public:
  /*!
   * @brief Add a new path into ENV $path, not a file name but a path.
   */
  static void add_path(const std::string&);
  /*!
   * @brief Load and read a new configure file in the runtime.
   */
  bool add_config(const std::string&);

public:
  /**
   *  @brief  Find the value of @p.@attr in the configure file.
   *  @param p[in]     The parent tag of attr
   *  @param attr[in]  Attribute name to locate.
   *  @param val[out]  The value associate to @attr.
   *  @param def[in]   default value.
   *  @return  Return true if this attribute has found, or return false.
  */
  template<class _Type>
  bool get_value(const std::string& p, const std::string& attr, _Type& val, const _Type& def);
  template<class _Type>
  bool get_value(const std::string& p, const std::string& attr, _Type& val);
  template<class _Type>
  bool get_value(const std::string& p, const std::string& attr, std::vector<_Type>& vals);
  ////////////////// Template Specialization
  bool get_value(const std::string& p, const std::string& attr, std::string&);
  bool get_value(const std::string& p, const std::string& attr, std::vector<bool>&);
  bool get_value(const std::string& p, const std::string& attr, std::vector<char>&);
  bool get_value(const std::string& p, const std::string& attr, std::vector<unsigned char>&);
  bool get_value(const std::string& p, const std::string& attr, std::vector<JntType>&);
  bool get_value(const std::string& p, const std::string& attr, std::vector<LegType>&);
  bool get_value(const std::string& p, const std::string& attr, std::vector<JntCmdType>&);

  /**
   *  @brief  Find the value of @p.@attr in the configure file.
   *          If not exist, This methods will be throwed an fatal exception.
   *  @param p[in]     The parent of attr
   *  @param attr[in]  Attribute name to locate.
   *  @param val[out]  The value associate to @attr.
   *  @return  Return true if this attribute has found, or return false.
  */
  template<class _Type>
  void get_value_fatal(const std::string& p, const std::string& attr, _Type& val);
  template<class _Type>
  void get_value_fatal(const std::string& p, const std::string& attr, std::vector<_Type>& vals);

private:
  TiXmlDocument**      cfg_docs_;
  // TiXmlElement*        cfg_root_;

  size_t               n_config_;
  size_t               N_config_;

  std::vector<std::string>          opened_cfg_file_;
  static std::vector<std::string>   s_cfg_paths_;
};






///////////////////////////////////////////////////////////////////////////////
////////////        The implementation of template methods         ////////////
///////////////////////////////////////////////////////////////////////////////
// The helper method's forward declaration
bool __get_value_helper(TiXmlElement* __root, const std::string& p,
    const std::string& __attr, bool fatal, std::string& __pAttr);

template<class _Type>
bool CfgReader::get_value(const std::string& p, const std::string& attr, _Type& val, const _Type& def) {
  if (!get_value(p, attr, val))
    val = def;

  return true;
}

template<class _Type>
bool CfgReader::get_value(const std::string& p, const std::string& attr, _Type& val) {
  std::vector<_Type> __vals;
  if ((!get_value(p, attr, __vals)) || (__vals.empty())) return false;

  val = __vals.back();
  return true;
}

template<class _Type>
bool CfgReader::get_value(const std::string& p, const std::string& attr, std::vector<_Type>& vals) {
  std::string __pAttr = "";
  for (size_t i = 0; i < n_config_; ++i) {
    auto _cfg_root = cfg_docs_[i]->RootElement();
    if (__get_value_helper(_cfg_root, p, attr, false, __pAttr)) break;
  }
  if (__pAttr.empty()) return false;

  std::stringstream __ss;
  __ss << __pAttr;
  vals.clear();
  _Type tmp;
  while (__ss >> tmp) vals.push_back(tmp);
  return !vals.empty();
}

template<class _Type>
void CfgReader::get_value_fatal(const std::string& p, const std::string& attr, _Type& val) {
  std::vector<_Type> __vals;
  if ((!get_value(p, attr, __vals)) || (__vals.empty()))
    LOG_FATAL << "CfgReader can't found the configure '" << Label::make_label(p, attr) << "' in the configure file.";

  val = __vals.back();
}

template<class _Type>
void CfgReader::get_value_fatal(const std::string& p, const std::string& attr, std::vector<_Type>& vals) {
  if (!get_value(p, attr, vals))
    LOG_FATAL << "CfgReader can't found the confgiure '" << Label::make_label(p, attr) << "' in the configure file.";
}

//} /* namespace middleware */

#endif /* INCLUDE_FOUNDATION_CFG_READER_H_ */
