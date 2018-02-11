/*
 * xacroparser.h
 *
 *  Created on: Dec 1, 2017
 *      Author: bibei
 */

#ifndef INTERNAL_MII_CONFIG_H_
#define INTERNAL_MII_CONFIG_H_

// #define LINK_CFG_READER

#include "../utf.h"
#include <functional>
#include <tinyxml.h>

namespace internal {

#define MACRO_CREATE          internal::MiiConfig::create_instance();
#define MACRO_DESTROY         internal::MiiConfig::destroy_instance();
#define MACRO_INST            internal::MiiConfig::instance()
#define MACRO_PARSE(...)      internal::MiiConfig::instance()->parse(__VA_ARGS__);

class MiiConfig {
  SINGLETON_DECLARE(MiiConfig)

public:
  void parse(const MiiString& _fn, TiXmlDocument** _out_xml = nullptr,
      const MiiString& _out_fn = "", bool print = false);

  static void add_path(const MiiString& _p);

protected:
  void parse_helper(TiXmlDocument*, TiXmlDocument* root, const MiiString& _out = "", bool print = false);

  void init_path     (TiXmlDocument*, TiXmlElement*);
  void init_include  (TiXmlDocument*, TiXmlElement*);
  void init_property (TiXmlDocument*, TiXmlElement*);
  void init_protoType(TiXmlDocument*, TiXmlElement*);

  /*!
   * @brief The callback is type as void(const MiiString&, void*), The first
   *        parameter is the label with given tag( or attribute), The second
   *        parameter is the TiXmlElement*( or char* for the attribute content).
   */
  void regTagCb(TiXmlDocument*, const MiiString& _tag,
      std::function<void(TiXmlElement*)>&);

protected:
  TiXmlDocument*                             root_file_;
  static MiiMap<MiiString, MiiString>        s_property_map_;
  static MiiMap<MiiString, class ProtoInfo*> s_prototype_map_;

  static MiiVector<MiiString>                s_cfg_paths_;
};

} /* namespace internal */

#endif /* INTERNAL_MII_CONFIG_H_ */
