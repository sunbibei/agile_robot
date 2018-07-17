/*
 * cfg_reader.cpp
 *
 *  Created on: Aug 28, 2017
 *      Author: silence
 */

#include "foundation/cfg_reader.h"
#include "foundation/internal/mii_config.h"

#include <fstream>
#include <dirent.h>
#include <boost/algorithm/string.hpp>

// Cancel the namespace middleware
// namespace middleware {

#define SUFFIX        ("cfg")
#define KEYWORD_INC   ("include")

bool __recurse_get_value(TiXmlElement* __ele, std::vector<std::string> __path,
                      const std::string& attr, std::string& __val) {
  std::string tag = __path.back();
  __path.pop_back();

  for (TiXmlElement* __next_ele = __ele->FirstChildElement(tag);
      nullptr != __next_ele; __next_ele = __next_ele->NextSiblingElement(tag)) {
    if (__path.empty()) {
      if (nullptr != __next_ele->Attribute(attr.c_str())) {
        __val = __next_ele->Attribute(attr.c_str());
        return true;
      }
    } else {
      if (__recurse_get_value(__next_ele, __path, attr, __val)) {
        return true;
      }
    }
  }

  return false;
}

bool __get_value_helper(TiXmlElement* __root, const std::string& p,
    const std::string& __attr, bool fatal, std::string& __pAttr) {
  std::vector<std::string> __ps;
  std::string __p = p; // Label::make_label("cfg", p);
  while (Label::null != __p) {
    std::string __l;
    Label::split_label(__p, __p, __l);
    if (Label::null != __l) __ps.push_back(__l);
  }
  if ((__ps.empty()) || (0 != __ps.back().compare(__root->Value()))) return false;

  __ps.pop_back();
  if (__ps.empty()) {
    if (nullptr == __root->Attribute(__attr.c_str()))
      return false;

    __pAttr = __root->Attribute(__attr.c_str());
    return true;
  }

  // __ps.pop_back();
  if (!__recurse_get_value(__root, __ps, __attr, __pAttr)) {
    if (fatal) LOG_FATAL << "Can't found the parameter '" << Label::make_label(p, __attr) << "'";
    else return false;
  }
  return true;
}

bool __file_list(const std::string& _root, std::map<std::string, std::string>& _map) {
  DIR* dir = opendir(_root.c_str());
  if (!dir) return false;

  struct dirent* ptr;
  std::string tmp_str;
  while ((ptr = readdir(dir)) != NULL) {
    if( (0 == strcmp(ptr->d_name,"."))
        || (0 == strcmp(ptr->d_name,"..")) )
        continue; // Skip the .. and .
    switch (ptr->d_type) {
    case DT_REG:
    case DT_LNK:
      tmp_str = ptr->d_name;
      if (std::string::npos != tmp_str.find(SUFFIX))
        _map[ptr->d_name] = _root;

      break;
    case DT_DIR:
      __file_list(_root + "/" + ptr->d_name, _map); break;
    default: break;
    }
  }
  closedir(dir);
  return true;
}

bool __get_full_fn(const std::vector<std::string>& _ps, const std::string& _f, std::string& _fn) {
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

bool __parse_root_file(const std::string& _f, const std::vector<std::string>& _ps, std::vector<std::string>& fs) {
  std::string tmp;
  if (!__get_full_fn(_ps, _f, tmp)) return false;

  std::ifstream ifd(tmp);
  while (ifd >> tmp) {
    if (0 == tmp.compare(KEYWORD_INC)) {
      ifd >> tmp;
      boost::algorithm::erase_all(tmp, "\"");
      boost::algorithm::erase_all(tmp, "<");
      boost::algorithm::erase_all(tmp, ">");
      fs.push_back(tmp);
    } else {
      ; // No more key words
    }
  }

  ifd.close();
  return true;
}


void __findAttr(TiXmlElement* __curr, const std::string& __p,
    const std::string& attr, CfgReader::Callback cb) {
  for (auto __next = __curr->FirstChildElement();
      nullptr != __next; __next = __next->NextSiblingElement()) {
    std::string __next_p = Label::make_label(__p, __next->Value());
    if (nullptr != __next->Attribute(attr.c_str()))
      cb(__next_p, __next->Attribute(attr.c_str()));

    __findAttr(__next, __next_p, attr, cb);
  }
}

void __findAttr(TiXmlElement* __curr, const std::string& __p,
    const std::string& attr, CfgReader::Callback1 cb) {
  for (auto __next = __curr->FirstChildElement();
      nullptr != __next; __next = __next->NextSiblingElement()) {
    std::string __next_p = Label::make_label(__p, __next->Value());
    if (nullptr != __next->Attribute(attr.c_str()))
      cb(__next_p, __next->Attribute(attr.c_str()));

    __findAttr(__next, __next_p, attr, cb);
  }
}

/*void __findTag(TiXmlElement* __curr, const std::string& __p,
    const std::string& _tag, CfgReader::Callback1 cb) {
  for (auto __next = __curr->FirstChildElement();
      nullptr != __next; __next = __next->NextSiblingElement()) {
    std::string __next_p = Label::make_label(__p, __next->Value());
    if (0 == _tag.compare(__next->Value()))
      cb(__next_p, __next);
    __findTag(__next, __next_p, _tag, cb);
  }
}*/

TiXmlElement* __findTag(TiXmlElement* __prefix, const std::string& tag) {
  if (nullptr == __prefix) return nullptr;

  for (auto __tag = __prefix->FirstChildElement();
      nullptr != __tag; __tag = __tag->NextSiblingElement())
    return ((0 == tag.compare(__tag->Value())) ? __tag : __findTag(__tag, tag));

  return nullptr;
}

TiXmlElement* __findLabel(TiXmlElement* __root, std::string __label) {
  if (__label.empty()) return nullptr;

  std::string __l;
  Label::split_label(__label, __label, __l);
  if (0 != __l.compare(__root->Value())) return nullptr;

  while (Label::null != __label) {
    Label::split_label(__label, __label, __l);
    __root = __root->FirstChildElement(__l);
    if (!__root) return nullptr;
  }

  return __root;
}

SINGLETON_IMPL(CfgReader)

//CfgReader* CfgReader::create_instance(const std::string& file) {
//  if (nullptr == s_inst_) {
//    add_path(".");
//    s_inst_ = new CfgReader(file);
//  } else {
//    LOG_WARNING << "This method 'CfgReader::create_instance' is called twice!";
//  }
//
//  return s_inst_;
//}

CfgReader::CfgReader(/*const std::string& file*/)
: cfg_docs_(nullptr), /*cfg_root_(nullptr),*/
  n_config_(0), N_config_(8) {

  cfg_docs_ = new TiXmlDocument*[N_config_];
  for (size_t i = 0; i < N_config_; ++i) {
    cfg_docs_[i] = nullptr;
  }

  MACRO_CREATE
  // add_config(file);
}

CfgReader::~CfgReader() {
  if (nullptr != cfg_docs_) {
    for (size_t i = 0; i < N_config_; ++i)
      if (nullptr != cfg_docs_[i]) delete cfg_docs_[i];
  delete[] cfg_docs_;
  cfg_docs_ = nullptr;
  }

  MACRO_DESTROY
}

std::vector<std::string> CfgReader::s_cfg_paths_;

void CfgReader::add_path(const std::string& _p) {
  for (const auto& p : s_cfg_paths_)
    if (0 == p.compare(_p)) return;

  s_cfg_paths_.push_back(_p);
}

bool CfgReader::add_config(const std::string& _f) {
  std::string _fn;
  if (!__get_full_fn(s_cfg_paths_, _f, _fn)) {
    LOG_ERROR << "Could not found the " << _f
            << ", did you forget define the file?";
    return false;
  }
  for (const auto& _f : opened_cfg_file_)
    if (0 == _f.compare(_fn)) return true;

  if (n_config_ == N_config_) {
    N_config_ *= 2;
    auto tmp = new TiXmlDocument*[N_config_];
    for (size_t i = 0; i < N_config_;  ++i) tmp[i] = nullptr;
    for (size_t i = 0; i < n_config_; ++i)  tmp[i] = cfg_docs_[i];

    delete[] cfg_docs_;
    cfg_docs_ = tmp;
  }

  /*auto xml             = new TiXmlDocument;
  if (!xml->LoadFile(_fn)) {
    LOG_ERROR << "Open the " << _fn << " fail.";
    delete xml;
    xml = nullptr;
    return false;
  }*/

  // cfg_docs_[n_config_] = new TiXmlDocument;
  MACRO_PARSE(_fn, cfg_docs_ + n_config_, "/home/bibei/Workspaces/qr_ws/src/qr-driver-0.2.9/config/all.cfg");

  if (!cfg_docs_[n_config_]) return false;

  opened_cfg_file_.push_back(_fn);
  ++n_config_;
  return true;
}

void CfgReader::regAttrCb(const std::string& attr, Callback cb,
    const std::string& prefix) {
  for (size_t i = 0; i < n_config_; ++i) {
    auto cfg_root = cfg_docs_[i]->RootElement();
    if (!prefix.empty())
      cfg_root = __findLabel(cfg_root, prefix);

    if (nullptr == cfg_root) continue;

    if (nullptr != cfg_root->Attribute(attr.c_str()))
      cb(Label::null, cfg_root->Attribute(attr.c_str()));

    LOG_ERROR << cfg_root->Value();
    __findAttr(cfg_root, cfg_root->Value(), attr, cb);
  }
}

void CfgReader::regAttrCb(const std::string& attr, Callback1 cb,
      const std::string& prefix) {
  for (size_t i = 0; i < n_config_; ++i) {
    auto cfg_root = cfg_docs_[i]->RootElement();
    if (!prefix.empty())
      cfg_root = __findLabel(cfg_root, prefix);

    if (nullptr == cfg_root) continue;

    if (nullptr != cfg_root->Attribute(attr.c_str()))
      cb(Label::null, cfg_root->Attribute(attr.c_str()));

    LOG_ERROR << cfg_root->Value();
    __findAttr(cfg_root, cfg_root->Value(), attr, cb);
  }
}

/*void CfgReader::regTagCb(const std::string& _tag, Callback1 cb, const std::string& _prefix) {
  for (size_t i = 0; i < n_config_; ++i) {
    auto cfg_root = cfg_docs_[i]->RootElement();
    if (!_prefix.empty()) {
      cfg_root = __findLabel(cfg_root, _prefix);
    }
    if (nullptr == cfg_root) continue;
    if (0 == _tag.compare(cfg_root->Value()))
      cb(Label::null, cfg_root);
    __findTag(cfg_root, cfg_root->Value(), _tag, cb);
  }
}*/

bool CfgReader::get_value(const std::string& p, const std::string& attr, std::string& val) {
  size_t i = 0;
  for (; i < n_config_; ++i) {
    auto _cfg_root = cfg_docs_[i]->RootElement();
    if (__get_value_helper(_cfg_root, p, attr, false, val)) break;
  }
  return (i != n_config_);
}

bool CfgReader::get_value(const std::string& p, const std::string& attr,
    std::vector<bool>& vals) {

  std::vector<std::string> vec_str;
  if (!get_value(p, attr, vec_str)) return false;

  vals.clear();
  for (auto& tmp : vec_str) {
    boost::to_lower(tmp);
    if (0 == tmp.compare("false"))
      vals.push_back(false);
    else if (0 == tmp.compare("true"))
      vals.push_back(true);
    else
      LOG_WARNING << "What a fucking configure? '" << tmp << "'";
  }
  return !vals.empty();
}

bool CfgReader::get_value(const std::string& p, const std::string& attr,
    std::vector<char>& vals) {

  std::vector<std::string> vals_str;
  if (!get_value(p, attr, vals_str)) return false;

  std::string template_str;
  for (const auto& str : vals_str) {
    if (('0' == str[0]) && ('x' == str[1])) {
      // Hex to id
      template_str = "0x%x";
    } else {
      template_str = "%d";
    }
    unsigned int val_i;
    sscanf(str.c_str(), template_str.c_str(), &val_i);
    vals.push_back((char)val_i);
  }

  return true;
}

bool CfgReader::get_value(const std::string& p, const std::string& attr, std::vector<unsigned char>& vals) {
  std::vector<char> vals_char;
  if (!get_value(p, attr, vals_char) || vals_char.empty()) return false;

  for (auto c : vals_char)
    vals.push_back(c);

  return true;
}

bool CfgReader::get_value(const std::string& p, const std::string& attr, std::vector<JntType>& vals) {
  std::vector<std::string> vals_str;
  if (!get_value(p, attr, vals_str)) return false;

  for (auto str : vals_str) {
    boost::to_upper(str);
    // boost::to_lower(str);
    JntType type = JntType::UNKNOWN_JNT;
    if (0 == str.compare(JNTTYPE2STR(JntType::HAA))) {
      type = JntType::HAA;
    } else if (0 == str.compare(JNTTYPE2STR(JntType::HFE))) {
      type = JntType::HFE;
    } else if (0 == str.compare(JNTTYPE2STR(JntType::KFE))) {
      type = JntType::KFE;
    } else {
      LOG_WARNING << "Error the 'jnt' TAG(" << str << ") in the 'joint' TAG, "
          << "require 'haa', 'hfe' or 'kfe'";
    }

    vals.push_back(type);
  }

  return true;
}

bool CfgReader::get_value(const std::string& p, const std::string& attr, std::vector<LegType>& vals) {
  std::vector<std::string> vals_str;
  if (!get_value(p, attr, vals_str)) return false;

  for (auto str : vals_str) {
    // boost::to_lower(str);
    boost::to_upper(str);
    LegType type = LegType::UNKNOWN_LEG;
    FOREACH_LEG(l) {
      if (0 == str.compare(LEGTYPE2STR(l))) {
        type = l;
        break;
      }
    }
    if (LegType::UNKNOWN_LEG == type) {
      LOG_WARNING << "Error the 'leg' TAG(" << str << ") in the 'joint' TAG, "
          << "require 'hl', 'fr', 'hl' or 'hr'";
    }

    vals.push_back(type);
  }

  return true;
}

bool CfgReader::get_value(const std::string& p, const std::string& attr, std::vector<JntCmdType>& vals) {
  std::vector<std::string> vals_str;
  if (!get_value(p, attr, vals_str)) return false;

  for (auto str : vals_str) {
    // boost::to_lower(str);
    boost::to_upper(str);
    JntCmdType type = JntCmdType::UNKNOWN_CMD_TYPE;
    for (const auto& jct : {JntCmdType::CMD_POS, JntCmdType::CMD_TOR, JntCmdType::CMD_VEL,
            JntCmdType::CMD_MOTOR_VEL, JntCmdType::CMD_POS_VEL}) {
      if (0 == str.compare(JNTCMDTYPE2STR(jct))) {
        type = jct;
        break;
      }
    }
    if (JntCmdType::UNKNOWN_CMD_TYPE == type) {
      LOG_WARNING << "Error the 'mode' TAG(" << str << ") in the TAG, "
          << "require 'POS', 'VEL', 'TOR', 'POS-VEL' or 'MOTOR-VEL'";
    }

    vals.push_back(type);
  }

  return true;
}

//} /* namespace middleware */
