/*
 * xacroparser.cpp
 *
 *  Created on: Dec 1, 2017
 *      Author: bibei
 */

#include "foundation/internal/expr.h"
#include "foundation/internal/mii_config.h"

#include <boost/algorithm/string.hpp>
#include <fstream>

namespace internal {

SINGLETON_IMPL(MiiConfig)

#define ESCAPE          ('$')
#define START_CHAR      ('{')
#define END_CHAR        ('}')
#define PROPERTY        ("macro:property")
#define PROTOTYPE       ("macro:prototype")
#define PATH            ("macro:path")
#define INCLUDE         ("macro:include")

const MiiString PROTO_PREFIX = "prototype:";

struct ProtoInfo {
  const TiXmlElement*          proto;
  MiiMap<MiiString, MiiString> attr_val_map;
};

MiiMap<MiiString, MiiString>  MiiConfig::s_property_map_;
MiiMap<MiiString, ProtoInfo*> MiiConfig::s_prototype_map_;

MiiVector<MiiString>          MiiConfig::s_cfg_paths_;

void __reset_xml_doc(TiXmlDocument** _xml) {
  if (_xml && *_xml)
    delete *_xml;

  *_xml = new TiXmlDocument;
  (*_xml)->LinkEndChild(new TiXmlDeclaration("1.0","UTF-8",""));
  /*auto root = new TiXmlElement("cfg");
  root->SetAttribute("description", "This is auto");
  (*_xml)->LinkEndChild(root);*/
}


void __findTag(TiXmlElement* __curr, const MiiString& __p,
    const MiiString& _tag, std::function<void(TiXmlElement*)>& cb) {
  for (auto __next = __curr->FirstChildElement();
      nullptr != __next; __next = __next->NextSiblingElement()) {
#ifdef LINK_CFG_READER
    MiiString __next_p = Label::make_label(__p, __next->Value());
#else
    MiiString __next_p = __p + "." + __next->Value();
#endif
    if (0 == _tag.compare(__next->Value()))
      cb(__next);
    __findTag(__next, __next_p, _tag, cb);
  }
}

size_t __recurse_escape(const MiiMap<MiiString, MiiString>& map,
    const MiiString& _raw, size_t pos, MiiString& _val) {
  MiiString _v;
  MiiString _name;
  size_t sta = pos;
  size_t off = pos;
  bool start = false;
  for (; off < _raw.size(); ++off) {
    switch(_raw[off]) {
    case ESCAPE:
      if (START_CHAR != _raw[off+1]) {
        _val = _raw.substr(pos);
        return _raw.size();
      }

      if (!start) {
        sta   = off;
        start = true;
        break;
      }
      _v = "";
      off = __recurse_escape(map, _raw, off, _v);
      _name += _v;
      break;
    case END_CHAR:
      if (map.end() == map.find(_name))
        _val += _raw.substr(sta, off+1);
      else
        _val += map.find(_name)->second;
      return off;
    default:
      if (start) _name.push_back(_raw[off]);
      else _val.push_back(_raw[off]);
    }
  }

  return off;
}

bool __parseMath(const MiiString& _raw, const MiiMap<MiiString, MiiString>& map,
    double& _out) {
  MiiVector<MiiString> sp;
  boost::split(sp, _raw, boost::is_any_of("+-*/()"));
  MiiString _r = _raw;
  for (const auto& _v : sp) {
    const auto& _m = map.find(_v);
    if (map.end() != _m) {
      auto pos = _r.find(_m->first);
      while (MiiString::npos != pos) {
        _r.replace(pos, _m->first.size(), _m->second);
        pos = _r.find(_m->first, pos + _m->second.size());
      }
    }
  }

  return eval(_r, _out);
}

bool __parseProperty(const MiiString& _raw, MiiString& _val,
    const MiiMap<MiiString, MiiString>& map, bool math = false) {
  size_t p0    = 0;
  size_t p1    = 0;
  _val         = "";
  MiiString _r = _raw;
  while (p1 < _r.size()) {
    _r = _r.substr(p1);

    p0 = _r.find(ESCAPE);
    if ((MiiString::npos == p0) || (p0 + 2) >= _r.size()) {
      _val += _r;
      return true;
    }
    if (START_CHAR != _r[p0 + 1])
      return false;

    p0 += 2;
    p1 = _r.find(END_CHAR, p0);
    if (MiiString::npos == p1) {
      LOG_ERROR << "The useage of macro: ${...}";
      return false;
    }
    auto _n = _r.substr(p0, p1 - p0);
    double _math_v = 0;
    MiiString new_val = "";
    if (math && __parseMath(_n, map, _math_v)) {
      new_val = std::to_string(_math_v);
    } else {
      if (map.end() != map.find(_n)) new_val = map.find(_n)->second;
      else return false;
    }
    _val += _r.substr(0, p0 - 2);
    _val += new_val;
    ++p1; // eat the END_CHAR
  }

  return true;
}

void __copy_element(TiXmlElement* _to, const TiXmlElement* _from,
    const MiiMap<MiiString, MiiString>& _map) {
  MiiString tmp;
  for (auto _attr = _from->FirstAttribute();
      nullptr != _attr; _attr = _attr->Next()) {
    if (!__parseProperty(_attr->ValueStr(), tmp, _map))
      tmp = _attr->ValueStr();
    _to->SetAttribute(_attr->Name(), tmp);
  }

  for (auto _tag = _from->FirstChildElement();
      nullptr != _tag; _tag = _tag->NextSiblingElement()) {
    TiXmlElement child(_tag->ValueStr());
    __copy_element(&child, _tag, _map);
    _to->InsertEndChild(child);
  }
}

void __copy_attribute(TiXmlElement* _to, const TiXmlElement* _from) {
  for (auto _attr = _from->FirstAttribute();
      nullptr != _attr; _attr = _attr->Next()) {
    _to->SetAttribute(_attr->Name(), _attr->ValueStr());
  }
}

bool __parsePrototype(TiXmlElement* _curr, const MiiMap<MiiString, ProtoInfo*>& _map) {
  if (!_curr || !_curr->Attribute("tag")) {
    LOG_ERROR << "The prototype'" << _curr->ValueStr() << "' no 'tag' attribute!";
    return false;
  }
  ///! Get the name of prototype
  MiiString proto_name = _curr->ValueStr().substr(PROTO_PREFIX.size());
  if (_map.end() == _map.find(proto_name)) {
    LOG_ERROR << "No such prototype:'" << proto_name << "' define before here.";
    return false;
  }

  ///! Create the property Map for the prototype
  auto _p = _map.find(proto_name)->second;
  auto map = _p->attr_val_map;
  for (auto& m : map) {
    if (!_curr->Attribute(m.first)) {
      LOG_WARNING << "The arg is incomplete, Could not found the arg: " << m.first;
      m.second = "${" + m.first + "}"; // keep the property.
      continue;
    }

    m.second = _curr->Attribute(m.first.c_str());
    _curr->RemoveAttribute(m.first);
  }

  /*for (const auto& m : map) {
    LOG_INFO << m.first << " -> " << m.second;
  }*/
  ///! Update the tag name of prototype, and remove the 'tag' attribute.
  _curr->SetValue(_curr->Attribute("tag"));
  _curr->RemoveAttribute("tag");
  ///! Replace the prototype
  __copy_element(_curr, _p->proto, map);
  return true;
}

void __recurse_parse(TiXmlElement* __curr,
    const MiiMap<MiiString, MiiString>&  _property_map,
    const MiiMap<MiiString, ProtoInfo*>& _prototype_map) {
  if (!__curr || MiiString::npos != __curr->ValueStr().find("macro:")) return;
  ///! first parse the prototype
  if (MiiString::npos != __curr->ValueStr().find(PROTO_PREFIX)) {
    if (!__parsePrototype(__curr, _prototype_map)) {
      LOG_ERROR << "The prototype'" << __curr->ValueStr() << "' has error!";
      return; // Don't need to parse the error prototype.
    }
  }

  ///! parse the proproperty of the current tag.
  for (auto _attr = __curr->FirstAttribute();
      nullptr != _attr; _attr = _attr->Next()) {
    MiiString val;
    if (__parseProperty(_attr->ValueStr(), val, _property_map, true)) {
      _attr->SetValue(val);
    }
  }

  ///! parse the each child element of the current tag.
  for (auto __next = __curr->FirstChildElement();
      nullptr != __next; __next = __next->NextSiblingElement())
    __recurse_parse(__next, _property_map, _prototype_map);
}

void __remove_macro(TiXmlElement* __root) {
  auto __next = __root->FirstChildElement();
  while (nullptr != __next) {
    if (MiiString::npos != __next->ValueStr().find("macro:")) {
      __root->RemoveChild(__next);
      __next = __root->FirstChildElement();
    } else {
      __remove_macro(__next);
      __next = __next->NextSiblingElement();
    }
  }
}

bool __get_full_fn(const MiiVector<MiiString>& _ps, const MiiString& _f, MiiString& _fn) {
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

void MiiConfig::regTagCb(TiXmlDocument* _doc, const MiiString& _tag,
    std::function<void(TiXmlElement*)>& cb) {
  if (!_doc || !_doc->RootElement()) return;

  auto cfg_root = _doc->RootElement();
  if (0 == _tag.compare(cfg_root->Value()))
    cb(cfg_root);
  __findTag(cfg_root, cfg_root->Value(), _tag, cb);
}

void MiiConfig::init_path(TiXmlDocument*, TiXmlElement* root) {
  if (!root) return;

  auto pv = root->Attribute("path");
  if (!pv) return;

  add_path(pv);
}

void MiiConfig::init_include(TiXmlDocument* _doc, TiXmlElement* root) {
  if (!root) return;

  auto pv = root->Attribute("file");
  if (!pv) return;

  MiiString _f_pv;
  if (!__get_full_fn(s_cfg_paths_, pv, _f_pv)) {
    LOG_WARNING << "No such file named '" << pv << "'";
    return;
  }

  TiXmlDocument* _xml = new TiXmlDocument;
  if (!_xml->LoadFile(_f_pv)) {
    LOG_WARNING << "Load the file named '" << _f_pv << "' has failed!";
    return;
  }

  parse_helper(_xml, _doc);
  delete _xml;
}

void MiiConfig::init_property(TiXmlDocument*,  TiXmlElement* root) {
  if (!root) return;

  auto pn = root->Attribute("name");
  auto pv = root->Attribute("value");
  if (!pn || !pv || (s_property_map_.end() != s_property_map_.find(pn))) return;

  if (s_property_map_.end() == s_property_map_.find(pn))
    s_property_map_[pn] = pv; //.insert(std::make_pair(MiiString(pn), MiiString(pv)));

}

void MiiConfig::init_protoType(TiXmlDocument*, TiXmlElement* root) {
  if (!root->Attribute("name") || !root->FirstChild("tag"))
    LOG_FATAL << "The prototype'" << root->ValueStr() << "' has name attribute!";

  ProtoInfo* info = new ProtoInfo;
  info->proto = root->FirstChildElement("tag");
  s_prototype_map_[root->Attribute("name")] = info;

  if (nullptr == root->Attribute("args")) return;
  MiiString args = root->Attribute("args");
  std::stringstream ss;
  ss << args;
  MiiString arg;
  while (ss >> arg)
    info->attr_val_map[arg] = "arg";
}

void MiiConfig::parse(const MiiString& _fn, TiXmlDocument** _out_xml,
      const MiiString& _out_fn, bool print) {
  MiiString _f_pv;
  if (!__get_full_fn(s_cfg_paths_, _fn, _f_pv)) {
    LOG_WARNING << "No such file named '" << _fn << "'";
    return;
  }

  TiXmlDocument* _xml = new TiXmlDocument;
  if (!_xml->LoadFile(_f_pv)) {
    LOG_WARNING << "Load the file named '" << _f_pv << "' has failed!";
    return;
  }

  if (!root_file_) __reset_xml_doc(&root_file_);
  /*auto _r = new TiXmlElement(_xml->RootElement()->ValueStr());
  __copy_attribute(_r, _xml->RootElement());
  root_file_->LinkEndChild(_r);*/

  parse_helper(_xml, root_file_);
  delete _xml;

  if (!_out_fn.empty())
    root_file_->SaveFile(_out_fn);
  if (print)
    root_file_->Print();
  if (_out_xml) {
    if (*_out_xml)
      delete *_out_xml;

    *_out_xml  = root_file_; // swap the pointer.
    root_file_ = nullptr;
  }

  __reset_xml_doc(&root_file_);
}

void MiiConfig::parse_helper(TiXmlDocument* doc, TiXmlDocument* root,
    const MiiString& _out, bool print) {

  std::function<void(TiXmlElement*)>
  cb = std::bind(&MiiConfig::init_path,      this, doc, std::placeholders::_1);
  regTagCb(doc, PATH,      cb);
  cb = std::bind(&MiiConfig::init_include,   this, doc, std::placeholders::_1);
  regTagCb(doc, INCLUDE,   cb);
  cb = std::bind(&MiiConfig::init_property,  this, doc, std::placeholders::_1);
  regTagCb(doc, PROPERTY,  cb);
  cb = std::bind(&MiiConfig::init_protoType, this, doc, std::placeholders::_1);
  regTagCb(doc, PROTOTYPE, cb);

  __recurse_parse(doc->RootElement(), s_property_map_, s_prototype_map_);
  __remove_macro (doc->RootElement());

  if (root->RootElement())
    root->RootElement()->LinkEndChild(doc->RootElement()->Clone());
  else
    root->LinkEndChild(doc->RootElement()->Clone());

  if (!_out.empty())
    root->SaveFile(_out);
  if (print)
    root->Print();
}

void MiiConfig::add_path(const MiiString& _p) {
  for (const auto& p : s_cfg_paths_)
    if (0 == p.compare(_p)) return;

  s_cfg_paths_.push_back(_p);
}

MiiConfig::MiiConfig() : root_file_(nullptr) {
  add_path(".");
};

MiiConfig::~MiiConfig() {
  if (root_file_) delete root_file_;

  for (auto& _m : s_prototype_map_) {
    if (_m.second) {
      delete _m.second;
      _m.second = nullptr;
    }
  }
};

} /* namespace internal */
