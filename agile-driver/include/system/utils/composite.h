/*
 * component.h
 *
 *  Created on: Jan 11, 2017
 *      Author: silence
 */

#ifndef INCLUDE_SYSTEM_UTILS_COMPOSITE_H_
#define INCLUDE_SYSTEM_UTILS_COMPOSITE_H_

#include <map>
#include <string>
#include <boost/shared_ptr.hpp>
#include <system/utils/utf.h>

namespace agile_robot {
/**
 * 完成组合模式的基类.
 * 组合模式以名称为标识符
 */
template <typename T>
class Composite : public T {
public:
  // TODO
  typedef typename std::map<std::string, boost::shared_ptr<T>> map_type;
  typedef typename map_type::iterator iterator;
  /**************************************************
   * 下述两个函数完成component的新增与删除
   * 以名称为标识符
   **************************************************/
  void add(const std::string& name, T* component) {
    boost::shared_ptr<T> ptr(component);
    add(name, ptr);
  }

  void add(const std::string& name, boost::shared_ptr<T> component) {
    auto itr = composite_map_.find(name);
    if (composite_map_.end() == itr) {
      // LOG_INFO << "Addition component( " << name << " )";
      composite_map_.insert(std::make_pair(name, component));
    } else {
      LOG_WARNING << "Replace component( " << name << " )";
      itr->second.swap(component);
    }
  }

  void remove(T* component) {
    for (auto itr : composite_map_) {
      if (itr->second.get() == component) {
        // LOG_INFO << "Remove component( " << itr->first << " )";
        composite_map_.erase(itr);
        return;
      }
    }
    LOG_WARNING << "Can't found the handle by addr: (" << component << " )";
  }

  void remove(boost::shared_ptr<T> component) {
    remove(component.get());
  }

  void remove(const std::string& name) {
    auto itr = composite_map_.find(name);
    if (composite_map_.end() != itr) {
      composite_map_.erase(itr);
    }
    LOG_INFO << "Remove component( " << name << " )";
  }

  /**************************************************
   * 查询是否包含某个名称的零件, 并返回迭代器, 不存在, 则返回end()
   * 并定义begin(), end()及一些常用操作
   **************************************************/
  iterator find(const std::string& name) { return composite_map_.find(name); }
  iterator begin() { return composite_map_.begin(); }
  iterator end()   { return composite_map_.end(); }
  boost::shared_ptr<T>& operator[](const std::string& key) { return composite_map_[key]; }

  void clear()  { composite_map_.clear(); };
  size_t size() { return composite_map_.size(); };
  bool empty()  { return composite_map_.empty(); };

  std::string label_;
protected:
  // std::string composite_name_;
  std::map<std::string, boost::shared_ptr<T>> composite_map_;
};

} /* namespace middleware */

#endif /* INCLUDE_SYSTEM_UTILS_COMPOSITE_H_ */
