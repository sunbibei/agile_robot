/*
 * mii_shm.h
 *
 *  Created on: Feb 1, 2018
 *      Author: bibei
 */

#ifndef SHARED_MEM_H_
#define SHARED_MEM_H_

#include "../utf.h"
#include <sys/shm.h>

class SharedMem {
  SINGLETON_DECLARE(SharedMem)

public:
  /*!
   * @brief Create the shared memory with the given name.
   */
  bool create_shm(const std::string&, size_t);
  /*!
   * @brief Get the data.
   */
  template<typename _T>
  bool read_from_shm(const std::string& _n, _T&);
  /*!
   * @brief Get the data.
   */
  template<typename _T>
  bool write_to_shm(const std::string& _n, const _T&);
  /*!
   * @brief Get the shared memory, If no such named shared memory, return nullptr.
   *        NOTE, the pointer which returned from this method, the user need to detach
   *        from the current process.
   */
  void* get_addr_from_shm(const std::string&);

  ///! Just for debug
  void printAllSharedMem();

  ///! tool
  static void clear();

private:
  std::map<std::string, key_t>                   key_map_;
  std::map<std::string, void*>                   addr_list_;
};

///////////////////////////////////////////////////////////////////////////////
////////////        The implementation of template methods         ////////////
///////////////////////////////////////////////////////////////////////////////
/*!
 * @brief Get the data.
 */
template<typename _T>
bool SharedMem::read_from_shm(const std::string& _n, _T& _data) {
  auto mem = get_addr_from_shm(_n);
  if (nullptr == mem) return false;

  memcpy(&_data, mem, sizeof(_T));
  return true;
}


/*!
 * @brief Get the data.
 */
template<typename _T>
bool SharedMem::write_to_shm(const std::string& _n, const _T& _data) {
  auto mem = get_addr_from_shm(_n);
  if (nullptr == mem) return false;

  memcpy(mem, &_data, sizeof(_T));
  return true;
}

#endif /* SHARED_MEM_H_ */
