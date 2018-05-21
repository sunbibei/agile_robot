/*
 * ipc.h
 *
 *  Created on: Apr 16, 2018
 *      Author: bibei
 */

#ifndef INCLUDE_FOUNDATION_IPC_IPC_H_
#define INCLUDE_FOUNDATION_IPC_IPC_H_

#include <map>
#include <string>

class MiiIPC {
protected:
  MiiIPC(const std::string& _n);
  virtual ~MiiIPC();

///! The interfaces definition.
public:
  /*!
   * @brief This method initialize the everything for this channel, include create the channel.
   * @param args  Cause we don't known what parameter is necessary, so we use
   *              the void* act as the parameter, analogous to the thread function.
   */
  virtual bool init (const void* args) = 0;
  /*!
   * @brief read the data to @_to, it maybe a structure with @_len bytes.
   */
  virtual bool read (void* _to,         const size_t _len) = 0;
  /*!
   * @brief write the data from @_from, it maybe a structure with @_len bytes
   */
  virtual bool write(const void* _from, const size_t _len) = 0;

protected:
  std::string ipc_name_;

////////////////////////////////////////////////////////////////////////////////
////////////  The manager of IPCs
////////////////////////////////////////////////////////////////////////////////
public:
  static MiiIPC* ipc_channel(const std::string& _n);

protected:
  static std::map<const std::string&, MiiIPC*> s_ipcs_;
};


#endif /* INCLUDE_FOUNDATION_IPC_IPC_H_ */
