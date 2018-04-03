/*
 * gait_base.h
 *
 *  Created on: Nov 20, 2017
 *      Author: bibei
 */

#ifndef INCLUDE_GAIT_GAIT_BASE_H_
#define INCLUDE_GAIT_GAIT_BASE_H_

#include <foundation/label.h>

namespace agile_control {

class StateMachineBase {
public:
  StateMachineBase();
  virtual ~StateMachineBase();

public:
  ///! The callback method for each state.
  ///! e.g. StateMachine::CallBack cb = std::bind(&SomeClass::SomeMethod, this);
  typedef std::function<void()> CallBack;

  /*!
   * The abstract method, the sub-class is asked to complete.
   */
  virtual void operator()() = 0;
};


template <typename _State>
class StateMachine : public StateMachineBase {
  friend class GaitManager;
public:
  StateMachine(const _State& _state_ref);
  virtual ~StateMachine();

public:
  virtual void operator()() override;

  template<typename _Func, typename... _BoundArgs>
  void registerStateCallback(const _State& _s, _Func&& _f, _BoundArgs&&... _args);

protected:
  const _State&            current_state_;
  std::map<_State, CallBack> cb_map_;
};


///////////////////////////////////////////////////////////////////////////////
////////////              The define of GaitBase class           //////////////
///////////////////////////////////////////////////////////////////////////////
class GaitBase: public Label {
  friend class GaitManager;
public:
  GaitBase(const std::string& _l = "gait-base");
  ///! For auto-instance
  virtual bool auto_init() override;

  virtual ~GaitBase();

public:
  /*!
   * @brief The name of gait.
   */
  const std::string& name() const { return gait_name_;  }

  /**
   * @brief The interface that get the current state. This method
   *        will be implemented by sub-class.This method should be
   *        a abstract method, but this class is needs to auto-instance,
   *        so cancel '=0'.
   * @return Return the current state, it could returns INVALID_STATE.
   */
//   template<typename _State>
//   _State currentState() const /*=0*/;

///! These methods are asked to implement by sub-class.
protected:
  /*!
   * @brief This method will be called at every tick, it should change
   *        the current state. This method should be a abstract method,
   *        but this class is needs to auto-instance, so cancel '=0'.
   */
  virtual void checkState() = 0;

  /**
   * @brief Get the state machine. This method will be implemented
   *        by sub-class.This method should be a abstract method, but
   *        this class is needs to auto-instance, so cancel '=0'.
   */
  virtual StateMachineBase* state_machine() { return state_machine_; };

  /*!
   * @brief This method will be called when the gait is activated and
   *        ready to run. If return false, it don't be running, instead
   *        of nothing to run, until the user activate the next gait.
   */
  virtual bool starting() /*= 0*/;

  /*!
   * @brief This method will be called when the gait is stopping and switch
   *        to the other gait.
   */
  virtual void stopping() /*= 0*/;

  /*!
   * @brief The current state whether could switch to ohter gait object.
   */
  virtual bool canSwitch() /*= 0*/;

  /*!
   * @brief This method will be called before the callback of state.
   */
  virtual void prev_tick()  /*= 0*/;
  /*!
   * @brief This method will be called after the callback of state.
   */
  virtual void post_tick() /*= 0*/;

protected:
  /*!
   * @brief This method will be called every tick.
   */
  // void update();

protected:
  std::string           gait_name_;
  StateMachineBase*   state_machine_;
};



///////////////////////////////////////////////////////////////////////////////
////////////        The implementation of template methods         ////////////
///////////////////////////////////////////////////////////////////////////////
template <typename _State>
StateMachine<_State>::StateMachine(const _State& _state_ref)
  : StateMachineBase(), current_state_(_state_ref) {
  ;
}

template <typename _State>
StateMachine<_State>::~StateMachine() {
  ;
}

template<typename _State>
template<typename _Func, typename... _BoundArgs>
void StateMachine<_State>::registerStateCallback(const _State& _s, _Func&& _f, _BoundArgs&&... _args) {
  if (cb_map_.end() != cb_map_.find(_s)) {
    /*cb_map_.insert(std::make_pair(_s, std::bind(_f, _args)));
  } else {*/
    LOG_WARNING << "Replace the old callback method for state."; // (" << _s << ").";
  }

  cb_map_[_s] = std::bind(_f, _args...);
}

template<typename _State>
void StateMachine<_State>::operator()() {
  if (cb_map_.end() == cb_map_.find(current_state_)) {
    LOG_WARNING << "There is no registered callback in the callback table.";
    return;
  }

  cb_map_[current_state_]();
}

//template<typename _State>
//_State GaitBase::currentState() const {
//  LOG_ERROR << "Call the base method 'currentState'";
//  return _State();
//}


} /* namespace qr_control */

#endif /* INCLUDE_GAIT_GAIT_BASE_H_ */
