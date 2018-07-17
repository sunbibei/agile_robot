/*
 * app.h
 *
 *  Created on: Jul 17, 2018
 *      Author: bibei
 */

#ifndef INCLUDE_FOUNDATION_APP_H_
#define INCLUDE_FOUNDATION_APP_H_

#include "foundation/cfg_reader.h"

#include <string>
/*!
 * @brief The main class(using in the main function) must be inherited from this
 *        class, This class offer some interfaces which called by the fixed order.
 *        THE ORDER:
 *            Constructor() -> init() -> run() -> DeConstructor()
 *        The ONLY public interface ( start() ) can be called using the object,
 *        the order is fixed by this method, for example:
 *          MiiApp* app = SomeApp::create_instance();
 *          app->start();
 *          // waiting shutdown by user
 *          ... ...
 *          SomeApp->destroy_instance();
 */
class MiiApp {
public:
  /*!
   * @brief The ONLY public interface can be called.
   */
  virtual void start();

protected:
////////////////////////////////////////////////////////////////////
////////////             THE CREATING STAGE             ////////////
////////////////////////////////////////////////////////////////////
  /*!
   * @brief STEP 1:
   *        The Constructor().
   */
  MiiApp();
  /*!
   * @brief STEP 2:
   *        Create the all of singleton in our system, this method will be
   *        called before the @init() after the @prev_init(),
   *        If something was wrong, return NOTHING, SHUTDOWN the process directly.
   */
  virtual void create_system_singleton()/* = 0*/;
  /*!
   * @brief STEP 3:
   *        Create the all of instance in the configure file using the given
   *        Callback method or the default Callback @auto_inst_cb(), If you
   *        want to move the default callback to a self-define callback, calling
   *        the @mv_auto_inst_cb()
   */
  virtual void auto_inst()/* = 0*/;
  static  bool auto_inst_cb(const std::string&, const std::string&);
  ///! The interface of change the callback.
  static  void mv_auto_inst_cb(CfgReader::Callback1&);
  /*!
   * @brief STEP 4:
   *        This function will be called lastly after the @init(), it must
   *        be completed the process of AutoInst and register the all of
   *        thread what our system need.
   */
  virtual bool init() = 0;

////////////////////////////////////////////////////////////////////
////////////              THE RUNNING STAGE             ////////////
////////////////////////////////////////////////////////////////////
  /*!
   * @brief The ONLY method at the running stage, returned immediately.
   *        Add a default implemented in this function, called the
   *        @ThreadPool::start() to launch the all of registered thread.
   */
  virtual bool run()/* = 0*/;

////////////////////////////////////////////////////////////////////
////////////            THE DESTROYING STAGE            ////////////
////////////////////////////////////////////////////////////////////
  /*!
   * @brief The destroying stage, contains ONLY @Deconstructor()
   */
  virtual ~MiiApp();

private:
  static CfgReader::Callback1 s_auto_inst_cb_;
};

#endif /* INCLUDE_FOUNDATION_APP_H_ */
