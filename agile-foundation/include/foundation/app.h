/*
 * app.h
 *
 *  Created on: Jul 17, 2018
 *      Author: bibei
 */

#ifndef INCLUDE_FOUNDATION_APP_H_
#define INCLUDE_FOUNDATION_APP_H_

/*!
 * @brief The main class(using in the main function) must be inherited from this
 *        class, This class offer some interfaces which called by the fixed order.
 *        THE ORDER:
 *            Constructor() -> init() -> run() -> DeConstructor()
 *        The ONLY public interface ( start() ) can be called using the object,
 *        the order is fixed by this method.
 */
class MiiApp {
public:
  /*!
   * @brief The ONLY public interface can be called.
   */
  virtual void start();

protected:
  /*!
   * @brief The creating stage, contains the Constructor(), init() and
   *        create_system_instance().
   */
  MiiApp();
  virtual bool init() = 0;
  virtual void create_system_instance() = 0;

  /*!
   * @brief The running stage, ONLY this method, returned immediately.
   */
  virtual bool run()/* = 0*/;

  /*!
   * @brief The destroying stage, contains ONLY Deconstructor()
   */
  virtual ~MiiApp();
};

#endif /* INCLUDE_FOUNDATION_APP_H_ */
