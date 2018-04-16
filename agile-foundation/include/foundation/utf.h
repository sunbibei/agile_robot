/*
 * utf.h
 * Uniform Type define File defines all of the typedef or macro.
 *
 *  Created on: Jan 16, 2017
 *      Author: silence
 */

#ifndef INCLUDE_SYSTEM_UTILS_UTF_H_
#define INCLUDE_SYSTEM_UTILS_UTF_H_

#define USING_GLOG
#ifdef  USING_GLOG
#include <glog/logging.h>
#include <glog/log_severity.h>
#else
#include <iostream>
#endif

// #include <Eigen/Dense>
#include <vector>
#include <string>
#include <map>
// #include <iostream>
//

#define EV2 Eigen::Vector2d
#define EV3 Eigen::Vector3d
#define EVX Eigen::VectorXd
#define EM3 Eigen::Matrix3d
#define EMX Eigen::MatrixXd
 
// typedef Eigen::Vector2d EV2;
// typedef Eigen::Vector3d EV3;
// typedef Eigen::Matrix3d EM3;
// typedef Eigen::MatrixXd EMX;

// cancel the namespace middleware
// namespace middleware {
#define TIMER_INIT \
    std::chrono::high_resolution_clock::time_point t0; \
    std::chrono::milliseconds sleep_time; \
    t0 = std::chrono::high_resolution_clock::now();

#define TIMER_CONTROL(duration) \
    sleep_time = std::chrono::milliseconds(duration) - std::chrono::duration_cast<std::chrono::milliseconds>( \
        std::chrono::high_resolution_clock::now() - t0); \
    if (sleep_time.count() > 0) { \
      std::this_thread::sleep_for(sleep_time); \
    } \
    t0 = std::chrono::high_resolution_clock::now();

#define SINGLETON_DECLARE(TYPE, ...) \
    protected: \
    TYPE(__VA_ARGS__); \
    virtual ~TYPE(); \
    static TYPE* instance_; \
    public: \
    static TYPE* create_instance(__VA_ARGS__); \
    static TYPE* instance(); \
    static void  destroy_instance(); \
    private:

#define SINGLETON_IMPL(TYPE) \
    TYPE* TYPE::instance_ = nullptr; \
    TYPE* TYPE::create_instance() { \
      if (nullptr != instance_) { \
        LOG_WARNING << "This method 'create_instance()' is called twice."; \
      } else { \
        instance_ = new TYPE(); \
      } \
      return instance_; \
    } \
    TYPE* TYPE::instance() { \
      if (nullptr == instance_) \
        LOG_WARNING << "This method instance() should be called after " \
            << "create_instance()"; \
      return instance_; \
    } \
    void TYPE::destroy_instance() { \
      if (nullptr != instance_) { \
        delete instance_; \
        instance_ = nullptr; \
      } \
    }

#define SINGLETON_IMPL_NO_CREATE(TYPE) \
    TYPE* TYPE::instance_ = nullptr; \
    TYPE* TYPE::instance() { \
      if (nullptr == instance_) \
        LOG_WARNING << "This method instance() should be called after " \
            << "create_instance()"; \
      return instance_; \
    } \
    void TYPE::destroy_instance() { \
      if (nullptr != instance_) { \
        delete instance_; \
        instance_ = nullptr; \
      } \
    }

// #define PRESS_THEN_GO do {LOG_WARNING << " -> Press any key to continue."; } while(0);
#define PRESS_THEN_GO do {LOG_WARNING << " -> Press any key to continue."; getchar();} while(0);
//   {LOG_WARNING << std::string(__FILE__).substr(std::string(__FILE__).rfind('/')+1) << ":" << __LINE__ << " -> Press any key to continue."; getchar();}


//template<class _Type>
//using MiiVector =  std::vector<_Type>;

//template<class _Key, class _Value>
//using MiiMap    =  std::map<_Key, _Value>;

// typedef std::string MiiString;
// using MiiString = std::string;

enum JntCmdType {
  UNKNOWN_CMD_TYPE = -1,
  CMD_POS = 0,
  CMD_VEL,
  CMD_TOR,
  CMD_POS_VEL,
  CMD_MOTOR_VEL,
  N_JNT_CMD_TYPES,
};
#define JNTCMDTYPE2STR(l) \
    ( ( (const char*[]){"UNKNOWN_CMD_TYPE", "CMD_POS", "CMD_VEL", "CMD_TOR", "CMD_POS_VEL", "CMD_MOTOR_VEL", "N_JNT_CMD_TYPES"} )[(l) + 1] )

enum JntDataType {
  UNKNOWN_TYPE = -1,
  POS = 0,
  VEL = 1,
  TOR = 2,
  N_JNT_DATA_TYPES = 3
};
#define JNTDATATYPE2STR(l) \
    ( ( (const char*[]){"UNKNOWN_TYPE", "POS", "VEL", "TOR", "N_JNT_DATA_TYPES"} )[(l) + 1] )

enum LegType {
  UNKNOWN_LEG = -1,
  FL = 0,
  FR = 1,
  HL = 2,
  HR = 3,
  N_LEGS = 4
};
#define FOREACH_LEG(l) for (const auto& l : {LegType::FL, LegType::FR, LegType::HL, LegType::HR})

#define LEGTYPE2STR(l) \
    ( ( (const char*[]){"UNKNOWN_LEG", "FL", "FR", "HL", "HR", "N_LEGS"} )[(l) + 1] )

///! whether is belong to front legs
#define LEGTYPE_IS_FRONT(l) \
    ( (FL == l) || (FR == l) )

///! whether is belong to hind leg
#define LEGTYPE_IS_HIND(l) \
    ( (HL == l) || (HR == l) )

///! whether is belong to left legs
#define LEGTYPE_IS_LEFT(l) \
    ( (FL == l) || (HL == l) )

///! whether is belong to right leg
#define LEGTYPE_IS_RIGHT(l) \
    ( (HR == l) || (FR == l) )

///! diagonal leg
#define LEGTYPE_DL(l) \
    ( ( (const LegType[]){UNKNOWN_LEG, HR, HL, FR, FL, UNKNOWN_LEG} )[(l) + 1] )

///! same side, both front or hind legs
#define LEGTYPE_SL(l) \
    ( ( (const LegType[]){UNKNOWN_LEG, FR, FL, HR, HL, UNKNOWN_LEG} )[(l) + 1] )

///! ipsilateral leg, both right or left
#define LEGTYPE_IL(l) \
    ( ( (const LegType[]){UNKNOWN_LEG, HL, HR, FL, FR, UNKNOWN_LEG} )[(l) + 1] )

///! contralateral front leg
#define LEGTYPE_CF(l) \
    ( ( (const LegType[]){UNKNOWN_LEG, FR, FL, FR, FL, UNKNOWN_LEG} )[(l) + 1] )

///! contralateral hind leg
#define LEGTYPE_CH(l) \
    ( ( (const LegType[]){UNKNOWN_LEG, HR, HL, HR, HL, UNKNOWN_LEG} )[(l) + 1] )

enum JntType {
  UNKNOWN_JNT = -1,
  ///! HIP abduction/adduction
  HAA = 0,
  ///! HIP flexion/extension
  HFE = 1,
  ///! KNEE flexion/extension
  KFE = 2,
  N_JNTS = 3
};

#define FOREACH_JNT(j) for (const auto& j : {JntType::HAA, JntType::HFE, JntType::KFE})

#define JNTTYPE2STR(l) \
    ( ( (const char*[]){"UNKNOWN_JNT", "HAA", "HFE", "KFE", "N_JNTS"} )[(l) + 1] )

#define _DEBUG_INFO_FLAG (true)

#ifdef USING_GLOG
#define LOG_DEBUG     if (_DEBUG_INFO_FLAG) LOG(WARNING)  << "\t"
#define LOG_INFO      LOG(INFO)     << "\t"
#define LOG_WARNING   LOG(WARNING)  << "\t"
#define LOG_ERROR     LOG(ERROR)    << "\t"
#define LOG_FATAL     LOG(FATAL)    << "\t"
#else
#define LOG_DEBUG     if (_DEBUG_INFO_FLAG) std::cout  << "\t"
#define LOG_INFO      std::cout << std::string(__FILE__).substr(std::string(__FILE__).rfind('/')+1) << ":" << __LINE__ << " -> "
#define LOG_WARNING   std::cout << std::string(__FILE__).substr(std::string(__FILE__).rfind('/')+1) << ":" << __LINE__ << " -> "
#define LOG_ERROR     std::cout << std::string(__FILE__).substr(std::string(__FILE__).rfind('/')+1) << ":" << __LINE__ << " -> "
#define LOG_FATAL     std::cout << std::string(__FILE__).substr(std::string(__FILE__).rfind('/')+1) << ":" << __LINE__ << " -> "
#endif
//} /* namespace middleware */



#endif /* INCLUDE_SYSTEM_UTILS_UTF_H_ */
