cmake_minimum_required(VERSION 2.8.3)
project(agile_foundation)

add_definitions(-std=c++11)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  class_loader
)

# Include a custom cmake file for TinyXML
find_package(TinyXML REQUIRED)
find_package(Eigen3  REQUIRED)

# Declare a catkin package
catkin_package(
  LIBRARIES
  INCLUDE_DIRS
    include
  DEPENDS
    TinyXML
    class_loader
)

###########
## Build ##
###########

# Build
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${TinyXML_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

set(SRC_ROOT            src)
set(SRC_TEST_ROOT       test)
set(SRC_FOUNDATION      ${SRC_ROOT}/foundation)

set(SRC_FDT_THREAD      ${SRC_FOUNDATION}/thread)

set(SRC_REP             ${SRC_ROOT}/repository)
set(SRC_REP_RES         ${SRC_REP}/resource)
set(SRC_REP_EST         ${SRC_REP}/estimator)
set(SRC_REP_BOX         ${SRC_REP}/control_toolbox)

set(SRC_TOOLBOX         ${SRC_ROOT}/toolbox)

# Files define.
set(SYS_FOUDATION_FILES ${SRC_FOUNDATION}/label.cpp
                        ${SRC_FOUNDATION}/auto_instanceor.cpp

                        ${SRC_FOUNDATION}/cfg_reader.cpp
                        ${SRC_FOUNDATION}/internal/mii_config.cpp
                        ${SRC_FOUNDATION}/internal/expr.cpp
                        
                        ${SRC_FDT_THREAD}/threadpool.cpp

                        ${SRC_FOUNDATION}/internal/sync.cpp
                        ${SRC_FOUNDATION}/ipc/msg_queue.cpp
                        ${SRC_FOUNDATION}/ipc/shared_mem.cpp
)

set(SYS_REGISTRY_FILES  ${SRC_REP}/registry.cpp
)

set(SYS_REPOSITORY_FILES  ${SRC_REP_RES}/imu_sensor.cpp
                          ${SRC_REP_RES}/joint.cpp
                          ${SRC_REP_RES}/motor.cpp
                          ${SRC_REP_RES}/power.cpp
                          ${SRC_REP_RES}/force_sensor.cpp
                          ${SRC_REP_RES}/joint_manager.cpp

                          ${SRC_REP_EST}/touchdown_estimator.cpp

                          ${SRC_REP_BOX}/pid.cpp
                          ${SRC_REP}/data_service.cpp
)

set(SYS_TOOLBOX_FILES     ${SRC_TOOLBOX}/time_control.cpp
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  ${SYS_FOUDATION_FILES}
)

add_library(${PROJECT_NAME}_registry
  ${SYS_REGISTRY_FILES}
)

add_library(${PROJECT_NAME}_repository
  ${SYS_REPOSITORY_FILES}
)

add_library(${PROJECT_NAME}_toolbox
  ${SYS_TOOLBOX_FILES}
)

add_executable(ipc_clear
  ${SRC_FOUNDATION}/ipc/ipc_clear.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${TinyXML_LIBRARIES}
  glog
)

target_link_libraries(${PROJECT_NAME}_registry
  ${EIGEN3_LIBRARIES}
  ${PROJECT_NAME}
)

target_link_libraries(${PROJECT_NAME}_repository
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${PROJECT_NAME}
  ${PROJECT_NAME}_toolbox
)

target_link_libraries(ipc_clear
  ${PROJECT_NAME}
)

# target_link_libraries(${PROJECT_NAME}_repository
#   ${catkin_LIBRARIES}
#   ${EIGEN3_LIBRARIES}
#   ${PROJECT_NAME}
# )

#############
## Install ##
#############

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
                ${PROJECT_NAME}_registry
                ${PROJECT_NAME}_repository
                ${PROJECT_NAME}_toolbox
                ipc_clear
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# foreach(dir include)
#   install(DIRECTORY ${dir}/
#     DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
# endforeach(dir)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}_test
#   ${SYS_TEST_FILES}
# )

# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}_test
#     ${PROJECT_NAME}_sys_robot
#   )
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)