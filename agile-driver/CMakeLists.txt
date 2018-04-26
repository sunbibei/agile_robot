cmake_minimum_required(VERSION 2.8.3)
project(agile_driver)

add_definitions(-std=c++11)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  class_loader
  agile_foundation
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

set(SRC_SYS             ${SRC_ROOT})
set(SRC_SYS_PLATFORM    ${SRC_SYS}/platform)
set(SRC_SYS_ROBOT       ${SRC_SYS}/robot)
set(SRC_SYS_UTILS       ${SRC_SYS}/utils)

# Files define.
set(SYS_FLATFORM_FILES  ${SRC_SYS_PLATFORM}/sw_node/sw_node_manager.cpp
                        ${SRC_SYS_PLATFORM}/sw_node/sw_node.cpp
                        ${SRC_SYS_PLATFORM}/sw_node/leg_node.cpp
                        ${SRC_SYS_PLATFORM}/sw_node/motor_node.cpp
                        ${SRC_SYS_PLATFORM}/sw_node/power_node.cpp
                        ${SRC_SYS_PLATFORM}/sw_node/test_node.cpp

                        ${SRC_SYS_PLATFORM}/propagate/propagate_manager.cpp
                        ${SRC_SYS_PLATFORM}/propagate/propagate.cpp
                        # ${SRC_SYS_PLATFORM}/propagate/pcan_fake.cpp
                        # ${SRC_SYS_PLATFORM}/propagate/pcan.cpp
                        # ${SRC_SYS_PLATFORM}/propagate/arm_pcan.cpp
                        # ${SRC_SYS_PLATFORM}/propagate/motor_pcan.cpp
                        ${SRC_SYS_PLATFORM}/propagate/usb.cpp
                        ${SRC_SYS_PLATFORM}/propagate/gazebo_propagate.cpp

                        ${SRC_SYS_PLATFORM}/master.cpp
)

set(SYS_ROBOT_FILES     ${SRC_SYS_ROBOT}/mii_robot.cpp
)

set(SYS_TEST_FILES      ${SRC_TEST_ROOT}/main.cpp)

## Declare a C++ library
add_library(${PROJECT_NAME}_sys_platform
  ${SYS_FLATFORM_FILES}
)

add_library(${PROJECT_NAME}_sys_robot
  ${SYS_ROBOT_FILES}
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_sys_platform
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  agile_foundation
  agile_foundation_repository
  # pcanbasic
)

target_link_libraries(${PROJECT_NAME}_sys_robot
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_sys_platform
  agile_foundation_registry
)

#############
## Install ##
#############

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}_sys_platform
                ${PROJECT_NAME}_sys_robot
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir images config)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
catkin_add_gtest(${PROJECT_NAME}_test
  ${SYS_TEST_FILES}
)

if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}_test
    ${PROJECT_NAME}_sys_robot
  )
endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)