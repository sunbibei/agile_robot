cmake_minimum_required(VERSION 2.8.3)
project(agile_apps)

add_definitions(-std=c++11)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  rospack

  agile_driver
  agile_control
  agile_foundation
)

# Declare a catkin package
catkin_package(
  LIBRARIES
  INCLUDE_DIRS
    include
  DEPENDS
    roscpp
)

find_package(Eigen3 REQUIRED)

###########
## Build ##
###########

# Build
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

set(SRC_ROOT            src)
# set(SRC_EXAMPLES        examples/src)

set(SRC_APP             ${SRC_ROOT}/apps)

set(ENTRY_FILES         ${SRC_ROOT}/entry.cpp
)

# Files define.
set(ROBOT_FILES         ${SRC_ROOT}/robot.cpp
                        ${SRC_APP}/robot_wrapper.cpp
)

# Files define.
set(CONTROL_FILES       ${SRC_ROOT}/control.cpp
                        ${SRC_APP}/control_wrapper.cpp
)

## Declare a C++ executable
# add_executable(mii_apps
#   ${SRC_ROOT}/main.cpp
#   ${SRC_APP}/ros_wrapper.cpp
# )

add_executable(pd_apps
  ${SRC_APP}/pd_wrapper.cpp
)

# add_executable(mii_launcher
#   ${ENTRY_FILES}
# )

add_executable(leg_robot
  ${SRC_APP}/robot_wrapper.cpp
)

# add_executable(mii_control
#   ${CONTROL_FILES}
# )

add_executable(ipc_clear
  ${SRC_ROOT}/ipc_clear.cpp
)

add_executable(registry_monitor
  ${SRC_ROOT}/registry_monitor.cpp
)

## Specify libraries to link a library or executable target against
# target_link_libraries(mii_apps
#   ${catkin_LIBRARIES}
#   agile_foundation
#   agile_driver_sys_robot
#   agile_control_framwork
# )

target_link_libraries(pd_apps
  ${catkin_LIBRARIES}
  agile_foundation
  agile_driver_sys_robot
)

# target_link_libraries(mii_launcher
#   ${catkin_LIBRARIES}
#   agile_foundation
# )

target_link_libraries(leg_robot
  ${catkin_LIBRARIES}
  agile_driver_sys_robot
)

# target_link_libraries(mii_control
#   ${catkin_LIBRARIES}
#   agile_control_framwork
# )

target_link_libraries(ipc_clear
  ${catkin_LIBRARIES}
  agile_foundation
)

target_link_libraries(registry_monitor
  ${catkin_LIBRARIES}
  agile_foundation
)

#############
## Install ##
#############

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS #mii_apps
                #mii_launcher
                leg_robot
                #mii_control
                # ${PROJECT_NAME}_test_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir examples/launch examples/config launch config)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
