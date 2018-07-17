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
set(SRC_TOOLBOX         ${SRC_ROOT}/toolbox)

set(SRC_TEST            ${SRC_TEST_ROOT}/test_registry2.cpp
                        ${SRC_TEST_ROOT}/main.cpp
)

# Files define.
set(SYS_FOUDATION_FILES ${SRC_FOUNDATION}/app.cpp
                        ${SRC_FOUNDATION}/label.cpp
                        ${SRC_FOUNDATION}/auto_instor.cpp

                        ${SRC_FOUNDATION}/cfg_reader.cpp
                        ${SRC_FOUNDATION}/internal/mii_config.cpp
                        ${SRC_FOUNDATION}/internal/expr.cpp
                        
                        ${SRC_FOUNDATION}/thread/threadpool.cpp

                        ${SRC_FOUNDATION}/internal/sync.cpp
                        ${SRC_FOUNDATION}/ipc/msg_queue.cpp
                        ${SRC_FOUNDATION}/ipc/shared_mem.cpp

                        ${SRC_FOUNDATION}/registry/registry.cpp
                        ${SRC_FOUNDATION}/registry/registry2.cpp
)

set(SYS_TOOLBOX_FILES     ${SRC_TOOLBOX}/time_control.cpp
                          ${SRC_TOOLBOX}/pid.cpp
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  ${SYS_FOUDATION_FILES}
)

add_library(${PROJECT_NAME}_toolbox
  ${SYS_TOOLBOX_FILES}
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${TinyXML_LIBRARIES}
  glog
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
                ${PROJECT_NAME}_toolbox
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