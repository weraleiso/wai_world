cmake_minimum_required(VERSION 3.0.2)
project(picovoice_driver)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  ddynamic_reconfigure
  picovoice_msgs
  roscpp
  roslib
)

catkin_package(
  CATKIN_DEPENDS
    actionlib
    ddynamic_reconfigure
    picovoice_msgs
    roscpp
    roslib
)

############
### Build ##
############

find_package(PkgConfig)
pkg_check_modules(YAML_CPP yaml-cpp)
if(${YAML_CPP_VERSION} VERSION_GREATER 0.5)
  add_definitions(-DHAVE_NEW_YAMLCPP)
endif()
link_directories(${YAML_CPP_LIBRARY_DIRS})
include_directories(${YAML_CPP_INCLUDE_DIRS})

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake/)
find_package(LIBSNDFILE REQUIRED)

set(EXTERN ${PROJECT_SOURCE_DIR}/extern)
set(PICOVOICE_INCLUDE_DIRS
  ${EXTERN}/pvrecorder/project/include
  ${EXTERN}/porcupine/include
  ${EXTERN}/rhino/include
  ${EXTERN}/cheetah/include
  ${EXTERN}/picollm/include
  ${EXTERN}/orca/include
)
set(PICOVOICE_LIBRARIES
  ${EXTERN}/pvrecorder/lib/linux/x86_64/libpv_recorder.so
  ${EXTERN}/porcupine/lib/linux/x86_64/libpv_porcupine.so
  ${EXTERN}/rhino/lib/linux/x86_64/libpv_rhino.so
  ${EXTERN}/cheetah/lib/linux/x86_64/libpv_cheetah.so
  ${EXTERN}/picollm/lib/linux/x86_64/libpv_picollm.so
  ${EXTERN}/orca/lib/linux/x86_64/libpv_orca.so
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${LIBSNDFILE_INCLUDE_DIRS}
  ${PICOVOICE_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/recognizer.cpp
  src/porcupine_recognizer.cpp
  src/rhino_recognizer.cpp
  src/cheetah_recognizer.cpp
  src/picollm_recognizer.cpp
  src/orca_recognizer.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${LIBSNDFILE_LIBRARIES}
  ${PICOVOICE_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

add_executable(test_${PROJECT_NAME}_porcupine
  test/test_porcupine.cpp
)
target_link_libraries(test_${PROJECT_NAME}_porcupine ${PROJECT_NAME})

add_executable(test_${PROJECT_NAME}_rhino
  test/test_rhino.cpp
)
target_link_libraries(test_${PROJECT_NAME}_rhino ${PROJECT_NAME})

add_executable(${PROJECT_NAME}_porcupine
  src/porcupine_node.cpp
  src/ros_util.cpp
)
target_link_libraries(${PROJECT_NAME}_porcupine ${PROJECT_NAME})

add_executable(${PROJECT_NAME}_rhino
  src/rhino_node.cpp
  src/ros_util.cpp
)
target_link_libraries(${PROJECT_NAME}_rhino ${PROJECT_NAME})

add_executable(${PROJECT_NAME}_cheetah
  src/cheetah_node.cpp
  src/ros_util.cpp
)
target_link_libraries(${PROJECT_NAME}_cheetah ${PROJECT_NAME})

add_executable(${PROJECT_NAME}_picollm
  src/picollm_node.cpp
  src/ros_util.cpp
)
target_link_libraries(${PROJECT_NAME}_picollm ${PROJECT_NAME})

add_executable(${PROJECT_NAME}_orca
  src/orca_node.cpp
  src/ros_util.cpp
)
target_link_libraries(${PROJECT_NAME}_orca ${PROJECT_NAME})

##############
### Install ##
##############
#
#install(FILES ${PICOVOICE_LIBRARIES}
#  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#)
#
#install(DIRECTORY ${PICOVOICE_INCLUDE_DIRS}
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.h"
#)
#
#install(DIRECTORY ${PROJECT_SOURCE_DIR}/resources
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)
#
#install(
#  TARGETS
#    ${PROJECT_NAME}
#    ${PROJECT_NAME}_porcupine
#    ${PROJECT_NAME}_rhino
#    test_${PROJECT_NAME}_porcupine
#    test_${PROJECT_NAME}_rhino
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#)
