cmake_minimum_required(VERSION 3.5)

project(wai_world_launcher LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# QtCreator supports the following variables for Android, which are identical to qmake Android variables.
# Check http://doc.qt.io/qt-5/deployment-android.html for more information.
# They need to be set before the find_package(Qt5 ...) call.

#if(ANDROID)
#    set(ANDROID_PACKAGE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/android")
#    if (ANDROID_ABI STREQUAL "armeabi-v7a")
#        set(ANDROID_EXTRA_LIBS
#            ${CMAKE_CURRENT_SOURCE_DIR}/path/to/libcrypto.so
#            ${CMAKE_CURRENT_SOURCE_DIR}/path/to/libssl.so)
#    endif()
#endif()

find_package(Qt5 COMPONENTS Widgets Network REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp roslib)

catkin_package(CATKIN_DEPENDS roscpp)

qt5_add_resources(QT_RESOURCES_CPP resources/resources.qrc)

include_directories(include ${catkin_INCLUDE_DIRS})
#include_directories(${Qt5Widgets_INCLUDE_DIRS})

if(ANDROID)
  add_library(wai_world_launcher_node SHARED
    src/wai_world_launcher_node.cpp
    src/win_wai_world.cpp
    src/win_wai_world.ui
    include/win_wai_world.h
    src/win_wai_oa.cpp
    src/win_wai_oa.ui
    include/win_wai_oa.h
  )
else()
  add_executable(wai_world_launcher_node
    src/wai_world_launcher_node.cpp
    src/win_wai_world.cpp
    src/win_wai_world.ui
    include/win_wai_world.h
    src/win_wai_oa.cpp
    src/win_wai_oa.ui
    include/win_wai_oa.h
  )
endif()

target_link_libraries(wai_world_launcher_node Qt5::Widgets Qt5::Network ${QT_LIBRARIES} ${catkin_LIBRARIES})
