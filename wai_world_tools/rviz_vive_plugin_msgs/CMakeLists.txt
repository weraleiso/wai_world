cmake_minimum_required(VERSION 3.2)
project(rviz_vive_plugin_msgs)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    message_runtime
    std_msgs
    geometry_msgs)

add_message_files(
    DIRECTORY
        msg
    FILES
        Controller.msg
        ControllerVibration.msg)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs geometry_msgs)
