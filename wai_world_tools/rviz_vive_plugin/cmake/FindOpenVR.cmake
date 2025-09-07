#if (NOT OPENVR_DIR)
#  message(FATAL_ERROR "OPENVR_DIR variable is not set")
#endif ()

set(OpenVR_LIBRARIES /home/ias/catkin_ws/src/wai_world/wai_world_tools/openvr/lib/linux64/libopenvr_api.so)
set(OpenVR_INCLUDE_DIRS /home/ias/catkin_ws/src/wai_world/wai_world_tools/openvr/headers)
