# CMake generated Testfile for 
# Source directory: /home/ias/picovoice_ws/src/wai_picovoice/picovoice_modules/pvrecorder_new/project
# Build directory: /home/ias/picovoice_ws/src/wai_picovoice/picovoice_modules/pvrecorder_new/project/build
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(test_circular_buffer "/home/ias/picovoice_ws/src/wai_picovoice/picovoice_modules/pvrecorder_new/project/build/test_circular_buffer")
set_tests_properties(test_circular_buffer PROPERTIES  _BACKTRACE_TRIPLES "/home/ias/picovoice_ws/src/wai_picovoice/picovoice_modules/pvrecorder_new/project/CMakeLists.txt;105;add_test;/home/ias/picovoice_ws/src/wai_picovoice/picovoice_modules/pvrecorder_new/project/CMakeLists.txt;0;")
add_test(test_recorder "/home/ias/picovoice_ws/src/wai_picovoice/picovoice_modules/pvrecorder_new/project/build/test_recorder")
set_tests_properties(test_recorder PROPERTIES  _BACKTRACE_TRIPLES "/home/ias/picovoice_ws/src/wai_picovoice/picovoice_modules/pvrecorder_new/project/CMakeLists.txt;112;add_test;/home/ias/picovoice_ws/src/wai_picovoice/picovoice_modules/pvrecorder_new/project/CMakeLists.txt;0;")
subdirs("node")
