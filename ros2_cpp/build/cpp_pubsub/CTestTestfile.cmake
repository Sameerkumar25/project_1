# CMake generated Testfile for 
# Source directory: /home/sameer/cpp_files/ros2_cpp/src/cpp_pubsub
# Build directory: /home/sameer/cpp_files/ros2_cpp/build/cpp_pubsub
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(pubsub_test "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/sameer/cpp_files/ros2_cpp/build/cpp_pubsub/test_results/cpp_pubsub/pubsub_test.gtest.xml" "--package-name" "cpp_pubsub" "--output-file" "/home/sameer/cpp_files/ros2_cpp/build/cpp_pubsub/ament_cmake_gtest/pubsub_test.txt" "--command" "/home/sameer/cpp_files/ros2_cpp/build/cpp_pubsub/pubsub_test" "--gtest_output=xml:/home/sameer/cpp_files/ros2_cpp/build/cpp_pubsub/test_results/cpp_pubsub/pubsub_test.gtest.xml")
set_tests_properties(pubsub_test PROPERTIES  LABELS "gtest" REQUIRED_FILES "/home/sameer/cpp_files/ros2_cpp/build/cpp_pubsub/pubsub_test" TIMEOUT "60" WORKING_DIRECTORY "/home/sameer/cpp_files/ros2_cpp/build/cpp_pubsub" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest_test.cmake;86;ament_add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest.cmake;93;ament_add_gtest_test;/home/sameer/cpp_files/ros2_cpp/src/cpp_pubsub/CMakeLists.txt;81;ament_add_gtest;/home/sameer/cpp_files/ros2_cpp/src/cpp_pubsub/CMakeLists.txt;0;")
subdirs("gtest")
