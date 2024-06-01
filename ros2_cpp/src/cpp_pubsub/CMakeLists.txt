cmake_minimum_required(VERSION 3.8)
project(cpp_pubsub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

include_directories(include
${rclcpp_INCLUDE_DIRS}
${lifecycle_msgs_INCLUDE_DIRS}
${rclcpp_lifecycle_INCLUDE_DIRS}
${rclcpp_action_INCLUDE_DIRS}
${example_interfaces_INCLUDE_DIRS}
${geometry_msgs_INCLUDE_DIRS}
${tf2_ros_INCLUDE_DIRS}
${tf2_INCLUDE_DIRS}
${tf2_geometry_msgs_INCLUDE_DIRS}
${nav_msgs_INCLUDE_DIRS}
)

set(LIBRARY_NAME "${PROJECT_NAME}_lib")
add_library(${LIBRARY_NAME} INTERFACE)

#target_include_directories(${LIBRABRY_NAME} INTERFACE $<BUILD_INTERFACE:{CMAKE_CURRENT_SOURCE_DIRS}/include>$<INSTALL_INTERFACE:include>)
target_include_directories(${LIBRARY_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${LIBRARY_NAME} INTERFACE 
${rclcpp_LIBRARIES}
${lifecycle_msgs_LIBRARIES}
${rclcpp_lifecycle_LIBRARIES}
${rclcpp_action_LIBRARIES}
${example_interfaces_LIBRARIES}
${geometry_msgs_LIBRARIES}
${tf2_ros_LIBRARIES}
${tf2_LIBRARIES}
${tf2_geometry_msgs_LIBRARIES}
${nav_msgs_LIBRARIES}
)

install(TARGETS ${LIBRARY_NAME}
EXPORT "export_${PROJECT_NAME}"
ARCHIVE DESTINATION lib
LIBRARY DESTINATION lib
RUNTIME DESTINATION lib
INCLUDES DESTINATION include
)

install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

set(dependencies
"rclcpp"
"lifecycle_msgs"
"rclcpp_lifecycle"
"rclcpp_action"
"example_interfaces"
"geometry_msgs"
"tf2_ros"
"tf2"
"tf2_geometry_msgs"
"nav_msgs"
)

find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(pubsub_test test/pubsub_test.cpp src/publisher_member_function.cpp src/subscriber_member_function.cpp src/single_thread_spinner.cpp)

#include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)


#if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/pubsub_test.cpp")
#    ament_add_gtest(pubsub_test test/pubsub_test.cpp)
 #   target_link_libraries(pubsub_test PRIVATE ${LIBRARY_NAME})
#else()
 #   message(FATAL_ERROR "Test source file not found: ${CMAKE_CURRENT_SOURCE_DIR}/test/pubsub_test.cpp")
#endif()

target_include_directories(pubsub_test PUBLIC include/ src/)
#add_executable(publisher src/publisher_member_function.cpp)

ament_target_dependencies(pubsub_test ${dependencies})
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

#if(BUILD_TESTING)
#find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
#  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
#  set(ament_cmake_cpplint_FOUND TRUE)
#  ament_lint_auto_find_test_dependencies()
#endif()
install(TARGETS
pubsub_test 
DESTINATION lib/${PROJECT_NAME})

ament_package()
