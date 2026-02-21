cmake_minimum_required(VERSION 3.8)
project(sigyn_behavior_trees)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(action_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)

# find_package(nav2_costmap_2d REQUIRED)
# find_package(nav2_util REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2_ros REQUIRED)

set(dependencies
  behaviortree_cpp
  builtin_interfaces
  geometry_msgs
  nav2_behavior_tree
  rclcpp
  rclcpp_action
  rclcpp_components
  tf2_ros
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveAShortDistanceAhead.action"
  "action/SaySomething.action"
  DEPENDENCIES builtin_interfaces geometry_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_library(sigyn_say_something_plugin_node SHARED
  src/say_something.cpp
)

list(APPEND plugin_libs sigyn_say_something_plugin_node)

add_library(sigyn_move_a_short_distance_ahead_plugin_node SHARED
  src/move_a_short_distance_ahead.cpp
)

list(APPEND plugin_libs sigyn_move_a_short_distance_ahead_plugin_node)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
  target_link_libraries(${bt_plugin} "${cpp_typesupport_target}")
  target_include_directories(${bt_plugin} PRIVATE include)
endforeach()

add_library(sigyn_SaySomethingActionServer SHARED
  src/say_something_action_server.cpp
)
ament_target_dependencies(sigyn_SaySomethingActionServer ${dependencies})
target_include_directories(sigyn_SaySomethingActionServer PRIVATE include)
target_link_libraries(sigyn_SaySomethingActionServer ${plugin_libs})
rclcpp_components_register_node(sigyn_SaySomethingActionServer
  PLUGIN "sigyn_behavior_trees::SaySomethingActionServer"
  EXECUTABLE SaySomethingActionServer)
install(TARGETS sigyn_SaySomethingActionServer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_library(sigyn_MoveAShortDistanceAheadActionServer SHARED
  src/move_a_short_distance_ahead_action_server.cpp
)
ament_target_dependencies(sigyn_MoveAShortDistanceAheadActionServer ${dependencies})
target_include_directories(sigyn_MoveAShortDistanceAheadActionServer PRIVATE include)
target_link_libraries(sigyn_MoveAShortDistanceAheadActionServer ${plugin_libs})
rclcpp_components_register_node(sigyn_MoveAShortDistanceAheadActionServer
  PLUGIN "sigyn_behavior_trees::MoveAShortDistanceAheadActionServer"
  EXECUTABLE MoveAShortDistanceAheadActionServer)
install(TARGETS sigyn_MoveAShortDistanceAheadActionServer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_executable(bt_test1
  src/bt_test1_main.cpp
)
ament_target_dependencies(bt_test1
  ${dependencies}
)
target_include_directories(bt_test1 PRIVATE include)
target_link_libraries(bt_test1
  "${cpp_typesupport_target}"
  sigyn_say_something_plugin_node
)

install(TARGETS bt_test1
  DESTINATION lib/${PROJECT_NAME})

add_executable(bt_test2
  src/bt_test2_main.cpp
)
ament_target_dependencies(bt_test2
  ${dependencies}
)
target_include_directories(bt_test2 PRIVATE include)
target_link_libraries(bt_test2
  "${cpp_typesupport_target}"
  sigyn_say_something_plugin_node
)

install(TARGETS bt_test2
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

# Install config files.
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(${dependencies})
ament_export_include_directories(include)
ament_export_libraries(${plugin_libs})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  # Exclude uncrustify: this package uses clang-format (Google style), which is
  # incompatible with the ament default uncrustify configuration.
  # Exclude copyright: this package uses SPDX-License-Identifier headers which
  # are not recognised by the ament_copyright version shipped with Jazzy.
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_uncrustify
    ament_cmake_copyright
  )
  ament_lint_auto_find_test_dependencies()

  # ── Unit test: SS synchronous BT node (no ROS required) ──────────────────
  ament_add_gtest(test_ss_node
    test/test_ss_node.cpp
    TIMEOUT 30
  )
  ament_target_dependencies(test_ss_node behaviortree_cpp)
  target_include_directories(test_ss_node PRIVATE include)

  # ── Integration test: SaySomething BT node + mock action server ──────────
  ament_add_gtest(test_say_something_node
    test/test_say_something_node.cpp
    TIMEOUT 60
  )
  ament_target_dependencies(test_say_something_node ${dependencies})
  target_include_directories(test_say_something_node PRIVATE include)
  target_link_libraries(test_say_something_node
    sigyn_say_something_plugin_node
    "${cpp_typesupport_target}"
  )
  target_compile_definitions(test_say_something_node PRIVATE
    TEST_BT_XML_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/bt_xml"
  )

  # ── Integration test: MoveAShortDistanceAhead BT node + mock server ───────
  ament_add_gtest(test_move_a_short_distance_ahead_node
    test/test_move_a_short_distance_ahead_node.cpp
    TIMEOUT 60
  )
  ament_target_dependencies(test_move_a_short_distance_ahead_node ${dependencies})
  target_include_directories(test_move_a_short_distance_ahead_node PRIVATE include)
  target_link_libraries(test_move_a_short_distance_ahead_node
    sigyn_move_a_short_distance_ahead_plugin_node
    "${cpp_typesupport_target}"
  )
  target_compile_definitions(test_move_a_short_distance_ahead_node PRIVATE
    TEST_BT_XML_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/bt_xml"
  )
endif()

ament_package()
