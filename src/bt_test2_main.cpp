// Copyright 2026 Wimble Robotics
// SPDX-License-Identifier: Apache-2.0

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>

#include <stdexcept>

#include <rclcpp/rclcpp.hpp>
#include <sigyn_behavior_trees/ss.hpp>

int main(int argc, char **argv) {
  if (argc < 2) {
    throw std::runtime_error("Usage: bt_test2 <xml_path>");
  }
  rclcpp::init(argc, argv);
  const std::string xml_path = argv[1];

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<sigyn_behavior_trees::SS>("SS");

  auto tree = factory.createTreeFromFile(xml_path);
  tree.tickWhileRunning();
  rclcpp::shutdown();
  return 0;
}
