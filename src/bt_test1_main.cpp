// Copyright 2026 Wimble Robotics
// SPDX-License-Identifier: Apache-2.0

#include <atomic>
#include <chrono>
#include <cstddef>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"

#include "sigyn_behavior_trees/say_something.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr g_node = rclcpp::Node::make_shared("bt_test1");

  std::string xml_path;
  g_node->declare_parameter<std::string>("xml_path", "foo");
  g_node->get_parameter("xml_path", xml_path);

  std::shared_ptr<BT::BehaviorTreeFactory> factory = std::make_shared<BT::BehaviorTreeFactory>();
  auto config = std::make_unique<BT::NodeConfiguration>();
  config->blackboard = BT::Blackboard::create();
  config->blackboard->set("bt_loop_duration", std::chrono::milliseconds(10));
  config->blackboard->set("node", g_node);
  config->blackboard->set<std::chrono::milliseconds>("server_timeout",
                                                     std::chrono::milliseconds(20));
  config->blackboard->set("wait_for_service_timeout", std::chrono::milliseconds(10'000));

  BT::NodeBuilder builder = [](const std::string &name, const BT::NodeConfiguration &config) {
    return std::make_unique<sigyn_behavior_trees::SaySomething>(name, "say_something", config);
  };
  factory->registerBuilder<sigyn_behavior_trees::SaySomething>("SaySomething", builder);

  RCLCPP_INFO(g_node->get_logger(), "bt_test1 starting, xml_path: %s", xml_path.c_str());
  auto tree = factory->createTreeFromFile(xml_path, config->blackboard);
  for (auto &subtree : tree.subtrees) {
    auto &blackboard = subtree->blackboard;
    blackboard->set("node", g_node);
  }

  BT::StdCoutLogger logger(tree);

  std::atomic<bool> running(true);

  std::thread action_server([&]() {
    while (running) {
      tree.tickOnce();
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  });

  std::cin.get();
  running = false;
  action_server.join();

  RCLCPP_INFO(g_node->get_logger(), "bt_test1 finished");
  rclcpp::shutdown();
  return 0;
}
