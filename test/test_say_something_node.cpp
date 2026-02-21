// Copyright 2026 Wimble Robotics
// SPDX-License-Identifier: Apache-2.0

/**
 * @file test_say_something_node.cpp
 * @brief Integration tests for the SaySomething BT action node.
 *
 * A mock SaySomethingActionServer is spun in a background thread.  Each test
 * builds a BT tree containing a SaySomething node, ticks it to completion,
 * and verifies SUCCESS.
 *
 * The mock mirrors the real server's contract: accepts every goal and
 * immediately calls succeed().  No hardware or external process is needed.
 *
 * Design note: BT::BehaviorTreeFactory is stored as a std::shared_ptr (static
 * class member) to avoid the = default move-assignment / PImpl incomplete-type
 * compile error present in behaviortree_cpp 4.8.3 with GCC 13.
 */

#include <behaviortree_cpp/bt_factory.h>
#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "sigyn_behavior_trees/action/say_something.hpp"
#include "sigyn_behavior_trees/say_something.hpp"

using SaySomethingAction = sigyn_behavior_trees::action::SaySomething;
using GoalHandle = rclcpp_action::ServerGoalHandle<SaySomethingAction>;

// ---------------------------------------------------------------------------
// MockSaySomethingServer
// ---------------------------------------------------------------------------
class MockSaySomethingServer : public rclcpp::Node {
 public:
  MockSaySomethingServer()
      : Node("mock_say_something_server") {
    server_ = rclcpp_action::create_server<SaySomethingAction>(
        this, "say_something",
        [](auto /*uuid*/, auto /*goal*/) {
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](auto /*gh*/) { return rclcpp_action::CancelResponse::ACCEPT; },
        [](std::shared_ptr<GoalHandle> gh) {
          std::thread([gh]() {
            gh->succeed(std::make_shared<SaySomethingAction::Result>());
          }).detach();
        });
  }

 private:
  rclcpp_action::Server<SaySomethingAction>::SharedPtr server_;
};

// ---------------------------------------------------------------------------
// Test fixture
// ---------------------------------------------------------------------------
class SaySomethingNodeTest : public ::testing::Test {
 public:
  static void SetUpTestCase() {
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    bt_node_ = rclcpp::Node::make_shared("say_something_bt_test_node");
    mock_server_ = std::make_shared<MockSaySomethingServer>();
    executor_->add_node(bt_node_);
    executor_->add_node(mock_server_);
    spin_thread_ = std::thread([]() { executor_->spin(); });
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // Factory held as shared_ptr to avoid stack-destroying BehaviorTreeFactory
    // (which would trigger the = default move-assignment / PImpl issue).
    factory_ = std::make_shared<BT::BehaviorTreeFactory>();
    factory_->registerBuilder<sigyn_behavior_trees::SaySomething>(
        "SaySomething",
        [](const std::string &name, const BT::NodeConfiguration &conf) {
          return std::make_unique<sigyn_behavior_trees::SaySomething>(name, "say_something",
                                                                       conf);
        });

    config_ = new BT::NodeConfiguration();
    config_->blackboard = BT::Blackboard::create();
    config_->blackboard->set("node", bt_node_);
    config_->blackboard->set("server_timeout", std::chrono::milliseconds(5000));
    config_->blackboard->set("bt_loop_duration", std::chrono::milliseconds(10));
    config_->blackboard->set("wait_for_service_timeout",
                              std::chrono::milliseconds(5000));
  }

  static void TearDownTestCase() {
    executor_->cancel();
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    delete config_;
    config_ = nullptr;
    factory_.reset();
    mock_server_.reset();
    bt_node_.reset();
    executor_.reset();
  }

 protected:
  static BT::NodeStatus TickUntilDone(
      BT::Tree &tree, std::chrono::seconds timeout = std::chrono::seconds(10)) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    while (status == BT::NodeStatus::RUNNING &&
           std::chrono::steady_clock::now() < deadline) {
      status = tree.tickOnce();
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    return status;
  }

  static rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;
  static rclcpp::Node::SharedPtr bt_node_;
  static std::shared_ptr<MockSaySomethingServer> mock_server_;
  static std::shared_ptr<BT::BehaviorTreeFactory> factory_;
  static BT::NodeConfiguration *config_;
  static std::thread spin_thread_;
};

rclcpp::executors::MultiThreadedExecutor::SharedPtr
    SaySomethingNodeTest::executor_ = nullptr;
rclcpp::Node::SharedPtr SaySomethingNodeTest::bt_node_ = nullptr;
std::shared_ptr<MockSaySomethingServer> SaySomethingNodeTest::mock_server_ =
    nullptr;
std::shared_ptr<BT::BehaviorTreeFactory> SaySomethingNodeTest::factory_ =
    nullptr;
BT::NodeConfiguration *SaySomethingNodeTest::config_ = nullptr;
std::thread SaySomethingNodeTest::spin_thread_;

// ---------------------------------------------------------------------------
// TreeSucceedsWithMessageOnly
// ---------------------------------------------------------------------------
TEST_F(SaySomethingNodeTest, TreeSucceedsWithMessageOnly) {
  auto tree = factory_->createTreeFromFile(
      std::string(TEST_BT_XML_DIR) + "/say_something_test.xml",
      config_->blackboard);
  EXPECT_EQ(TickUntilDone(tree), BT::NodeStatus::SUCCESS);
}

// ---------------------------------------------------------------------------
// TreeSucceedsWithMessageAndPose
// ---------------------------------------------------------------------------
TEST_F(SaySomethingNodeTest, TreeSucceedsWithMessageAndPose) {
  static const char kXml[] = R"(
    <root BTCPP_format="4" main_tree_to_execute="MainTree">
      <BehaviorTree ID="MainTree">
        <SaySomething message="test with pose" pose="{goal_pose}"/>
      </BehaviorTree>
    </root>)";

  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.position.x = 1.0;
  pose.pose.position.y = 2.0;
  pose.pose.orientation.w = 1.0;
  config_->blackboard->set("goal_pose", pose);

  auto tree = factory_->createTreeFromText(kXml, config_->blackboard);
  EXPECT_EQ(TickUntilDone(tree), BT::NodeStatus::SUCCESS);
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
