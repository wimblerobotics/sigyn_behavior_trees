// Copyright 2026 Wimble Robotics
// SPDX-License-Identifier: Apache-2.0

/**
 * @file test_move_a_short_distance_ahead_node.cpp
 * @brief Integration tests for the MoveAShortDistanceAhead BT action node.
 *
 * A mock action server replaces the real motor controller.  The mock accepts
 * every goal and immediately calls succeed(), recording the requested distance
 * so tests can verify the BT node sent the correct goal.
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

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "sigyn_behavior_trees/action/move_a_short_distance_ahead.hpp"
#include "sigyn_behavior_trees/move_a_short_distance_ahead.hpp"



using MoveAction = sigyn_behavior_trees::action::MoveAShortDistanceAhead;
using GoalHandle = rclcpp_action::ServerGoalHandle<MoveAction>;

// ---------------------------------------------------------------------------
// MockMoveAShortDistanceAheadServer
// ---------------------------------------------------------------------------
class MockMoveAShortDistanceAheadServer : public rclcpp::Node {
 public:
  explicit MockMoveAShortDistanceAheadServer(
      const std::string &name = "mock_move_a_short_distance_ahead_server")
      : Node(name) {
    server_ = rclcpp_action::create_server<MoveAction>(
        this, "move_a_short_distance_ahead",
        [](auto /*uuid*/, auto /*goal*/) {
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](auto /*gh*/) { return rclcpp_action::CancelResponse::ACCEPT; },
        [this](std::shared_ptr<GoalHandle> gh) {
          std::thread([this, gh]() { execute(gh); }).detach();
        });
  }

  double last_received_distance() const { return last_distance_; }

 private:
  void execute(std::shared_ptr<GoalHandle> gh) {
    last_distance_ = gh->get_goal()->distance;
    RCLCPP_INFO(get_logger(), "[MockMoveServer] distance=%.3f", last_distance_);
    gh->succeed(std::make_shared<MoveAction::Result>());
  }

  rclcpp_action::Server<MoveAction>::SharedPtr server_;
  double last_distance_{0.0};
};

// ---------------------------------------------------------------------------
// Test fixture
// ---------------------------------------------------------------------------
class MoveAShortDistanceAheadNodeTest : public ::testing::Test {
 public:
  static void SetUpTestCase() {
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    bt_node_ = rclcpp::Node::make_shared("move_bt_test_node");
    mock_server_ = std::make_shared<MockMoveAShortDistanceAheadServer>();
    executor_->add_node(bt_node_);
    executor_->add_node(mock_server_);
    spin_thread_ = std::thread([]() { executor_->spin(); });
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    factory_ = std::make_shared<BT::BehaviorTreeFactory>();
    factory_->registerBuilder<sigyn_behavior_trees::MoveAShortDistanceAhead>(
        "MoveAShortDistanceAhead",
        [](const std::string &name, const BT::NodeConfiguration &conf) {
          return std::make_unique<sigyn_behavior_trees::MoveAShortDistanceAhead>(
              name, "move_a_short_distance_ahead", conf);
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
  static std::shared_ptr<MockMoveAShortDistanceAheadServer> mock_server_;
  static std::shared_ptr<BT::BehaviorTreeFactory> factory_;
  static BT::NodeConfiguration *config_;
  static std::thread spin_thread_;
};

rclcpp::executors::MultiThreadedExecutor::SharedPtr
    MoveAShortDistanceAheadNodeTest::executor_ = nullptr;
rclcpp::Node::SharedPtr MoveAShortDistanceAheadNodeTest::bt_node_ = nullptr;
std::shared_ptr<MockMoveAShortDistanceAheadServer>
    MoveAShortDistanceAheadNodeTest::mock_server_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory>
    MoveAShortDistanceAheadNodeTest::factory_ = nullptr;
BT::NodeConfiguration *MoveAShortDistanceAheadNodeTest::config_ = nullptr;
std::thread MoveAShortDistanceAheadNodeTest::spin_thread_;

// ---------------------------------------------------------------------------
// TreeSucceedsFromFile
// ---------------------------------------------------------------------------
TEST_F(MoveAShortDistanceAheadNodeTest, TreeSucceedsFromFile) {
  auto tree = factory_->createTreeFromFile(
      std::string(TEST_BT_XML_DIR) + "/move_a_short_distance_ahead_test.xml",
      config_->blackboard);
  EXPECT_EQ(TickUntilDone(tree), BT::NodeStatus::SUCCESS);
}

// ---------------------------------------------------------------------------
// MockServerReceivesCorrectDistance
// ---------------------------------------------------------------------------
TEST_F(MoveAShortDistanceAheadNodeTest, MockServerReceivesCorrectDistance) {
  static const char kXml[] = R"(
    <root BTCPP_format="4" main_tree_to_execute="MainTree">
      <BehaviorTree ID="MainTree">
        <MoveAShortDistanceAhead distance="0.42"/>
      </BehaviorTree>
    </root>)";

  auto tree = factory_->createTreeFromText(kXml, config_->blackboard);
  const BT::NodeStatus status = TickUntilDone(tree);

  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
  EXPECT_NEAR(mock_server_->last_received_distance(), 0.42, 1e-3);
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
