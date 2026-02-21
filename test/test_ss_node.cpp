// Copyright 2026 Wimble Robotics
// SPDX-License-Identifier: Apache-2.0

/**
 * @file test_ss_node.cpp
 * @brief Unit tests for the SS synchronous BT action node.
 *
 * SS is a pure BT::SyncActionNode with no external dependencies.
 * Nodes are instantiated directly (without BT::BehaviorTreeFactory) to keep
 * the test simple and dependency-free.
 */

#include <behaviortree_cpp/action_node.h>
#include <gtest/gtest.h>

#include <sigyn_behavior_trees/ss.hpp>

// ---------------------------------------------------------------------------
// TicksSuccessWithValidMessage
// ---------------------------------------------------------------------------
TEST(SsNodeTest, TicksSuccessWithValidMessage) {
  auto bb = BT::Blackboard::create();
  bb->set("message", std::string("hello from unit test"));

  BT::NodeConfiguration config;
  config.blackboard = bb;
  config.input_ports["message"] = "{message}";

  sigyn_behavior_trees::SS ss("test_ss", config);
  EXPECT_EQ(ss.executeTick(), BT::NodeStatus::SUCCESS);
}

// ---------------------------------------------------------------------------
// ThrowsRuntimeErrorOnMissingMessage
// ---------------------------------------------------------------------------
TEST(SsNodeTest, ThrowsRuntimeErrorOnMissingMessage) {
  auto bb = BT::Blackboard::create();
  // "message" key NOT in blackboard → getInput returns error → RuntimeError.

  BT::NodeConfiguration config;
  config.blackboard = bb;
  config.input_ports["message"] = "{message}";

  sigyn_behavior_trees::SS ss("test_ss_no_msg", config);
  EXPECT_THROW(ss.executeTick(), BT::RuntimeError);
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
