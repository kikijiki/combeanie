// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <action_msgs/msg/goal_status.hpp>
#include <action_msgs/msg/goal_status_array.hpp>
#include <action_msgs/srv/cancel_goal.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <restocker_interfaces/action/restock_product.hpp>

#include "restocker_task_executor/action_client_finality.hpp"

#include "ros_domain_lease_guard.hpp"

namespace restocker_task_executor
{
namespace
{

using namespace std::chrono_literals;
using Action = restocker_interfaces::action::RestockProduct;
using action_msgs::msg::GoalStatus;

TEST(DeliveredTerminal, OnlyTerminalGoalStatusesAreDeliveredTerminals)
{
  EXPECT_TRUE(delivered_terminal(GoalStatus::STATUS_SUCCEEDED));
  EXPECT_TRUE(delivered_terminal(GoalStatus::STATUS_CANCELED));
  EXPECT_TRUE(delivered_terminal(GoalStatus::STATUS_ABORTED));
  // rclcpp_action's answer to a result request that found no registered goal.
  EXPECT_FALSE(delivered_terminal(GoalStatus::STATUS_UNKNOWN));
  EXPECT_FALSE(delivered_terminal(GoalStatus::STATUS_ACCEPTED));
  EXPECT_FALSE(delivered_terminal(GoalStatus::STATUS_EXECUTING));
  EXPECT_FALSE(delivered_terminal(GoalStatus::STATUS_CANCELING));
  EXPECT_FALSE(delivered_terminal(-1));
  EXPECT_FALSE(delivered_terminal(7));
}

// A raw action server: it accepts every goal, answers every result request with STATUS_UNKNOWN
// (what rclcpp_action sends when a result request found no registered goal) and records every
// cancel request it receives.
class UnknownResultActionServer final
{
public:
  UnknownResultActionServer(const rclcpp::Node::SharedPtr & node, const std::string & action_name)
  {
    send_goal_ = node->create_service<Action::Impl::SendGoalService>(
      action_name + "/_action/send_goal",
      [](const Action::Impl::SendGoalService::Request::SharedPtr,
      Action::Impl::SendGoalService::Response::SharedPtr response) {
        response->accepted = true;
      });
    get_result_ = node->create_service<Action::Impl::GetResultService>(
      action_name + "/_action/get_result",
      [](const Action::Impl::GetResultService::Request::SharedPtr,
      Action::Impl::GetResultService::Response::SharedPtr response) {
        response->status = GoalStatus::STATUS_UNKNOWN;
      });
    cancel_goal_ = node->create_service<action_msgs::srv::CancelGoal>(
      action_name + "/_action/cancel_goal",
      [this](const action_msgs::srv::CancelGoal::Request::SharedPtr request,
      action_msgs::srv::CancelGoal::Response::SharedPtr) {
        std::lock_guard lock(mutex_);
        cancels_.push_back(*request);
      });
    // An rclcpp_action client counts a server as available once its topics are matched too.
    feedback_ = node->create_publisher<Action::Impl::FeedbackMessage>(
      action_name + "/_action/feedback", rclcpp::ServicesQoS());
    status_ = node->create_publisher<action_msgs::msg::GoalStatusArray>(
      action_name + "/_action/status", rclcpp::QoS(1).reliable().transient_local());
  }

  [[nodiscard]] std::vector<action_msgs::srv::CancelGoal::Request> cancels() const
  {
    std::lock_guard lock(mutex_);
    return cancels_;
  }

private:
  mutable std::mutex mutex_;
  std::vector<action_msgs::srv::CancelGoal::Request> cancels_;
  rclcpp::Service<Action::Impl::SendGoalService>::SharedPtr send_goal_;
  rclcpp::Service<Action::Impl::GetResultService>::SharedPtr get_result_;
  rclcpp::Service<action_msgs::srv::CancelGoal>::SharedPtr cancel_goal_;
  rclcpp::Publisher<Action::Impl::FeedbackMessage>::SharedPtr feedback_;
  rclcpp::Publisher<action_msgs::msg::GoalStatusArray>::SharedPtr status_;
};

class ActionClientFinality : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    server_node_ = std::make_shared<rclcpp::Node>("action_client_finality_server");
    client_node_ = std::make_shared<rclcpp::Node>("action_client_finality_client");
    server_ = std::make_unique<UnknownResultActionServer>(server_node_, kActionName);
    client_ = rclcpp_action::create_client<Action>(client_node_, kActionName);
    cancel_client_ = create_goal_cancel_client(*client_node_, kActionName);
    executor_.add_node(server_node_);
    executor_.add_node(client_node_);
    spinner_ = std::thread([this]() {executor_.spin();});
  }

  void TearDown() override
  {
    executor_.cancel();
    spinner_.join();
  }

  // An accepted goal whose result future was fulfilled with STATUS_UNKNOWN.
  [[nodiscard]] std::pair<rclcpp_action::ClientGoalHandle<Action>::SharedPtr,
    rclcpp_action::ClientGoalHandle<Action>::WrappedResult>
  accepted_goal_with_unknown_result()
  {
    EXPECT_TRUE(client_->wait_for_action_server(5s));
    EXPECT_TRUE(cancel_client_->wait_for_service(5s));
    auto accepted = client_->async_send_goal(Action::Goal{});
    EXPECT_EQ(accepted.wait_for(5s), std::future_status::ready);
    auto handle = accepted.get();
    EXPECT_TRUE(handle);
    auto result = client_->async_get_result(handle);
    EXPECT_EQ(result.wait_for(5s), std::future_status::ready);
    return {handle, result.get()};
  }

  [[nodiscard]] std::vector<action_msgs::srv::CancelGoal::Request> wait_for_cancels(
    std::size_t count, std::chrono::milliseconds timeout) const
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    auto cancels = server_->cancels();
    while (cancels.size() < count && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(5ms);
      cancels = server_->cancels();
    }
    return cancels;
  }

  static constexpr const char * kActionName = "/action_client_finality/restock_product";
  rclcpp::Node::SharedPtr server_node_;
  rclcpp::Node::SharedPtr client_node_;
  std::unique_ptr<UnknownResultActionServer> server_;
  rclcpp_action::Client<Action>::SharedPtr client_;
  GoalCancelClient::SharedPtr cancel_client_;
  rclcpp::executors::MultiThreadedExecutor executor_{rclcpp::ExecutorOptions{}, 2U};
  std::thread spinner_;
};

TEST_F(ActionClientFinality, UndeliveredResultCancelsExactlyThatGoal)
{
  const auto [handle, wrapped] = accepted_goal_with_unknown_result();
  ASSERT_TRUE(handle);
  EXPECT_EQ(wrapped.code, rclcpp_action::ResultCode::UNKNOWN);
  // The cancel does not go through the goal handle: rclcpp_action's client erases the goal right
  // after fulfilling its result future (client.hpp make_result_aware), after which
  // async_cancel_goal(handle) throws UnknownGoalHandleError. Whether that erase has happened yet
  // is a race, so the helper never depends on it.
  EXPECT_FALSE(
    accept_delivered_terminal_or_cancel(
      static_cast<std::int8_t>(wrapped.code), handle->get_goal_id(), *cancel_client_));
  const auto cancels = wait_for_cancels(1U, 5s);
  ASSERT_EQ(cancels.size(), 1U);
  EXPECT_EQ(cancels.front().goal_info.goal_id.uuid, handle->get_goal_id());
  EXPECT_EQ(cancels.front().goal_info.stamp.sec, 0);
  EXPECT_EQ(cancels.front().goal_info.stamp.nanosec, 0U);
}

TEST_F(ActionClientFinality, DeliveredTerminalSendsNoCancel)
{
  const auto [handle, wrapped] = accepted_goal_with_unknown_result();
  ASSERT_TRUE(handle);
  for (const auto code :
    {GoalStatus::STATUS_SUCCEEDED, GoalStatus::STATUS_CANCELED, GoalStatus::STATUS_ABORTED})
  {
    EXPECT_TRUE(accept_delivered_terminal_or_cancel(code, handle->get_goal_id(), *cancel_client_));
  }
  EXPECT_TRUE(wait_for_cancels(1U, 300ms).empty());
}

}  // namespace
}  // namespace restocker_task_executor
