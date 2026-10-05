// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <chrono>
#include <condition_variable>
#include <future>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <restocker_interfaces/action/restock_product.hpp>

#include "restocker_task_executor/action_result_publisher.hpp"

namespace restocker_task_executor
{
namespace
{

using namespace std::chrono_literals;
using Action = restocker_interfaces::action::RestockProduct;
using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;

class ExecutorSpinGuard final
{
public:
  explicit ExecutorSpinGuard(rclcpp::Executor & executor)
  : executor_(executor), thread_([this]() {executor_.spin();})
  {
  }

  ~ExecutorSpinGuard()
  {
    executor_.cancel();
    if (thread_.joinable()) {
      thread_.join();
    }
  }

  ExecutorSpinGuard(const ExecutorSpinGuard &) = delete;
  ExecutorSpinGuard & operator=(const ExecutorSpinGuard &) = delete;

private:
  rclcpp::Executor & executor_;
  std::thread thread_;
};

class HeldRestockActionServer final
{
public:
  HeldRestockActionServer(const rclcpp::Node::SharedPtr & node, std::string action_name)
  {
    server_ = rclcpp_action::create_server<Action>(
      node->get_node_base_interface(), node->get_node_clock_interface(),
      node->get_node_logging_interface(), node->get_node_waitables_interface(),
      std::move(action_name),
      [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](const std::shared_ptr<GoalHandle> &) {
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](std::shared_ptr<GoalHandle> handle) {
        {
          std::lock_guard lock(mutex_);
          handle_ = std::move(handle);
        }
        changed_.notify_all();
      });
  }

  [[nodiscard]] std::shared_ptr<GoalHandle> wait_for_handle(
    std::chrono::milliseconds timeout)
  {
    std::unique_lock lock(mutex_);
    if (!changed_.wait_for(lock, timeout, [this]() {return handle_ != nullptr;})) {
      return {};
    }
    return handle_;
  }

private:
  rclcpp_action::Server<Action>::SharedPtr server_;
  std::mutex mutex_;
  std::condition_variable changed_;
  std::shared_ptr<GoalHandle> handle_;
};

class ActionResultPublisherTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void verify_mapping(
    const std::string & suffix, ActionTerminalKind kind,
    rclcpp_action::ResultCode expected_code, bool request_cancel)
  {
    const auto action_name = "/action_result_publisher_" + suffix;
    auto server_node = std::make_shared<rclcpp::Node>(suffix + "_publisher_server");
    auto client_node = std::make_shared<rclcpp::Node>(suffix + "_publisher_client");
    HeldRestockActionServer server(server_node, action_name);
    auto client = rclcpp_action::create_client<Action>(client_node, action_name);
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions{}, 2U);
    executor.add_node(server_node);
    executor.add_node(client_node);
    ExecutorSpinGuard spin(executor);
    ASSERT_TRUE(client->wait_for_action_server(2s));

    auto sent = client->async_send_goal(Action::Goal{});
    ASSERT_EQ(sent.wait_for(2s), std::future_status::ready);
    const auto client_handle = sent.get();
    ASSERT_TRUE(client_handle);
    const auto server_handle = server.wait_for_handle(2s);
    ASSERT_TRUE(server_handle);
    auto result_future = client->async_get_result(client_handle);

    if (request_cancel) {
      auto canceled = client->async_cancel_goal(client_handle);
      ASSERT_EQ(canceled.wait_for(2s), std::future_status::ready);
      ASSERT_FALSE(canceled.get()->goals_canceling.empty());
    }

    auto result = std::make_shared<Action::Result>();
    result->detail = suffix;
    RosActionResultPublisher publisher;
    publisher.publish(server_handle, kind, result);

    ASSERT_EQ(result_future.wait_for(2s), std::future_status::ready);
    const auto wrapped = result_future.get();
    EXPECT_EQ(wrapped.code, expected_code);
    ASSERT_TRUE(wrapped.result);
    EXPECT_EQ(wrapped.result->detail, suffix);
  }
};

TEST_F(ActionResultPublisherTest, PublishesCanceledThroughRealRosGoalHandle)
{
  verify_mapping(
    "canceled", ActionTerminalKind::kCanceled,
    rclcpp_action::ResultCode::CANCELED, true);
}

TEST_F(ActionResultPublisherTest, PublishesAbortedThroughRealRosGoalHandle)
{
  verify_mapping(
    "aborted", ActionTerminalKind::kAborted,
    rclcpp_action::ResultCode::ABORTED, false);
}

TEST_F(ActionResultPublisherTest, PublishesSucceededThroughRealRosGoalHandle)
{
  verify_mapping(
    "succeeded", ActionTerminalKind::kSucceeded,
    rclcpp_action::ResultCode::SUCCEEDED, false);
}

TEST_F(ActionResultPublisherTest, RejectsInvalidKindWithoutPublishing)
{
  const auto action_name = "/action_result_publisher_invalid_kind";
  auto server_node = std::make_shared<rclcpp::Node>("invalid_kind_publisher_server");
  auto client_node = std::make_shared<rclcpp::Node>("invalid_kind_publisher_client");
  HeldRestockActionServer server(server_node, action_name);
  auto client = rclcpp_action::create_client<Action>(client_node, action_name);
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions{}, 2U);
  executor.add_node(server_node);
  executor.add_node(client_node);
  ExecutorSpinGuard spin(executor);
  ASSERT_TRUE(client->wait_for_action_server(2s));

  auto sent = client->async_send_goal(Action::Goal{});
  ASSERT_EQ(sent.wait_for(2s), std::future_status::ready);
  const auto client_handle = sent.get();
  ASSERT_TRUE(client_handle);
  const auto server_handle = server.wait_for_handle(2s);
  ASSERT_TRUE(server_handle);
  auto result_future = client->async_get_result(client_handle);
  auto result = std::make_shared<Action::Result>();
  RosActionResultPublisher publisher;

  EXPECT_THROW(
    publisher.publish(
      server_handle, static_cast<ActionTerminalKind>(255U), result),
    std::invalid_argument);
  EXPECT_EQ(result_future.wait_for(100ms), std::future_status::timeout);

  publisher.publish(server_handle, ActionTerminalKind::kAborted, result);
  ASSERT_EQ(result_future.wait_for(2s), std::future_status::ready);
  EXPECT_EQ(result_future.get().code, rclcpp_action::ResultCode::ABORTED);
}

}  // namespace
}  // namespace restocker_task_executor
