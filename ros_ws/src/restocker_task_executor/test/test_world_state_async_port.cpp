// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>

#include "restocker_task_executor/world_state_async_port.hpp"

namespace restocker_task_executor
{
namespace
{

using namespace std::chrono_literals;

class WorldStateAsyncPortTest : public ::testing::Test
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
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    static std::size_t instance = 0;
    const std::string suffix = "case_" + std::to_string(++instance);
    names_ = {
      "/test/world_state_async_port/" + suffix + "/snapshot",
      "/test/world_state_async_port/" + suffix + "/reserve",
      "/test/world_state_async_port/" + suffix + "/validate",
      "/test/world_state_async_port/" + suffix + "/release",
      "/test/world_state_async_port/" + suffix + "/execution_authority"};
    client_node_ = std::make_shared<rclcpp::Node>("world_state_port_client_" + suffix);
    server_node_ = std::make_shared<rclcpp::Node>("world_state_port_server_" + suffix);
    completion_group_ = client_node_->create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
    std::weak_ptr<rclcpp::CallbackGroup> group_lifetime = completion_group_;
    port_ = std::make_unique<WorldStateAsyncPort>(
      *client_node_, completion_group_, names_);
    completion_group_.reset();
    ASSERT_FALSE(group_lifetime.expired()) << "the port must retain its callback group";
    executor_.add_node(client_node_);
    executor_.add_node(server_node_);
  }

  void TearDown() override
  {
    executor_.remove_node(server_node_);
    executor_.remove_node(client_node_);
    port_.reset();
    completion_group_.reset();
    server_node_.reset();
    client_node_.reset();
  }

  template<typename Predicate>
  bool spin_until(Predicate predicate, std::chrono::milliseconds timeout = 1s)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (!predicate() && std::chrono::steady_clock::now() < deadline) {
      executor_.spin_some();
    }
    return predicate();
  }

  WorldStateServiceNames names_;
  rclcpp::Node::SharedPtr client_node_;
  rclcpp::Node::SharedPtr server_node_;
  rclcpp::CallbackGroup::SharedPtr completion_group_;
  std::unique_ptr<WorldStateAsyncPort> port_;
  rclcpp::executors::SingleThreadedExecutor executor_;
};

TEST_F(WorldStateAsyncPortTest, ReportsEachEndpointWithoutWaiting)
{
  EXPECT_FALSE(port_->readiness().all_available());

  auto snapshot = server_node_->create_service<WorldStateAsyncPort::GetSnapshot>(
    names_.get_snapshot,
    [](
      const WorldStateAsyncPort::GetSnapshot::Request::SharedPtr,
      WorldStateAsyncPort::GetSnapshot::Response::SharedPtr) {});
  auto reserve = server_node_->create_service<WorldStateAsyncPort::ReserveTask>(
    names_.reserve_task,
    [](
      const WorldStateAsyncPort::ReserveTask::Request::SharedPtr,
      WorldStateAsyncPort::ReserveTask::Response::SharedPtr) {});
  auto validate = server_node_->create_service<WorldStateAsyncPort::ValidateReservation>(
    names_.validate_reservation,
    [](
      const WorldStateAsyncPort::ValidateReservation::Request::SharedPtr,
      WorldStateAsyncPort::ValidateReservation::Response::SharedPtr) {});
  auto release = server_node_->create_service<WorldStateAsyncPort::ReleaseReservation>(
    names_.release_reservation,
    [](
      const WorldStateAsyncPort::ReleaseReservation::Request::SharedPtr,
      WorldStateAsyncPort::ReleaseReservation::Response::SharedPtr) {});
  auto execution_authority =
    server_node_->create_service<WorldStateAsyncPort::ValidateExecutionAuthority>(
    names_.validate_execution_authority,
    [](
      const WorldStateAsyncPort::ValidateExecutionAuthority::Request::SharedPtr,
      WorldStateAsyncPort::ValidateExecutionAuthority::Response::SharedPtr) {});

  ASSERT_TRUE(spin_until([this]() {return port_->readiness().all_available();}));
  const auto readiness = port_->readiness();
  EXPECT_TRUE(readiness.get_snapshot);
  EXPECT_TRUE(readiness.reserve_task);
  EXPECT_TRUE(readiness.validate_reservation);
  EXPECT_TRUE(readiness.release_reservation);
  EXPECT_TRUE(readiness.validate_execution_authority);
}

TEST_F(WorldStateAsyncPortTest, RejectsMutuallyExclusiveCompletionGroup)
{
  const auto exclusive_group = client_node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  EXPECT_THROW(
    WorldStateAsyncPort(*client_node_, exclusive_group, names_), std::invalid_argument);
}

TEST_F(WorldStateAsyncPortTest, PreservesCorrelationAndResponseForEveryService)
{
  auto snapshot = server_node_->create_service<WorldStateAsyncPort::GetSnapshot>(
    names_.get_snapshot,
    [](
      const WorldStateAsyncPort::GetSnapshot::Request::SharedPtr request,
      WorldStateAsyncPort::GetSnapshot::Response::SharedPtr response)
    {
      response->snapshot.revision = request->include_events ? 41U : 0U;
    });
  auto reserve = server_node_->create_service<WorldStateAsyncPort::ReserveTask>(
    names_.reserve_task,
    [](
      const WorldStateAsyncPort::ReserveTask::Request::SharedPtr request,
      WorldStateAsyncPort::ReserveTask::Response::SharedPtr response)
    {
      response->world_revision = request->selected_snapshot_revision + 1U;
    });
  auto validate = server_node_->create_service<WorldStateAsyncPort::ValidateReservation>(
    names_.validate_reservation,
    [](
      const WorldStateAsyncPort::ValidateReservation::Request::SharedPtr request,
      WorldStateAsyncPort::ValidateReservation::Response::SharedPtr response)
    {
      response->has_reservation = request->token == "capability";
    });
  auto release = server_node_->create_service<WorldStateAsyncPort::ReleaseReservation>(
    names_.release_reservation,
    [](
      const WorldStateAsyncPort::ReleaseReservation::Request::SharedPtr request,
      WorldStateAsyncPort::ReleaseReservation::Response::SharedPtr response)
    {
      response->world_revision = request->operation_id == "release-1" ? 99U : 0U;
    });
  auto execution_authority =
    server_node_->create_service<WorldStateAsyncPort::ValidateExecutionAuthority>(
    names_.validate_execution_authority,
    [](
      const WorldStateAsyncPort::ValidateExecutionAuthority::Request::SharedPtr request,
      WorldStateAsyncPort::ValidateExecutionAuthority::Response::SharedPtr response)
    {
      response->has_proof = request->expected_reservation_id == 42U;
      response->world_revision = request->expected_reservation_revision;
    });
  ASSERT_TRUE(spin_until([this]() {return port_->readiness().all_available();}));

  std::optional<AsyncServiceCompletion<WorldStateAsyncPort::GetSnapshot::Response>> snapshot_done;
  auto snapshot_request = std::make_shared<WorldStateAsyncPort::GetSnapshot::Request>();
  snapshot_request->include_events = true;
  const auto snapshot_sent = port_->get_snapshot(
    {7, 11}, snapshot_request,
    [&snapshot_done](auto completion) {snapshot_done = std::move(completion);});
  ASSERT_TRUE(snapshot_sent) << snapshot_sent.detail;
  ASSERT_TRUE(spin_until([&snapshot_done]() {return snapshot_done.has_value();}));
  ASSERT_TRUE(snapshot_done->has_response());
  EXPECT_EQ(snapshot_done->correlation.goal_generation, 7U);
  EXPECT_EQ(snapshot_done->correlation.operation_generation, 11U);
  EXPECT_EQ(snapshot_done->response->snapshot.revision, 41U);

  std::optional<AsyncServiceCompletion<WorldStateAsyncPort::ReserveTask::Response>> reserve_done;
  auto reserve_request = std::make_shared<WorldStateAsyncPort::ReserveTask::Request>();
  reserve_request->selected_snapshot_revision = 52U;
  const auto reserve_sent = port_->reserve_task(
    {7, 12}, reserve_request,
    [&reserve_done](auto completion) {reserve_done = std::move(completion);});
  ASSERT_TRUE(reserve_sent) << reserve_sent.detail;
  ASSERT_TRUE(spin_until([&reserve_done]() {return reserve_done.has_value();}));
  ASSERT_TRUE(reserve_done->has_response());
  EXPECT_EQ(reserve_done->response->world_revision, 53U);

  std::optional<AsyncServiceCompletion<WorldStateAsyncPort::ValidateReservation::Response>>
  validate_done;
  auto validate_request = std::make_shared<WorldStateAsyncPort::ValidateReservation::Request>();
  validate_request->token = "capability";
  const auto validate_sent = port_->validate_reservation(
    {7, 13}, validate_request,
    [&validate_done](auto completion) {validate_done = std::move(completion);});
  ASSERT_TRUE(validate_sent) << validate_sent.detail;
  ASSERT_TRUE(spin_until([&validate_done]() {return validate_done.has_value();}));
  ASSERT_TRUE(validate_done->has_response());
  EXPECT_TRUE(validate_done->response->has_reservation);

  std::optional<AsyncServiceCompletion<WorldStateAsyncPort::ReleaseReservation::Response>>
  release_done;
  auto release_request = std::make_shared<WorldStateAsyncPort::ReleaseReservation::Request>();
  release_request->operation_id = "release-1";
  const auto release_sent = port_->release_reservation(
    {7, 14}, release_request,
    [&release_done](auto completion) {release_done = std::move(completion);});
  ASSERT_TRUE(release_sent) << release_sent.detail;
  ASSERT_TRUE(spin_until([&release_done]() {return release_done.has_value();}));
  ASSERT_TRUE(release_done->has_response());
  EXPECT_EQ(release_done->response->world_revision, 99U);

  std::optional<
    AsyncServiceCompletion<WorldStateAsyncPort::ValidateExecutionAuthority::Response>>
  authority_done;
  auto authority_request =
    std::make_shared<WorldStateAsyncPort::ValidateExecutionAuthority::Request>();
  authority_request->expected_reservation_id = 42U;
  authority_request->expected_reservation_revision = 100U;
  const auto authority_sent = port_->validate_execution_authority(
    {7, 15}, authority_request,
    [&authority_done](auto completion) {authority_done = std::move(completion);});
  ASSERT_TRUE(authority_sent) << authority_sent.detail;
  ASSERT_TRUE(spin_until([&authority_done]() {return authority_done.has_value();}));
  ASSERT_TRUE(authority_done->has_response());
  EXPECT_EQ(authority_done->correlation.goal_generation, 7U);
  EXPECT_EQ(authority_done->correlation.operation_generation, 15U);
  EXPECT_TRUE(authority_done->response->has_proof);
  EXPECT_EQ(authority_done->response->world_revision, 100U);
}

TEST_F(WorldStateAsyncPortTest, RejectsInvalidSubmissionWithoutCallingCallback)
{
  bool callback_called = false;
  const auto result = port_->get_snapshot(
    {0, 1}, std::make_shared<WorldStateAsyncPort::GetSnapshot::Request>(),
    [&callback_called](auto) {callback_called = true;});
  EXPECT_FALSE(result);
  EXPECT_EQ(result.error, AsyncSendErrorCode::kInvalidArgument);
  EXPECT_FALSE(result.handle);
  EXPECT_FALSE(callback_called);
}

TEST_F(WorldStateAsyncPortTest, ExplicitlyRemovesKnownPendingRequest)
{
  bool callback_called = false;
  const auto result = port_->get_snapshot(
    {3, 9}, std::make_shared<WorldStateAsyncPort::GetSnapshot::Request>(),
    [&callback_called](auto) {callback_called = true;});
  ASSERT_TRUE(result) << result.detail;
  ASSERT_TRUE(result.handle);
  EXPECT_TRUE(port_->remove_pending_request(*result.handle));
  EXPECT_FALSE(port_->remove_pending_request(*result.handle));
  executor_.spin_some();
  EXPECT_FALSE(callback_called);

  auto malformed = *result.handle;
  malformed.correlation.goal_generation = 0;
  EXPECT_FALSE(port_->remove_pending_request(malformed));
}

TEST_F(WorldStateAsyncPortTest, RemovesPendingExecutionAuthorityRead)
{
  bool callback_called = false;
  const auto result = port_->validate_execution_authority(
    {3, 10},
    std::make_shared<WorldStateAsyncPort::ValidateExecutionAuthority::Request>(),
    [&callback_called](auto) {callback_called = true;});
  ASSERT_TRUE(result) << result.detail;
  ASSERT_TRUE(result.handle);
  EXPECT_EQ(result.handle->service, WorldStateServiceKind::kValidateExecutionAuthority);
  EXPECT_TRUE(port_->remove_pending_request(*result.handle));
  EXPECT_FALSE(port_->remove_pending_request(*result.handle));
  executor_.spin_some();
  EXPECT_FALSE(callback_called);
}

TEST(WorldStateAsyncPortStandalone, NamesStableSendErrors)
{
  EXPECT_STREQ(to_string(AsyncSendErrorCode::kNone), "none");
  EXPECT_STREQ(to_string(AsyncSendErrorCode::kInvalidArgument), "invalid_argument");
  EXPECT_STREQ(to_string(AsyncSendErrorCode::kTransportRejected), "transport_rejected");
}

TEST(WorldStateAsyncPortStandalone, SendResultMakesSuccessAndFailureStructurallyExclusive)
{
  const WorldStateRequestHandle handle{
    {3U, 9U}, WorldStateServiceKind::kGetSnapshot, 1};
  EXPECT_NO_THROW(AsyncSendResult(AsyncSendErrorCode::kNone, handle, ""));
  EXPECT_NO_THROW(
    AsyncSendResult(AsyncSendErrorCode::kTransportRejected, std::nullopt, "rejected"));
  EXPECT_THROW(
    AsyncSendResult(AsyncSendErrorCode::kNone, std::nullopt, ""), std::invalid_argument);
  EXPECT_THROW(
    AsyncSendResult(AsyncSendErrorCode::kNone, handle, "unexpected"),
    std::invalid_argument);
  EXPECT_THROW(
    AsyncSendResult(AsyncSendErrorCode::kTransportRejected, handle, "rejected"),
    std::invalid_argument);
  EXPECT_THROW(
    AsyncSendResult(AsyncSendErrorCode::kTransportRejected, std::nullopt, ""),
    std::invalid_argument);
}

}  // namespace
}  // namespace restocker_task_executor
