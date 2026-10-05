// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <deque>
#include <future>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

#include "attachment_port_harness.hpp"

namespace restocker_task_executor
{
namespace
{
using attachment_test::Behavior;
using attachment_test::Harness;
using attachment_test::Step;
using attachment_test::Status;
using attachment_test::goal;
using std::chrono_literals::operator""ms;

TEST(RosAttachmentSettlement, PreparesRequestBeforePhysicalSendAndReusesExactIdentity)
{
  Harness io;
  io.steps = {{Behavior::kReply}, {Behavior::kReply}, {Behavior::kReply, Status::OK, true}};
  RosAttachmentPort port(io.config, io.operations());
  io.on_physical = [&]() {
    const auto record = RosAttachmentPortTestPeer::retained(port);
    ASSERT_TRUE(record);
    ASSERT_TRUE(record->attachment_request);
    EXPECT_EQ(
      record->attachment_request->operation_id,
      io.physical_request->operation_id + "-commit");
    EXPECT_EQ(record->attachment_request->token, io.physical_request->reservation_token);
    EXPECT_EQ(
      record->attachment_request->grasp_center_from_held_object,
      io.physical_request->expected_grasp_center_to_child);
  };
  const auto result = io.run(port, goal());
  EXPECT_EQ(result.outcome, AttachmentOutcome::kSucceeded);
  EXPECT_EQ(io.physical_sends, 1);
  EXPECT_EQ(io.acquired, 1);
  EXPECT_EQ(io.released, 1);
  ASSERT_EQ(io.requests.size(), 3U);
  EXPECT_EQ(io.requests[0].get(), io.requests[1].get());
  EXPECT_EQ(io.requests[0].get(), io.requests[2].get());
  EXPECT_EQ(io.release_request->required_semantic_revision, 73U);
  const auto record = RosAttachmentPortTestPeer::retained(port);
  ASSERT_TRUE(record);
  ASSERT_TRUE(record->physical_receipt);
  EXPECT_EQ(record->physical_receipt->simulator_epoch, "original-epoch");
  EXPECT_EQ(record->physical_receipt->sequence, 19U);
  EXPECT_EQ(record->physical_receipt->simulator_iteration, 211U);
  EXPECT_DOUBLE_EQ(record->physical_receipt->fidelity_translation_residual_m, 0.001);
  EXPECT_FALSE(record->unsafe_terminal);
}

TEST(RosAttachmentSettlement, ReadinessAndNoSendAreDefiniteButDoNotEraseOtherSends)
{
  struct Case
  {
    std::deque<Step> steps;
    AttachmentOutcome outcome;
  };
  const std::vector<Case> cases{
    {{{Behavior::kUnready}, {Behavior::kUnready}}, AttachmentOutcome::kUncommitted},
    {{{Behavior::kReply}, {Behavior::kReply}}, AttachmentOutcome::kUncommitted},
    {{{Behavior::kReply}, {Behavior::kTimeout}}, AttachmentOutcome::kIndeterminate},
    {{{Behavior::kTimeout}, {Behavior::kReply}}, AttachmentOutcome::kIndeterminate},
    {{{Behavior::kTimeout}, {Behavior::kUnready}}, AttachmentOutcome::kIndeterminate},
    {{{Behavior::kTimeout}, {Behavior::kReply, Status::TOKEN_MISMATCH}},
      AttachmentOutcome::kIndeterminate},
    {{{Behavior::kReply}, {Behavior::kReply, Status::INVALID_ARGUMENT}},
      AttachmentOutcome::kUncommitted},
    {{{Behavior::kReply, Status::PREDICATE_FAILED}}, AttachmentOutcome::kUncommitted},
    {{{Behavior::kReply, Status::IDEMPOTENCY_CONFLICT}}, AttachmentOutcome::kIndeterminate},
    {{{Behavior::kReply, Status::ATTACHMENT_CLOCK_NOT_READY, true}},
      AttachmentOutcome::kIndeterminate},
    {{{Behavior::kReply, Status::INVALID_ARGUMENT, true}}, AttachmentOutcome::kIndeterminate},
    {{{Behavior::kReply, Status::OK, false}}, AttachmentOutcome::kIndeterminate},
    {{{Behavior::kReply, 999}}, AttachmentOutcome::kIndeterminate},
    {{{Behavior::kGetThrows}}, AttachmentOutcome::kIndeterminate},
  };
  for (const auto & item : cases) {
    Harness io;
    io.steps = item.steps;
    RosAttachmentPort port(io.config, io.operations());
    const auto result = io.run(port, goal());
    EXPECT_EQ(result.outcome, item.outcome);
    EXPECT_EQ(io.physical_sends, 1);
    EXPECT_EQ(io.released, 0);
    EXPECT_FALSE(result.planning_scene_lease_released);
    const auto record = RosAttachmentPortTestPeer::retained(port);
    ASSERT_TRUE(record);
    EXPECT_TRUE(record->unsafe_terminal);
    EXPECT_EQ(record->lease_token, "retained-scene-capability");
  }
}

TEST(RosAttachmentSettlement, SendBeforeThrowIsMarkedBeforeTheClientReturnsAFuture)
{
  Harness io;
  io.steps = {{Behavior::kSendThenThrow}, {Behavior::kReply}};
  RosAttachmentPort port(io.config, io.operations());
  io.on_send = [&]() {
    const auto record = RosAttachmentPortTestPeer::retained(port);
    ASSERT_TRUE(record);
    EXPECT_TRUE(record->commit_history.unresolved);
    EXPECT_GE(record->commit_history.submitted, 1U);
  };
  EXPECT_EQ(io.run(port, goal()).outcome, AttachmentOutcome::kIndeterminate);
  const auto record = RosAttachmentPortTestPeer::retained(port);
  ASSERT_TRUE(record);
  EXPECT_EQ(record->commit_history.submitted, 2U);
  EXPECT_EQ(record->commit_history.matched, 1U);
  EXPECT_TRUE(record->commit_history.unresolved);
  EXPECT_EQ(io.removed, 1);  // Throwing send returned no local handle to erase.
  EXPECT_EQ(io.released, 0);
}

TEST(RosAttachmentSettlement, ExactSuccessResolvesEarlierUncertainty)
{
  Harness io;
  io.steps = {{Behavior::kTimeout}, {Behavior::kReply, Status::OK, true}};
  RosAttachmentPort port(io.config, io.operations());
  EXPECT_EQ(io.run(port, goal()).outcome, AttachmentOutcome::kSucceeded);
  EXPECT_EQ(io.released, 1);
  EXPECT_EQ(io.removed, 2);
  const auto record = RosAttachmentPortTestPeer::retained(port);
  ASSERT_TRUE(record);
  EXPECT_FALSE(record->commit_history.unresolved);
  EXPECT_EQ(record->historical_receipt->world_revision, 73U);
}

TEST(RosAttachmentSettlement, ClockInhibitionPreservesHistoricalReceiptAndNeverReleases)
{
  for (const bool receipt : {false, true}) {
    Harness io;
    io.steps = {{Behavior::kTimeout},
      {Behavior::kReply, Status::CLOCK_AUTHORITY_INHIBITED, receipt},
      {Behavior::kReply, Status::OK, true}};
    RosAttachmentPort port(io.config, io.operations());
    const auto result = io.run(port, goal());
    EXPECT_EQ(result.outcome, AttachmentOutcome::kIndeterminate);
    EXPECT_EQ(attachment_world_effect(result.outcome), AttachmentWorldEffect::kIndeterminate);
    EXPECT_EQ(result.world_revision, 0U);
    EXPECT_EQ(io.requests.size(), 2U);
    EXPECT_EQ(io.released, 0);
    const auto record = RosAttachmentPortTestPeer::retained(port);
    ASSERT_TRUE(record);
    EXPECT_TRUE(record->commit_history.inhibited);
    EXPECT_EQ(static_cast<bool>(record->historical_receipt), receipt);
    if (receipt) {
      EXPECT_EQ(record->historical_receipt->world_revision, 73U);
      EXPECT_EQ(record->historical_receipt->reservation.reservation_id, 31U);
    }
  }
}

TEST(RosAttachmentSettlement, OriginalDeadlineCapsEveryRpcAndPollEvenWithPausedRosTime)
{
  Harness io;
  io.steps = {{Behavior::kReply}, {Behavior::kTimeout}, {Behavior::kReply},
    {Behavior::kTimeout}, {Behavior::kReply}, {Behavior::kTimeout}};
  RosAttachmentPort port(io.config, io.operations());
  auto request = goal();
  request.commit_timeout = 75ms;
  const auto deadline = io.now + request.commit_timeout;
  EXPECT_EQ(io.run(port, request).outcome, AttachmentOutcome::kIndeterminate);
  for (const auto observed : io.semantic_deadlines) {
    EXPECT_EQ(observed, deadline);
  }
  for (const auto observed : io.wait_deadlines) {
    EXPECT_LE(observed, deadline);
  }
  for (const auto observed : io.polls) {
    EXPECT_LE(observed, deadline);
  }
  EXPECT_EQ(io.now, deadline);
  EXPECT_EQ(io.physical_sends, 1);
  EXPECT_EQ(io.released, 0);
}

TEST(RosAttachmentSettlement, ContinuingReadinessFrontierCannotRenewTheOriginalBudget)
{
  Harness io;
  for (int index = 0; index < 30; ++index) {
    io.steps.push_back({Behavior::kReply});
  }
  RosAttachmentPort port(io.config, io.operations());
  const auto deadline = io.now + goal().commit_timeout;
  EXPECT_EQ(io.run(port, goal()).outcome, AttachmentOutcome::kUncommitted);
  EXPECT_EQ(io.now, deadline);
  EXPECT_FALSE(io.steps.empty());
  EXPECT_EQ(io.physical_sends, 1);
  EXPECT_EQ(io.released, 0);
  for (const auto & request : io.requests) {
    EXPECT_EQ(request.get(), io.requests.front().get());
  }
}

TEST(RosAttachmentSettlement, ReadinessCheckCrossingDeadlineDoesNotSend)
{
  Harness io;
  io.steps = {{Behavior::kReply, Status::OK, true, 120ms}};
  RosAttachmentPort port(io.config, io.operations());
  EXPECT_EQ(io.run(port, goal()).outcome, AttachmentOutcome::kUncommitted);
  EXPECT_TRUE(io.requests.empty());
  EXPECT_EQ(io.released, 0);
}

TEST(RosAttachmentSettlement, LateSuccessFromSendOrResponseProcessingIsEvidenceOnly)
{
  for (const bool delay_send : {false, true}) {
    Harness io;
    Step step{Behavior::kReply, Status::OK, true};
    (delay_send ? step.send_delay : step.get_delay) = 120ms;
    io.steps.push_back(step);
    RosAttachmentPort port(io.config, io.operations());
    const auto result = io.run(port, goal());
    EXPECT_EQ(result.outcome, AttachmentOutcome::kIndeterminate);
    EXPECT_EQ(io.released, 0);
    const auto record = RosAttachmentPortTestPeer::retained(port);
    ASSERT_TRUE(record);
    EXPECT_TRUE(record->unsafe_terminal);
    ASSERT_TRUE(record->historical_receipt);
    EXPECT_EQ(record->historical_receipt->world_revision, 73U);
  }
}

TEST(RosAttachmentSettlement, CancellationAfterPhysicalSendNeverReportsCleanCancellation)
{
  for (const auto behavior : {Behavior::kReply, Behavior::kTimeout}) {
    Harness io;
    io.steps = {{behavior}};
    RosAttachmentPort port(io.config, io.operations());
    io.on_send = [&]() {port.cancel();};
    const auto result = io.run(port, goal());
    EXPECT_EQ(
      result.outcome, behavior == Behavior::kReply ?
      AttachmentOutcome::kUncommitted : AttachmentOutcome::kIndeterminate);
    EXPECT_EQ(io.released, 0);
    EXPECT_EQ(io.physical_sends, 1);
  }
}

TEST(RosAttachmentSettlement, ShutdownJoinsWorkerAndPreservesUnsafeTerminalRecord)
{
  Harness io;
  io.steps = {{Behavior::kTimeout}};
  io.poll_continues = false;
  std::promise<void> entered_poll;
  auto entered = entered_poll.get_future();
  std::promise<void> unblock_poll;
  auto unblock = unblock_poll.get_future();
  io.on_poll = [&]() {
    entered_poll.set_value();
    unblock.wait();
  };
  RosAttachmentPort port(io.config, io.operations());
  std::promise<AttachmentCompletion> completed;
  auto result = completed.get_future();
  ASSERT_TRUE(
    port.submit(
      {1, 1}, goal(), [&](auto completion) {
        completed.set_value(std::move(completion));
      }));
  // Release the worker on every path so an assertion cannot strand shutdown's join.
  const bool reached_poll = entered.wait_for(std::chrono::seconds(2)) == std::future_status::ready;
  if (!reached_poll) {
    unblock_poll.set_value();
    FAIL() << "worker did not reach controlled poll boundary";
  }
  std::thread shutdown([&]() {port.shutdown();});
  const bool stopping = RosAttachmentPortTestPeer::wait_for_shutdown_request(port);
  unblock_poll.set_value();
  shutdown.join();
  EXPECT_TRUE(stopping);
  ASSERT_EQ(result.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  EXPECT_EQ(result.get().outcome, AttachmentOutcome::kIndeterminate);
  EXPECT_EQ(io.released, 0);
  const auto record = RosAttachmentPortTestPeer::retained(port);
  ASSERT_TRUE(record);
  EXPECT_TRUE(record->unsafe_terminal);
  EXPECT_TRUE(record->commit_history.unresolved);
  EXPECT_TRUE(record->physical_receipt);
}

TEST(RosAttachmentSettlement, UnsafeRecordBlocksBothAttachAndDetachReplacement)
{
  Harness io;
  io.steps = {{Behavior::kReply}};
  RosAttachmentPort port(io.config, io.operations());
  EXPECT_EQ(io.run(port, goal()).outcome, AttachmentOutcome::kUncommitted);
  const auto original = RosAttachmentPortTestPeer::retained(port);
  ASSERT_TRUE(original);
  for (const auto direction : {AttachmentDirection::kAttach, AttachmentDirection::kDetach}) {
    auto next = goal();
    next.direction = direction;
    next.grasp_center_to_child.setIdentity();
    EXPECT_EQ(port.submit({2, 2}, next, [](auto) {}).status, AttachmentSubmitStatus::kUnavailable);
  }
  const auto retained = RosAttachmentPortTestPeer::retained(port);
  ASSERT_TRUE(retained);
  EXPECT_EQ(retained->operation_id, original->operation_id);
  EXPECT_EQ(retained->lease_token, original->lease_token);
  EXPECT_EQ(retained->physical_receipt, original->physical_receipt);
  EXPECT_EQ(io.physical_sends, 1);
}

TEST(RosAttachmentSettlement, ThrowingCompletionSinkCannotDiscardUnsafeEvidence)
{
  Harness io;
  io.steps = {{Behavior::kReply, Status::CLOCK_AUTHORITY_INHIBITED, true}};
  RosAttachmentPort port(io.config, io.operations());
  std::promise<AttachmentCompletion> delivered;
  auto result = delivered.get_future();
  ASSERT_TRUE(
    port.submit(
      {1, 1}, goal(), [&](auto completion) {
        delivered.set_value(std::move(completion));
        throw std::runtime_error("completion sink failed");
      }));
  ASSERT_EQ(result.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  EXPECT_EQ(result.get().outcome, AttachmentOutcome::kIndeterminate);
  EXPECT_EQ(port.submit({2, 2}, goal(), [](auto) {}).status, AttachmentSubmitStatus::kUnavailable);
  port.shutdown();
  const auto record = RosAttachmentPortTestPeer::retained(port);
  ASSERT_TRUE(record);
  ASSERT_TRUE(record->historical_receipt);
  EXPECT_EQ(record->historical_receipt->world_revision, 73U);
  EXPECT_TRUE(record->unsafe_terminal);
  EXPECT_EQ(io.released, 0);
}

TEST(RosAttachmentSettlement, SafeSettlementAllowsNextRecordButLostReleaseIsIndependent)
{
  Harness io;
  io.release_status =
    restocker_interfaces::msg::PlanningSceneLeaseOperationStatus::INVALID_ARGUMENT;
  io.steps = {{Behavior::kReply, Status::OK, true}, {Behavior::kReply, Status::OK, true}};
  RosAttachmentPort port(io.config, io.operations());
  const auto first = io.run(port, goal());
  EXPECT_EQ(first.outcome, AttachmentOutcome::kSucceeded);
  EXPECT_FALSE(first.planning_scene_lease_released);
  EXPECT_EQ(io.run(port, goal(), {2, 2}).outcome, AttachmentOutcome::kSucceeded);
  ASSERT_EQ(io.requests.size(), 2U);
  EXPECT_NE(io.requests[0]->operation_id, io.requests[1]->operation_id);
  EXPECT_EQ(io.physical_sends, 2);
}

AttachmentGoal detach_goal(DetachDisposition disposition)
{
  auto request = goal();
  request.direction = AttachmentDirection::kDetach;
  request.grasp_center_to_child.setIdentity();
  request.detach_disposition = disposition;
  return request;
}

TEST(RosAttachmentDetach, FullDetachKeepsDispositionStampAndReleaseRevision)
{
  for (const auto disposition : {DetachDisposition::kPlaceInReservedDestination,
      DetachDisposition::kReleaseWithoutMembership})
  {
    Harness io;
    io.detach_codes = {Status::OK};
    RosAttachmentPort port(io.config, io.operations());
    const auto result = io.run(port, detach_goal(disposition));
    port.shutdown();
    EXPECT_EQ(result.outcome, AttachmentOutcome::kSucceeded);
    EXPECT_EQ(result.world_revision, 83U);
    EXPECT_TRUE(result.planning_scene_lease_released);
    EXPECT_EQ(io.acquired, 1);
    EXPECT_EQ(io.physical_sends, 1);
    EXPECT_EQ(io.physical_request->command, attachment_test::Physical::Request::COMMAND_DETACH);
    EXPECT_EQ(io.detach_sends, 1);
    EXPECT_TRUE(io.requests.empty());
    EXPECT_EQ(io.completions.load(), 1U);
    EXPECT_EQ(io.released, 1);
    ASSERT_EQ(io.detach_requests.size(), 1U);
    const auto & semantic = *io.detach_requests.front();
    const auto expected_disposition = disposition ==
      DetachDisposition::kPlaceInReservedDestination ?
      attachment_test::Detach::Request::PLACE_IN_RESERVED_DESTINATION :
      attachment_test::Detach::Request::RELEASE_WITHOUT_MEMBERSHIP;
    EXPECT_EQ(semantic.disposition, expected_disposition);
    EXPECT_EQ(semantic.released_at.sec, 9);
    EXPECT_EQ(semantic.released_at.nanosec, 650000000U);
    EXPECT_EQ(semantic.token, io.physical_request->reservation_token);
    EXPECT_EQ(semantic.operation_id, io.physical_request->operation_id + "-commit");
    EXPECT_EQ(io.release_request->token, io.physical_request->planning_scene_lease_token);
    EXPECT_EQ(io.release_request->required_semantic_revision, 83U);
    EXPECT_FALSE(RosAttachmentPortTestPeer::retained(port));
  }
}

TEST(RosAttachmentDetach, SplitPlacementReusesPhysicalLeaseAndStampWhileEvidenceSettles)
{
  Harness io;
  io.detach_codes = {Status::PREDICATE_FAILED, Status::OK};
  RosAttachmentPort port(io.config, io.operations());
  auto physical = detach_goal(DetachDisposition::kPlaceInReservedDestination);
  physical.scope = AttachmentScope::kPhysicalOnly;
  const auto first = io.run(port, physical);
  ASSERT_EQ(first.outcome, AttachmentOutcome::kSucceeded);
  EXPECT_FALSE(first.planning_scene_lease_released);
  EXPECT_EQ(first.world_revision, 0U);
  EXPECT_EQ(first.retained_lease_token, "retained-scene-capability");
  EXPECT_EQ(first.released_at.sec, 9);
  EXPECT_EQ(first.released_at.nanosec, 650000000U);
  EXPECT_EQ(io.acquired, 1);
  EXPECT_EQ(io.physical_sends, 1);
  EXPECT_EQ(io.detach_sends, 0);
  EXPECT_EQ(io.released, 0);
  EXPECT_EQ(io.completions.load(), 1U);

  auto semantic = detach_goal(DetachDisposition::kPlaceInReservedDestination);
  semantic.scope = AttachmentScope::kSemanticOnly;
  semantic.planning_scene_lease_token = first.retained_lease_token;
  semantic.released_at = first.released_at;
  const auto second = io.run(port, semantic, {2, 2});
  port.shutdown();
  EXPECT_EQ(second.outcome, AttachmentOutcome::kSucceeded);
  EXPECT_EQ(second.world_revision, 83U);
  EXPECT_TRUE(second.planning_scene_lease_released);
  EXPECT_EQ(io.acquired, 1);
  EXPECT_EQ(io.physical_sends, 1);
  EXPECT_EQ(io.detach_sends, 2);
  EXPECT_TRUE(io.requests.empty());
  EXPECT_EQ(io.completions.load(), 2U);
  EXPECT_EQ(io.released, 1);
  EXPECT_EQ(io.release_request->token, first.retained_lease_token);
  EXPECT_EQ(io.release_request->required_semantic_revision, 83U);
  ASSERT_EQ(io.detach_requests.size(), 2U);
  EXPECT_EQ(io.detach_requests[0]->operation_id, io.detach_requests[1]->operation_id);
  for (const auto & request : io.detach_requests) {
    EXPECT_EQ(
      request->disposition,
      attachment_test::Detach::Request::PLACE_IN_RESERVED_DESTINATION);
    EXPECT_EQ(request->released_at.sec, first.released_at.sec);
    EXPECT_EQ(request->released_at.nanosec, first.released_at.nanosec);
    EXPECT_EQ(request->token, semantic.reservation_token);
  }
  EXPECT_FALSE(io.polls.empty());
  EXPECT_FALSE(RosAttachmentPortTestPeer::retained(port));
}

TEST(RosAttachmentDetach, ReleaseWithoutMembershipDoesNotRetryPlacementRefusal)
{
  Harness io;
  io.detach_codes = {Status::PREDICATE_FAILED, Status::OK};
  RosAttachmentPort port(io.config, io.operations());
  const auto result = io.run(port, detach_goal(DetachDisposition::kReleaseWithoutMembership));
  port.shutdown();
  EXPECT_EQ(result.outcome, AttachmentOutcome::kUncommitted);
  EXPECT_FALSE(result.planning_scene_lease_released);
  EXPECT_EQ(io.physical_sends, 1);
  EXPECT_EQ(io.detach_sends, 1);
  EXPECT_EQ(io.detach_codes.size(), 1U);
  EXPECT_EQ(io.released, 0);
  EXPECT_EQ(io.completions.load(), 1U);
  EXPECT_TRUE(io.requests.empty());
}

TEST(RosAttachmentTerminal, ReleaseExceptionPreservesSemanticSuccessAndWorkerInBothDirections)
{
  for (const auto direction : {AttachmentDirection::kAttach, AttachmentDirection::kDetach}) {
    Harness io;
    io.steps = {{Behavior::kReply, Status::OK, true}, {Behavior::kReply, Status::OK, true}};
    io.detach_codes = {Status::OK, Status::OK};
    io.on_release = []() {
      throw std::runtime_error("release binding failed after semantic success");
    };
    RosAttachmentPort port(io.config, io.operations());
    const auto request = direction == AttachmentDirection::kAttach ? goal() :
      detach_goal(DetachDisposition::kReleaseWithoutMembership);
    const auto first = io.run(port, request);
    EXPECT_EQ(first.outcome, AttachmentOutcome::kSucceeded);
    EXPECT_EQ(first.world_revision, direction == AttachmentDirection::kAttach ? 73U : 83U);
    EXPECT_FALSE(first.planning_scene_lease_released);
    EXPECT_EQ(io.physical_sends, 1);
    EXPECT_EQ(io.released, 1);
    EXPECT_EQ(io.completions.load(), 1U);

    // An exception at terminal lease disposition must not kill the worker or duplicate a command.
    // A second authorized transaction here proves worker liveness, not remote lease settlement.
    io.on_release = {};
    const auto second = io.run(port, request, {2, 2});
    port.shutdown();
    EXPECT_EQ(second.outcome, AttachmentOutcome::kSucceeded);
    EXPECT_TRUE(second.planning_scene_lease_released);
    EXPECT_EQ(io.physical_sends, 2);
    EXPECT_EQ(io.released, 2);
    EXPECT_EQ(io.completions.load(), 2U);
    EXPECT_EQ(io.requests.size(), direction == AttachmentDirection::kAttach ? 2U : 0U);
    EXPECT_EQ(io.detach_sends, direction == AttachmentDirection::kDetach ? 2 : 0);
  }
}

TEST(RosAttachmentDetach, PostPhysicalSemanticBindingExceptionIsIndeterminateWithoutRelease)
{
  Harness io;
  io.on_detach = []() {
    throw std::runtime_error("detach semantic I/O failed after physical detach");
  };
  RosAttachmentPort port(io.config, io.operations());
  const auto result = io.run(port, detach_goal(DetachDisposition::kPlaceInReservedDestination));
  port.shutdown();
  EXPECT_EQ(result.outcome, AttachmentOutcome::kIndeterminate);
  EXPECT_TRUE(attachment_requires_operator(result.outcome));
  EXPECT_FALSE(attachment_definitely_not_applied(result.outcome));
  EXPECT_FALSE(result.planning_scene_lease_released);
  EXPECT_EQ(io.acquired, 1);
  EXPECT_EQ(io.physical_sends, 1);
  EXPECT_EQ(io.detach_sends, 1);
  EXPECT_EQ(io.released, 0);
  EXPECT_EQ(io.completions.load(), 1U);
  EXPECT_TRUE(io.requests.empty());
  // Detach still has its legacy stack-local lifetime; this test does not claim retained recovery.
  EXPECT_FALSE(RosAttachmentPortTestPeer::retained(port));
}

}  // namespace
}  // namespace restocker_task_executor
