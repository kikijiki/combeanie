// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <atomic>
#include <cstddef>
#include <string>
#include <thread>
#include <vector>

#include "restocker_gazebo/attachment_mailbox.hpp"

namespace restocker_gazebo
{
namespace
{

msgs::SetAttachmentRequest attach_message(std::string operation_id = "attach-17")
{
  msgs::SetAttachmentRequest result;
  result.set_expected_simulator_epoch("sim-epoch-a");
  result.set_operation_id(std::move(operation_id));
  auto * request = result.mutable_request();
  request->set_command(msgs::ATTACHMENT_COMMAND_ATTACH);
  request->set_reservation_id(81);
  auto * identity = request->mutable_identity();
  identity->set_object_id(17);
  identity->set_object_source_id("sim:stock_can_01");
  identity->set_parent_model("restocker");
  identity->set_parent_link("gripper");
  identity->set_child_model("stock_can_01");
  identity->set_child_link("product_body");
  request->mutable_expected_grasp_center_to_child()->set_qw(1.0);
  return result;
}

msgs::QueryAttachmentRequest query_message(std::string operation_id = {})
{
  msgs::QueryAttachmentRequest result;
  result.set_expected_simulator_epoch("sim-epoch-a");
  result.set_operation_id(std::move(operation_id));
  return result;
}

AttachmentJournalReply decode(const msgs::AttachmentReply & message)
{
  AttachmentStatus error;
  const auto result = decode_reply(message, error);
  EXPECT_TRUE(result) << error.detail;
  return result.value_or(AttachmentJournalReply{});
}

AttachmentEvidence evidence()
{
  AttachmentEvidence result;
  result.joint_observed = true;
  result.simulator_iteration = 20;
  result.simulation_time_ns = 1'000'000;
  return result;
}

TEST(AttachmentMailbox, RejectsMalformedAndStaleEpochCommandsWithoutQueueing)
{
  AttachmentMailbox mailbox("sim-epoch-a", 8);
  auto malformed = attach_message();
  malformed.clear_operation_id();
  EXPECT_EQ(
    decode(mailbox.handle_set(malformed)).status.code,
    AttachmentStatusCode::kInvalidArgument);

  auto stale = attach_message();
  stale.set_expected_simulator_epoch("old-epoch");
  const auto rejected = decode(mailbox.handle_set(stale));
  EXPECT_EQ(rejected.status.code, AttachmentStatusCode::kSimulatorEpochChanged);
  EXPECT_EQ(rejected.state.motion_gate, AttachmentMotionGate::kValid);
  EXPECT_FALSE(mailbox.take_pending());
}

TEST(AttachmentMailbox, ExactReplayQueuesOneImmutableUpdateThreadCommand)
{
  AttachmentMailbox mailbox("sim-epoch-a", 8);
  const auto accepted = decode(mailbox.handle_set(attach_message()));
  EXPECT_EQ(accepted.status.code, AttachmentStatusCode::kPending);
  EXPECT_FALSE(accepted.replayed);

  const auto replay = decode(mailbox.handle_set(attach_message()));
  EXPECT_EQ(replay.status.code, AttachmentStatusCode::kPending);
  EXPECT_TRUE(replay.replayed);

  const auto pending = mailbox.take_pending();
  ASSERT_TRUE(pending);
  EXPECT_EQ(pending->operation_id, "attach-17");
  EXPECT_EQ(pending->request.identity.child_model, "stock_can_01");
  EXPECT_FALSE(mailbox.take_pending());
}

TEST(AttachmentMailbox, UpdateThreadTransitionsArePollableThroughTransportQuery)
{
  AttachmentMailbox mailbox("sim-epoch-a", 8);
  ASSERT_EQ(
    decode(mailbox.handle_set(attach_message())).status.code,
    AttachmentStatusCode::kPending);
  ASSERT_TRUE(mailbox.take_pending());
  ASSERT_EQ(
    mailbox.mark_validation_succeeded("attach-17").status.code,
    AttachmentStatusCode::kPending);
  ASSERT_EQ(
    mailbox.mark_mutation_started("attach-17").status.code,
    AttachmentStatusCode::kPending);
  ASSERT_EQ(
    mailbox.mark_verification_succeeded("attach-17", evidence()).status.code,
    AttachmentStatusCode::kAttached);

  const auto query = decode(mailbox.handle_query(query_message("attach-17")));
  EXPECT_TRUE(query.replayed);
  EXPECT_EQ(query.status.code, AttachmentStatusCode::kAttached);
  EXPECT_EQ(query.state.phase, AttachmentPhase::kAttached);
  ASSERT_TRUE(query.operation);
  ASSERT_TRUE(query.operation->evidence);
  EXPECT_EQ(query.operation->evidence->simulator_iteration, 20U);
}

TEST(AttachmentMailbox, ConcurrentReplaysStillProduceOnePhysicalWorkItem)
{
  AttachmentMailbox mailbox("sim-epoch-a", 8);
  constexpr std::size_t kThreadCount = 16;
  std::atomic<std::size_t> accepted{0};
  std::vector<std::thread> threads;
  threads.reserve(kThreadCount);
  for (std::size_t index = 0; index < kThreadCount; ++index) {
    threads.emplace_back(
      [&mailbox, &accepted]() {
        AttachmentStatus error;
        const auto reply = decode_reply(mailbox.handle_set(attach_message()), error);
        if (reply && reply->status.code == AttachmentStatusCode::kPending) {
          ++accepted;
        }
      });
  }
  for (auto & thread : threads) {
    thread.join();
  }
  EXPECT_EQ(accepted, kThreadCount);
  EXPECT_TRUE(mailbox.take_pending());
  EXPECT_FALSE(mailbox.take_pending());
}

TEST(AttachmentMailbox, SerializesDifferentOperationIdsBehindTheActiveMutation)
{
  AttachmentMailbox mailbox("sim-epoch-a", 8);
  EXPECT_EQ(
    decode(mailbox.handle_set(attach_message("attach-first"))).status.code,
    AttachmentStatusCode::kPending);
  EXPECT_EQ(
    decode(mailbox.handle_set(attach_message("attach-second"))).status.code,
    AttachmentStatusCode::kConflict);

  const auto pending = mailbox.take_pending();
  ASSERT_TRUE(pending);
  EXPECT_EQ(pending->operation_id, "attach-first");
  EXPECT_FALSE(mailbox.take_pending());
}

TEST(AttachmentMailbox, EpochRotationDropsQueuedWorkAndRejectsOldQueries)
{
  AttachmentMailbox mailbox("sim-epoch-a", 8);
  ASSERT_EQ(
    decode(mailbox.handle_set(attach_message())).status.code,
    AttachmentStatusCode::kPending);
  mailbox.rotate_epoch("sim-epoch-b");
  EXPECT_FALSE(mailbox.take_pending());
  EXPECT_EQ(mailbox.simulator_epoch(), "sim-epoch-b");
  EXPECT_EQ(mailbox.state().motion_gate, AttachmentMotionGate::kInhibited);

  const auto stale_query = decode(mailbox.handle_query(query_message("attach-17")));
  EXPECT_EQ(stale_query.status.code, AttachmentStatusCode::kSimulatorEpochChanged);
  EXPECT_EQ(stale_query.state.simulator_epoch, "sim-epoch-b");
}

TEST(AttachmentMailbox, WatchdogFaultIsImmediatelyVisibleToTransportQueries)
{
  AttachmentMailbox mailbox("sim-epoch-a", 8);
  ASSERT_EQ(
    decode(mailbox.handle_set(attach_message())).status.code,
    AttachmentStatusCode::kPending);
  ASSERT_TRUE(mailbox.take_pending());
  ASSERT_EQ(
    mailbox.mark_validation_succeeded("attach-17").status.code,
    AttachmentStatusCode::kPending);
  ASSERT_EQ(
    mailbox.mark_mutation_started("attach-17").status.code,
    AttachmentStatusCode::kPending);
  ASSERT_EQ(
    mailbox.mark_verification_succeeded("attach-17", evidence()).status.code,
    AttachmentStatusCode::kAttached);

  ASSERT_EQ(
    mailbox.mark_external_inconsistency("held object drifted").status.code,
    AttachmentStatusCode::kExternalInconsistency);
  const auto query = decode(mailbox.handle_query(query_message()));
  EXPECT_EQ(query.status.code, AttachmentStatusCode::kExternalInconsistency);
  EXPECT_EQ(query.state.phase, AttachmentPhase::kInconsistent);
  EXPECT_EQ(query.state.motion_gate, AttachmentMotionGate::kInhibited);
}

}  // namespace
}  // namespace restocker_gazebo
