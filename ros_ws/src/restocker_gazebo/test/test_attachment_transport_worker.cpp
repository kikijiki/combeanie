// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <thread>

#include <gz/transport/Node.hh>

#include "restocker_gazebo/attachment_protocol.hpp"
#include "restocker_gazebo/attachment_transport_worker.hpp"

namespace restocker_gazebo
{
namespace
{

using namespace std::chrono_literals;

AttachmentJournalReply detached_reply()
{
  AttachmentJournalReply reply;
  reply.status = AttachmentStatus{AttachmentStatusCode::kDetached, "detached"};
  reply.state.simulator_epoch = "sim-epoch";
  reply.state.sequence = 1;
  reply.state.phase = AttachmentPhase::kDetached;
  reply.state.motion_gate = AttachmentMotionGate::kValid;
  reply.state.status = reply.status;
  return reply;
}

class QueryServer
{
public:
  explicit QueryServer(bool slow = false)
  : slow_(slow)
  {
    static std::atomic<std::uint64_t> sequence{0};
    const std::string suffix = std::to_string(sequence.fetch_add(1));
    set_service = "/attachment_worker_test/set_" + suffix;
    query_service = "/attachment_worker_test/query_" + suffix;
  }

  bool advertise()
  {
    return node.Advertise(query_service, &QueryServer::on_query, this);
  }

  bool on_query(const msgs::QueryAttachmentRequest & request, msgs::AttachmentReply & reply)
  {
    if (slow_) {
      std::this_thread::sleep_for(80ms);
    }
    AttachmentStatus error;
    decoded = decode_query(request, error);
    reply = encode_reply(detached_reply());
    return true;
  }

  gz::transport::Node node;
  std::string set_service;
  std::string query_service;
  std::optional<DecodedAttachmentQuery> decoded;

private:
  bool slow_;
};

std::optional<AttachmentTransportCompletion> await_completion(
  AttachmentTransportWorker & worker,
  std::chrono::milliseconds timeout = 1s)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (auto completion = worker.take_completion()) {
      return completion;
    }
    std::this_thread::sleep_for(2ms);
  }
  return std::nullopt;
}

TEST(AttachmentTransportWorkerTest, ExecutesQueryOutsideCallerAndReturnsTypedCompletion)
{
  QueryServer server;
  ASSERT_TRUE(server.advertise());
  AttachmentTransportWorker worker(server.set_service, server.query_service, 500ms);
  ASSERT_TRUE(worker.submit_query("sim-epoch", "operation-1"));
  const auto completion = await_completion(worker);
  ASSERT_TRUE(completion);
  EXPECT_EQ(completion->kind, AttachmentTransportJobKind::kQuery);
  EXPECT_EQ(completion->operation_id, "operation-1");
  EXPECT_EQ(completion->result.outcome, AttachmentTransportOutcome::kReply);
  ASSERT_TRUE(completion->result.reply);
  EXPECT_EQ(completion->result.reply->status.code, AttachmentStatusCode::kDetached);
  ASSERT_TRUE(server.decoded);
  EXPECT_EQ(server.decoded->expected_simulator_epoch, "sim-epoch");
  EXPECT_EQ(server.decoded->operation_id, "operation-1");
  EXPECT_FALSE(worker.busy());
}

TEST(AttachmentTransportWorkerTest, RejectsConcurrentAndUnconsumedJobs)
{
  QueryServer server(true);
  ASSERT_TRUE(server.advertise());
  AttachmentTransportWorker worker(server.set_service, server.query_service, 500ms);
  ASSERT_TRUE(worker.submit_query("sim-epoch", "operation-1"));
  EXPECT_FALSE(worker.submit_query("sim-epoch", "operation-2"));
  const auto completion = await_completion(worker);
  ASSERT_TRUE(completion);
  EXPECT_TRUE(worker.submit_query("sim-epoch", "operation-2"));
  EXPECT_TRUE(await_completion(worker).has_value());
}

TEST(AttachmentTransportWorkerTest, MissingServiceCompletesAsOutcomeUnknown)
{
  QueryServer names;
  AttachmentTransportWorker worker(names.set_service, names.query_service, 10ms);
  ASSERT_TRUE(worker.submit_query("sim-epoch"));
  const auto completion = await_completion(worker);
  ASSERT_TRUE(completion);
  EXPECT_EQ(completion->result.outcome, AttachmentTransportOutcome::kNoReply);
  EXPECT_EQ(completion->result.status.code, AttachmentStatusCode::kOutcomeUnknown);
}

}  // namespace
}  // namespace restocker_gazebo
