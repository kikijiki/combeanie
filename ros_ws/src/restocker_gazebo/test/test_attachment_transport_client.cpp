// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>

#include <gz/transport/Node.hh>

#include "restocker_gazebo/attachment_protocol.hpp"
#include "restocker_gazebo/attachment_transport_client.hpp"

namespace restocker_gazebo
{
namespace
{

using namespace std::chrono_literals;

AttachmentRequest attach_request()
{
  AttachmentRequest request;
  request.command = AttachmentCommand::kAttach;
  request.reservation_id = 41;
  request.identity = AttachmentIdentity{
    7, "sim:stock_can_01", "restocker", "gripper", "stock_can_01", "product_body"};
  request.has_expected_grasp = true;
  request.expected_grasp_center_to_child.rotation_xyzw = {0.0, 0.0, 0.0, 1.0};
  return request;
}

AttachmentJournalReply detached_reply(AttachmentStatusCode code = AttachmentStatusCode::kDetached)
{
  AttachmentJournalReply reply;
  reply.status = AttachmentStatus{code, "transport result"};
  reply.state.simulator_epoch = "sim-epoch";
  reply.state.sequence = 1;
  reply.state.phase = AttachmentPhase::kDetached;
  reply.state.motion_gate = AttachmentMotionGate::kValid;
  reply.state.status = reply.status;
  return reply;
}

class TransportServer
{
public:
  TransportServer()
  {
    static std::atomic<std::uint64_t> sequence{0};
    const std::string suffix = std::to_string(sequence.fetch_add(1));
    set_service = "/attachment_transport_test/set_" + suffix;
    query_service = "/attachment_transport_test/query_" + suffix;
  }

  bool advertise_valid()
  {
    return node.Advertise(set_service, &TransportServer::on_set, this) &&
           node.Advertise(query_service, &TransportServer::on_query, this);
  }

  bool advertise_malformed_query()
  {
    malformed_query = true;
    return node.Advertise(query_service, &TransportServer::on_query, this);
  }

  bool advertise_rejected_set()
  {
    reject_set = true;
    return node.Advertise(set_service, &TransportServer::on_set, this);
  }

  bool on_set(const msgs::SetAttachmentRequest & request, msgs::AttachmentReply & reply)
  {
    AttachmentStatus error;
    command = decode_command(request, error);
    reply = encode_reply(detached_reply(AttachmentStatusCode::kPending));
    return !reject_set;
  }

  bool on_query(const msgs::QueryAttachmentRequest & request, msgs::AttachmentReply & reply)
  {
    AttachmentStatus error;
    query = decode_query(request, error);
    if (malformed_query) {
      reply.mutable_status()->set_code(msgs::ATTACHMENT_STATUS_DETACHED);
    } else {
      reply = encode_reply(detached_reply());
    }
    return true;
  }

  gz::transport::Node node;
  std::string set_service;
  std::string query_service;
  std::optional<DecodedAttachmentCommand> command;
  std::optional<DecodedAttachmentQuery> query;
  bool malformed_query{false};
  bool reject_set{false};
};

TEST(AttachmentTransportClientTest, RejectsInvalidConstruction)
{
  EXPECT_THROW(AttachmentTransportClient("", "/query", 10ms), std::invalid_argument);
  EXPECT_THROW(AttachmentTransportClient("/set", "/query", 0ms), std::invalid_argument);
}

TEST(AttachmentTransportClientTest, SendsExactTokenFreeTypedCommandAndQuery)
{
  TransportServer server;
  ASSERT_TRUE(server.advertise_valid());
  AttachmentTransportClient client(server.set_service, server.query_service, 500ms);

  const auto command = client.command("sim-epoch", "attach-1", attach_request());
  ASSERT_EQ(command.outcome, AttachmentTransportOutcome::kReply);
  ASSERT_TRUE(command.reply);
  EXPECT_EQ(command.status.code, AttachmentStatusCode::kPending);
  ASSERT_TRUE(server.command);
  EXPECT_EQ(server.command->expected_simulator_epoch, "sim-epoch");
  EXPECT_EQ(server.command->operation_id, "attach-1");
  EXPECT_EQ(server.command->request, attach_request());

  const auto query = client.query("sim-epoch", "attach-1");
  ASSERT_EQ(query.outcome, AttachmentTransportOutcome::kReply);
  EXPECT_EQ(query.status.code, AttachmentStatusCode::kDetached);
  ASSERT_TRUE(server.query);
  EXPECT_EQ(server.query->expected_simulator_epoch, "sim-epoch");
  EXPECT_EQ(server.query->operation_id, "attach-1");
}

TEST(AttachmentTransportClientTest, ClassifiesMalformedReplyAsExternalInconsistency)
{
  TransportServer server;
  ASSERT_TRUE(server.advertise_malformed_query());
  AttachmentTransportClient client(server.set_service, server.query_service, 500ms);
  const auto result = client.query("sim-epoch");
  EXPECT_EQ(result.outcome, AttachmentTransportOutcome::kMalformedReply);
  EXPECT_EQ(result.status.code, AttachmentStatusCode::kExternalInconsistency);
  EXPECT_FALSE(result.reply);
}

TEST(AttachmentTransportClientTest, RejectedCommandRemainsOutcomeUnknown)
{
  TransportServer server;
  ASSERT_TRUE(server.advertise_rejected_set());
  AttachmentTransportClient client(server.set_service, server.query_service, 500ms);
  const auto result = client.command("sim-epoch", "attach-1", attach_request());
  EXPECT_EQ(result.outcome, AttachmentTransportOutcome::kServiceRejected);
  EXPECT_EQ(result.status.code, AttachmentStatusCode::kOutcomeUnknown);
  EXPECT_FALSE(result.reply);
}

TEST(AttachmentTransportClientTest, MissingServiceRemainsOutcomeUnknown)
{
  TransportServer names;
  AttachmentTransportClient client(names.set_service, names.query_service, 10ms);
  const auto result = client.query("sim-epoch");
  EXPECT_EQ(result.outcome, AttachmentTransportOutcome::kNoReply);
  EXPECT_EQ(result.status.code, AttachmentStatusCode::kOutcomeUnknown);
  EXPECT_FALSE(result.reply);
}

}  // namespace
}  // namespace restocker_gazebo
