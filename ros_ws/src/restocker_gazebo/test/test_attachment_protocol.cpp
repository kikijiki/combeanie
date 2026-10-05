// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <google/protobuf/descriptor.h>

#include <algorithm>
#include <cstdint>
#include <string>

#include "restocker_gazebo/attachment_journal.hpp"
#include "restocker_gazebo/attachment_protocol.hpp"

namespace restocker_gazebo
{
namespace
{

msgs::SetAttachmentRequest attach_message()
{
  msgs::SetAttachmentRequest result;
  result.set_expected_simulator_epoch("sim-epoch-a");
  result.set_operation_id("attach-17");
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
  auto * pose = request->mutable_expected_grasp_center_to_child();
  pose->set_z(0.14);
  pose->set_qw(-1.0);
  return result;
}

TEST(AttachmentProtocol, DecodesAndCanonicalizesAnAttachCommand)
{
  AttachmentStatus error;
  const auto decoded = decode_command(attach_message(), error);
  ASSERT_TRUE(decoded);
  EXPECT_EQ(error.code, AttachmentStatusCode::kUnset);
  EXPECT_EQ(decoded->expected_simulator_epoch, "sim-epoch-a");
  EXPECT_EQ(decoded->operation_id, "attach-17");
  EXPECT_EQ(decoded->request.command, AttachmentCommand::kAttach);
  EXPECT_EQ(decoded->request.reservation_id, 81U);
  EXPECT_EQ(decoded->request.identity.object_id, 17U);
  EXPECT_TRUE(decoded->request.has_expected_grasp);
  EXPECT_DOUBLE_EQ(decoded->request.expected_grasp_center_to_child.translation[2], 0.14);
  EXPECT_DOUBLE_EQ(decoded->request.expected_grasp_center_to_child.rotation_xyzw[3], 1.0);
}

TEST(AttachmentProtocol, CarriesTheAuthorizingCovarianceToTheBoundary)
{
  // The fidelity budget is sized from the estimate's claim, so the covariance must survive the
  // wire.
  auto message = attach_message();
  auto * request = message.mutable_request();
  for (int index = 0; index < 36; ++index) {
    request->add_expected_pose_covariance(index == 0 ? 4.0e-6 : 0.0);
  }
  AttachmentStatus error;
  const auto decoded = decode_command(message, error);
  ASSERT_TRUE(decoded);
  EXPECT_DOUBLE_EQ(decoded->request.expected_pose_covariance[0], 4.0e-6);
  EXPECT_DOUBLE_EQ(decoded->request.expected_pose_covariance[35], 0.0);

  // Absent is a caller that stated nothing, which the boundary reads as no claim.
  const auto silent = decode_command(attach_message(), error);
  ASSERT_TRUE(silent);
  EXPECT_TRUE(
    std::all_of(
      silent->request.expected_pose_covariance.begin(),
      silent->request.expected_pose_covariance.end(),
      [](double entry) {return entry == 0.0;}));

  // A wrong-length matrix is a malformed caller, not an uncertainty claim.
  auto truncated = attach_message();
  truncated.mutable_request()->add_expected_pose_covariance(1.0);
  EXPECT_FALSE(decode_command(truncated, error));
  EXPECT_EQ(error.code, AttachmentStatusCode::kInvalidArgument);
}

TEST(AttachmentProtocol, CarriesTheFidelityResidualBackOnRefusal)
{
  AttachmentJournal journal("sim-epoch-a", 8);
  AttachmentStatus error;
  const auto decoded = decode_command(attach_message(), error);
  ASSERT_TRUE(decoded);
  ASSERT_EQ(
    journal.submit(decoded->operation_id, decoded->request).status.code,
    AttachmentStatusCode::kPending);

  const AttachmentFidelity fidelity{0.0042, 0.0011, 0.0130, 0.0040, 0.0185};
  const auto refused = journal.reject_before_mutation(
    decoded->operation_id,
    AttachmentStatus{
        AttachmentStatusCode::kOutOfTolerance,
        "observed grasp transform differs from the authorized candidate"},
    fidelity);
  ASSERT_EQ(refused.status.code, AttachmentStatusCode::kOutOfTolerance);
  ASSERT_TRUE(refused.operation);
  ASSERT_TRUE(refused.operation->fidelity);
  EXPECT_EQ(*refused.operation->fidelity, fidelity);
  // Mirrored onto the state so the refusal's residual reaches the adapter's state topic.
  ASSERT_TRUE(refused.state.fidelity);
  EXPECT_EQ(*refused.state.fidelity, fidelity);

  const auto encoded = encode_reply(refused);
  ASSERT_TRUE(encoded.state().has_fidelity());
  EXPECT_DOUBLE_EQ(encoded.state().fidelity().translation_residual_m(), 0.0042);
  EXPECT_DOUBLE_EQ(encoded.state().fidelity().translation_budget_m(), 0.0130);
  const auto round_trip = decode_reply(encoded, error);
  ASSERT_TRUE(round_trip);
  EXPECT_EQ(*round_trip, refused);
}

TEST(AttachmentProtocol, RejectsMissingEpochUnknownCommandAndInvalidPose)
{
  AttachmentStatus error;
  auto message = attach_message();
  message.clear_expected_simulator_epoch();
  EXPECT_FALSE(decode_command(message, error));
  EXPECT_EQ(error.code, AttachmentStatusCode::kInvalidArgument);

  message = attach_message();
  message.mutable_request()->set_command(
    static_cast<msgs::AttachmentCommandValue>(99));
  EXPECT_FALSE(decode_command(message, error));
  EXPECT_EQ(error.code, AttachmentStatusCode::kInvalidArgument);

  message = attach_message();
  message.mutable_request()->mutable_expected_grasp_center_to_child()->set_qw(2.0);
  EXPECT_FALSE(decode_command(message, error));
  EXPECT_EQ(error.code, AttachmentStatusCode::kInvalidArgument);
}

TEST(AttachmentProtocol, QueryRequiresAnExpectedSimulatorEpoch)
{
  AttachmentStatus error;
  msgs::QueryAttachmentRequest message;
  EXPECT_FALSE(decode_query(message, error));
  EXPECT_EQ(error.code, AttachmentStatusCode::kInvalidArgument);

  message.set_expected_simulator_epoch("sim-epoch-a");
  message.set_operation_id("attach-17");
  const auto decoded = decode_query(message, error);
  ASSERT_TRUE(decoded);
  EXPECT_EQ(decoded->expected_simulator_epoch, "sim-epoch-a");
  EXPECT_EQ(decoded->operation_id, "attach-17");
}

TEST(AttachmentProtocol, EncodesPresenceAndEveryPieceOfPhysicalEvidence)
{
  AttachmentJournal journal("sim-epoch-a", 8);
  AttachmentStatus error;
  const auto decoded = decode_command(attach_message(), error);
  ASSERT_TRUE(decoded);
  ASSERT_EQ(
    journal.submit(decoded->operation_id, decoded->request).status.code,
    AttachmentStatusCode::kPending);
  ASSERT_EQ(
    journal.mark_validation_succeeded(decoded->operation_id).status.code,
    AttachmentStatusCode::kPending);
  ASSERT_EQ(
    journal.mark_mutation_started(decoded->operation_id).status.code,
    AttachmentStatusCode::kPending);

  AttachmentEvidence evidence;
  evidence.joint_observed = true;
  evidence.parent_to_child.translation[2] = 0.14;
  evidence.child_pose_in_world.translation = {0.4, -0.6, 0.9};
  evidence.relative_twist_in_parent = {0.1, 0.2, 0.3, -0.1, -0.2, -0.3};
  evidence.simulator_iteration = 123;
  evidence.simulation_time_ns = 456'000'000;
  const auto terminal = journal.mark_verification_succeeded(decoded->operation_id, evidence);
  const auto encoded = encode_reply(terminal);

  EXPECT_EQ(encoded.status().code(), msgs::ATTACHMENT_STATUS_ATTACHED);
  ASSERT_TRUE(encoded.has_operation());
  EXPECT_EQ(encoded.operation().operation_id(), "attach-17");
  EXPECT_TRUE(encoded.operation().has_evidence());
  EXPECT_EQ(encoded.operation().evidence().relative_twist_in_parent_size(), 6);
  EXPECT_EQ(encoded.operation().evidence().simulator_iteration(), 123U);
  ASSERT_TRUE(encoded.has_state());
  EXPECT_EQ(encoded.state().motion_gate(), msgs::ATTACHMENT_MOTION_GATE_VALID);
  ASSERT_TRUE(encoded.state().has_attached_identity());
  EXPECT_EQ(encoded.state().attached_identity().child_model(), "stock_can_01");

  const auto round_trip = decode_reply(encoded, error);
  ASSERT_TRUE(round_trip);
  EXPECT_EQ(*round_trip, terminal);
}

TEST(AttachmentProtocol, RejectsMalformedRepliesAtTheAdapterBoundary)
{
  AttachmentStatus error;
  msgs::AttachmentReply missing;
  EXPECT_FALSE(decode_reply(missing, error));
  EXPECT_EQ(error.code, AttachmentStatusCode::kInvalidArgument);

  AttachmentJournal journal("sim-epoch-a", 8);
  auto encoded = encode_reply(journal.query());
  encoded.mutable_status()->set_code(
    static_cast<msgs::AttachmentStatusCodeValue>(99));
  EXPECT_FALSE(decode_reply(encoded, error));

  encoded = encode_reply(journal.query());
  encoded.mutable_state()->set_motion_gate(
    static_cast<msgs::AttachmentMotionGateValue>(99));
  EXPECT_FALSE(decode_reply(encoded, error));
}

TEST(AttachmentProtocol, GazeboWireDescriptorCannotCarryCapabilityTokens)
{
  const auto * file = msgs::SetAttachmentRequest::descriptor()->file();
  ASSERT_NE(file, nullptr);
  for (int message_index = 0; message_index < file->message_type_count(); ++message_index) {
    const auto * message = file->message_type(message_index);
    for (int field_index = 0; field_index < message->field_count(); ++field_index) {
      const auto name = message->field(field_index)->name();
      EXPECT_EQ(name.find("token"), std::string::npos) << message->full_name() << '.' << name;
      EXPECT_EQ(name.find("capability"), std::string::npos) << message->full_name() << '.' << name;
    }
  }
}

TEST(AttachmentProtocol, EncodesTokenFreeCommandsAndQueries)
{
  AttachmentStatus error;
  const auto source = decode_command(attach_message(), error);
  ASSERT_TRUE(source);
  const AttachmentRequest request = source->request;
  const auto command_message = encode_command("epoch-a", "operation-a", request);
  const auto decoded_command = decode_command(command_message, error);
  ASSERT_TRUE(decoded_command);
  EXPECT_EQ(decoded_command->expected_simulator_epoch, "epoch-a");
  EXPECT_EQ(decoded_command->operation_id, "operation-a");
  EXPECT_EQ(decoded_command->request, request);

  const auto query_message = encode_query("epoch-a", "operation-a");
  const auto decoded_query = decode_query(query_message, error);
  ASSERT_TRUE(decoded_query);
  EXPECT_EQ(decoded_query->expected_simulator_epoch, "epoch-a");
  EXPECT_EQ(decoded_query->operation_id, "operation-a");
}

}  // namespace
}  // namespace restocker_gazebo
