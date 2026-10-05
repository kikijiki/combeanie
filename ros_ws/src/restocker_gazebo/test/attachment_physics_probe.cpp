// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_gazebo/attachment_protocol.hpp"

#include <gz/msgs/double.pb.h>

#include <chrono>
#include <cmath>
#include <iostream>
#include <optional>
#include <string>
#include <thread>

#include <gz/transport/Node.hh>

namespace restocker_gazebo
{
namespace
{

using namespace std::chrono_literals;

constexpr char kSetService[] = "/world/restocking/restocker/attachment/set";
constexpr char kQueryService[] = "/world/restocking/restocker/attachment/query";

template<typename Request>
[[nodiscard]] std::optional<AttachmentJournalReply> request(
  gz::transport::Node & node, const std::string & service,
  const Request & message)
{
  msgs::AttachmentReply wire_reply;
  bool service_result = false;
  const bool executed = node.Request(service, message, 1000U, wire_reply, service_result);
  if (!executed || !service_result) {
    return std::nullopt;
  }
  AttachmentStatus error;
  auto decoded = decode_reply(wire_reply, error);
  if (!decoded) {
    std::cerr << "attachment reply decode failed: " << error.detail << '\n';
  }
  return decoded;
}

[[nodiscard]] std::optional<std::string> discover_epoch(gz::transport::Node & node)
{
  const auto deadline = std::chrono::steady_clock::now() + 10s;
  while (std::chrono::steady_clock::now() < deadline) {
    msgs::QueryAttachmentRequest query;
    query.set_expected_simulator_epoch("physics-probe-discovery");
    const auto reply = request(node, kQueryService, query);
    if (reply && reply->status.code == AttachmentStatusCode::kSimulatorEpochChanged &&
      !reply->state.simulator_epoch.empty())
    {
      return reply->state.simulator_epoch;
    }
    std::this_thread::sleep_for(50ms);
  }
  return std::nullopt;
}

[[nodiscard]] msgs::SetAttachmentRequest command(
  const std::string & epoch, const std::string & operation_id,
  msgs::AttachmentCommandValue operation)
{
  msgs::SetAttachmentRequest result;
  result.set_expected_simulator_epoch(epoch);
  result.set_operation_id(operation_id);
  auto * payload = result.mutable_request();
  payload->set_command(operation);
  payload->set_reservation_id(41);
  auto * identity = payload->mutable_identity();
  identity->set_object_id(7);
  identity->set_object_source_id("sim:stock_can_01");
  identity->set_parent_model("restocker");
  identity->set_parent_link("gripper");
  identity->set_child_model("stock_can_01");
  identity->set_child_link("product_body");
  if (operation == msgs::ATTACHMENT_COMMAND_ATTACH) {
    payload->mutable_expected_grasp_center_to_child()->set_qw(1.0);
  }
  return result;
}

[[nodiscard]] std::optional<AttachmentJournalReply> await_terminal(
  gz::transport::Node & node, const std::string & epoch,
  const std::string & operation_id)
{
  const auto deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline) {
    msgs::QueryAttachmentRequest query;
    query.set_expected_simulator_epoch(epoch);
    query.set_operation_id(operation_id);
    const auto reply = request(node, kQueryService, query);
    if (reply && reply->status.code != AttachmentStatusCode::kPending) {
      return reply;
    }
    std::this_thread::sleep_for(20ms);
  }
  return std::nullopt;
}

[[nodiscard]] bool near(double left, double right, double tolerance)
{
  return std::isfinite(left) && std::abs(left - right) <= tolerance;
}

int fail(const std::string & detail)
{
  std::cerr << "attachment physics probe failed: " << detail << '\n';
  return 1;
}

}  // namespace
}  // namespace restocker_gazebo

int main()
{
  using restocker_gazebo::AttachmentMotionGate;
  using restocker_gazebo::AttachmentPhase;
  using restocker_gazebo::AttachmentStatusCode;
  using restocker_gazebo::await_terminal;
  using restocker_gazebo::command;
  using restocker_gazebo::discover_epoch;
  using restocker_gazebo::fail;
  using restocker_gazebo::kSetService;
  using restocker_gazebo::near;
  using restocker_gazebo::request;
  using namespace std::chrono_literals;

  gz::transport::Node node;
  const auto epoch = discover_epoch(node);
  if (!epoch) {
    return fail("query service did not expose the simulator epoch");
  }

  // Let the fixture's finger controllers reach the can-specific hold target.
  std::this_thread::sleep_for(500ms);
  const auto attach_request = command(
    *epoch, "attachment-physics-attach", restocker_gazebo::msgs::ATTACHMENT_COMMAND_ATTACH);
  const auto accepted = request(node, kSetService, attach_request);
  if (!accepted || accepted->status.code != AttachmentStatusCode::kPending) {
    return fail("attach command was not journaled as pending");
  }
  const auto attached = await_terminal(node, *epoch, "attachment-physics-attach");
  if (!attached || attached->status.code != AttachmentStatusCode::kAttached ||
    attached->state.phase != AttachmentPhase::kAttached ||
    attached->state.motion_gate != AttachmentMotionGate::kValid ||
    !attached->operation || !attached->operation->mutation_started ||
    !attached->operation->evidence || !attached->operation->evidence->joint_observed ||
    !near(attached->operation->evidence->parent_to_child.translation[2], 0.14, 1.0e-4))
  {
    return fail("attach did not reach a physics-verified terminal state");
  }

  const auto replay = request(node, kSetService, attach_request);
  if (!replay || replay->status.code != AttachmentStatusCode::kAttached || !replay->replayed) {
    return fail("exact attach replay did not return the original terminal record");
  }
  auto changed_request = attach_request;
  changed_request.mutable_request()->mutable_expected_grasp_center_to_child()->set_x(0.001);
  const auto conflict = request(node, kSetService, changed_request);
  if (!conflict || conflict->status.code != AttachmentStatusCode::kIdempotencyConflict) {
    return fail("changed attach replay did not fail with idempotency conflict");
  }

  auto left = node.Advertise<gz::msgs::Double>("/attachment_test/left_finger_position");
  auto right = node.Advertise<gz::msgs::Double>("/attachment_test/right_finger_position");
  if (!left.Valid() || !right.Valid()) {
    return fail("finger command publishers could not be advertised");
  }
  gz::msgs::Double open;
  // The plugin decouples only at its configured release aperture, so this must be
  // restocker_description/config/gripper_geometry.yaml's attachment.open_target_m, not the
  // fixture joint's upper limit (deliberately wider so the jaws never park on it).
  open.set_data(0.032);
  for (std::size_t attempt = 0; attempt < 20; ++attempt) {
    if (!left.Publish(open) || !right.Publish(open)) {
      return fail("finger open command publication failed");
    }
    std::this_thread::sleep_for(20ms);
  }
  std::this_thread::sleep_for(300ms);

  const auto detach_request = command(
    *epoch, "attachment-physics-detach", restocker_gazebo::msgs::ATTACHMENT_COMMAND_DETACH);
  const auto detach_accepted = request(node, kSetService, detach_request);
  if (!detach_accepted || detach_accepted->status.code != AttachmentStatusCode::kPending) {
    return fail("detach command was not journaled as pending");
  }
  const auto detached = await_terminal(node, *epoch, "attachment-physics-detach");
  if (!detached || detached->status.code != AttachmentStatusCode::kDetached ||
    detached->state.phase != AttachmentPhase::kDetached ||
    detached->state.motion_gate != AttachmentMotionGate::kValid ||
    detached->state.attached_identity || !detached->operation ||
    !detached->operation->mutation_started || !detached->operation->evidence ||
    detached->operation->evidence->joint_observed)
  {
    return fail("detach did not reach a physics-verified terminal state");
  }

  std::cout << "attachment physics probe completed attach and detach at epoch " << *epoch << '\n';
  return 0;
}
