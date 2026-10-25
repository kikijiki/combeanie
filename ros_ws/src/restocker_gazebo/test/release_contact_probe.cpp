// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

// Exercise the path a real placement takes and measure whether the released product is solid to
// the pinned column it is released into:
//
//   * attach  - the production attachment plugin welds `stock_can_01` into the gripper's
//               skeleton through the same services the coordinator uses, from the jaws closing
//               on the resting can to a physics-verified terminal state;
//   * carry   - `carry_joint` performs the arm's 0.12 m straight-line insert toward lane depth
//               0.088 m (`lane_release_product_center_depth_m` for the can). The welded
//               product's own contact stops the insert at a measured stall point; that stall
//               is the actual path's state, so the release happens from there rather than by
//               forcing the product through the contact;
//   * release - jaws open to `attachment.open_target_m` and the plugin detaches, returning the
//               product to free physics;
//   * settle  - wherever free physics leaves it, the probe measures the result against the
//               pinned column.
//
// It fails when the released product passes through the pinned column or remains
// interpenetrating with it, which is the unmeasured half of the product-contact claim Card 019
// owns, and it prints a receipt with the final separation, the retained penetration, the
// measured column growth against the catalogued pitch, and the terminal placement-evidence
// verdict the world state's `placement_require_column_growth` predicate would reach from those
// numbers.

// cpplint's enforced order ("release_contact_probe.h, c system, c++ system, other"): the angle
// `<gz/msgs/*.pb.h>` headers classify as C system here, the C++ standard headers follow, and
// only then the other headers (the quoted project header shares no basename prefix with this
// file, so it gets no own-header grace).
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/model.pb.h>
#include <gz/msgs/pose.pb.h>
#include <gz/msgs/pose_v.pb.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <mutex>
#include <optional>
#include <cstdlib>
#include <string>
#include <thread>
#include <vector>

#include <gz/transport/Node.hh>

#include "restocker_gazebo/attachment_protocol.hpp"

namespace restocker_gazebo
{
namespace
{

using namespace std::chrono_literals;

constexpr char kPoseTopic[] = "/world/restocking/pose/info";
constexpr char kJointStateTopic[] = "/release_contact/joint_state";
constexpr char kSetPoseService[] = "/world/restocking/set_pose";
constexpr char kSetService[] = "/world/restocking/restocker/attachment/set";
constexpr char kQueryService[] = "/world/restocking/restocker/attachment/query";
constexpr char kLeftFingerTopic[] = "/release_contact/left_finger_position";
constexpr char kRightFingerTopic[] = "/release_contact/right_finger_position";
constexpr char kCarryTopic[] = "/release_contact/carry_position";
constexpr char kCarryJoint[] = "carry_joint";

// The can column of restocker_gazebo/restocker_gazebo/lane_columns.py: the product this fixture
// stands and releases, restated rather than read so the probe stays a single translation unit.
// test_release_contact_runtime.py and fixtures/release_contact.sdf carry the same numbers.
constexpr double kProductRadiusM = 0.033;
constexpr double kProductHeightM = 0.122;
constexpr double kLaneInclineRad = 0.06981317007977318;  // 4 degrees, shelf.lane_incline_deg
constexpr double kLaneDatumDepthM = 0.84;                // bed datum, at the retainer face
constexpr double kLaneBaseHeightM = 0.75;                // world height of the lane datum
constexpr double kRearClearanceM = 0.01;                 // lane rear_clearance_m
constexpr double kStartDepthM = -0.032;                  // release depth minus 0.12 pre-insert
constexpr double kReleaseDepthM = 0.088;                 // 0.01 + 0.033 + insert_entry 0.045
constexpr double kCarryHeightM = 0.8717385220325486;     // rear-lip insert height, can centre
constexpr double kColumnPitchM = 2.0 * kProductRadiusM * std::cos(kLaneInclineRad);
// How far short of a full pitch a settled column may read and still prove a placement: the
// world state's placement_growth_tolerance_m, including the mixed-height footprint term and
// contact penetration. A single-class column has no mixed-height term.
constexpr double kGrowthToleranceM = 0.010;
// DART's default penetration allowance is 1 mm; 4 mm allows the settling transient and stays
// well away from the 0 m separation of an interpenetrating pair (product_contact_probe's rule).
constexpr double kPenetrationAllowanceM = 0.004;
// A resting contact may sit this far from the analytic bed pose without having left the bed:
// floor-vs-through-bed is a 0.75 m error, so this only absorbs the solver's allowance.
constexpr double kBedRestAllowanceM = 0.004;
// Static bodies do not move; anything else means the measurement is not the one it claims.
constexpr double kPinStillAllowanceM = 0.001;
// gripper_geometry.yaml grasp_center: the point 0.14 m along the gripper's +z with rpy 0. The
// probe restates fixture constants rather than parsing the yaml, as it does for the lane numbers.
constexpr double kGraspCenterZM = 0.14;
// attachment_boundary.yaml tolerances.expected_rotation_rad: how far the settled product may lean
// from upright before the fixture itself is misdelivered (Card 047, Milestone 10 §7). Rotation
// about the product's own vertical axis is free — the can is axisymmetric and the solver sets
// that angle during settle.
constexpr double kSpawnLeanMaxRad = 0.03;
// The pinned column, at lane_columns.py index 0, 1, 2 poses. Index 2 is the rearmost product
// and the one the released can must come to rest against.
constexpr std::array<double, 3> kPinDepthM{0.807000000, 0.741160773, 0.675321545};
constexpr std::array<double, 3> kPinHeightM{0.813456541, 0.818060468, 0.822664395};
constexpr char kCarriedName[] = "stock_can_01";

// Which jaw-hold policy this run is exercising, and the aperture it commands. Both default to
// the shipped physical policy (spec 11.3 D3); the Card 019 evidence run pins weld through the
// environment, so the same probe covers both and the attach assertion follows the mode.
constexpr std::array<const char *, 3> kPinNames{"lane_pinned_can_01", "lane_pinned_can_02",
  "lane_pinned_can_03"};

// Axis-aligned depth bound of a tilted cylinder, the same expression lane_columns.py's
// ground_truth_available_depth_m takes, so expectation and fixture agree.
[[nodiscard]] double kProductExtentM()
{
  const double axial = std::abs(std::sin(kLaneInclineRad));
  return kProductRadiusM * std::sqrt(1.0 - axial * axial) + 0.5 * kProductHeightM * axial;
}

// Where a can of this size comes to rest on the bed at a given lane depth: base one half-height
// of tilt behind the centre, surface height at that base, centre half a height along the normal.
[[nodiscard]] double bed_rest_height_m(double depth_m)
{
  const double base_depth = depth_m - 0.5 * kProductHeightM * std::sin(kLaneInclineRad);
  const double surface = (kLaneDatumDepthM - base_depth) * std::tan(kLaneInclineRad);
  return kLaneBaseHeightM + surface + 0.5 * kProductHeightM * std::cos(kLaneInclineRad);
}

struct Sample
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  // Orientation, captured so a red can show whether the product was already leaning before
  // the jaws moved (the attach rotation bound is the quantity that misses).
  double qw{1.0};
  double qx{0.0};
  double qy{0.0};
  double qz{0.0};
};

// Angle between the sampled orientation and the fixture's expected upright-can orientation
// (roll pi about x: w=0, x=1, y=0, z=0), via the quaternion inner product.
[[nodiscard]] double tilt_from_upright_rad(const Sample & sample)
{
  const double aligned = std::abs(sample.qx);
  return 2.0 * std::acos(std::min(1.0, aligned));
}

struct Quat
{
  double w{1.0};
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

[[nodiscard]] Quat quat_inverse(const Quat & q)
{
  // Unit quaternion: conjugate is the inverse.
  return Quat{q.w, -q.x, -q.y, -q.z};
}

[[nodiscard]] Quat quat_mul(const Quat & left, const Quat & right)
{
  return Quat{
    left.w * right.w - left.x * right.x - left.y * right.y - left.z * right.z,
    left.w * right.x + left.x * right.w + left.y * right.z - left.z * right.y,
    left.w * right.y - left.x * right.z + left.y * right.w + left.z * right.x,
    left.w * right.z + left.x * right.y - left.y * right.x + left.z * right.w};
}

[[nodiscard]] std::array<double, 3> quat_rotate(const Quat & q, const std::array<double, 3> & v)
{
  const std::array<double, 3> vector_part{q.x, q.y, q.z};
  const std::array<double, 3> cross_vector{
    vector_part[1] * v[2] - vector_part[2] * v[1],
    vector_part[2] * v[0] - vector_part[0] * v[2],
    vector_part[0] * v[1] - vector_part[1] * v[0]};
  const std::array<double, 3> cross_cross{
    vector_part[1] * cross_vector[2] - vector_part[2] * cross_vector[1],
    vector_part[2] * cross_vector[0] - vector_part[0] * cross_vector[2],
    vector_part[0] * cross_vector[1] - vector_part[1] * cross_vector[0]};
  std::array<double, 3> result{};
  for (std::size_t index = 0; index < 3; ++index) {
    result[index] = v[index] + 2.0 * (q.w * cross_vector[index] + cross_cross[index]);
  }
  return result;
}

// Split the can's world orientation error against upright (roll pi) into yaw about the world
// vertical (free: the product is axisymmetric) and lean about horizontal axes (asserted: a
// settled lean means the product took the bed's slope instead of standing on the pad).
struct OrientationError
{
  double yaw_rad{0.0};
  double lean_rad{0.0};
};

[[nodiscard]] OrientationError orientation_error(const Sample & sample)
{
  const Quat upright{0.0, 1.0, 0.0, 0.0};
  Quat relative = quat_mul(
    Quat{sample.qw, sample.qx, sample.qy, sample.qz}, quat_inverse(upright));
  if (relative.w < 0.0) {
    relative = Quat{-relative.w, -relative.x, -relative.y, -relative.z};
  }
  const double vector_norm = std::sqrt(
    relative.x * relative.x + relative.y * relative.y + relative.z * relative.z);
  OrientationError result;
  if (vector_norm < 1.0e-12) {
    return result;
  }
  const double angle = 2.0 * std::atan2(vector_norm, relative.w);
  const double scale = angle / vector_norm;
  const double omega_x = relative.x * scale;
  const double omega_y = relative.y * scale;
  const double omega_z = relative.z * scale;
  result.lean_rad = std::hypot(omega_x, omega_y);
  result.yaw_rad = omega_z;
  return result;
}

class PoseWatcher
{
public:
  void Ingest(const gz::msgs::Pose_V & message)
  {
    const std::lock_guard<std::mutex> guard(mutex_);
    for (const auto & pose : message.pose()) {
      const std::string & name = pose.name();
      if (std::find(kPinNames.begin(), kPinNames.end(), name) != kPinNames.end() ||
        name == kCarriedName)
      {
        poses_[name] = Sample{
          pose.position().x(), pose.position().y(), pose.position().z(),
          pose.orientation().w(), pose.orientation().x(), pose.orientation().y(),
          pose.orientation().z()};
      } else if (name == "restocker") {
        // The scene broadcaster publishes link poses in their model's frame, so the `gripper`
        // link (model-local identity) always reads (0,0,0). The model pose is the world pose of
        // the canonical link, and `gripper` is that link (no relative pose of its own), so this
        // is the gripper's world pose and orientation while the insert runs. Only this branch
        // writes it: a model-frame link pose arriving afterwards would clobber it with identity.
        gripper_ = Sample{
          pose.position().x(), pose.position().y(), pose.position().z(),
          pose.orientation().w(), pose.orientation().x(), pose.orientation().y(),
          pose.orientation().z()};
        gripper_tracked_ = true;
      }
    }
  }

  [[nodiscard]] std::optional<std::map<std::string, Sample>> Snapshot() const
  {
    const std::lock_guard<std::mutex> guard(mutex_);
    if (poses_.size() != 4U) {
      return std::nullopt;
    }
    return poses_;
  }

  [[nodiscard]] std::optional<Sample> Gripper() const
  {
    const std::lock_guard<std::mutex> guard(mutex_);
    if (!gripper_tracked_) {
      return std::nullopt;
    }
    return gripper_;
  }

  // Which tracked names have appeared, and how often, for a settle-timeout diagnosis.
  [[nodiscard]] std::string Summary() const
  {
    const std::lock_guard<std::mutex> guard(mutex_);
    std::string text;
    for (const char * pin : kPinNames) {
      text += std::string(pin) + ":" + std::to_string(poses_.count(pin) ? 1 : 0) + " ";
    }
    text += std::string(kCarriedName) + ":" + std::to_string(poses_.count(kCarriedName) ? 1 : 0);
    return text;
  }

private:
  mutable std::mutex mutex_;
  std::map<std::string, Sample> poses_;
  Sample gripper_;
  bool gripper_tracked_{false};
};

// Joint positions from the fixture's JointStatePublisher: the only direct reading of how far
// the insert has travelled, because the chassis weld keeps the model pose fixed.
class JointStateWatcher
{
public:
  void Ingest(const gz::msgs::Model & message)
  {
    const std::lock_guard<std::mutex> guard(mutex_);
    for (const auto & joint : message.joint()) {
      if (joint.has_axis1()) {
        positions_[joint.name()] = joint.axis1().position();
        velocities_[joint.name()] = joint.has_axis1() ? joint.axis1().velocity() : 0.0;
      }
    }
    seen_ = true;
  }

  [[nodiscard]] std::optional<std::map<std::string, double>> Snapshot() const
  {
    const std::lock_guard<std::mutex> guard(mutex_);
    if (!seen_) {
      return std::nullopt;
    }
    return positions_;
  }

  [[nodiscard]] std::optional<std::map<std::string, double>> Velocities() const
  {
    const std::lock_guard<std::mutex> guard(mutex_);
    if (!seen_) {
      return std::nullopt;
    }
    return velocities_;
  }

private:
  mutable std::mutex mutex_;
  std::map<std::string, double> positions_;
  std::map<std::string, double> velocities_;
  bool seen_{false};
};

// Each finger's contact point with the carried product, published by the fixture's test-only
// GripForceProbe: the probe's own view of the pads, independent of the attachment boundary.
constexpr char kPadTopic[] = "/release_contact/pad_contacts";

class PadWatcher
{
public:
  void Ingest(const gz::msgs::Pose_V & message)
  {
    std::map<std::string, std::array<double, 3>> points;
    for (const auto & pose : message.pose()) {
      points[pose.name()] = {pose.position().x(), pose.position().y(), pose.position().z()};
    }
    std::scoped_lock lock(mutex_);
    points_ = std::move(points);
  }

  [[nodiscard]] std::map<std::string, std::array<double, 3>> Snapshot() const
  {
    std::scoped_lock lock(mutex_);
    return points_;
  }

private:
  mutable std::mutex mutex_;
  std::map<std::string, std::array<double, 3>> points_;
};

[[nodiscard]] double distance(const Sample & left, const Sample & right)
{
  const double dx = left.x - right.x;
  const double dy = left.y - right.y;
  const double dz = left.z - right.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// Wait for the carried product and all three pinned products to exist and stop moving.
// Mid-fall, a product that never collides and one that has not landed yet look the same.
[[nodiscard]] std::optional<std::map<std::string, Sample>> await_settled(
  const PoseWatcher & watcher, std::chrono::seconds deadline)
{
  constexpr double kStillnessM = 1.0e-4;
  const auto until = std::chrono::steady_clock::now() + deadline;
  std::optional<std::map<std::string, Sample>> previous;
  int still = 0;
  while (std::chrono::steady_clock::now() < until) {
    std::this_thread::sleep_for(250ms);
    auto current = watcher.Snapshot();
    if (!current) {
      previous.reset();
      still = 0;
      continue;
    }
    double largest = 0.0;
    if (previous) {
      for (const auto &[name, sample] : *current) {
        const auto found = previous->find(name);
        if (found == previous->end()) {
          largest = std::numeric_limits<double>::infinity();
          break;
        }
        largest = std::max(largest, distance(sample, found->second));
      }
    }
    if (previous && largest < kStillnessM) {
      // Four consecutive quiet samples, so a pause at the top of a bounce is not read as rest.
      if (++still >= 4) {
        return current;
      }
    } else {
      still = 0;
    }
    previous = current;
  }
  return std::nullopt;
}

template<typename Request>
[[nodiscard]] std::optional<AttachmentJournalReply> request(
  gz::transport::Node & node,
  const std::string & service,
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
    query.set_expected_simulator_epoch("release-contact-discovery");
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
  const std::string & epoch,
  const std::string & operation_id,
  msgs::AttachmentCommandValue operation,
  const std::array<double, 3> & expected_translation,
  const std::array<double, 4> & expected_rotation_xyzw)
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
    // The authorized candidate is the probe's own settle observation expressed through the
    // fixture grasp center (Milestone 10 §7, Card 047), not the analytic identity: the bounds
    // then measure the product's motion from rest to attach.
    auto * expected = payload->mutable_expected_grasp_center_to_child();
    expected->set_x(expected_translation[0]);
    expected->set_y(expected_translation[1]);
    expected->set_z(expected_translation[2]);
    expected->set_qx(expected_rotation_xyzw[0]);
    expected->set_qy(expected_rotation_xyzw[1]);
    expected->set_qz(expected_rotation_xyzw[2]);
    expected->set_qw(expected_rotation_xyzw[3]);
  }
  return result;
}

[[nodiscard]] std::optional<AttachmentJournalReply> await_terminal(
  gz::transport::Node & node,
  const std::string & epoch,
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

// Diagnosis receipts: every attach attempt's outcome, so a red names the budget, the refusal
// or the predicate that missed instead of only the shared terminal message.
[[nodiscard]] std::string load1()
{
  std::ifstream stream("/proc/loadavg");
  std::string line;
  if (std::getline(stream, line)) {
    const auto space = line.find(' ');
    if (space != std::string::npos) {
      line.resize(space);
    }
    return line;
  }
  return "unknown";
}

[[nodiscard]] std::int64_t ms_since(std::chrono::steady_clock::time_point start)
{
  return std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - start).count();
}

[[nodiscard]] const char * status_name(AttachmentStatusCode code)
{
  switch (code) {
    case AttachmentStatusCode::kUnset: return "kUnset";
    case AttachmentStatusCode::kPending: return "kPending";
    case AttachmentStatusCode::kAttached: return "kAttached";
    case AttachmentStatusCode::kDetached: return "kDetached";
    case AttachmentStatusCode::kInvalidArgument: return "kInvalidArgument";
    case AttachmentStatusCode::kAuthorizationFailed: return "kAuthorizationFailed";
    case AttachmentStatusCode::kExternalInconsistency: return "kExternalInconsistency";
    case AttachmentStatusCode::kConflict: return "kConflict";
    case AttachmentStatusCode::kTokenMismatch: return "kTokenMismatch";
    case AttachmentStatusCode::kIdempotencyConflict: return "kIdempotencyConflict";
    case AttachmentStatusCode::kResourceExhausted: return "kResourceExhausted";
    case AttachmentStatusCode::kObjectNotFound: return "kObjectNotFound";
    case AttachmentStatusCode::kEntityAmbiguous: return "kEntityAmbiguous";
    case AttachmentStatusCode::kGripperNotReady: return "kGripperNotReady";
    case AttachmentStatusCode::kOutOfTolerance: return "kOutOfTolerance";
    case AttachmentStatusCode::kStateMismatch: return "kStateMismatch";
    case AttachmentStatusCode::kSimulatorEpochChanged: return "kSimulatorEpochChanged";
    case AttachmentStatusCode::kOutcomeUnknown: return "kOutcomeUnknown";
    case AttachmentStatusCode::kOperationNotFound: return "kOperationNotFound";
    case AttachmentStatusCode::kInternalError: return "kInternalError";
  }
  return "kUnrecognizedCode";
}

[[nodiscard]] const char * phase_name(AttachmentPhase phase)
{
  switch (phase) {
    case AttachmentPhase::kUnknown: return "kUnknown";
    case AttachmentPhase::kDetached: return "kDetached";
    case AttachmentPhase::kValidatingAttach: return "kValidatingAttach";
    case AttachmentPhase::kApplyingAttach: return "kApplyingAttach";
    case AttachmentPhase::kVerifyingAttach: return "kVerifyingAttach";
    case AttachmentPhase::kAttached: return "kAttached";
    case AttachmentPhase::kValidatingDetach: return "kValidatingDetach";
    case AttachmentPhase::kApplyingDetach: return "kApplyingDetach";
    case AttachmentPhase::kVerifyingDetach: return "kVerifyingDetach";
    case AttachmentPhase::kInconsistent: return "kInconsistent";
  }
  return "kUnrecognizedPhase";
}

[[nodiscard]] const char * gate_name(AttachmentMotionGate gate)
{
  switch (gate) {
    case AttachmentMotionGate::kUnknown: return "kUnknown";
    case AttachmentMotionGate::kInhibited: return "kInhibited";
    case AttachmentMotionGate::kValid: return "kValid";
  }
  return "kUnrecognizedGate";
}

// One receipt line per reply plus the terminal-state predicate values line 515's shared
// message collapses: operation presence, mutation start, evidence, joint observation and the
// weld-offset z against the 0.14 m expectation.
void print_reply_receipt(const char * label, const AttachmentJournalReply & reply)
{
  std::cout << "release contact receipt: " << label << " status " << status_name(reply.status.code)
            << " (" << reply.status.detail << ") phase " << phase_name(reply.state.phase)
            << " gate " << gate_name(reply.state.motion_gate) << '\n';
  if (!reply.operation) {
    std::cout << "release contact receipt: " << label << " operation record absent\n";
    return;
  }
  std::cout << "release contact receipt: " << label << " operation phase "
            << phase_name(reply.operation->phase) << " mutation_started "
            << (reply.operation->mutation_started ? "yes" : "no");
  if (reply.operation->fidelity) {
    std::cout << " fidelity translation_residual_m "
              << reply.operation->fidelity->translation_residual_m << " of budget "
              << reply.operation->fidelity->translation_budget_m << " (ceiling "
              << reply.operation->fidelity->translation_ceiling_m
              << ") rotation_residual_rad " << reply.operation->fidelity->rotation_residual_rad;
  }
  if (!reply.operation->evidence) {
    std::cout << " evidence absent\n";
    return;
  }
  const double weld_z = reply.operation->evidence->parent_to_child.translation[2];
  std::cout << " joint_observed "
            << (reply.operation->evidence->joint_observed ? "yes" : "no") << " weld_z " << weld_z
            << " (delta " << (weld_z - 0.14) << " of 0.0001)\n";
}

int fail(const std::string & detail)
{
  std::cerr << "release contact probe failed: " << detail << '\n';
  return 1;
}

// Joint positions and speeds for the fixture's three joints as one receipt line: the velocity
// gate is only trustworthy next to where the jaws and the insert actually are, and a velocity
// number with a stationary joint behind it is a phantom.
void print_joint_state_receipt(const char * label, const JointStateWatcher & joints)
{
  const auto positions = joints.Snapshot();
  if (!positions) {
    std::cout << "release contact receipt: " << label << " joint states absent\n";
    return;
  }
  const auto velocities = joints.Velocities();
  const auto left = positions->find("left_finger_joint");
  const auto right = positions->find("right_finger_joint");
  const auto carry = positions->find("carry_joint");
  std::cout << "release contact receipt: " << label << " joints left "
            << (left != positions->end() ? std::to_string(left->second) : "absent") << " right "
            << (right != positions->end() ? std::to_string(right->second) : "absent") << " carry "
            << (carry != positions->end() ? std::to_string(carry->second) : "absent");
  if (velocities) {
    const auto & v = *velocities;
    const auto left_v = v.find("left_finger_joint");
    const auto right_v = v.find("right_finger_joint");
    const auto carry_v = v.find("carry_joint");
    std::cout << " velocities left "
              << (left_v != v.end() ? std::to_string(left_v->second) : "absent") << " right "
              << (right_v != v.end() ? std::to_string(right_v->second) : "absent") << " carry "
              << (carry_v != v.end() ? std::to_string(carry_v->second) : "absent");
  }
  std::cout << '\n';
}

// Product pose during an attach attempt against the pre-attach sample: the refusal's velocity
// number is only trustworthy next to how far the product actually travelled while measured.
void print_attempt_pose_receipt(
  int attempt, const PoseWatcher & watcher, const Sample & pre_attach)
{
  if (const auto snapshot = watcher.Snapshot()) {
    const Sample & can = snapshot->at(kCarriedName);
    std::cout << "release contact receipt: attempt " << attempt << " pose x " << can.x
              << " y " << can.y << " z " << can.z << " (pre-attach delta "
              << (can.x - pre_attach.x) << " " << (can.y - pre_attach.y) << " "
              << (can.z - pre_attach.z) << ")";
    // The close's rotation split: yaw is a spin about the can's own axis, lean is a tip. Both
    // count against the attach rotation bound, and they have different causes.
    const auto now = orientation_error(can);
    const auto before = orientation_error(pre_attach);
    std::cout << " yaw_rad " << now.yaw_rad << " (pre-attach " << before.yaw_rad << ") lean_rad "
              << now.lean_rad << " (pre-attach " << before.lean_rad << ")\n";
  }
}

// One line of the carry trace: the product's offset in the gripper frame beside the jaw and
// carry positions, so a friction-grip slip shows which body moved and along which axis.
void print_carry_trace_receipt(const PoseWatcher & watcher, const JointStateWatcher & joints)
{
  const auto grip = watcher.Gripper();
  const auto snapshot = watcher.Snapshot();
  const auto positions = joints.Snapshot();
  if (!grip || !snapshot || !positions || snapshot->count(kCarriedName) == 0U) {
    std::cout << "release contact receipt: carry trace sample unavailable\n";
    return;
  }
  const Sample & can = snapshot->at(kCarriedName);
  const auto in_grip = quat_rotate(
    quat_inverse(Quat{grip->qw, grip->qx, grip->qy, grip->qz}),
    {can.x - grip->x, can.y - grip->y, can.z - grip->z});
  const auto joint = [&positions](const char * name) {
    const auto found = positions->find(name);
    return found != positions->end() ? found->second : std::numeric_limits<double>::quiet_NaN();
  };
  std::cout << "release contact receipt: carry trace carry " << joint(kCarryJoint) << " left "
            << joint("left_finger_joint") << " right " << joint("right_finger_joint")
            << " product y " << can.y << " z " << can.z << " tilt_rad "
            << tilt_from_upright_rad(can) << " in-grip xyz " << in_grip[0] << " " << in_grip[1]
            << " " << in_grip[2] << '\n';
}

// Which jaw-hold policy this run exercises: defaults to the shipped physical policy (spec 11.3
// D3); the Card 019 evidence run pins weld in the environment, so one probe covers both.
[[nodiscard]] std::string configured_hold_mode()
{
  const char * raw = std::getenv("RESTOCKER_RELEASE_CONTACT_HOLD_MODE");
  return std::string(raw ? raw : "physical");
}

// The aperture the run commands: physical-mode hold target for the can is
// 0.5 * (diameter - 2*interference - zero_gap) = 0.0093 at the shipped 0.0027 m interference; the
// env var pins it for weld runs.
[[nodiscard]] double configured_hold_target()
{
  if (const char * raw = std::getenv("RESTOCKER_RELEASE_CONTACT_HOLD_TARGET")) {
    return std::stod(raw);
  }
  return 0.0093;
}

// One position command, published repeatedly the way the fixture's own probe does: a single
// sample can be dropped while the controller is still subscribing.
[[nodiscard]] bool command_position(
  gz::transport::Node & node, const std::string & topic,
  double value, std::size_t attempts = 15U)
{
  auto publisher = node.Advertise<gz::msgs::Double>(topic);
  if (!publisher.Valid()) {
    return false;
  }
  gz::msgs::Double message;
  message.set_data(value);
  for (std::size_t attempt = 0; attempt < attempts; ++attempt) {
    if (!publisher.Publish(message)) {
      return false;
    }
    std::this_thread::sleep_for(20ms);
  }
  return true;
}

struct ReleaseSample
{
  std::optional<Sample> release;
  std::optional<Sample> last_seen;
};

// The first free pose after the release is the carry proof: the product must appear where the
// carry joint left it (stall point or target). Take the first sample inside that window rather
// than the last: from the target the bed's feed carries the product away within a few
// centimetres of the release instant, and a later sample would be mid-flight.
[[nodiscard]] ReleaseSample await_release_pose(const PoseWatcher & watcher, double expected_y)
{
  ReleaseSample result;
  const auto deadline = std::chrono::steady_clock::now() + 2s;
  while (std::chrono::steady_clock::now() < deadline) {
    if (const auto pose = watcher.Snapshot()) {
      result.last_seen = pose->at(kCarriedName);
      if (std::abs(result.last_seen->y - expected_y) <= 0.03) {
        result.release = result.last_seen;
        break;
      }
    }
    std::this_thread::sleep_for(20ms);
  }
  return result;
}

// The pinned column must stand where lane_columns.py puts it before anything is commanded.
[[nodiscard]] std::optional<std::string> misplaced_pin(const std::map<std::string, Sample> & spawn)
{
  for (std::size_t index = 0; index < kPinNames.size(); ++index) {
    const Sample & pin = spawn.at(kPinNames[index]);
    if (std::abs(pin.y - kPinDepthM[index]) > kPinStillAllowanceM ||
      std::abs(pin.z - kPinHeightM[index]) > kPinStillAllowanceM)
    {
      return std::string("pinned product ") + kPinNames[index] +
             " did not spawn at its column pose: y " + std::to_string(pin.y) + " z " +
             std::to_string(pin.z);
    }
  }
  return std::nullopt;
}

// Both jaws in one publish loop, as production's single gripper trajectory moves them together.
// Commanding one topic's burst and then the other's closed the jaws 300 ms apart, and under an
// interference target the first jaw shoved the product across the pad alone (measured, Card 057
// run M3: the can yawed 0.50 rad before the second jaw arrived).
[[nodiscard]] bool command_jaws(gz::transport::Node & node, double value)
{
  auto left = node.Advertise<gz::msgs::Double>(kLeftFingerTopic);
  auto right = node.Advertise<gz::msgs::Double>(kRightFingerTopic);
  if (!left.Valid() || !right.Valid()) {
    return false;
  }
  gz::msgs::Double message;
  message.set_data(value);
  for (int attempt = 0; attempt < 15; ++attempt) {
    if (!left.Publish(message) || !right.Publish(message)) {
      return false;
    }
    std::this_thread::sleep_for(20ms);
  }
  return true;
}

// Where the product sits in the gripper frame: its origin and its orientation.
struct InGripper
{
  std::array<double, 3> translation{};
  Quat rotation;
};

[[nodiscard]] InGripper product_in_gripper(const Sample & grip, const Sample & can)
{
  const Quat inverse = quat_inverse(Quat{grip.qw, grip.qx, grip.qy, grip.qz});
  return InGripper{
    quat_rotate(inverse, {can.x - grip.x, can.y - grip.y, can.z - grip.z}),
    quat_mul(inverse, Quat{can.qw, can.qx, can.qy, can.qz})};
}

[[nodiscard]] double distance3(const std::array<double, 3> & a, const std::array<double, 3> & b)
{
  return std::sqrt(
    (a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1]) + (a[2] - b[2]) * (a[2] - b[2]));
}

// The state the held measurement is taken against, sampled after the attach and before the insert.
struct HeldReference
{
  Sample grip;
  Sample can;
  std::array<std::array<double, 3>, 2> pads{};
  std::array<double, 2> finger_positions{};
};

// A finger's origin in the gripper frame: gripper_geometry.yaml's joint origins (0, +-0.03, 0.07)
// and axes (+-y), with the link declared at joint zero as physical_grasp.sdf declares it.
[[nodiscard]] std::array<double, 3> finger_in_gripper(std::size_t index, double position)
{
  const double side = index == 0U ? 1.0 : -1.0;
  return {0.0, side * (0.03 + position), 0.07};
}

[[nodiscard]] std::array<double, 3> world_to_gripper(
  const Sample & grip, const std::array<double, 3> & point)
{
  return quat_rotate(
    quat_inverse(Quat{grip.qw, grip.qx, grip.qy, grip.qz}),
    {point[0] - grip.x, point[1] - grip.y, point[2] - grip.z});
}

[[nodiscard]] std::optional<HeldReference> capture_held_reference(
  const PoseWatcher & watcher, const JointStateWatcher & joints, const PadWatcher & pads)
{
  const auto grip = watcher.Gripper();
  const auto snapshot = watcher.Snapshot();
  const auto positions = joints.Snapshot();
  const auto points = pads.Snapshot();
  if (!grip || !snapshot || !positions || points.count("left") == 0U ||
    points.count("right") == 0U || positions->count("left_finger_joint") == 0U ||
    positions->count("right_finger_joint") == 0U)
  {
    return std::nullopt;
  }
  return HeldReference{
    *grip, snapshot->at(kCarriedName), {points.at("left"), points.at("right")},
    {positions->at("left_finger_joint"), positions->at("right_finger_joint")}};
}

[[nodiscard]] std::array<double, 3> minus(
  const std::array<double, 3> & a, const std::array<double, 3> & b)
{
  return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}

[[nodiscard]] std::array<double, 3> plus(
  const std::array<double, 3> & a, const std::array<double, 3> & b)
{
  return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
}

// Spec M10 section 11 D5 amendment (as corrected), the probe's own half: over 2.4 s after the
// insert, each pad's slip -- the product's material point where the pad touched it at the
// reference, against that pad point carried by its finger -- stays within 1 mm, and the product's
// origin swings at most 3 mm about the pads' midpoint. The settle rule is on what load hopping
// between pad rows cannot move: the centre (per-axis midpoint of min and max) of the product
// origin's oscillation relative to the finger links' midpoint, which may move at most 0.1 mm
// between the two consecutive 1.2 s windows (each over two 0.56 s swing periods). Computed from
// the contact publisher, the pose topic and the joint states, not from the boundary; the
// boundary's watchdog verdict is then read back as a second, separate check.
[[nodiscard]] std::optional<std::string> assert_held_slip_and_swing(
  gz::transport::Node & node, const std::string & epoch, const std::string & attach_id,
  const PoseWatcher & watcher, const JointStateWatcher & joints, const HeldReference & reference)
{
  const InGripper pre = product_in_gripper(reference.grip, reference.can);
  std::array<std::array<double, 3>, 2> pad_in_finger{};
  std::array<std::array<double, 3>, 2> pad_in_child{};
  std::array<double, 3> middle_then{};
  for (std::size_t index = 0; index < 2; ++index) {
    const auto point = world_to_gripper(reference.grip, reference.pads[index]);
    pad_in_finger[index] =
      minus(point, finger_in_gripper(index, reference.finger_positions[index]));
    pad_in_child[index] =
      quat_rotate(quat_inverse(pre.rotation), minus(point, pre.translation));
    for (std::size_t axis = 0; axis < 3; ++axis) {
      middle_then[axis] += 0.5 * point[axis];
    }
  }
  std::array<double, 2> peak_slip{};
  double peak_swing = 0.0;
  std::string pad_rows;
  std::array<std::array<double, 3>, 2> window_min{};
  std::array<std::array<double, 3>, 2> window_max{};
  std::array<std::size_t, 2> window_samples{};
  std::size_t samples = 0U;
  const auto started = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - started < 2400ms) {
    std::this_thread::sleep_for(50ms);
    const auto grip = watcher.Gripper();
    const auto snapshot = watcher.Snapshot();
    const auto positions = joints.Snapshot();
    if (!grip || !snapshot || !positions || positions->count("left_finger_joint") == 0U ||
      positions->count("right_finger_joint") == 0U)
    {
      continue;
    }
    const InGripper now = product_in_gripper(*grip, snapshot->at(kCarriedName));
    const std::array<double, 2> fingers{
      positions->at("left_finger_joint"), positions->at("right_finger_joint")};
    // The compliant pad rows' own joint positions (spec M10 section 11 D10 item 3), left and
    // right per row: that is the pad compression, and it decides whether the 40 N press reaches
    // the product through the springs at all.
    {
      std::ostringstream rows;
      for (int index = 0; index < 4; ++index) {
        const auto left = "left_finger_pad_row_" + std::to_string(index) + "_joint";
        const auto right = "right_finger_pad_row_" + std::to_string(index) + "_joint";
        const auto left_position = positions->find(left);
        const auto right_position = positions->find(right);
        if (left_position == positions->end() || right_position == positions->end()) {
          continue;
        }
        rows << " [" << left_position->second << " " << right_position->second << "]";
      }
      if (rows.tellp() > 0) {
        pad_rows = rows.str();
      }
    }
    std::array<double, 3> middle_now{};
    for (std::size_t index = 0; index < 2; ++index) {
      const auto pad = plus(finger_in_gripper(index, fingers[index]), pad_in_finger[index]);
      const auto material = plus(now.translation, quat_rotate(now.rotation, pad_in_child[index]));
      const auto slip = minus(material, pad);
      peak_slip[index] = std::max(peak_slip[index], distance3(slip, {0.0, 0.0, 0.0}));
      for (std::size_t axis = 0; axis < 3; ++axis) {
        middle_now[axis] += 0.5 * pad[axis];
      }
    }
    peak_swing = std::max(
      peak_swing,
      distance3(minus(now.translation, middle_now), minus(pre.translation, middle_then)));
    // The settle quantity: the product origin against the finger links' midpoint.
    const auto links = plus(finger_in_gripper(0U, fingers[0]), finger_in_gripper(1U, fingers[1]));
    const std::array<double, 3> settle = minus(
      now.translation, {0.5 * links[0], 0.5 * links[1], 0.5 * links[2]});
    const std::size_t window =
      std::chrono::steady_clock::now() - started < 1200ms ? 0U : 1U;
    for (std::size_t axis = 0; axis < 3; ++axis) {
      window_min[window][axis] = window_samples[window] == 0U ?
        settle[axis] : std::min(window_min[window][axis], settle[axis]);
      window_max[window][axis] = window_samples[window] == 0U ?
        settle[axis] : std::max(window_max[window][axis], settle[axis]);
    }
    ++window_samples[window];
    ++samples;
  }
  std::array<std::array<double, 3>, 2> centre{};
  for (std::size_t window = 0; window < 2; ++window) {
    for (std::size_t axis = 0; axis < 3; ++axis) {
      centre[window][axis] = 0.5 * (window_min[window][axis] + window_max[window][axis]);
    }
  }
  const double settle_drift = distance3(centre[0], centre[1]);
  std::cout << "release contact receipt: held over 2.4 s after the insert (" << samples
            << " samples): pad slip peak left " << peak_slip[0] << " right " << peak_slip[1]
            << " of 0.001 m, swing peak " << peak_swing << " of 0.003 m, oscillation centre "
            << "drift against the finger links " << settle_drift
            << " of 0.0001 m between the two 1.2 s windows (" << window_samples[0] << " + "
            << window_samples[1] << " samples)"
            << "; product grip-frame y " << pre.translation[1]
            << "; pad row joints (left right)" << pad_rows << "\n";
  restocker_gazebo::msgs::QueryAttachmentRequest query;
  query.set_expected_simulator_epoch(epoch);
  query.set_operation_id(attach_id);
  const auto state = request(node, kQueryService, query);
  if (state) {
    print_reply_receipt("held after the insert", *state);
  }
  if (samples < 20U || window_samples[0] < 10U || window_samples[1] < 10U) {
    return std::string("held slip and swing could not be sampled after the insert");
  }
  if (std::max(peak_slip[0], peak_slip[1]) > 0.001) {
    return std::string("the product slipped at a pad by more than 1 mm during carry");
  }
  if (peak_swing > 0.003) {
    return std::string("the product swung more than the 3 mm allowance during carry");
  }
  if (settle_drift > 1.0e-4) {
    return std::string("the product never settled against the finger links after the insert");
  }
  if (!state || state->state.motion_gate != AttachmentMotionGate::kValid) {
    return std::string("the boundary's held-object watchdog did not keep the grip valid");
  }
  return std::nullopt;
}

// The fixture's finger links are declared at joint zero, where the faces are 12 mm inside a can
// standing between them, so the carried product loads parked on the floor and is placed at the
// pre-insert pose only once both jaws have opened to their initial target. Declaring the links
// at the open pose instead would shift every joint position by that offset (a joint reads zero
// at its declared pose; the controller's initial_position is only a command), and the aperture
// the plugin checks against gripper_geometry.yaml would then not be the aperture in the world.
[[nodiscard]] std::optional<std::string> place_carried_product(
  gz::transport::Node & node, const JointStateWatcher & joints)
{
  const auto deadline = std::chrono::steady_clock::now() + 10s;
  bool open = false;
  while (!open && std::chrono::steady_clock::now() < deadline) {
    if (const auto positions = joints.Snapshot()) {
      const auto left = positions->find("left_finger_joint");
      const auto right = positions->find("right_finger_joint");
      open = left != positions->end() && right != positions->end() && left->second >= 0.031 &&
        right->second >= 0.031;
    }
    if (!open) {
      std::this_thread::sleep_for(50ms);
    }
  }
  if (!open) {
    return std::string(
      "the jaws never opened to their initial target, so the product was not placed");
  }
  gz::msgs::Pose pose;
  pose.set_name(kCarriedName);
  pose.mutable_position()->set_x(0.0);
  pose.mutable_position()->set_y(kStartDepthM);
  pose.mutable_position()->set_z(kCarryHeightM);
  // Roll pi: the inverted grasp frame's upright can, as the spawn pose on the floor.
  pose.mutable_orientation()->set_w(0.0);
  pose.mutable_orientation()->set_x(1.0);
  gz::msgs::Boolean reply;
  bool result = false;
  if (!node.Request(kSetPoseService, pose, 5000U, reply, result) || !result || !reply.data()) {
    return std::string("the set_pose request placing the carried product was refused");
  }
  std::cout << "release contact receipt: product placed between the opened jaws\n";
  // Let pose samples from before the move age out of the settle window.
  std::this_thread::sleep_for(500ms);
  return std::nullopt;
}

}  // namespace
}  // namespace restocker_gazebo

int main()
{
  const std::string hold_mode = restocker_gazebo::configured_hold_mode();
  const double hold_target = restocker_gazebo::configured_hold_target();
  using restocker_gazebo::AttachmentMotionGate;
  using restocker_gazebo::AttachmentPhase;
  using restocker_gazebo::AttachmentStatusCode;
  using restocker_gazebo::await_settled;
  using restocker_gazebo::await_terminal;
  using restocker_gazebo::bed_rest_height_m;
  using restocker_gazebo::command;
  using restocker_gazebo::command_position;
  using restocker_gazebo::discover_epoch;
  using restocker_gazebo::distance;
  using restocker_gazebo::fail;
  using restocker_gazebo::kBedRestAllowanceM;
  using restocker_gazebo::kCarriedName;
  using restocker_gazebo::kCarryHeightM;
  using restocker_gazebo::kCarryTopic;
  using restocker_gazebo::kColumnPitchM;
  using restocker_gazebo::kGrowthToleranceM;
  using restocker_gazebo::kPenetrationAllowanceM;
  using restocker_gazebo::kPinDepthM;
  using restocker_gazebo::kPinHeightM;
  using restocker_gazebo::kPinNames;
  using restocker_gazebo::kPinStillAllowanceM;
  using restocker_gazebo::kProductExtentM;
  using restocker_gazebo::kQueryService;
  using restocker_gazebo::kRearClearanceM;
  using restocker_gazebo::kReleaseDepthM;
  using restocker_gazebo::kSetService;
  using restocker_gazebo::kStartDepthM;
  using restocker_gazebo::near;
  using restocker_gazebo::orientation_error;
  using restocker_gazebo::phase_name;
  using restocker_gazebo::print_reply_receipt;
  using restocker_gazebo::kGraspCenterZM;
  using restocker_gazebo::kSpawnLeanMaxRad;
  using restocker_gazebo::load1;
  using restocker_gazebo::ms_since;
  using restocker_gazebo::quat_inverse;
  using restocker_gazebo::quat_mul;
  using restocker_gazebo::quat_rotate;
  using restocker_gazebo::Quat;
  using restocker_gazebo::request;
  using restocker_gazebo::Sample;
  using restocker_gazebo::status_name;
  using restocker_gazebo::tilt_from_upright_rad;
  using namespace std::chrono_literals;

  const auto probe_started = std::chrono::steady_clock::now();
  // Receipts go to a pipe, so cout is fully buffered and a crash would erase them; unitbuf
  // flushes after every insertion so a red always keeps its evidence.
  std::cout.setf(std::ios::unitbuf);
  // Declaration order is the teardown contract: gz-transport's NodeShared::Unsubscribe is not
  // synchronous with an already-dispatched delivery (NodeShared::RecvMsgUpdate copies the
  // handler under its lock, then runs TriggerCallbacks unlocked), so the node must be destroyed
  // FIRST -- while the watchers its callbacks reference are still alive -- or an in-flight
  // joint-state callback writes into a destroyed map and the probe dies SIGSEGV on its way out
  // (coredumpctl 2026-09-27, pid 2275286: main in Node::~Node/Unsubscribe,
  // reception thread in JointStateWatcher::Ingest). Same order as product_contact_probe.
  restocker_gazebo::PoseWatcher watcher;
  restocker_gazebo::JointStateWatcher joints;
  restocker_gazebo::PadWatcher pads;
  gz::transport::Node node;
  if (!node.Subscribe<gz::msgs::Pose_V>(
      restocker_gazebo::kPoseTopic,
      [&watcher](const gz::msgs::Pose_V & message) {watcher.Ingest(message);}))
  {
    return fail(std::string("could not subscribe to ") + restocker_gazebo::kPoseTopic);
  }
  if (!node.Subscribe<gz::msgs::Model>(
      restocker_gazebo::kJointStateTopic,
      [&joints](const gz::msgs::Model & message) {joints.Ingest(message);}))
  {
    return fail(std::string("could not subscribe to ") + restocker_gazebo::kJointStateTopic);
  }
  if (!node.Subscribe<gz::msgs::Pose_V>(
      restocker_gazebo::kPadTopic,
      [&pads](const gz::msgs::Pose_V & message) {pads.Ingest(message);}))
  {
    return fail(std::string("could not subscribe to ") + restocker_gazebo::kPadTopic);
  }

  // The carried product starts level on the pad and the three pinned products are already at
  // their column poses; all four must exist and be at rest before anything is commanded.
  if (const auto placement = restocker_gazebo::place_carried_product(node, joints)) {
    return fail(*placement);
  }
  const auto settle_started = std::chrono::steady_clock::now();
  const auto spawn = await_settled(watcher, 30s);
  if (!spawn) {
    return fail(
      "the carried product and the three pinned products never all appeared and rested (" +
      watcher.Summary() + ")");
  }
  std::cout << "release contact receipt: spawn settled after " << ms_since(settle_started)
            << " ms (load1 " << load1() << ")\n";
  const Sample carried_spawn = spawn->at(kCarriedName);
  std::cout << "release contact receipt: spawn pose x " << carried_spawn.x << " y "
            << carried_spawn.y << " z " << carried_spawn.z << " (expected y " << kStartDepthM
            << " z " << kCarryHeightM << ", dz " << (carried_spawn.z - kCarryHeightM)
            << ") tilt_rad " << tilt_from_upright_rad(carried_spawn) << '\n';
  if (std::abs(carried_spawn.y - kStartDepthM) > 0.01 ||
    std::abs(carried_spawn.z - kCarryHeightM) > 0.01)
  {
    return fail(
      "the carried product is not at the pre-insert pose: y " +
      std::to_string(carried_spawn.y) + " z " + std::to_string(carried_spawn.z));
  }
  if (const auto misplaced = restocker_gazebo::misplaced_pin(*spawn)) {
    return fail(*misplaced);
  }

  // Authorize the attach from the settle observation (Milestone 10 §7, Card 047): the product
  // pose relative to the gripper model pose, expressed through the fixture grasp center. The
  // plugin's 3 mm / 0.03 rad bounds then measure the product's motion from rest to attach
  // instead of agreement with the analytic spawn pose, which the solver's settle yaw never
  // satisfied in the leaning micro-state.
  const auto gripper = watcher.Gripper();
  if (!gripper) {
    return fail(
      "the gripper pose never appeared, so the attach candidate cannot be authorized from the "
      "settle observation");
  }
  const Quat gripper_rotation{gripper->qw, gripper->qx, gripper->qy, gripper->qz};
  const Quat can_rotation{
    carried_spawn.qw, carried_spawn.qx, carried_spawn.qy, carried_spawn.qz};
  const Quat gripper_inverse = quat_inverse(gripper_rotation);
  const std::array<double, 3> child_offset{
    carried_spawn.x - gripper->x, carried_spawn.y - gripper->y, carried_spawn.z - gripper->z};
  const auto parent_from_child_translation = quat_rotate(gripper_inverse, child_offset);
  const auto parent_from_child_rotation = quat_mul(gripper_inverse, can_rotation);
  // expected = grasp_center_from_gripper * parent_from_child; the grasp center rotation is
  // identity, so only the translation drops the 0.14 m offset along the gripper's +z.
  const std::array<double, 3> expected_translation{
    parent_from_child_translation[0], parent_from_child_translation[1],
    parent_from_child_translation[2] - kGraspCenterZM};
  const std::array<double, 4> expected_rotation_xyzw{
    parent_from_child_rotation.x, parent_from_child_rotation.y, parent_from_child_rotation.z,
    parent_from_child_rotation.w};
  const auto spawn_orientation = orientation_error(carried_spawn);
  std::cout << "release contact receipt: spawn orientation yaw_rad "
            << spawn_orientation.yaw_rad << " lean_rad " << spawn_orientation.lean_rad
            << " (lean bound " << kSpawnLeanMaxRad << "); authorized candidate t "
            << expected_translation[0] << " " << expected_translation[1] << " "
            << expected_translation[2] << " q " << expected_rotation_xyzw[0] << " "
            << expected_rotation_xyzw[1] << " " << expected_rotation_xyzw[2] << " "
            << expected_rotation_xyzw[3] << '\n';
  if (spawn_orientation.lean_rad > kSpawnLeanMaxRad) {
    return fail(
      "the settled product leans " + std::to_string(spawn_orientation.lean_rad) +
      " rad from upright, beyond the " + std::to_string(kSpawnLeanMaxRad) +
      " rad bound: it did not come to rest standing on the pad, so this is not the "
      "attachment path the fixture exists to measure.");
  }

  const auto epoch = discover_epoch(node);
  if (!epoch) {
    return fail("query service did not expose the simulator epoch");
  }

  // SC-002: the product's position in the gripper frame, sampled immediately before the insert
  // and compared after it, is what a friction grip has to hold still (spec 11.9, 1 mm bound).
  std::optional<restocker_gazebo::HeldReference> held_reference;

  // Attach: close the jaws to the can's hold target, then let the plugin weld the product into
  // the gripper's skeleton. The journal refuses a premature attempt outright, so a retry with a
  // fresh operation id covers the fingers still settling without masking a real rejection.
  if (!restocker_gazebo::command_jaws(node, hold_target)) {
    return fail("finger close command publication failed");
  }
  // How long the jaws are given to finish the close before the plugin is asked to verify it.
  // Physical mode contacts the product, so the contact has to be settled first; the default
  // keeps Card 019's weld-era timing, and the physical run widens it from measurements.
  std::this_thread::sleep_for(
    std::chrono::milliseconds(
      [] {
        if (const char * raw = std::getenv("RESTOCKER_RELEASE_CONTACT_SETTLE_MS")) {
          return std::stoi(raw);
        }
        return 800;
      }()));
  // Pre-attach state receipt: has the close disturbed the product, and are the jaws at the
  // hold target? This is the moment the plugin's transform assertion will sample.
  Sample pre_attach = carried_spawn;
  if (const auto pre_attach_snapshot = watcher.Snapshot()) {
    pre_attach = pre_attach_snapshot->at(kCarriedName);
    std::cout << "release contact receipt: pre-attach pose x " << pre_attach.x
              << " y " << pre_attach.y << " z " << pre_attach.z << " tilt_rad "
              << tilt_from_upright_rad(pre_attach)
              << " (spawn tilt_rad " << tilt_from_upright_rad(carried_spawn) << ")\n";
  }
  std::cout << "release contact receipt: pre-attach hold target " << hold_target << " mode "
            << hold_mode << '\n';
  restocker_gazebo::print_joint_state_receipt("pre-attach", joints);

  const auto attach_loop_started = std::chrono::steady_clock::now();
  std::optional<restocker_gazebo::AttachmentJournalReply> attached;
  std::string attach_id;
  for (int attempt = 0; attempt < 4 && !attached; ++attempt) {
    attach_id = "release-contact-attach-" + std::to_string(attempt);
    const auto request_started = std::chrono::steady_clock::now();
    const auto accepted =
      request(
      node, kSetService,
      command(
        *epoch, attach_id, restocker_gazebo::msgs::ATTACHMENT_COMMAND_ATTACH,
        expected_translation, expected_rotation_xyzw));
    if (!accepted) {
      std::cout << "release contact receipt: attach attempt " << attempt
                << " set request got no reply in " << ms_since(request_started) << " ms\n";
      std::this_thread::sleep_for(300ms);
      continue;
    }
    if (accepted->status.code != AttachmentStatusCode::kPending) {
      std::cout << "release contact receipt: attach attempt " << attempt
                << " was not journaled pending after " << ms_since(request_started) << " ms\n";
      print_reply_receipt("attach submit", *accepted);
      std::this_thread::sleep_for(300ms);
      continue;
    }
    const auto terminal_started = std::chrono::steady_clock::now();
    auto terminal = await_terminal(node, *epoch, attach_id);
    const auto terminal_ms = ms_since(terminal_started);
    if (!terminal) {
      // One more query so a budget expiry records the state it expired on rather than nothing.
      restocker_gazebo::msgs::QueryAttachmentRequest query;
      query.set_expected_simulator_epoch(*epoch);
      query.set_operation_id(attach_id);
      std::cout << "release contact receipt: attach attempt " << attempt
                << " still pending after the " << terminal_ms << " ms terminal budget\n";
      if (const auto last = request(node, kQueryService, query)) {
        print_reply_receipt("attach at budget expiry", *last);
      }
      std::this_thread::sleep_for(300ms);
      continue;
    }
    std::cout << "release contact receipt: attach attempt " << attempt << " terminal after "
              << terminal_ms << " ms\n";
    print_reply_receipt("attach terminal", *terminal);
    restocker_gazebo::print_attempt_pose_receipt(attempt, watcher, pre_attach);
    restocker_gazebo::print_joint_state_receipt(
      ("attempt " + std::to_string(attempt)).c_str(), joints);
    if (terminal->status.code == AttachmentStatusCode::kAttached) {
      attached = terminal;
      break;
    }
    std::this_thread::sleep_for(300ms);
  }
  if (!attached || attached->status.code != AttachmentStatusCode::kAttached ||
    attached->state.phase != AttachmentPhase::kAttached ||
    attached->state.motion_gate != AttachmentMotionGate::kValid || !attached->operation ||
    !attached->operation->mutation_started || !attached->operation->evidence ||
    (hold_mode == "weld" ?
    !attached->operation->evidence->joint_observed :
    attached->operation->evidence->joint_observed) ||
    attached->state.hold_mode != hold_mode ||
    // A weld pins the product at the grasp center by construction; a squeeze leaves it where
    // contact put it, which only section 7's fidelity budget bounds (Card 057: no weld-era
    // tolerance is evidence for the physical grip).
    (hold_mode == "weld" ?
    !near(attached->operation->evidence->parent_to_child.translation[2], 0.14, 1.0e-4) :
    (!attached->operation->fidelity ||
    attached->operation->fidelity->translation_residual_m >
    attached->operation->fidelity->translation_budget_m)))
  {
    std::cout << "release contact receipt: attach ended unverified after "
              << ms_since(attach_loop_started) << " ms in the loop, " << ms_since(probe_started)
              << " ms of probe time (load1 " << load1() << ")\n";
    return fail("attach did not reach a physics-verified terminal state");
  }
  std::cout << "release contact receipt: attach verified after " << ms_since(attach_loop_started)
            << " ms in the loop (load1 " << load1() << ")\n";
  std::cout << "release contact receipt: attached at epoch " << *epoch << " operation " << attach_id
            << '\n';

  // Carry: the prismatic insert drives the welded product toward the release depth. Arrival is
  // read from the fixture's joint states (the chassis weld pins the model pose). When the
  // welded product's contact stops the insert short of the target, that stall is the measured
  // end of the actual path: the probe releases from the stall point instead of forcing the
  // product through contact, and the release-pose assertion below holds the joint's own
  // reading to where the product actually ended up. If joint states never publish, the
  // joint's velocity bound (0.05 m/s over 0.12 m) leaves a safe fixed wait and the commanded
  // target stays the expectation.
  held_reference = restocker_gazebo::capture_held_reference(watcher, joints, pads);
  const double carry_target = kReleaseDepthM - kStartDepthM;
  double carry_position = carry_target;
  if (!command_position(node, kCarryTopic, carry_target, 10U)) {
    return fail("carry command publication failed");
  }
  {
    const auto deadline = std::chrono::steady_clock::now() + 12s;
    bool states_seen = false;
    bool arrived = false;
    double previous = std::numeric_limits<double>::quiet_NaN();
    int quiet_ticks = 0;
    while (!arrived && std::chrono::steady_clock::now() < deadline) {
      if (hold_mode == "physical") {
        restocker_gazebo::print_carry_trace_receipt(watcher, joints);
      }
      if (const auto snapshot = joints.Snapshot()) {
        states_seen = true;
        const auto found = snapshot->find(restocker_gazebo::kCarryJoint);
        if (found != snapshot->end()) {
          carry_position = found->second;
          arrived = carry_position >= carry_target - 0.003;
          if (std::isfinite(previous) && std::abs(carry_position - previous) < 5.0e-4 &&
            std::abs(carry_position) > 1.0e-4 && carry_position < carry_target - 0.003)
          {
            // Two full seconds without a tenth of a millimetre of travel, after the insert has
            // already moved: it is against its contact, not on its way. The motion threshold
            // keeps command latency from being misread as a stall.
            if (++quiet_ticks >= 20) {
              break;
            }
          } else {
            quiet_ticks = 0;
          }
          previous = carry_position;
        }
      }
      if (!arrived) {
        std::this_thread::sleep_for(100ms);
      }
    }
    if (arrived) {
      std::cout << "release contact receipt: carry reached " << carry_position << " m of "
                << carry_target << " m\n";
    } else if (states_seen) {
      std::cout << "release contact receipt: carry stalled at " << carry_position << " m of "
                << carry_target
                << " m against the welded product's contact; releasing from the observed "
        "contact point\n";
    } else {
      // No joint-state publisher observed: the velocity bound makes 6 s safe to spend, and the
      // release-pose assertion below still fails if the product never moved.
      std::this_thread::sleep_for(6s);
      std::cout << "release contact receipt: carry joint states unavailable; expecting the "
        "commanded "
                << carry_target << " m\n";
    }

    // Park the actuator where it ended before the jaws open. The carry command is still live;
    // once the weld is gone an unloaded gripper would surge to the commanded target and its
    // trailing finger would sweep the freed product, so the release point would no longer be
    // the one the joint just measured. Re-commanding the measured position holds the gripper
    // still through the release, as the real arm does.
    if (states_seen) {
      if (!command_position(node, kCarryTopic, carry_position, 6U)) {
        return fail("carry hold command publication failed");
      }
      std::this_thread::sleep_for(400ms);
      if (const auto snapshot = joints.Snapshot()) {
        const auto found = snapshot->find(restocker_gazebo::kCarryJoint);
        if (found != snapshot->end()) {
          carry_position = found->second;
        }
      }
      std::cout << "release contact receipt: carry parked at joint position " << carry_position
                << " m for the release\n";
    }
  }

  if (hold_mode == "physical") {
    if (!held_reference) {
      return fail("the product and pad contacts were not sampled before the insert");
    }
    if (const auto held = restocker_gazebo::assert_held_slip_and_swing(
        node, *epoch, attach_id, watcher, joints, *held_reference))
    {
      return fail(*held);
    }
  } else if (const auto during_carry = await_settled(watcher, 4s)) {
    // Under the weld Gazebo stops updating the carried model's pose, so this read may still show
    // the attach pose; recorded for the receipt only.
    const Sample & held = during_carry->at(kCarriedName);
    std::cout << "release contact receipt: held pose during carry y " << held.y << " z " << held.z
              << '\n';
  }

  // Release: open the jaws to the decouple aperture, then detach through the plugin.
  if (!restocker_gazebo::command_jaws(node, 0.032)) {
    return fail("finger open command publication failed");
  }
  // Under a friction grip the opening jaws are the release and the bed's feed starts at once;
  // the detach below is bookkeeping, so the release pose is sampled now. A weld holds the product
  // until the detach, so there it is sampled after.
  const double expected_release_y = kStartDepthM + carry_position;
  std::optional<restocker_gazebo::ReleaseSample> physical_release;
  if (hold_mode == "physical") {
    physical_release = restocker_gazebo::await_release_pose(watcher, expected_release_y);
  }
  std::this_thread::sleep_for(700ms);

  std::optional<restocker_gazebo::AttachmentJournalReply> detached;
  std::string detach_id;
  for (int attempt = 0; attempt < 4 && !detached; ++attempt) {
    detach_id = "release-contact-detach-" + std::to_string(attempt);
    const auto accepted =
      request(
      node, kSetService,
      command(
        *epoch, detach_id, restocker_gazebo::msgs::ATTACHMENT_COMMAND_DETACH,
        expected_translation, expected_rotation_xyzw));
    if (!accepted || accepted->status.code != AttachmentStatusCode::kPending) {
      std::this_thread::sleep_for(300ms);
      continue;
    }
    auto terminal = await_terminal(node, *epoch, detach_id);
    if (terminal && terminal->status.code == AttachmentStatusCode::kDetached) {
      detached = terminal;
      break;
    }
    std::this_thread::sleep_for(300ms);
  }
  if (!detached || detached->status.code != AttachmentStatusCode::kDetached ||
    detached->state.phase != AttachmentPhase::kDetached ||
    detached->state.motion_gate != AttachmentMotionGate::kValid ||
    detached->state.attached_identity || !detached->operation ||
    !detached->operation->mutation_started || !detached->operation->evidence ||
    detached->operation->evidence->joint_observed)
  {
    return fail("detach did not reach a physics-verified terminal state");
  }

  const auto release_sample =
    physical_release ? *physical_release : restocker_gazebo::await_release_pose(
    watcher,
    expected_release_y);
  const std::optional<Sample> & release_pose = release_sample.release;
  if (!release_pose) {
    if (!release_sample.last_seen) {
      return fail("no post-detach pose of the released product appeared after the joint was gone");
    }
    return fail(
      "released at y " + std::to_string(release_sample.last_seen->y) +
      " against the carry joint's "
      "own end position y " +
      std::to_string(expected_release_y) + " (attach y " +
      std::to_string(carried_spawn.y) + ")");
  }
  std::cout << "release contact receipt: release pose y " << release_pose->y << " z "
            << release_pose->z << " (attach y " << carried_spawn.y
            << ", carry ended at joint position " << carry_position << " m)\n";

  // Settle: free physics decides where the product ends up relative to the pinned column;
  // the assertions below record pass-through, retained interpenetration and placement
  // evidence from that result.
  const auto settled = await_settled(watcher, 45s);
  if (!settled) {
    return fail(
      "the released product and the pinned column never came to rest (" +
      watcher.Summary() + ")");
  }
  // The measurement is complete: stop delivery ourselves and let an already-dispatched
  // callback drain while the watchers are still alive. Unsubscribe removes the handler for
  // future dispatches, but NodeShared::TriggerCallbacks runs unlocked after the handler was
  // copied under the lock, so the 100 ms drain (one delivery is microseconds) closes the
  // residue the declaration order alone leaves between ~Node and ~watcher.
  node.Unsubscribe(restocker_gazebo::kPoseTopic);
  node.Unsubscribe(restocker_gazebo::kJointStateTopic);
  std::this_thread::sleep_for(100ms);

  const Sample & released = settled->at(kCarriedName);
  const Sample & rearmost_pin = settled->at(kPinNames[2]);
  for (std::size_t index = 0; index < kPinNames.size(); ++index) {
    const Sample & pin = settled->at(kPinNames[index]);
    if (std::abs(pin.y - kPinDepthM[index]) > kPinStillAllowanceM ||
      std::abs(pin.z - kPinHeightM[index]) > kPinStillAllowanceM)
    {
      return fail(
        std::string("the pinned product ") + kPinNames[index] + " moved: y " +
        std::to_string(pin.y) + " z " + std::to_string(pin.z) + " against y " +
        std::to_string(kPinDepthM[index]) + " z " + std::to_string(kPinHeightM[index]));
    }
  }

  int failures = 0;

  // Control: a product resting on the bed is at a known height for its depth. A product on the
  // floor (it fell through) or on its side (it tipped off the feed) is not, which separates
  // "contact never happened because the product left the lane" from the contact question.
  const double expected_rest_z = bed_rest_height_m(released.y);
  if (std::abs(released.z - expected_rest_z) > kBedRestAllowanceM) {
    failures += fail(
      "the released product is not resting on the bed at depth " +
      std::to_string(released.y) + ": z " + std::to_string(released.z) +
      " against an expected " + std::to_string(expected_rest_z) +
      ". It left the lane surface, so this is not the "
      "product-to-pinned contact the fixture exists to measure.");
  }
  if (std::abs(released.x) > 0.05) {
    failures += fail(
      "the released product drifted laterally out of the lane: x " +
      std::to_string(released.x));
  }

  // Pass-through: the released product must end behind the rearmost pinned product. Ending at
  // or beyond it means it went through the column, which is exactly the case that would make
  // free depth grow back after a proven placement.
  const double separation = rearmost_pin.y - released.y;
  if (released.y >= rearmost_pin.y) {
    failures += fail(
      "the released product passed through the pinned column: it settled at depth " +
      std::to_string(released.y) + " against the rearmost pinned product at " +
      std::to_string(rearmost_pin.y) +
      ". Column growth measured from this placement would read "
      "zero while the lane still holds a product it cannot see.");
  } else if (separation < kColumnPitchM - kPenetrationAllowanceM) {
    failures += fail(
      "the released product remains interpenetrating with the pinned column: centre separation " +
      std::to_string(separation) + " m of lane depth against a catalogued pitch of " +
      std::to_string(kColumnPitchM) + " m (penetration " +
      std::to_string(kColumnPitchM - separation) + " m, allowance " +
      std::to_string(kPenetrationAllowanceM) + " m).");
  }

  // Terminal placement evidence: the quantity `placement_require_column_growth` judges. Free
  // depth is the rearmost product's depth bound behind the rear clearance, before the carry
  // (rearmost pinned product) and after the settle (whichever product is rearmost now).
  const double extent = kProductExtentM();
  const double available_before = rearmost_pin.y - extent - kRearClearanceM;
  const double available_after =
    (released.y < rearmost_pin.y ? released.y : rearmost_pin.y) - extent - kRearClearanceM;
  const double growth = available_before - available_after;
  const bool evidence_proven = growth >= kColumnPitchM - kGrowthToleranceM;
  if (!evidence_proven) {
    failures +=
      fail(
      "terminal placement evidence would be refused: the column grew " +
      std::to_string(growth) + " m against a pitch of " + std::to_string(kColumnPitchM) +
      " m less tolerance " + std::to_string(kGrowthToleranceM) + " m.");
  }

  const double penetration = kColumnPitchM - separation;
  std::cout << "release contact receipt: settled released depth " << released.y << " height "
            << released.z << " (expected bed rest " << expected_rest_z << ")\n";
  for (std::size_t index = 0; index < kPinNames.size(); ++index) {
    const Sample & pin = settled->at(kPinNames[index]);
    std::cout << "release contact receipt: settled " << kPinNames[index] << " depth " << pin.y
              << " height " << pin.z << '\n';
  }
  std::cout << "release contact receipt: final separation " << separation
            << " m of lane depth (pitch " << kColumnPitchM << " m, 3D "
            << distance(released, rearmost_pin) << " m)\n";
  std::cout << "release contact receipt: retained penetration " << penetration << " m (allowance "
            << kPenetrationAllowanceM << " m)\n";
  std::cout << "release contact receipt: column growth " << growth << " m (available "
            << available_before << " -> " << available_after << ", pitch " << kColumnPitchM
            << ", tolerance " << kGrowthToleranceM << ")\n";
  std::cout << "release contact receipt: terminal placement evidence "
            << (evidence_proven ? "PROVEN" : "REFUSED") << " by column growth\n";

  if (failures > 0) {
    return 1;
  }
  const bool packed = separation >= kColumnPitchM - kPenetrationAllowanceM &&
    separation <= kColumnPitchM + kGrowthToleranceM;
  std::cout << "release contact probe succeeded: attach, carry, release and settle measured "
            << (packed ? "a product packed solid against the pinned column" :
  "a product released clear of the pinned column without pass-through or "
  "interpenetration")
            << " (separation " << separation << " m)\n";
  return 0;
}
