// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_world_state/world_state.hpp"

#include <sys/random.h>

#include <bit>

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <deque>
#include <iomanip>
#include <limits>
#include <locale>
#include <map>
#include <mutex>
#include <shared_mutex>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace restocker_world_state
{
namespace
{

constexpr double kRotationTolerance = 1e-6;
constexpr double kCovarianceSymmetryTolerance = 1e-9;
constexpr double kCovarianceEigenvalueTolerance = 1e-10;

[[nodiscard]] bool valid_telemetry_source_id(const std::string & source_id) noexcept
{
  const auto alphanumeric = [](const char value) {
    return (value >= 'a' && value <= 'z') || (value >= 'A' && value <= 'Z') ||
           (value >= '0' && value <= '9');
  };
  if (source_id.empty() || source_id.size() > 128U || !alphanumeric(source_id.front())) {
    return false;
  }
  return std::all_of(
    source_id.begin() + 1, source_id.end(), [&alphanumeric](const char value) {
      return alphanumeric(value) || value == '_' || value == '-' || value == '.' ||
             value == ':' || value == '/';
    });
}

template<typename T>
Result<T> failure(WorldStateErrorCode code, std::string detail)
{
  return Result<T>::failure(WorldStateError{code, std::move(detail)});
}

// Stored, not re-derived, so a malformed grasp transform would be trusted while the product is
// held.
[[nodiscard]] bool valid_rigid_transform(const Eigen::Isometry3d & transform)
{
  return transform.matrix().allFinite() &&
         std::abs(Eigen::Quaterniond(transform.linear()).norm() - 1.0) <= kRotationTolerance;
}

std::optional<WorldStateError> validate_clock(
  const rclcpp::Time & value, rcl_clock_type_t expected,
  const char * field)
{
  if (value.get_clock_type() == expected) {
    return std::nullopt;
  }
  std::ostringstream detail;
  detail << field << " uses clock type " << value.get_clock_type() << ", expected " << expected;
  return WorldStateError{WorldStateErrorCode::ClockMismatch, detail.str()};
}

std::optional<WorldStateError> validate_observation_time(
  const rclcpp::Time & observation_time,
  const rclcpp::Time & received_at,
  const WorldStateConfig & config)
{
  if (auto error = validate_clock(observation_time, config.clock_type, "observation_time")) {
    return error;
  }
  if (auto error = validate_clock(received_at, config.clock_type, "received_at")) {
    return error;
  }

  const auto observation_ns = observation_time.nanoseconds();
  const auto received_ns = received_at.nanoseconds();
  if (observation_ns > received_ns) {
    if (observation_ns - received_ns > config.maximum_future_skew.count()) {
      return WorldStateError{WorldStateErrorCode::FutureObservation,
        "observation exceeds the permitted future skew"};
    }
  } else if (received_ns - observation_ns > config.maximum_observation_age.count()) {
    return WorldStateError{WorldStateErrorCode::StaleObservation,
      "observation exceeds the configured maximum age"};
  }
  return std::nullopt;
}

// Lane evidence under a moving eye: usable when a survey was accepted, not invalidated by an
// event, and inside the configured validity horizon.
[[nodiscard]] std::optional<WorldStateError> lane_evidence_validity_error(
  const ShelfLane & lane, const rclcpp::Time & now, const WorldStateConfig & config,
  const char * context)
{
  if (lane.evidence_revision == 0 || lane.last_verified.nanoseconds() <= 0) {
    return WorldStateError{
      WorldStateErrorCode::StaleObservation,
      std::string(context) + " has no accepted depletion evidence"};
  }
  if (lane.evidence_invalidated) {
    return WorldStateError{
      WorldStateErrorCode::StaleObservation,
      std::string(context) + " depletion evidence was invalidated and has not been re-surveyed"};
  }
  if (lane.last_verified.get_clock_type() != now.get_clock_type()) {
    return WorldStateError{
      WorldStateErrorCode::ClockMismatch,
      std::string(context) + " evidence clock type mismatches the request"};
  }
  const std::int64_t age_ns = now.nanoseconds() - lane.last_verified.nanoseconds();
  if (age_ns > config.lane_evidence_validity.count()) {
    return WorldStateError{
      WorldStateErrorCode::StaleObservation,
      std::string(context) + " depletion evidence is outside its validity horizon"};
  }
  if (age_ns < -config.maximum_future_skew.count()) {
    return WorldStateError{
      WorldStateErrorCode::FutureObservation,
      std::string(context) + " depletion evidence is too far in the future"};
  }
  return std::nullopt;
}

std::optional<WorldStateError> validate_optional_text(
  const std::optional<std::string> & value,
  const char * field)
{
  if (value && value->empty()) {
    return WorldStateError{WorldStateErrorCode::InvalidArgument,
      std::string(field) + " cannot be empty when present"};
  }
  return std::nullopt;
}

std::optional<WorldStateError> validate_pose(const Eigen::Isometry3d & pose)
{
  if (!pose.matrix().allFinite()) {
    return WorldStateError{WorldStateErrorCode::InvalidArgument, "pose contains non-finite values"};
  }

  const Eigen::Matrix3d rotation = pose.linear();
  if (!(rotation.transpose() * rotation)
    .isApprox(Eigen::Matrix3d::Identity(), kRotationTolerance) ||
    std::abs(rotation.determinant() - 1.0) > kRotationTolerance)
  {
    return WorldStateError{WorldStateErrorCode::InvalidArgument,
      "pose rotation is not a proper orthonormal matrix"};
  }

  const Eigen::Vector4d expected_last_row(0.0, 0.0, 0.0, 1.0);
  if (!pose.matrix().row(3).transpose().isApprox(expected_last_row, kRotationTolerance)) {
    return WorldStateError{WorldStateErrorCode::InvalidArgument, "pose homogeneous row is invalid"};
  }
  return std::nullopt;
}

std::optional<WorldStateError> validate_covariance(const PoseCovariance & covariance)
{
  if (!covariance.allFinite()) {
    return WorldStateError{WorldStateErrorCode::InvalidArgument,
      "pose covariance contains non-finite values"};
  }
  if (!covariance.isApprox(covariance.transpose(), kCovarianceSymmetryTolerance)) {
    return WorldStateError{WorldStateErrorCode::InvalidArgument,
      "pose covariance is not symmetric"};
  }

  Eigen::SelfAdjointEigenSolver<PoseCovariance> solver(covariance, Eigen::EigenvaluesOnly);
  if (solver.info() != Eigen::Success ||
    solver.eigenvalues().minCoeff() < -kCovarianceEigenvalueTolerance)
  {
    return WorldStateError{WorldStateErrorCode::InvalidArgument,
      "pose covariance is not positive-semidefinite"};
  }
  return std::nullopt;
}

bool lane_accepts_object(const ShelfLane & lane, const TrackedObject & object)
{
  return policy_accepts_object(lane.expected_product_class, lane.expected_sku, object);
}

[[nodiscard]] bool valid_operation_id(const std::string & value)
{
  return !value.empty() && value.size() <= 128;
}

void append_text(std::ostringstream & stream, const std::string & value)
{
  stream << value.size() << ':' << value << ';';
}

void append_lane(std::ostringstream & stream, const std::optional<LaneId> & lane)
{
  stream << lane.has_value() << ';';
  if (lane) {
    append_text(stream, lane->value);
  }
}

void append_revision(std::ostringstream & stream, const std::optional<Revision> & value)
{
  stream << value.has_value() << ';';
  if (value) {
    stream << *value << ';';
  }
}

[[nodiscard]] std::string format_depth(double value)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(4) << value << "m";
  return stream.str();
}

// Renders the destination evidence a placement predicate judged, so a refusal can be diagnosed
// from the log. `released_at` is present only for a placement.
[[nodiscard]] std::string describe_lane_evidence(
  const ShelfLane & lane, const TrackedObject & object, const rclcpp::Time & judged_at,
  const std::optional<rclcpp::Time> & released_at = std::nullopt)
{
  std::ostringstream stream;
  stream << " (lane " << lane.id.value << " target [" << object.source_object_id
         << "], observed [";
  for (std::size_t index = 0; index < lane.observed_source_object_ids.size(); ++index) {
    stream << (index == 0 ? "" : " ") << lane.observed_source_object_ids[index];
  }
  // Lane evidence and product pose come from the same sample; printing the tracked pose shows
  // which of the two disagrees.
  const Eigen::Vector3d position = object.pose_in_world.translation();
  stream << "], obstructed=" << (lane.obstructed ? "true" : "false")
         << ", evidence_revision=" << lane.evidence_revision
         << ", evidence_invalidated=" << (lane.evidence_invalidated ? "true" : "false")
         << ", last_verified_ns=" << lane.last_verified.nanoseconds()
         << ", available_depth_m=" << lane.available_depth_m
         << ", product_world_xyz=(" << position.x() << ' ' << position.y() << ' ' << position.z()
         << "), product_observed_ns=" << object.observation_time.nanoseconds();
  if (released_at) {
    stream << ", released_at_ns=" << released_at->nanoseconds();
  }
  stream << ", judged_at_ns=" << judged_at.nanoseconds() << ")";
  return stream.str();
}

[[nodiscard]] std::string reserve_fingerprint(const ReserveTaskRequest & request)
{
  std::ostringstream stream;
  stream << request.selected_snapshot_revision << ';' << request.object_id.value << ';'
         << request.object_revision << ';';
  append_lane(stream, request.source_lane);
  append_revision(stream, request.source_lane_revision);
  append_text(stream, request.destination_lane.value);
  stream << request.destination_lane_revision;
  return stream.str();
}

[[nodiscard]] std::string checkpoint_fingerprint(const ReservedTaskCheckpoint & checkpoint)
{
  std::ostringstream stream;
  append_text(stream, checkpoint.token);
  stream << checkpoint.expected_reservation_id << ';'
         << static_cast<unsigned int>(checkpoint.expected_reservation_stage) << ';'
         << checkpoint.expected_reservation_revision << ';'
         << static_cast<unsigned int>(checkpoint.task_phase) << ';'
         << static_cast<unsigned int>(checkpoint.fault_state);
  return stream.str();
}

[[nodiscard]] std::string attachment_fingerprint(const ReservedAttachmentRequest & request)
{
  std::ostringstream stream;
  stream.imbue(std::locale::classic());
  stream << "attach;";
  append_text(stream, request.token);
  stream << std::hex << std::setfill('0');
  for (Eigen::Index row = 0; row < 4; ++row) {
    for (Eigen::Index column = 0; column < 4; ++column) {
      const double value = request.grasp_center_from_held_object.matrix()(row, column);
      const double canonical = value == 0.0 ? 0.0 : value;
      stream << std::setw(16) << std::bit_cast<std::uint64_t>(canonical);
    }
  }
  return stream.str();
}

[[nodiscard]] std::string detach_fingerprint(const ReservedDetachmentRequest & request)
{
  std::ostringstream stream;
  append_text(stream, request.token);
  stream << static_cast<unsigned int>(request.disposition) << ';'
         << request.released_at.nanoseconds();
  return stream.str();
}

[[nodiscard]] std::string release_fingerprint(const ReleaseReservationRequest & request)
{
  std::ostringstream stream;
  append_text(stream, request.token);
  stream << request.expected_reservation_id << ';'
         << static_cast<unsigned int>(request.expected_reservation_stage) << ';'
         << request.expected_reservation_revision << ';'
         << static_cast<unsigned int>(request.outcome) << ';'
         << static_cast<unsigned int>(request.terminal_task_phase) << ';'
         << static_cast<unsigned int>(request.terminal_fault_state);
  return stream.str();
}

enum class JournalKind : std::uint8_t
{
  Reserve,
  Checkpoint,
  Attach,
  Detach,
  Release,
};

struct JournalEntry
{
  JournalKind kind{JournalKind::Reserve};
  std::string fingerprint;
  Revision revision{0};
  std::string token;
  std::optional<TaskReservation> reservation;
};

// Admission provenance shares the existing source identity entry: one bounded record per
// retained object, with no per-observation allocation or separate growing history.
struct SourceObjectEntry
{
  ObjectId object_id;
  std::int64_t received_at_ns{0};
  Revision observation_revision{0};
  std::optional<AuthorityClockSample> clock_sample;
};

[[nodiscard]] std::optional<std::string> generate_token()
{
  std::array<unsigned char, 16> bytes{};
  std::size_t offset = 0;
  while (offset < bytes.size()) {
    const ssize_t count = getrandom(bytes.data() + offset, bytes.size() - offset, GRND_NONBLOCK);
    if (count < 0) {
      if (errno == EINTR) {
        continue;
      }
      return std::nullopt;
    }
    if (count == 0) {
      return std::nullopt;
    }
    offset += static_cast<std::size_t>(count);
  }
  constexpr char kHex[] = "0123456789abcdef";
  std::string token(bytes.size() * 2, '0');
  for (std::size_t index = 0; index < bytes.size(); ++index) {
    token[2 * index] = kHex[bytes[index] >> 4U];
    token[2 * index + 1] = kHex[bytes[index] & 0x0fU];
  }
  return token;
}

}  // namespace

AttachmentDiagnosticIdentity AttachmentDiagnosticIdentity::capture(std::string_view value) noexcept
{
  AttachmentDiagnosticIdentity result;
  result.original_size = value.size();
  std::copy_n(value.begin(), std::min(value.size(), result.prefix.size()), result.prefix.begin());
  return result;
}

std::array<char, AttachmentDiagnosticIdentity::capacity * 2 + 1>
AttachmentDiagnosticIdentity::hex_prefix() const noexcept
{
  constexpr char digits[] = "0123456789abcdef";
  std::array<char, capacity * 2 + 1> result{};
  const auto count = std::min<std::uint64_t>(original_size, capacity);
  for (std::size_t index = 0; index < count; ++index) {
    result[index * 2] = digits[prefix[index] >> 4];
    result[index * 2 + 1] = digits[prefix[index] & 0x0f];
  }
  return result;
}

class WorldStateStore::Impl
{
public:
  explicit Impl(WorldStateConfig input_config)
  : config(std::move(input_config))
  {
    robot.telemetry_time = rclcpp::Time(std::int64_t {0}, config.clock_type);
    instance_id = generate_token().value_or("unavailable");
    if (config.planning_frame.empty()) {
      throw std::invalid_argument("planning_frame cannot be empty");
    }
    if (config.clock_type != RCL_ROS_TIME && config.clock_type != RCL_SYSTEM_TIME &&
      config.clock_type != RCL_STEADY_TIME)
    {
      throw std::invalid_argument("clock_type must be an initialized ROS, system, or steady clock");
    }
    if (config.maximum_observation_age.count() < 0) {
      throw std::invalid_argument("maximum_observation_age cannot be negative");
    }
    if (config.lane_evidence_validity.count() <= 0) {
      throw std::invalid_argument("lane_evidence_validity must be positive");
    }
    if (config.maximum_future_skew.count() < 0) {
      throw std::invalid_argument("maximum_future_skew cannot be negative");
    }
    if (config.event_capacity == 0) {
      throw std::invalid_argument("event_capacity must be positive");
    }
    if (config.operation_journal_capacity == 0) {
      throw std::invalid_argument("operation_journal_capacity must be positive");
    }
    if (!std::isfinite(config.reserved_pose_divergence_bound_m) ||
      config.reserved_pose_divergence_bound_m < 0.0)
    {
      throw std::invalid_argument(
              "reserved_pose_divergence_bound_m must be finite and non-negative");
    }
    for (std::size_t index = 0; index < kArmJointCount; ++index) {
      if (!std::isfinite(config.joint_lower_limits[index]) ||
        !std::isfinite(config.joint_upper_limits[index]) ||
        config.joint_lower_limits[index] > config.joint_upper_limits[index])
      {
        throw std::invalid_argument("robot joint limits must be finite and ordered");
      }
    }
    // Capped: the tolerance absorbs a float excess at a parked bound, never a widened limit.
    if (!std::isfinite(config.joint_limit_tolerance) || config.joint_limit_tolerance < 0.0 ||
      config.joint_limit_tolerance > kMaximumJointLimitTolerance)
    {
      throw std::invalid_argument(
              "robot joint limit tolerance must be finite and within [0, 0.01] rad");
    }
    if (!std::isfinite(config.rail_lower_limit) || !std::isfinite(config.rail_upper_limit) ||
      config.rail_lower_limit > config.rail_upper_limit)
    {
      throw std::invalid_argument("robot rail limits must be finite and ordered");
    }
    for (std::size_t index = 0; index < kGripperJointCount; ++index) {
      if (!std::isfinite(config.gripper_joint_lower_limits[index]) ||
        !std::isfinite(config.gripper_joint_upper_limits[index]) ||
        config.gripper_joint_lower_limits[index] > config.gripper_joint_upper_limits[index])
      {
        throw std::invalid_argument("robot gripper joint limits must be finite and ordered");
      }
    }
  }

  AuthorityClockSample capture_authority_clock(
    const rclcpp::Time & sampled_at, bool managed_ros_started)
  {
    std::unique_lock lock(mutex);
    const bool managed = managed_ros_started && config.clock_type == RCL_ROS_TIME &&
      sampled_at.get_clock_type() == RCL_ROS_TIME && sampled_at.nanoseconds() > 0;
    if (managed) {
      // This API is called at the ordered sampling boundary, not when an older retained sample
      // is delivered. Reordered delivery therefore cannot be misclassified as a clock rollback.
      if (last_ordered_managed_sample &&
        sampled_at.nanoseconds() < *last_ordered_managed_sample)
      {
        invalidate_clock_lineage();
      }
      last_ordered_managed_sample = sampled_at.nanoseconds();
    }
    return {sampled_at, clock_lineage, managed};
  }

  void notify_clock_discontinuity() noexcept
  {
    std::unique_lock lock(mutex);
    invalidate_clock_lineage();
  }

  Result<ObservationReceipt> observe_object(
    const ObjectObservation & observation,
    const rclcpp::Time & received_at,
    const std::optional<AuthorityClockSample> & clock_sample)
  {
    std::unique_lock lock(mutex);
    if (auto error = authority_sample_error(received_at, clock_sample)) {
      return Result<ObservationReceipt>::failure(std::move(*error));
    }
    if (observation.source_object_id.empty()) {
      return failure<ObservationReceipt>(
        WorldStateErrorCode::InvalidArgument,
        "source_object_id cannot be empty");
    }
    if (observation.frame_id != config.planning_frame) {
      return failure<ObservationReceipt>(
        WorldStateErrorCode::FrameMismatch,
        "observation frame '" + observation.frame_id +
        "' does not match planning frame '" +
        config.planning_frame + "'");
    }
    if (auto error = validate_optional_text(observation.sku, "sku")) {
      return Result<ObservationReceipt>::failure(std::move(*error));
    }
    if (auto error = validate_observation_time(observation.observation_time, received_at, config)) {
      return Result<ObservationReceipt>::failure(std::move(*error));
    }
    if (auto error = validate_pose(observation.pose_in_world)) {
      return Result<ObservationReceipt>::failure(std::move(*error));
    }
    if (auto error = validate_covariance(observation.pose_covariance)) {
      return Result<ObservationReceipt>::failure(std::move(*error));
    }

    const auto source = source_objects.find(observation.source_object_id);
    if (source == source_objects.end()) {
      const ObjectId object_id{next_object_id};
      const Revision new_revision = revision + 1;
      TrackedObject object{object_id,
        observation.source_object_id,
        observation.product_class,
        observation.sku,
        observation.pose_in_world,
        observation.pose_covariance,
        observation.orientation,
        TrackingState::Tracked,
        GraspState::Free,
        observation.observation_time,
        observation.observation_time,
        new_revision,
        PoseEstimate{observation.pose_in_world, observation.pose_covariance,
          observation.orientation, observation.observation_time}};

      ++next_object_id;
      revision = new_revision;
      source_objects.emplace(
        object.source_object_id,
        SourceObjectEntry{object_id, received_at.nanoseconds(), new_revision, clock_sample});
      clock_authority_armed = true;
      objects.emplace(object_id, std::move(object));
      append_event(
        WorldStateEvent{new_revision, observation.observation_time,
          EventKind::ObjectObserved, object_id, std::nullopt,
          "object created"});
      return Result<ObservationReceipt>::success(ObservationReceipt{object_id, new_revision, true});
    }

    auto object_iterator = objects.find(source->second.object_id);
    if (object_iterator == objects.end()) {
      return failure<ObservationReceipt>(
        WorldStateErrorCode::InvariantViolation,
        "source identity references a missing object");
    }
    TrackedObject & object = object_iterator->second;
    if (object.tracking_state == TrackingState::Removed) {
      return failure<ObservationReceipt>(
        WorldStateErrorCode::Removed,
        "source identity belongs to a removed object");
    }
    if (observation.observation_time.nanoseconds() <= object.observation_time.nanoseconds() ||
      observation.observation_time.nanoseconds() < object.transition_time.nanoseconds())
    {
      return failure<ObservationReceipt>(
        WorldStateErrorCode::OutOfOrder,
        "observation does not follow current object state");
    }
    if (object.product_class != ProductClass::Unknown &&
      observation.product_class != ProductClass::Unknown &&
      object.product_class != observation.product_class)
    {
      return failure<ObservationReceipt>(
        WorldStateErrorCode::IdentityConflict,
        "known product class cannot change");
    }
    if (object.sku && observation.sku && object.sku != observation.sku) {
      return failure<ObservationReceipt>(
        WorldStateErrorCode::IdentityConflict,
        "known SKU cannot change");
    }

    // A reservation pins the reserved product's pose across the pre-attach window: confirm-frame
    // noise must not rewrite the projected obstacle under a grasp staged from this pose
    // (ADR-0016, Milestone 10 §5). Freshness, tracking, identity and revision still advance;
    // the pose is replaced only when the observation diverges past the configured bound, which
    // is how a real movement is surfaced instead of hidden.
    const bool reserved_target = active_reservation &&
      active_reservation->object_id == object.id &&
      active_reservation->stage == ReservationStage::Reserved;
    const double divergence_m =
      (observation.pose_in_world.translation() - object.pose_in_world.translation()).norm();
    const bool hold_pose =
      reserved_target && divergence_m <= config.reserved_pose_divergence_bound_m;

    const Revision new_revision = ++revision;
    if (object.product_class == ProductClass::Unknown) {
      object.product_class = observation.product_class;
    }
    if (!object.sku) {
      object.sku = observation.sku;
    }
    object.tracking_state = TrackingState::Tracked;
    object.observation_time = observation.observation_time;
    object.transition_time = observation.observation_time;
    object.revision = new_revision;
    source->second.received_at_ns = received_at.nanoseconds();
    source->second.observation_revision = new_revision;
    source->second.clock_sample = clock_sample;
    clock_authority_armed = true;
    // The measurement is recorded whether or not the hold retains the act-on fields below.
    object.latest_measured = PoseEstimate{observation.pose_in_world, observation.pose_covariance,
      observation.orientation, observation.observation_time};
    if (!hold_pose) {
      object.pose_in_world = observation.pose_in_world;
      object.pose_covariance = observation.pose_covariance;
      object.orientation = observation.orientation;
    }
    std::string event_detail = "object updated";
    if (hold_pose) {
      event_detail = "object re-observed; pose held under active reservation";
    } else if (reserved_target) {
      // Carry the measured distance: it is the number the divergence bound is calibrated
      // against, and the only way to tell a false trip from a real movement after the fact.
      std::ostringstream distance;
      distance << std::fixed << std::setprecision(4) << divergence_m;
      event_detail =
        "object re-observed; pose applied beyond reserved divergence bound (distance " +
        distance.str() + " m)";
    }
    append_event(
      WorldStateEvent{
        new_revision, observation.observation_time,
        EventKind::ObjectObserved, object.id, std::nullopt, std::move(event_detail)});
    return Result<ObservationReceipt>::success(ObservationReceipt{object.id, new_revision, false});
  }

  Result<MutationReceipt> set_tracking_state(
    ObjectId object_id, TrackingState state,
    const rclcpp::Time & transition_time,
    const ObjectContextPreconditions & preconditions)
  {
    std::unique_lock lock(mutex);
    if (!object_id) {
      return failure<MutationReceipt>(WorldStateErrorCode::InvalidArgument, "object ID is invalid");
    }
    if (state == TrackingState::Tracked) {
      return failure<MutationReceipt>(
        WorldStateErrorCode::InvalidArgument,
        "only observations may restore tracked state");
    }
    if (auto error = validate_clock(transition_time, config.clock_type, "transition_time")) {
      return Result<MutationReceipt>::failure(std::move(*error));
    }
    if (active_reservation && active_reservation->object_id == object_id) {
      return failure<MutationReceipt>(
        WorldStateErrorCode::ReservationConflict,
        "reserved object lifecycle requires the active reservation token");
    }

    auto object_iterator = objects.find(object_id);
    if (object_iterator == objects.end()) {
      return failure<MutationReceipt>(WorldStateErrorCode::NotFound, "object was not found");
    }
    TrackedObject & object = object_iterator->second;
    if (object.tracking_state == TrackingState::Removed) {
      return failure<MutationReceipt>(WorldStateErrorCode::Removed, "object is removed");
    }
    if (object.revision != preconditions.object_revision) {
      return failure<MutationReceipt>(
        WorldStateErrorCode::RevisionConflict,
        "object revision changed");
    }
    if (object.tracking_state == state) {
      return failure<MutationReceipt>(
        WorldStateErrorCode::InvalidArgument,
        "tracking state is already set");
    }
    if (object.grasp_state == GraspState::Attached) {
      return failure<MutationReceipt>(
        WorldStateErrorCode::InvariantViolation,
        "attached object tracking cannot be changed");
    }
    if (transition_time.nanoseconds() < object.transition_time.nanoseconds() ||
      transition_time.nanoseconds() < object.observation_time.nanoseconds())
    {
      return failure<MutationReceipt>(
        WorldStateErrorCode::OutOfOrder,
        "lifecycle transition predates object state");
    }

    auto containing_lane = find_containing_lane(object_id);
    if (containing_lane != lanes.end()) {
      if (!preconditions.containing_lane_revision ||
        *preconditions.containing_lane_revision != containing_lane->second.revision)
      {
        return failure<MutationReceipt>(
          WorldStateErrorCode::RevisionConflict,
          "containing lane revision changed or is missing");
      }
    } else if (preconditions.containing_lane_revision) {
      return failure<MutationReceipt>(
        WorldStateErrorCode::RevisionConflict,
        "object no longer belongs to a lane");
    }

    const Revision new_revision = ++revision;
    object.tracking_state = state;
    object.transition_time = transition_time;
    object.revision = new_revision;
    EventKind event_kind = EventKind::ObjectTrackingChanged;
    std::optional<LaneId> event_lane;
    if (state == TrackingState::Removed) {
      event_kind = EventKind::ObjectRemoved;
      if (containing_lane != lanes.end()) {
        auto & contents = containing_lane->second.contents;
        contents.erase(std::remove(contents.begin(), contents.end(), object_id), contents.end());
        containing_lane->second.revision = new_revision;
        event_lane = containing_lane->first;
      }
    }
    append_event(
      WorldStateEvent{
        new_revision, transition_time, event_kind, object_id, event_lane,
        state == TrackingState::Removed ? "object removed" : "tracking state changed"});
    return Result<MutationReceipt>::success(MutationReceipt{new_revision});
  }

  Result<MutationReceipt> configure_lane(
    const LaneDefinition & definition,
    const rclcpp::Time & configured_at)
  {
    std::unique_lock lock(mutex);
    if (definition.id.value.empty()) {
      return failure<MutationReceipt>(
        WorldStateErrorCode::InvalidArgument,
        "lane ID cannot be empty");
    }
    if (!std::isfinite(definition.depth_m) || definition.depth_m <= 0.0) {
      return failure<MutationReceipt>(
        WorldStateErrorCode::InvalidArgument,
        "lane depth must be finite and positive");
    }
    if (auto error = validate_optional_text(definition.expected_sku, "expected_sku")) {
      return Result<MutationReceipt>::failure(std::move(*error));
    }
    if (auto error = validate_clock(configured_at, config.clock_type, "configured_at")) {
      return Result<MutationReceipt>::failure(std::move(*error));
    }
    if (lanes.contains(definition.id)) {
      return failure<MutationReceipt>(
        WorldStateErrorCode::IdentityConflict,
        "lane is already configured");
    }

    const Revision new_revision = ++revision;
    lanes.emplace(
      definition.id, ShelfLane{definition.id,
        definition.expected_product_class,
        definition.expected_sku,
        definition.target_count,
        {},
        {},
        definition.depth_m,
        definition.depth_m,
        false,
        configured_at,
        false,
        rclcpp::Time{std::int64_t{0}, config.clock_type},
        0,
        false,
        new_revision});
    append_event(
      WorldStateEvent{new_revision, configured_at, EventKind::LaneConfigured,
        std::nullopt, definition.id, "lane configured"});
    return Result<MutationReceipt>::success(MutationReceipt{new_revision});
  }

  Result<MutationReceipt> update_lane(
    const LaneObservation & observation,
    const rclcpp::Time & received_at,
    Revision expected_lane_revision,
    const std::optional<AuthorityClockSample> & clock_sample)
  {
    std::unique_lock lock(mutex);
    if (auto error = authority_sample_error(received_at, clock_sample)) {
      return Result<MutationReceipt>::failure(std::move(*error));
    }
    if (auto error = validate_observation_time(observation.observation_time, received_at, config)) {
      return Result<MutationReceipt>::failure(std::move(*error));
    }
    auto lane_iterator = lanes.find(observation.id);
    if (lane_iterator == lanes.end()) {
      return failure<MutationReceipt>(WorldStateErrorCode::NotFound, "lane was not found");
    }
    ShelfLane & lane = lane_iterator->second;
    if (lane.revision != expected_lane_revision) {
      return failure<MutationReceipt>(
        WorldStateErrorCode::RevisionConflict,
        "lane revision changed");
    }
    // Before the first observation, last_verified is only the configuration placeholder. It
    // cannot fence a new unarmed clock lineage. Invalidation never clears evidence_revision,
    // so every previously observed lane keeps the strict ordering fence.
    if (lane.evidence_revision != 0 &&
      observation.observation_time.nanoseconds() <= lane.last_verified.nanoseconds())
    {
      return failure<MutationReceipt>(
        WorldStateErrorCode::OutOfOrder,
        "lane observation is not newer");
    }
    if (!std::isfinite(observation.available_depth_m) || observation.available_depth_m < 0.0 ||
      observation.available_depth_m > lane.depth_m)
    {
      return failure<MutationReceipt>(
        WorldStateErrorCode::InvalidArgument,
        "available lane depth is outside lane bounds");
    }
    if (std::ranges::any_of(
        observation.observed_source_object_ids,
        [](const std::string & source_id) {return source_id.empty();}) ||
      !std::ranges::is_sorted(observation.observed_source_object_ids) ||
      std::ranges::adjacent_find(observation.observed_source_object_ids) !=
      observation.observed_source_object_ids.end())
    {
      return failure<MutationReceipt>(
        WorldStateErrorCode::InvalidArgument,
        "observed lane source IDs must be non-empty, sorted, and unique");
    }

    // ADR-0012 reconciliation, by length, before the evidence lands: the FIFO contents ledger
    // (push_back on placement, so front() is customer-facing) is compared against the measured
    // column length at the catalogued pitch. Within half a pitch they agree; a shortfall of one
    // or more whole pitches pops that many front entries as sales; growth beyond what the ledger
    // plus any in-flight placement can explain withdraws identity trust (latched) while the
    // measured geometry stands.
    //
    // Two boundaries shape the comparison, and both are load-bearing under the default
    // continuous lane-evidence topic (ground truth publishes while a reservation is Attached):
    //
    // - Sales are judged against contents alone. An in-flight placement has not been pushed yet,
    //   so counting it on the shortfall side would read every pre-insert survey — column still at
    //   the old length — as a phantom front sale (stocked lane) or an unpoppable fractional
    //   mismatch (empty lane) and latch `ledger_unreliable` on the first attach.
    // - Growth is judged against contents plus that in-flight placement, so the retreat survey
    //   that first shows the new pitch agrees instead of reading as unexplained growth.
    //
    // Length reconciliation also assumes the column is the product the ledger names. After an
    // identity policy change that leaves the old column in place, the new intent's pitch does not
    // describe the measured column, and popping at it would destroy the wrong-product conflict
    // selection is required to report. Such a ledger is left untouched (not marked unreliable —
    // the divergence is explained); selection reports `lane_wrong_product` instead. Without a
    // catalogued pitch the comparison is not computable either, so the ledger is left untouched
    // rather than guessed at; selection fails closed on the missing pitch instead.
    double measured_length_m = 0.0;
    std::size_t sold_count = 0U;
    bool ledger_diverged = false;
    std::string divergence_detail;
    if (!lane.ledger_unreliable) {
      bool ledger_identity_conflicts = false;
      for (const ObjectId content_id : lane.contents) {
        const auto content_object = objects.find(content_id);
        if (content_object == objects.end() ||
          !lane_accepts_object(lane, content_object->second))
        {
          ledger_identity_conflicts = true;
          break;
        }
      }
      const auto profile = resolve_product_lane_profile(
        config.product_lane_profiles, lane.expected_product_class, lane.expected_sku);
      if (!ledger_identity_conflicts && profile &&
        std::isfinite(profile->column_pitch_m) && profile->column_pitch_m > 0.0)
      {
        const double pitch_m = profile->column_pitch_m;
        measured_length_m = lane.depth_m - observation.available_depth_m;
        const std::size_t pending_placements =
          (active_reservation &&
          active_reservation->destination_lane == lane.id &&
          active_reservation->stage == ReservationStage::Attached) ? 1U : 0U;
        const double settled_length_m =
          static_cast<double>(lane.contents.size()) * pitch_m;
        const double expected_maximum_m =
          static_cast<double>(lane.contents.size() + pending_placements) * pitch_m;
        const double tolerance_m = 0.5 * pitch_m;
        if (measured_length_m - expected_maximum_m > tolerance_m) {
          ledger_diverged = true;
          std::ostringstream detail;
          detail << "ledger unreliable: measured column " << format_depth(measured_length_m)
                 << " exceeds ledger length " << format_depth(expected_maximum_m) << " by "
                 << format_depth(measured_length_m - expected_maximum_m)
                 << " (half-pitch tolerance " << format_depth(tolerance_m)
                 << "); geometry retained, contents withdrawn";
          divergence_detail = detail.str();
        } else if (settled_length_m - measured_length_m > tolerance_m) {
          const auto sold =
            static_cast<std::size_t>(
            std::llround((settled_length_m - measured_length_m) / pitch_m));
          if (sold >= 1U && sold <= lane.contents.size()) {
            sold_count = sold;
          } else {
            ledger_diverged = true;
            std::ostringstream detail;
            detail << "ledger unreliable: measured column " << format_depth(measured_length_m)
                   << " falls " << format_depth(settled_length_m - measured_length_m)
                   << " below settled ledger length " << format_depth(settled_length_m)
                   << " with no whole front-entry sale to pop (" << lane.contents.size()
                   << " entries, in-flight " << pending_placements << ", half-pitch tolerance "
                   << format_depth(tolerance_m) << "); geometry retained, contents withdrawn";
            divergence_detail = detail.str();
          }
        }
      }
    }

    std::vector<ObjectId> sold_ids;
    if (sold_count > 0U) {
      sold_ids.assign(lane.contents.begin(), lane.contents.begin() + sold_count);
      lane.contents.erase(lane.contents.begin(), lane.contents.begin() + sold_count);
    }

    const Revision new_revision = ++revision;
    lane.observed_source_object_ids = observation.observed_source_object_ids;
    lane.available_depth_m = observation.available_depth_m;
    lane.obstructed = observation.obstructed;
    lane.last_verified = observation.observation_time;
    clock_authority_armed = true;
    lane.evidence_invalidated = false;
    lane.evidence_invalidated_at = rclcpp::Time{std::int64_t{0}, config.clock_type};
    lane.evidence_revision = new_revision;
    if (ledger_diverged) {
      lane.ledger_unreliable = true;
    }
    lane.revision = new_revision;
    // A sold product has left the shop: mark it Removed so it stops counting as stock, stops
    // being projected, and refuses further observations, exactly as an operator removal does.
    for (const ObjectId sold_id : sold_ids) {
      auto sold_object = objects.find(sold_id);
      if (sold_object != objects.end() &&
        sold_object->second.tracking_state == TrackingState::Tracked &&
        sold_object->second.grasp_state == GraspState::Free)
      {
        sold_object->second.tracking_state = TrackingState::Removed;
        sold_object->second.transition_time = observation.observation_time;
        sold_object->second.revision = new_revision;
      }
      append_event(
        WorldStateEvent{new_revision, observation.observation_time,
          EventKind::ObjectRemoved, sold_id, lane.id,
          "sold: popped from the front of the lane ledger by reconciliation"});
    }
    std::ostringstream verified_detail;
    verified_detail << "lane verified";
    if (sold_count > 0U) {
      verified_detail << "; ledger reconciled " << sold_count << " front sale(s)";
    }
    append_event(
      WorldStateEvent{new_revision, observation.observation_time,
        EventKind::LaneVerified, std::nullopt, lane.id, verified_detail.str()});
    if (ledger_diverged) {
      append_event(
        WorldStateEvent{new_revision, observation.observation_time,
          EventKind::LaneLedgerUnreliable, std::nullopt, lane.id, divergence_detail});
    }
    return Result<MutationReceipt>::success(MutationReceipt{new_revision});
  }

  Result<MutationReceipt> invalidate_lane_evidence(
    LaneId lane_id,
    const rclcpp::Time & invalidated_at,
    Revision expected_lane_revision)
  {
    std::unique_lock lock(mutex);
    if (auto error = validate_clock(invalidated_at, config.clock_type, "invalidated_at")) {
      return Result<MutationReceipt>::failure(std::move(*error));
    }
    auto lane_iterator = lanes.find(lane_id);
    if (lane_iterator == lanes.end()) {
      return failure<MutationReceipt>(WorldStateErrorCode::NotFound, "lane was not found");
    }
    ShelfLane & lane = lane_iterator->second;
    if (lane.revision != expected_lane_revision) {
      return failure<MutationReceipt>(
        WorldStateErrorCode::RevisionConflict,
        "lane revision changed");
    }
    if (lane.evidence_revision == 0) {
      return failure<MutationReceipt>(
        WorldStateErrorCode::StaleObservation,
        "lane has no depletion evidence to invalidate");
    }
    if (lane.evidence_invalidated) {
      return Result<MutationReceipt>::success(MutationReceipt{lane.revision});
    }
    const Revision new_revision = ++revision;
    lane.evidence_invalidated = true;
    lane.evidence_invalidated_at = invalidated_at;
    lane.revision = new_revision;
    append_event(
      WorldStateEvent{new_revision, invalidated_at, EventKind::LaneVerified, std::nullopt, lane.id,
        "lane evidence invalidated"});
    return Result<MutationReceipt>::success(MutationReceipt{new_revision});
  }

  Result<MutationReceipt> set_lane_policy(
    const LanePolicy & policy,
    Revision expected_lane_revision,
    const rclcpp::Time & changed_at,
    const LanePolicyPersist & persist)
  {
    std::unique_lock lock(mutex);
    if (!persist) {
      return failure<MutationReceipt>(
        WorldStateErrorCode::InvalidArgument,
        "lane policy changes require a persistence callback");
    }
    if (policy.id.value.empty()) {
      return failure<MutationReceipt>(
        WorldStateErrorCode::InvalidArgument,
        "lane ID cannot be empty");
    }
    if (policy.expected_product_class == ProductClass::Unknown) {
      return failure<MutationReceipt>(
        WorldStateErrorCode::InvalidArgument,
        "lane policy requires a known expected product class");
    }
    if (auto error = validate_optional_text(policy.expected_sku, "expected_sku")) {
      return Result<MutationReceipt>::failure(std::move(*error));
    }
    if (auto error = validate_clock(changed_at, config.clock_type, "changed_at")) {
      return Result<MutationReceipt>::failure(std::move(*error));
    }
    auto lane_iterator = lanes.find(policy.id);
    if (lane_iterator == lanes.end()) {
      return failure<MutationReceipt>(WorldStateErrorCode::NotFound, "lane was not found");
    }
    ShelfLane & lane = lane_iterator->second;
    if (lane.revision != expected_lane_revision) {
      return failure<MutationReceipt>(
        WorldStateErrorCode::RevisionConflict,
        "lane revision changed");
    }
    if (lane.expected_product_class == policy.expected_product_class &&
      lane.expected_sku == policy.expected_sku && lane.target_count == policy.target_count)
    {
      // The requested policy is already in force: succeed without disturbing the revision any
      // reservation or selection may be holding, and without rewriting a file that already
      // says this.
      return Result<MutationReceipt>::success(MutationReceipt{revision});
    }

    // The full table is written first, under the same lock as the in-memory change, so a stale
    // revision or a failed write refuses without touching either the lane or the policy file.
    std::vector<LanePolicy> table;
    table.reserve(lanes.size());
    for (const auto & [lane_id, existing] : lanes) {
      table.push_back(
        lane_id == policy.id ? policy :
        LanePolicy{lane_id, existing.expected_product_class, existing.expected_sku,
          existing.target_count});
    }
    if (!persist(table)) {
      return failure<MutationReceipt>(
        WorldStateErrorCode::InvariantViolation,
        "lane policy persistence failed; the lane and the policy file are unchanged");
    }

    const Revision new_revision = ++revision;
    lane.expected_product_class = policy.expected_product_class;
    lane.expected_sku = policy.expected_sku;
    lane.target_count = policy.target_count;
    if (lane.evidence_revision != 0 && !lane.evidence_invalidated) {
      // The survey that produced this evidence was taken under the old intent; the policy
      // change invalidates it until the next accepted observation.
      lane.evidence_invalidated = true;
      lane.evidence_invalidated_at = changed_at;
    }
    lane.revision = new_revision;
    append_event(
      WorldStateEvent{new_revision, changed_at, EventKind::LanePolicySet, std::nullopt, lane.id,
        "lane policy set"});
    return Result<MutationReceipt>::success(MutationReceipt{new_revision});
  }

  Result<MutationReceipt> observe_robot_telemetry(
    const RobotTelemetryObservation & observation,
    const rclcpp::Time & received_at,
    const std::optional<AuthorityClockSample> & clock_sample)
  {
    std::unique_lock lock(mutex);
    if (auto error = authority_sample_error(received_at, clock_sample)) {
      return Result<MutationReceipt>::failure(std::move(*error));
    }
    if (!valid_telemetry_source_id(observation.source_id)) {
      return failure<MutationReceipt>(
        WorldStateErrorCode::InvalidArgument,
        "robot telemetry source ID is invalid");
    }
    if (robot.telemetry_revision != 0 &&
      observation.source_id != robot.telemetry_source_id)
    {
      return failure<MutationReceipt>(
        WorldStateErrorCode::IdentityConflict,
        "robot telemetry source identity changed");
    }
    if (observation.observation_time.nanoseconds() <= 0) {
      return failure<MutationReceipt>(
        WorldStateErrorCode::InvalidArgument,
        "robot telemetry timestamp must be positive");
    }
    if (auto error = validate_observation_time(observation.observation_time, received_at, config)) {
      return Result<MutationReceipt>::failure(std::move(*error));
    }
    if (observation.observation_time.nanoseconds() > received_at.nanoseconds()) {
      return failure<MutationReceipt>(
        WorldStateErrorCode::FutureObservation,
        "robot telemetry cannot postdate its authoritative receipt time");
    }
    if (!std::isfinite(observation.rail_position) ||
      observation.rail_position < config.rail_lower_limit ||
      observation.rail_position > config.rail_upper_limit)
    {
      return failure<MutationReceipt>(
        WorldStateErrorCode::InvalidArgument,
        "robot rail position is outside configured limits");
    }
    if (!std::isfinite(observation.rail_velocity)) {
      return failure<MutationReceipt>(
        WorldStateErrorCode::InvalidArgument,
        "robot rail velocity must be finite");
    }
    for (std::size_t index = 0; index < kArmJointCount; ++index) {
      const double position = observation.joint_positions[index];
      if (!std::isfinite(position) ||
        position < config.joint_lower_limits[index] - config.joint_limit_tolerance ||
        position > config.joint_upper_limits[index] + config.joint_limit_tolerance)
      {
        std::ostringstream detail;
        detail << "robot joint " << index + 1 << " position is outside configured limits";
        return failure<MutationReceipt>(WorldStateErrorCode::InvalidArgument, detail.str());
      }
      if (!std::isfinite(observation.joint_velocities[index])) {
        std::ostringstream detail;
        detail << "robot joint " << index + 1 << " velocity must be finite";
        return failure<MutationReceipt>(WorldStateErrorCode::InvalidArgument, detail.str());
      }
    }
    for (std::size_t index = 0; index < kGripperJointCount; ++index) {
      const double position = observation.gripper_joint_positions[index];
      if (!std::isfinite(position) || position < config.gripper_joint_lower_limits[index] ||
        position > config.gripper_joint_upper_limits[index])
      {
        std::ostringstream detail;
        detail << "robot gripper joint " << index + 1 <<
          " position is outside configured limits";
        return failure<MutationReceipt>(WorldStateErrorCode::InvalidArgument, detail.str());
      }
      if (!std::isfinite(observation.gripper_joint_velocities[index])) {
        std::ostringstream detail;
        detail << "robot gripper joint " << index + 1 << " velocity must be finite";
        return failure<MutationReceipt>(WorldStateErrorCode::InvalidArgument, detail.str());
      }
    }
    if (robot.telemetry_revision != 0 &&
      observation.observation_time.nanoseconds() <= robot.telemetry_time.nanoseconds())
    {
      return failure<MutationReceipt>(
        WorldStateErrorCode::OutOfOrder,
        "robot telemetry is not newer");
    }

    const Revision new_revision = ++revision;
    robot.joint_positions = observation.joint_positions;
    robot.joint_velocities = observation.joint_velocities;
    robot.rail_position = observation.rail_position;
    robot.rail_velocity = observation.rail_velocity;
    robot.gripper_joint_positions = observation.gripper_joint_positions;
    robot.gripper_joint_velocities = observation.gripper_joint_velocities;
    robot.telemetry_time = observation.observation_time;
    clock_authority_armed = true;
    robot.telemetry_source_id = observation.source_id;
    robot.telemetry_revision = new_revision;
    robot.revision = new_revision;
    return Result<MutationReceipt>::success(MutationReceipt{new_revision});
  }

  Result<MutationReceipt> commit_attachment(
    ObjectId object_id, const Eigen::Isometry3d & grasp_center_from_object,
    const rclcpp::Time & committed_at,
    const AttachPreconditions & preconditions)
  {
    std::unique_lock lock(mutex);
    if (auto error = validate_clock(committed_at, config.clock_type, "committed_at")) {
      return Result<MutationReceipt>::failure(std::move(*error));
    }
    if (!valid_rigid_transform(grasp_center_from_object)) {
      return failure<MutationReceipt>(
        WorldStateErrorCode::InvalidArgument, "grasp transform is not a finite rigid transform");
    }
    if (active_reservation) {
      return failure<MutationReceipt>(
        WorldStateErrorCode::ReservationConflict,
        "legacy attachment is disabled while a task reservation is active");
    }
    auto object_iterator = objects.find(object_id);
    if (object_iterator == objects.end()) {
      return failure<MutationReceipt>(WorldStateErrorCode::NotFound, "object was not found");
    }
    TrackedObject & object = object_iterator->second;
    if (object.tracking_state == TrackingState::Removed) {
      return failure<MutationReceipt>(WorldStateErrorCode::Removed, "object is removed");
    }
    if (object.revision != preconditions.object_revision ||
      robot.revision != preconditions.robot_revision)
    {
      return failure<MutationReceipt>(
        WorldStateErrorCode::RevisionConflict,
        "object or robot revision changed");
    }
    if (object.grasp_state != GraspState::Free || robot.held_object) {
      return failure<MutationReceipt>(
        WorldStateErrorCode::InvariantViolation,
        "object or robot is already attached");
    }
    if (object.tracking_state != TrackingState::Tracked) {
      return failure<MutationReceipt>(
        WorldStateErrorCode::InvariantViolation,
        "only a tracked object may be attached");
    }
    if (committed_at.nanoseconds() < object.transition_time.nanoseconds() ||
      committed_at.nanoseconds() < robot.telemetry_time.nanoseconds())
    {
      return failure<MutationReceipt>(
        WorldStateErrorCode::OutOfOrder,
        "attachment predates object or robot state");
    }

    auto source_lane = find_containing_lane(object_id);
    if (source_lane != lanes.end()) {
      if (!preconditions.source_lane_revision ||
        *preconditions.source_lane_revision != source_lane->second.revision)
      {
        return failure<MutationReceipt>(
          WorldStateErrorCode::RevisionConflict,
          "source lane revision changed or is missing");
      }
    } else if (preconditions.source_lane_revision) {
      return failure<MutationReceipt>(
        WorldStateErrorCode::RevisionConflict,
        "object no longer belongs to the source lane");
    }

    const Revision new_revision = ++revision;
    std::optional<LaneId> event_lane;
    if (source_lane != lanes.end()) {
      auto & contents = source_lane->second.contents;
      contents.erase(std::remove(contents.begin(), contents.end(), object_id), contents.end());
      source_lane->second.revision = new_revision;
      event_lane = source_lane->first;
    }
    object.grasp_state = GraspState::Attached;
    object.transition_time = committed_at;
    object.revision = new_revision;
    robot.held_object = object_id;
    robot.grasp_center_from_held_object = grasp_center_from_object;
    robot.revision = new_revision;
    append_event(
      WorldStateEvent{new_revision, committed_at, EventKind::ObjectAttached, object_id,
        event_lane, "attachment committed"});
    return Result<MutationReceipt>::success(MutationReceipt{new_revision});
  }

  Result<MutationReceipt> commit_detachment(
    ObjectId object_id,
    std::optional<LaneId> destination_lane,
    const rclcpp::Time & committed_at,
    const DetachPreconditions & preconditions)
  {
    std::unique_lock lock(mutex);
    if (auto error = validate_clock(committed_at, config.clock_type, "committed_at")) {
      return Result<MutationReceipt>::failure(std::move(*error));
    }
    if (active_reservation) {
      return failure<MutationReceipt>(
        WorldStateErrorCode::ReservationConflict,
        "legacy detachment is disabled while a task reservation is active");
    }
    auto object_iterator = objects.find(object_id);
    if (object_iterator == objects.end()) {
      return failure<MutationReceipt>(WorldStateErrorCode::NotFound, "object was not found");
    }
    TrackedObject & object = object_iterator->second;
    if (object.revision != preconditions.object_revision ||
      robot.revision != preconditions.robot_revision)
    {
      return failure<MutationReceipt>(
        WorldStateErrorCode::RevisionConflict,
        "object or robot revision changed");
    }
    if (object.grasp_state != GraspState::Attached || robot.held_object != object_id) {
      return failure<MutationReceipt>(
        WorldStateErrorCode::InvariantViolation,
        "object is not consistently attached and held");
    }
    if (committed_at.nanoseconds() < object.transition_time.nanoseconds() ||
      committed_at.nanoseconds() < robot.telemetry_time.nanoseconds())
    {
      return failure<MutationReceipt>(
        WorldStateErrorCode::OutOfOrder,
        "detachment predates object or robot state");
    }
    if (find_containing_lane(object_id) != lanes.end()) {
      return failure<MutationReceipt>(
        WorldStateErrorCode::InvariantViolation,
        "attached object already belongs to a lane");
    }

    auto lane_iterator = lanes.end();
    if (destination_lane) {
      lane_iterator = lanes.find(*destination_lane);
      if (lane_iterator == lanes.end()) {
        return failure<MutationReceipt>(
          WorldStateErrorCode::NotFound,
          "destination lane was not found");
      }
      if (!preconditions.destination_lane_revision ||
        *preconditions.destination_lane_revision != lane_iterator->second.revision)
      {
        return failure<MutationReceipt>(
          WorldStateErrorCode::RevisionConflict,
          "destination lane revision changed or is missing");
      }
      if (!lane_accepts_object(lane_iterator->second, object)) {
        return failure<MutationReceipt>(
          WorldStateErrorCode::InvariantViolation,
          "object identity is incompatible with destination lane");
      }
    } else if (preconditions.destination_lane_revision) {
      return failure<MutationReceipt>(
        WorldStateErrorCode::RevisionConflict,
        "destination lane is no longer selected");
    }

    const Revision new_revision = ++revision;
    object.grasp_state = GraspState::Free;
    object.transition_time = committed_at;
    object.revision = new_revision;
    robot.held_object.reset();
    robot.grasp_center_from_held_object = Eigen::Isometry3d::Identity();
    robot.revision = new_revision;
    if (lane_iterator != lanes.end()) {
      lane_iterator->second.contents.push_back(object_id);
      lane_iterator->second.revision = new_revision;
    }
    append_event(
      WorldStateEvent{new_revision, committed_at, EventKind::ObjectDetached, object_id,
        destination_lane, "detachment committed"});
    return Result<MutationReceipt>::success(MutationReceipt{new_revision});
  }

  Result<ReservationReceipt> reserve_task(
    const ReserveTaskRequest & request,
    const rclcpp::Time & reserved_at,
    const std::optional<AuthorityClockSample> & clock_sample)
  {
    std::unique_lock lock(mutex);
    if (auto error = authority_sample_error(reserved_at, clock_sample)) {
      return Result<ReservationReceipt>::failure(std::move(*error));
    }
    if (auto error = validate_clock(reserved_at, config.clock_type, "reserved_at")) {
      return Result<ReservationReceipt>::failure(std::move(*error));
    }
    if (!valid_operation_id(request.request_id) || !request.object_id ||
      request.destination_lane.value.empty() || request.selected_snapshot_revision == 0 ||
      request.object_revision == 0 || request.destination_lane_revision == 0 ||
      request.source_lane.has_value() != request.source_lane_revision.has_value() ||
      (request.source_lane && request.source_lane->value.empty()) ||
      (request.source_lane && *request.source_lane == request.destination_lane))
    {
      return failure<ReservationReceipt>(
        WorldStateErrorCode::InvalidArgument,
        "reservation request identity or revisions are invalid");
    }

    const std::string fingerprint = reserve_fingerprint(request);
    const auto replay = operation_journal.find(request.request_id);
    if (replay != operation_journal.end()) {
      if (replay->second.kind != JournalKind::Reserve ||
        replay->second.fingerprint != fingerprint || !replay->second.reservation)
      {
        return failure<ReservationReceipt>(
          WorldStateErrorCode::IdempotencyConflict,
          "request ID was already used with a different operation or payload");
      }
      return Result<ReservationReceipt>::success(
        ReservationReceipt{
          replay->second.revision, replay->second.token, *replay->second.reservation});
    }
    if (active_reservation) {
      return failure<ReservationReceipt>(
        WorldStateErrorCode::ReservationConflict,
        "another task reservation is active");
    }
    if (!has_journal_capacity(4)) {
      return failure<ReservationReceipt>(
        WorldStateErrorCode::ResourceExhausted,
        "operation journal cannot reserve the complete task lifecycle");
    }

    const auto object_iterator = objects.find(request.object_id);
    const auto destination_iterator = lanes.find(request.destination_lane);
    if (object_iterator == objects.end() || destination_iterator == lanes.end()) {
      return failure<ReservationReceipt>(
        WorldStateErrorCode::NotFound,
        "selected object or destination lane was not found");
    }
    const TrackedObject & object = object_iterator->second;
    const ShelfLane & destination = destination_iterator->second;
    if (request.selected_snapshot_revision > revision ||
      request.selected_snapshot_revision < request.object_revision ||
      request.selected_snapshot_revision < request.destination_lane_revision ||
      object.revision != request.object_revision ||
      destination.revision != request.destination_lane_revision)
    {
      return failure<ReservationReceipt>(
        WorldStateErrorCode::RevisionConflict,
        "selected snapshot entity revisions changed");
    }
    const std::optional<LaneId> membership = containing_lane_id(request.object_id);
    if (membership != request.source_lane) {
      return failure<ReservationReceipt>(
        WorldStateErrorCode::PredicateFailed,
        "object source membership changed");
    }
    if (request.source_lane) {
      const auto source = lanes.find(*request.source_lane);
      if (source == lanes.end()) {
        return failure<ReservationReceipt>(
          WorldStateErrorCode::NotFound,
          "selected source lane was not found");
      }
      if (source->second.revision != *request.source_lane_revision ||
        request.selected_snapshot_revision < *request.source_lane_revision)
      {
        return failure<ReservationReceipt>(
          WorldStateErrorCode::RevisionConflict,
          "selected source lane revision changed");
      }
    }
    if (robot.telemetry_revision == 0) {
      return failure<ReservationReceipt>(
        WorldStateErrorCode::StaleObservation,
        "robot telemetry has not been observed");
    }
    if (object.source_object_id.empty() || object.product_class == ProductClass::Unknown ||
      object.tracking_state != TrackingState::Tracked || object.grasp_state != GraspState::Free ||
      robot.held_object || robot.fault_state != FaultState::None ||
      (robot.task_phase != TaskPhase::Idle && robot.task_phase != TaskPhase::ValidatingScene) ||
      !lane_accepts_object(destination, object) ||
      destination.evidence_revision == 0 || destination.obstructed)
    {
      return failure<ReservationReceipt>(
        WorldStateErrorCode::PredicateFailed,
        "object, destination, or robot no longer satisfies reservation predicates");
    }
    // A gravity-fed lane holds a column against the front rail, so a placement needs the free
    // depth behind it to take one more of this product.
    const auto profile = resolve_product_lane_profile(
      config.product_lane_profiles, object.product_class, object.sku);
    if (!profile) {
      return failure<ReservationReceipt>(
        WorldStateErrorCode::PredicateFailed,
        "selected product has no catalogued lane depth, so no lane can be shown to hold it");
    }
    if (destination.available_depth_m + config.placement_entry_depth_tolerance_m <
      profile->required_rear_depth_m)
    {
      return failure<ReservationReceipt>(
        WorldStateErrorCode::PredicateFailed,
        "destination lane has no room at its rear for another product: " +
        format_depth(destination.available_depth_m) + " free, " +
        format_depth(profile->required_rear_depth_m) + " required" +
        describe_lane_evidence(destination, object, reserved_at));
    }
    if (reserved_at.nanoseconds() < object.transition_time.nanoseconds() ||
      reserved_at.nanoseconds() < destination.last_verified.nanoseconds() ||
      reserved_at.nanoseconds() < robot.telemetry_time.nanoseconds())
    {
      return failure<ReservationReceipt>(
        WorldStateErrorCode::OutOfOrder,
        "reservation time predates selected semantic state");
    }
    const auto maximum_age_ns = config.maximum_observation_age.count();
    if (reserved_at.nanoseconds() - object.observation_time.nanoseconds() > maximum_age_ns) {
      return failure<ReservationReceipt>(
        WorldStateErrorCode::StaleObservation,
        "selected object evidence is stale at reservation");
    }
    if (auto error = lane_evidence_validity_error(
        destination, reserved_at, config, "destination"))
    {
      return Result<ReservationReceipt>::failure(std::move(*error));
    }
    if (reserved_at.nanoseconds() - robot.telemetry_time.nanoseconds() > maximum_age_ns) {
      return failure<ReservationReceipt>(
        WorldStateErrorCode::StaleObservation,
        "robot telemetry is stale at reservation");
    }

    std::optional<std::string> token;
    for (std::size_t attempt = 0; attempt < 8 && !token; ++attempt) {
      auto candidate = generate_token();
      if (!candidate) {
        break;
      }
      const bool collision =
        std::any_of(
        operation_journal.begin(), operation_journal.end(),
        [&candidate](const auto & entry) {return entry.second.token == *candidate;});
      if (!collision) {
        token = std::move(candidate);
      }
    }
    if (!token) {
      return failure<ReservationReceipt>(
        WorldStateErrorCode::ResourceExhausted,
        "secure reservation token generation failed");
    }

    const Revision new_revision = ++revision;
    TaskReservation reservation{next_reservation_id++,
      request.request_id,
      object.id,
      object.source_object_id,
      object.product_class,
      object.sku,
      request.source_lane,
      request.destination_lane,
      ReservationStage::Reserved,
      false,
      reserved_at,
      new_revision,
      robot.telemetry_revision,
      destination.available_depth_m,
      destination.observed_source_object_ids,
      new_revision,
      destination.expected_product_class,
      destination.expected_sku};
    robot.task_phase = TaskPhase::Executing;
    robot.revision = new_revision;
    clock_authority_armed = true;
    active_token = *token;
    active_reservation = reservation;
    append_event(
      WorldStateEvent{
        new_revision, reserved_at, EventKind::TaskReserved, object.id, request.destination_lane,
        "task reservation " + std::to_string(reservation.reservation_id) + " created"});
    operation_journal.emplace(
      request.request_id, JournalEntry{JournalKind::Reserve, fingerprint,
        new_revision, *token, reservation});
    return Result<ReservationReceipt>::success(
      ReservationReceipt{new_revision, *token, std::move(reservation)});
  }

  Result<ReservationValidation> validate_task_reservation(const std::string & token) const
  {
    std::shared_lock lock(mutex);
    if (token.empty() || !active_reservation || token != active_token) {
      return failure<ReservationValidation>(
        WorldStateErrorCode::TokenMismatch,
        "task reservation token is not active");
    }
    if (auto error = reservation_predicate_error(*active_reservation)) {
      return Result<ReservationValidation>::failure(std::move(*error));
    }
    return Result<ReservationValidation>::success(
      ReservationValidation{revision, *active_reservation});
  }

  Result<ExecutionWorldAuthorityProof> validate_execution_world_authority(
    const std::string & token,
    const ExecutionWorldAuthorityExpectation & expected) const
  {
    std::shared_lock lock(mutex);
    if (token.empty() || !active_reservation || token != active_token) {
      return failure<ExecutionWorldAuthorityProof>(
        WorldStateErrorCode::TokenMismatch,
        "task reservation token is not active");
    }
    if (expected.reservation_id == 0U || expected.reservation_revision == 0U ||
      !expected.object_id || expected.destination_lane.value.empty())
    {
      return failure<ExecutionWorldAuthorityProof>(
        WorldStateErrorCode::InvalidArgument,
        "expected execution authority identity is malformed");
    }
    if (active_reservation->reservation_id != expected.reservation_id ||
      active_reservation->revision != expected.reservation_revision)
    {
      return failure<ExecutionWorldAuthorityProof>(
        WorldStateErrorCode::RevisionConflict,
        "expected reservation identity or revision is no longer active");
    }
    if (active_reservation->object_id != expected.object_id ||
      active_reservation->destination_lane != expected.destination_lane)
    {
      return failure<ExecutionWorldAuthorityProof>(
        WorldStateErrorCode::IdentityConflict,
        "requested execution selection does not match the active reservation");
    }
    if (auto error = reservation_predicate_error(*active_reservation)) {
      return Result<ExecutionWorldAuthorityProof>::failure(std::move(*error));
    }

    if (clock_authority_inhibited) {
      return failure<ExecutionWorldAuthorityProof>(
        WorldStateErrorCode::ClockAuthorityInhibited, "clock authority is inhibited");
    }

    const TrackedObject & object = objects.at(active_reservation->object_id);
    const ShelfLane & destination = lanes.at(active_reservation->destination_lane);
    std::optional<ShelfLane> source;
    if (active_reservation->source_lane) {
      source = lanes.at(*active_reservation->source_lane);
    }
    return Result<ExecutionWorldAuthorityProof>::success(
      ExecutionWorldAuthorityProof{
        revision, config.planning_frame, *active_reservation, object, destination,
        std::move(source), robot});
  }

  Result<ReservationReceipt> checkpoint_reserved_task(const ReservedTaskCheckpoint & checkpoint)
  {
    std::unique_lock lock(mutex);
    if (!valid_operation_id(checkpoint.operation_id) || checkpoint.token.empty() ||
      checkpoint.expected_reservation_id == 0 || checkpoint.expected_reservation_revision == 0)
    {
      return failure<ReservationReceipt>(
        WorldStateErrorCode::InvalidArgument,
        "checkpoint token or operation ID is invalid");
    }
    if (auto error =
      validate_clock(checkpoint.update_time, config.clock_type, "update_time"))
    {
      return Result<ReservationReceipt>::failure(std::move(*error));
    }
    if (!valid_active_task_semantics(checkpoint.task_phase, checkpoint.fault_state)) {
      return failure<ReservationReceipt>(
        WorldStateErrorCode::InvalidTransition,
        "checkpoint task/fault phase is invalid");
    }
    const std::string fingerprint = checkpoint_fingerprint(checkpoint);
    if (auto replay =
      replay_reservation(checkpoint.operation_id, JournalKind::Checkpoint, fingerprint))
    {
      return *replay;
    }
    if (!active_reservation || checkpoint.token != active_token) {
      return failure<ReservationReceipt>(
        WorldStateErrorCode::TokenMismatch, "task reservation token is not active");
    }
    if (checkpoint.expected_reservation_id != active_reservation->reservation_id ||
      checkpoint.expected_reservation_stage != active_reservation->stage ||
      checkpoint.expected_reservation_revision != active_reservation->revision)
    {
      return failure<ReservationReceipt>(
        WorldStateErrorCode::RevisionConflict,
        "reservation identity, stage, or revision changed before checkpoint");
    }
    if (checkpoint.update_time.nanoseconds() < robot.telemetry_time.nanoseconds() ||
      checkpoint.update_time.nanoseconds() < active_reservation->created_at.nanoseconds())
    {
      return failure<ReservationReceipt>(
        WorldStateErrorCode::OutOfOrder,
        "checkpoint time predates robot telemetry or reservation creation");
    }
    if (auto error =
      prepare_mutation(
        checkpoint.operation_id, checkpoint.token,
        checkpoint.expected_reservation_stage, JournalKind::Checkpoint, true))
    {
      return Result<ReservationReceipt>::failure(std::move(*error));
    }
    const Revision new_revision = ++revision;
    robot.task_phase = checkpoint.task_phase;
    robot.fault_state = checkpoint.fault_state;
    robot.revision = new_revision;
    active_reservation->revision = new_revision;
    append_event(
      WorldStateEvent{new_revision, checkpoint.update_time,
        EventKind::TaskCheckpointed, active_reservation->object_id,
        active_reservation->destination_lane,
        "reserved task checkpointed"});
    journal_reservation(
      checkpoint.operation_id, JournalKind::Checkpoint, fingerprint,
      new_revision);
    return Result<ReservationReceipt>::success(
      ReservationReceipt{new_revision, active_token, *active_reservation});
  }

  ReservedAttachmentResult commit_reserved_attachment(
    const ReservedAttachmentRequest & request,
    const rclcpp::Time & committed_at,
    const std::optional<AuthorityClockSample> & clock_sample)
  {
    std::unique_lock lock(mutex);
    if (!valid_operation_id(request.operation_id) || request.token.empty()) {
      return ReservedAttachmentResult::failure(
        {WorldStateErrorCode::InvalidArgument, "attachment token or operation ID is invalid"});
    }
    if (!valid_rigid_transform(request.grasp_center_from_held_object)) {
      return ReservedAttachmentResult::failure(
        {WorldStateErrorCode::InvalidArgument, "grasp transform is not a finite rigid transform"});
    }
    if (auto error = validate_clock(committed_at, config.clock_type, "committed_at")) {
      return ReservedAttachmentResult::failure(std::move(*error));
    }
    const std::string fingerprint = attachment_fingerprint(request);
    if (auto replay = replay_reservation(request.operation_id, JournalKind::Attach, fingerprint)) {
      if (!*replay) {
        return ReservedAttachmentResult::failure(replay->error());
      }
      if (auto error = authority_sample_error(committed_at, clock_sample)) {
        if (error->code == WorldStateErrorCode::ClockAuthorityInhibited) {
          return ReservedAttachmentResult::inhibited_history(std::move(replay->value()));
        }
        return ReservedAttachmentResult::failure(std::move(*error));
      }
      return ReservedAttachmentResult::success(std::move(replay->value()));
    }
    if (auto error = authority_sample_error(committed_at, clock_sample)) {
      return ReservedAttachmentResult::failure(std::move(*error));
    }
    if (auto error = prepare_mutation(
        request.operation_id, request.token,
        ReservationStage::Reserved, JournalKind::Attach, true))
    {
      return ReservedAttachmentResult::failure(std::move(*error));
    }
    TrackedObject & object = objects.at(active_reservation->object_id);
    if (committed_at.nanoseconds() < object.transition_time.nanoseconds() ||
      committed_at.nanoseconds() < robot.telemetry_time.nanoseconds())
    {
      const bool may_settle = attachment_clock_may_settle(object, committed_at, clock_sample);
      AttachmentTemporalRefusal decision{committed_at.nanoseconds(),
        object.observation_time.nanoseconds(), object.transition_time.nanoseconds(),
        robot.telemetry_time.nanoseconds(), revision, object.id, object.revision,
        robot.revision, robot.telemetry_revision, active_reservation->reservation_id,
        active_reservation->revision, static_cast<std::uint8_t>(active_reservation->stage),
        committed_at.nanoseconds() < object.transition_time.nanoseconds(),
        committed_at.nanoseconds() < robot.telemetry_time.nanoseconds(), {}, {}, {}, false, false};
      decision.decision_kind = may_settle ? WorldStateErrorCode::AttachmentClockNotReady :
        WorldStateErrorCode::OutOfOrder;
      decision.clock_inhibited = clock_authority_inhibited;
      decision.authority_clock_lineage = clock_lineage;
      decision.operation_id = AttachmentDiagnosticIdentity::capture(request.operation_id);
      decision.source_id = AttachmentDiagnosticIdentity::capture(object.source_object_id);
      const auto source = source_objects.find(object.source_object_id);
      if (source != source_objects.end() && source->second.object_id == object.id &&
        source->second.observation_revision == object.revision)
      {
        decision.object_received_at_ns = source->second.received_at_ns;
        if (source->second.clock_sample) {
          decision.object_clock_lineage = source->second.clock_sample->lineage;
          decision.object_managed_ros_started = source->second.clock_sample->managed_ros_started;
        }
      }
      if (clock_sample) {
        decision.commit_clock_lineage = clock_sample->lineage;
        decision.commit_managed_ros_started = clock_sample->managed_ros_started;
      }
      return ReservedAttachmentResult::failure(
        WorldStateError{decision.decision_kind,
          may_settle ? "attachment is waiting for admitted object observation time" :
          "attachment predates object or robot state", decision});
    }
    const Revision new_revision = ++revision;
    if (active_reservation->source_lane) {
      auto & contents = lanes.at(*active_reservation->source_lane).contents;
      contents.erase(std::remove(contents.begin(), contents.end(), object.id), contents.end());
      lanes.at(*active_reservation->source_lane).revision = new_revision;
    }
    object.grasp_state = GraspState::Attached;
    object.transition_time = committed_at;
    object.revision = new_revision;
    robot.held_object = object.id;
    robot.grasp_center_from_held_object = request.grasp_center_from_held_object;
    robot.revision = new_revision;
    active_reservation->stage = ReservationStage::Attached;
    active_reservation->revision = new_revision;
    append_event(
      WorldStateEvent{new_revision, committed_at, EventKind::ObjectAttached, object.id,
        active_reservation->source_lane, "reserved attachment committed"});
    journal_reservation(request.operation_id, JournalKind::Attach, fingerprint, new_revision);
    return ReservedAttachmentResult::success(
      ReservationReceipt{new_revision, active_token, *active_reservation});
  }

  // Decides whether the destination evidence proves a placement rather than a presence. A gripped
  // product is already inside the lane volume during the pre-insert, so the lane can show it
  // before anything was placed; the evidence must be unobstructed and name nothing the
  // reservation did not already account for, must be fresh under the validity horizon, and must
  // have been recorded strictly after the boundary verified the release — that ordering is what
  // separates placement from presence and is not negotiable. What proves the *landing* is
  // flag-selected (Milestone 10 §3): by default (`placement_require_column_growth`) the free rear
  // depth has shrunk against the grant-time baseline by at least one catalogued pitch less
  // tolerance, a proof any geometry producer can supply — including the wrist camera, which never
  // names identities in a lane; with the flag false the post-release evidence must instead name
  // the target (exact post-release source identity, the ground-truth simulator opt-out).
  //
  // The release ordering is what stops a held product hovering in the lane mouth from reading as
  // a grown column. A reading taken while the product is still rolling understates free depth,
  // which overstates the growth: safe for a proof that something was added.
  [[nodiscard]] std::optional<WorldStateError> placement_evidence_error(
    const ShelfLane & destination, const TrackedObject & object,
    const rclcpp::Time & released_at, const rclcpp::Time & committed_at) const
  {
    // A release outside the reservation's window is a caller defect: rejected as an argument.
    if (released_at.nanoseconds() <= 0 ||
      released_at.nanoseconds() < active_reservation->created_at.nanoseconds() ||
      released_at.nanoseconds() > committed_at.nanoseconds())
    {
      return WorldStateError{
        WorldStateErrorCode::InvalidArgument,
        "placement release time is outside the reservation's own window" +
        describe_lane_evidence(destination, object, committed_at, released_at)};
    }
    // The refusal carries the judged evidence to distinguish never observed, obstructed, and
    // holding something the reservation did not account for.
    if (destination.obstructed ||
      !destination_evidence_within_reservation(destination, object, true))
    {
      return WorldStateError{
        WorldStateErrorCode::PredicateFailed,
        "successful placement requires destination evidence that is not obstructed and names "
        "nothing the reservation did not already account for" +
        describe_lane_evidence(destination, object, committed_at, released_at)};
    }
    if (config.placement_require_column_growth) {
      const auto profile = resolve_product_lane_profile(
        config.product_lane_profiles, object.product_class, object.sku);
      if (!profile) {
        return WorldStateError{
          WorldStateErrorCode::PredicateFailed,
          "placed product has no catalogued lane pitch, so no growth can be measured" +
          describe_lane_evidence(destination, object, committed_at, released_at)};
      }
      const double growth_m =
        active_reservation->destination_available_depth_m - destination.available_depth_m;
      if (growth_m < profile->column_pitch_m - config.placement_growth_tolerance_m) {
        return WorldStateError{
          WorldStateErrorCode::PredicateFailed,
          "successful placement requires the destination column to have grown by one product "
          "pitch: grew " + format_depth(growth_m) + " from " +
          format_depth(active_reservation->destination_available_depth_m) + " free at reservation, "
          "pitch " + format_depth(profile->column_pitch_m) + " less tolerance " +
          format_depth(config.placement_growth_tolerance_m) +
          describe_lane_evidence(destination, object, committed_at, released_at)};
      }
    } else {
      const auto & observed = destination.observed_source_object_ids;
      if (std::find(observed.begin(), observed.end(), object.source_object_id) == observed.end()) {
        return WorldStateError{
          WorldStateErrorCode::PredicateFailed,
          "successful placement requires destination evidence naming the target" +
          describe_lane_evidence(destination, object, committed_at, released_at)};
      }
    }
    // Under a moving eye, evidence validity is the horizon plus the invalidation check, not the
    // ingest age.
    if (auto error = lane_evidence_validity_error(
        destination, committed_at, config, "destination"))
    {
      return WorldStateError{
        WorldStateErrorCode::PredicateFailed,
        "successful placement requires valid destination evidence: " + error->detail +
        describe_lane_evidence(destination, object, committed_at, released_at)};
    }
    if (destination.last_verified.nanoseconds() <= released_at.nanoseconds()) {
      return WorldStateError{
        WorldStateErrorCode::PredicateFailed,
        "successful placement requires destination evidence recorded after the release" +
        describe_lane_evidence(destination, object, committed_at, released_at)};
    }
    return std::nullopt;
  }

  Result<ReservationReceipt> commit_reserved_detachment(
    const ReservedDetachmentRequest & request,
    const rclcpp::Time & committed_at)
  {
    std::unique_lock lock(mutex);
    if (!valid_operation_id(request.operation_id) || request.token.empty()) {
      return failure<ReservationReceipt>(
        WorldStateErrorCode::InvalidArgument,
        "detachment token or operation ID is invalid");
    }
    if (auto error = validate_clock(committed_at, config.clock_type, "committed_at")) {
      return Result<ReservationReceipt>::failure(std::move(*error));
    }
    if (auto error = validate_clock(request.released_at, config.clock_type, "released_at")) {
      return Result<ReservationReceipt>::failure(std::move(*error));
    }
    const std::string fingerprint = detach_fingerprint(request);
    if (auto replay = replay_reservation(request.operation_id, JournalKind::Detach, fingerprint)) {
      return *replay;
    }
    if (auto error = prepare_mutation(
        request.operation_id, request.token,
        ReservationStage::Attached, JournalKind::Detach, true))
    {
      return Result<ReservationReceipt>::failure(std::move(*error));
    }
    TrackedObject & object = objects.at(active_reservation->object_id);
    ShelfLane & destination = lanes.at(active_reservation->destination_lane);
    if (request.disposition == DetachmentDisposition::PlaceInReservedDestination) {
      if (auto error = placement_evidence_error(
          destination, object, request.released_at,
          committed_at))
      {
        return Result<ReservationReceipt>::failure(std::move(*error));
      }
    }
    if (committed_at.nanoseconds() < object.transition_time.nanoseconds() ||
      committed_at.nanoseconds() < robot.telemetry_time.nanoseconds())
    {
      return failure<ReservationReceipt>(
        WorldStateErrorCode::OutOfOrder,
        "detachment predates object or robot state");
    }
    const Revision new_revision = ++revision;
    object.grasp_state = GraspState::Free;
    object.transition_time = committed_at;
    object.revision = new_revision;
    robot.held_object.reset();
    robot.grasp_center_from_held_object = Eigen::Isometry3d::Identity();
    robot.revision = new_revision;
    const bool place = request.disposition == DetachmentDisposition::PlaceInReservedDestination;
    if (place) {
      destination.contents.push_back(object.id);
      destination.revision = new_revision;
    }
    active_reservation->stage = ReservationStage::Detached;
    active_reservation->placed_in_destination = place;
    active_reservation->revision = new_revision;
    append_event(
      WorldStateEvent{
        new_revision, committed_at, EventKind::ObjectDetached, object.id,
        place ? std::optional<LaneId>(destination.id) : std::nullopt,
        place ? "reserved placement committed" : "reserved free release committed"});
    journal_reservation(request.operation_id, JournalKind::Detach, fingerprint, new_revision);
    return Result<ReservationReceipt>::success(
      ReservationReceipt{new_revision, active_token, *active_reservation});
  }

  Result<MutationReceipt> release_task_reservation(
    const ReleaseReservationRequest & request,
    const rclcpp::Time & released_at)
  {
    std::unique_lock lock(mutex);
    if (!valid_operation_id(request.operation_id) || request.token.empty() ||
      request.expected_reservation_id == 0 || request.expected_reservation_revision == 0)
    {
      return failure<MutationReceipt>(
        WorldStateErrorCode::InvalidArgument,
        "release token or operation ID is invalid");
    }
    if (auto error = validate_clock(released_at, config.clock_type, "released_at")) {
      return Result<MutationReceipt>::failure(std::move(*error));
    }
    const std::string fingerprint = release_fingerprint(request);
    const auto replay = operation_journal.find(request.operation_id);
    if (replay != operation_journal.end()) {
      if (replay->second.kind != JournalKind::Release ||
        replay->second.fingerprint != fingerprint)
      {
        return failure<MutationReceipt>(
          WorldStateErrorCode::IdempotencyConflict,
          "operation ID was already used with a different operation or payload");
      }
      return Result<MutationReceipt>::success(MutationReceipt{replay->second.revision});
    }
    if (!active_reservation || request.token != active_token) {
      return failure<MutationReceipt>(
        WorldStateErrorCode::TokenMismatch,
        "task reservation token is not active");
    }
    if (request.expected_reservation_id != active_reservation->reservation_id ||
      request.expected_reservation_stage != active_reservation->stage ||
      request.expected_reservation_revision != active_reservation->revision)
    {
      return failure<MutationReceipt>(
        WorldStateErrorCode::RevisionConflict,
        "reservation identity, stage, or revision changed before release");
    }
    if (operation_journal.size() >= config.operation_journal_capacity) {
      return failure<MutationReceipt>(
        WorldStateErrorCode::ResourceExhausted,
        "operation journal is full");
    }
    const bool cancel = active_reservation->stage == ReservationStage::Reserved;
    const bool detached = active_reservation->stage == ReservationStage::Detached;
    if ((!cancel && !detached) ||
      (cancel && request.outcome != ReservationOutcome::Canceled &&
      request.outcome != ReservationOutcome::FailedSafe) ||
      (detached && active_reservation->placed_in_destination &&
      request.outcome != ReservationOutcome::Succeeded) ||
      (detached && !active_reservation->placed_in_destination &&
      request.outcome != ReservationOutcome::FailedSafe))
    {
      return failure<MutationReceipt>(
        WorldStateErrorCode::InvalidTransition,
        "reservation stage, disposition, and terminal outcome disagree");
    }
    if (auto error = reservation_predicate_error(*active_reservation)) {
      return Result<MutationReceipt>::failure(std::move(*error));
    }
    const bool normal_terminal = request.outcome == ReservationOutcome::Succeeded ||
      request.outcome == ReservationOutcome::Canceled;
    if ((normal_terminal && (request.terminal_task_phase != TaskPhase::Idle ||
      request.terminal_fault_state != FaultState::None)) ||
      (!normal_terminal && (request.terminal_task_phase != TaskPhase::Fault &&
      request.terminal_task_phase != TaskPhase::RequestingOperator)) ||
      (!normal_terminal && request.terminal_fault_state == FaultState::None))
    {
      return failure<MutationReceipt>(
        WorldStateErrorCode::InvalidTransition,
        "terminal task and fault state are inconsistent");
    }
    if (released_at.nanoseconds() < robot.telemetry_time.nanoseconds()) {
      return failure<MutationReceipt>(
        WorldStateErrorCode::OutOfOrder,
        "release predates robot state");
    }

    const Revision new_revision = ++revision;
    const std::uint64_t reservation_id = active_reservation->reservation_id;
    const ObjectId object_id = active_reservation->object_id;
    const LaneId destination_lane = active_reservation->destination_lane;
    robot.task_phase = request.terminal_task_phase;
    robot.fault_state = request.terminal_fault_state;
    robot.revision = new_revision;
    append_event(
      WorldStateEvent{
        new_revision, released_at, EventKind::TaskReservationReleased, object_id, destination_lane,
        "task reservation " + std::to_string(reservation_id) + " released"});
    operation_journal.emplace(
      request.operation_id,
      JournalEntry{JournalKind::Release, fingerprint, new_revision, active_token, std::nullopt});
    active_token.clear();
    active_reservation.reset();
    return Result<MutationReceipt>::success(MutationReceipt{new_revision});
  }

  WorldStateSnapshot snapshot() const
  {
    std::shared_lock lock(mutex);
    return WorldStateSnapshot{revision,
      objects,
      lanes,
      robot,
      active_reservation,
      std::vector<WorldStateEvent>(events.begin(), events.end())};
  }

  std::vector<JournalRetentionSnapshot> retention_snapshot() const
  {
    std::shared_lock lock(mutex);
    std::size_t open = 0;
    if (active_reservation) {
      open = static_cast<std::size_t>(
        std::count_if(
          operation_journal.begin(), operation_journal.end(),
          [this](const auto & entry) {return entry.second.token == active_token;}));
    }
    JournalRetentionSnapshot operations;
    operations.journal = "world_state.operations";
    operations.epoch_id = instance_id;
    operations.capacity = config.operation_journal_capacity;
    operations.size = operation_journal.size();
    operations.open_obligations = open;
    operations.terminal_receipts = operation_journal.size() - open;
    operations.reserved_credits = settlement_credits();
    operations.inhibited = clock_authority_inhibited;
    operations.evicting = false;
    JournalRetentionSnapshot audit;
    audit.journal = "world_state.events";
    audit.epoch_id = instance_id;
    audit.capacity = config.event_capacity;
    audit.size = events.size();
    audit.open_obligations = 0;
    audit.terminal_receipts = events.size();
    audit.reserved_credits = 0;
    audit.inhibited = clock_authority_inhibited;
    audit.evicting = true;
    return {std::move(operations), std::move(audit)};
  }

private:
  using LaneIterator = std::map<LaneId, ShelfLane>::iterator;

  void invalidate_clock_lineage() noexcept
  {
    if (clock_authority_armed || clock_lineage == std::numeric_limits<std::uint64_t>::max()) {
      clock_authority_inhibited = true;
    }
    if (clock_lineage != std::numeric_limits<std::uint64_t>::max()) {
      ++clock_lineage;
    }
    last_ordered_managed_sample.reset();
  }

  [[nodiscard]] std::optional<WorldStateError> authority_sample_error(
    const rclcpp::Time & authority_time,
    const std::optional<AuthorityClockSample> & sample) const
  {
    if (clock_authority_inhibited || (sample && sample->lineage != clock_lineage)) {
      return WorldStateError{WorldStateErrorCode::ClockAuthorityInhibited,
        "clock authority is inhibited or the sampled clock lineage changed"};
    }
    if (sample && (sample->time.get_clock_type() != authority_time.get_clock_type() ||
      sample->time.nanoseconds() != authority_time.nanoseconds()))
    {
      return WorldStateError{WorldStateErrorCode::InvalidArgument,
        "authority time differs from its original clock sample"};
    }
    return std::nullopt;
  }

  [[nodiscard]] bool attachment_clock_may_settle(
    const TrackedObject & object, const rclcpp::Time & committed_at,
    const std::optional<AuthorityClockSample> & sample) const
  {
    if (!sample || !sample->managed_ros_started || clock_authority_inhibited ||
      sample->lineage != clock_lineage ||
      robot.telemetry_time.nanoseconds() > committed_at.nanoseconds())
    {
      return false;
    }
    const auto source = source_objects.find(object.source_object_id);
    if (source == source_objects.end()) {
      return false;
    }
    const auto & admitted = source->second;
    if (admitted.object_id != object.id || admitted.observation_revision != object.revision ||
      !admitted.clock_sample || !admitted.clock_sample->managed_ros_started ||
      admitted.clock_sample->lineage != sample->lineage ||
      object.transition_time.nanoseconds() != object.observation_time.nanoseconds())
    {
      return false;
    }
    const auto observed_ns = object.observation_time.nanoseconds();
    const auto committed_ns = committed_at.nanoseconds();
    return observed_ns > admitted.received_at_ns &&
           observed_ns - admitted.received_at_ns <= config.maximum_future_skew.count() &&
           committed_ns >= admitted.received_at_ns && observed_ns > committed_ns &&
           observed_ns - committed_ns <= config.maximum_future_skew.count();
  }

  [[nodiscard]] std::optional<LaneId> containing_lane_id(ObjectId object_id) const
  {
    const auto lane = std::find_if(
      lanes.begin(), lanes.end(), [object_id](const auto & entry) {
        const auto & contents = entry.second.contents;
        return std::find(contents.begin(), contents.end(), object_id) != contents.end();
      });
    if (lane == lanes.end()) {
      return std::nullopt;
    }
    return lane->first;
  }

  [[nodiscard]] std::optional<WorldStateError> reservation_predicate_error(
    const TaskReservation & reservation) const
  {
    const auto object_iterator = objects.find(reservation.object_id);
    const auto destination_iterator = lanes.find(reservation.destination_lane);
    if (object_iterator == objects.end() || destination_iterator == lanes.end()) {
      return WorldStateError{WorldStateErrorCode::InvariantViolation,
        "reserved object or destination lane disappeared"};
    }
    const TrackedObject & object = object_iterator->second;
    const ShelfLane & destination = destination_iterator->second;
    if (object.source_object_id != reservation.object_source_id ||
      object.product_class != reservation.product_class || object.sku != reservation.sku ||
      object.tracking_state != TrackingState::Tracked ||
      !policy_accepts_object(
        reservation.destination_expected_product_class,
        reservation.destination_expected_sku, object))
    {
      return WorldStateError{
        WorldStateErrorCode::PredicateFailed,
        "reserved object lifecycle/identity or captured destination policy failed"};
    }
    if (reservation.source_lane && !lanes.contains(*reservation.source_lane)) {
      return WorldStateError{WorldStateErrorCode::InvariantViolation,
        "reserved source lane disappeared"};
    }

    const std::optional<LaneId> membership = containing_lane_id(object.id);
    // Room at the rear for one more of this product. Measured live only at the reserved stage:
    // from the attached stage on, the arm stands in the lane mouth and the reported free depth
    // includes what the arm occupies. Nothing else takes that depth meanwhile (only this arm loads
    // from behind; customers only give depth back), and the placement generator re-checks the
    // room against live evidence before the insert.
    const auto profile = resolve_product_lane_profile(
      config.product_lane_profiles, object.product_class, object.sku);
    if (!profile) {
      return WorldStateError{
        WorldStateErrorCode::PredicateFailed,
        "reserved product has no catalogued lane depth, so its destination cannot be judged"};
    }
    switch (reservation.stage) {
      case ReservationStage::Reserved:
        if (object.grasp_state != GraspState::Free || robot.held_object ||
          membership != reservation.source_lane || destination.obstructed ||
          destination.available_depth_m + config.placement_entry_depth_tolerance_m <
          profile->required_rear_depth_m ||
          !destination_evidence_within_reservation(destination, object, false))
        {
          return WorldStateError{
            WorldStateErrorCode::PredicateFailed,
            "reserved-stage free, membership, held-state, or destination predicate failed" +
            describe_lane_evidence(destination, object, object.observation_time)};
        }
        break;
      case ReservationStage::Attached:
        if (object.grasp_state != GraspState::Attached || robot.held_object != object.id ||
          membership || destination.obstructed ||
          !destination_evidence_within_reservation(destination, object, true))
        {
          return WorldStateError{
            WorldStateErrorCode::PredicateFailed,
            "attached-stage coupling, membership, or destination predicate failed" +
            describe_lane_evidence(destination, object, object.observation_time)};
        }
        break;
      case ReservationStage::Detached: {
          const std::optional<LaneId> expected_membership =
            reservation.placed_in_destination ?
            std::optional<LaneId>(reservation.destination_lane) :
            std::nullopt;
          // The same flag-selected landing proof the commit accepted, re-judged against current
          // evidence: growth still standing under the default §3 proof, the target still named
          // under the identity opt-out. Identity-free (wrist) evidence cannot re-name the
          // target, so the naming re-proof only binds where a producer supplies identities.
          bool landing_still_proven = false;
          if (reservation.placed_in_destination) {
            if (config.placement_require_column_growth) {
              const double growth_m =
                reservation.destination_available_depth_m - destination.available_depth_m;
              landing_still_proven =
                growth_m >= profile->column_pitch_m - config.placement_growth_tolerance_m;
            } else {
              landing_still_proven =
                std::find(
                destination.observed_source_object_ids.begin(),
                destination.observed_source_object_ids.end(), object.source_object_id) !=
                destination.observed_source_object_ids.end();
            }
          }
          const bool placement_still_stands = !destination.obstructed &&
            destination_evidence_within_reservation(destination, object, true) &&
            landing_still_proven;
          if (object.grasp_state != GraspState::Free || robot.held_object ||
            membership != expected_membership ||
            (reservation.placed_in_destination && !placement_still_stands))
          {
            return WorldStateError{
              WorldStateErrorCode::PredicateFailed,
              "detached-stage free, membership, or destination evidence predicate failed" +
              describe_lane_evidence(destination, object, object.observation_time)};
          }
          break;
        }
    }
    return std::nullopt;
  }

  // Whether every product the destination's evidence names was already known to the reservation:
  // in the lane when it was granted, or the placed product when `target_admitted` allows it.
  [[nodiscard]] bool destination_evidence_within_reservation(
    const ShelfLane & destination, const TrackedObject & object, bool target_admitted) const
  {
    return std::all_of(
      destination.observed_source_object_ids.begin(),
      destination.observed_source_object_ids.end(),
      [&](const std::string & source_id) {
        if (target_admitted && source_id == object.source_object_id) {
          return true;
        }
        const auto & known = active_reservation->destination_source_ids;
        return std::find(known.begin(), known.end(), source_id) != known.end();
      });
  }

  [[nodiscard]] std::optional<Result<ReservationReceipt>> replay_reservation(
    const std::string & operation_id, JournalKind kind, const std::string & fingerprint) const
  {
    const auto replay = operation_journal.find(operation_id);
    if (replay == operation_journal.end()) {
      return std::nullopt;
    }
    if (replay->second.kind != kind || replay->second.fingerprint != fingerprint ||
      !replay->second.reservation)
    {
      return Result<ReservationReceipt>::failure(
        WorldStateError{WorldStateErrorCode::IdempotencyConflict,
          "operation ID was already used with a different operation or payload"});
    }
    return Result<ReservationReceipt>::success(
      ReservationReceipt{
        replay->second.revision, replay->second.token, *replay->second.reservation});
  }

  // Journal entries still owed to the active reservation's remaining lifecycle (attach, detach,
  // release); zero when no reservation is active.
  [[nodiscard]] std::size_t settlement_credits() const
  {
    if (!active_reservation) {
      return 0;
    }
    switch (active_reservation->stage) {
      case ReservationStage::Reserved:
        return 3;
      case ReservationStage::Attached:
        return 2;
      case ReservationStage::Detached:
        return 1;
    }
    return 0;
  }

  [[nodiscard]] bool has_journal_capacity(std::size_t required_slots) const
  {
    return required_slots <= config.operation_journal_capacity &&
           operation_journal.size() <= config.operation_journal_capacity - required_slots;
  }

  [[nodiscard]] std::optional<WorldStateError> prepare_mutation(
    const std::string & operation_id, const std::string & token,
    std::optional<ReservationStage> expected_stage, JournalKind kind,
    bool validate_predicates) const
  {
    if (!active_reservation || token != active_token) {
      return WorldStateError{WorldStateErrorCode::TokenMismatch,
        "task reservation token is not active"};
    }
    if (operation_journal.contains(operation_id)) {
      return WorldStateError{WorldStateErrorCode::IdempotencyConflict,
        "operation ID was already used with a different operation or payload"};
    }
    if (expected_stage && active_reservation->stage != *expected_stage) {
      return WorldStateError{WorldStateErrorCode::InvalidTransition,
        "reservation stage does not permit this mutation"};
    }
    // Protect semantic settlement before physical work can begin. Checkpoints spend only
    // unreserved entries; attach/detach exchange one lifecycle credit for their retained record.
    std::size_t required_slots = settlement_credits();
    if (kind == JournalKind::Checkpoint) {
      ++required_slots;
    }
    if (!has_journal_capacity(required_slots)) {
      return WorldStateError{WorldStateErrorCode::ResourceExhausted,
        "operation journal capacity is reserved for task settlement"};
    }
    if (validate_predicates) {
      return reservation_predicate_error(*active_reservation);
    }
    return std::nullopt;
  }

  void journal_reservation(
    const std::string & operation_id, JournalKind kind,
    const std::string & fingerprint, Revision operation_revision)
  {
    operation_journal.emplace(
      operation_id, JournalEntry{kind, fingerprint, operation_revision,
        active_token, *active_reservation});
  }

  LaneIterator find_containing_lane(ObjectId object_id)
  {
    return std::find_if(
      lanes.begin(), lanes.end(), [object_id](const auto & entry) {
        const auto & contents = entry.second.contents;
        return std::find(contents.begin(), contents.end(), object_id) != contents.end();
      });
  }

  void append_event(WorldStateEvent event)
  {
    events.push_back(std::move(event));
    if (events.size() > config.event_capacity) {
      events.pop_front();
    }
  }

  WorldStateConfig config;
  std::string instance_id;
  mutable std::shared_mutex mutex;
  Revision revision{0};
  std::uint64_t next_object_id{1};
  std::uint64_t next_reservation_id{1};
  std::map<ObjectId, TrackedObject> objects;
  std::map<std::string, SourceObjectEntry> source_objects;
  std::uint64_t clock_lineage{1};
  bool clock_authority_armed{false};
  bool clock_authority_inhibited{false};
  std::optional<std::int64_t> last_ordered_managed_sample;
  std::map<LaneId, ShelfLane> lanes;
  RobotExecutionState robot;
  std::optional<TaskReservation> active_reservation;
  std::string active_token;
  std::map<std::string, JournalEntry> operation_journal;
  std::deque<WorldStateEvent> events;
};

WorldStateStore::WorldStateStore(WorldStateConfig config)
: impl_(std::make_unique<Impl>(std::move(config)))
{
}

WorldStateStore::~WorldStateStore() = default;

AuthorityClockSample WorldStateStore::capture_authority_clock(
  const rclcpp::Time & sampled_at, bool managed_ros_started)
{
  return impl_->capture_authority_clock(sampled_at, managed_ros_started);
}

void WorldStateStore::notify_clock_discontinuity() noexcept
{
  impl_->notify_clock_discontinuity();
}

Result<ObservationReceipt> WorldStateStore::observe_object(
  const ObjectObservation & observation,
  const rclcpp::Time & received_at,
  const std::optional<AuthorityClockSample> & clock_sample)
{
  return impl_->observe_object(observation, received_at, clock_sample);
}

Result<MutationReceipt> WorldStateStore::set_tracking_state(
  ObjectId object_id, TrackingState state, const rclcpp::Time & transition_time,
  const ObjectContextPreconditions & preconditions)
{
  return impl_->set_tracking_state(object_id, state, transition_time, preconditions);
}

Result<MutationReceipt> WorldStateStore::configure_lane(
  const LaneDefinition & definition,
  const rclcpp::Time & configured_at)
{
  return impl_->configure_lane(definition, configured_at);
}

Result<MutationReceipt> WorldStateStore::update_lane(
  const LaneObservation & observation,
  const rclcpp::Time & received_at,
  Revision expected_lane_revision,
  const std::optional<AuthorityClockSample> & clock_sample)
{
  return impl_->update_lane(observation, received_at, expected_lane_revision, clock_sample);
}

Result<MutationReceipt> WorldStateStore::invalidate_lane_evidence(
  LaneId lane_id,
  const rclcpp::Time & invalidated_at,
  Revision expected_lane_revision)
{
  return impl_->invalidate_lane_evidence(lane_id, invalidated_at, expected_lane_revision);
}

Result<MutationReceipt> WorldStateStore::set_lane_policy(
  const LanePolicy & policy,
  Revision expected_lane_revision,
  const rclcpp::Time & changed_at,
  const LanePolicyPersist & persist)
{
  return impl_->set_lane_policy(policy, expected_lane_revision, changed_at, persist);
}

Result<MutationReceipt> WorldStateStore::observe_robot_telemetry(
  const RobotTelemetryObservation & observation, const rclcpp::Time & received_at,
  const std::optional<AuthorityClockSample> & clock_sample)
{
  return impl_->observe_robot_telemetry(observation, received_at, clock_sample);
}

Result<MutationReceipt> WorldStateStore::commit_attachment(
  ObjectId object_id,
  const Eigen::Isometry3d & grasp_center_from_object,
  const rclcpp::Time & committed_at,
  const AttachPreconditions & preconditions)
{
  return impl_->commit_attachment(
    object_id, grasp_center_from_object, committed_at, preconditions);
}

Result<MutationReceipt> WorldStateStore::commit_detachment(
  ObjectId object_id,
  std::optional<LaneId> destination_lane,
  const rclcpp::Time & committed_at,
  const DetachPreconditions & preconditions)
{
  return impl_->commit_detachment(
    object_id, std::move(destination_lane), committed_at,
    preconditions);
}

Result<ReservationReceipt> WorldStateStore::reserve_task(
  const ReserveTaskRequest & request,
  const rclcpp::Time & reserved_at,
  const std::optional<AuthorityClockSample> & clock_sample)
{
  return impl_->reserve_task(request, reserved_at, clock_sample);
}

Result<ReservationValidation> WorldStateStore::validate_task_reservation(
  const std::string & token) const
{
  return impl_->validate_task_reservation(token);
}

Result<ExecutionWorldAuthorityProof> WorldStateStore::validate_execution_world_authority(
  const std::string & token,
  const ExecutionWorldAuthorityExpectation & expected) const
{
  return impl_->validate_execution_world_authority(token, expected);
}

Result<ReservationReceipt> WorldStateStore::checkpoint_reserved_task(
  const ReservedTaskCheckpoint & checkpoint)
{
  return impl_->checkpoint_reserved_task(checkpoint);
}

ReservedAttachmentResult WorldStateStore::commit_reserved_attachment(
  const ReservedAttachmentRequest & request, const rclcpp::Time & committed_at,
  const std::optional<AuthorityClockSample> & clock_sample)
{
  return impl_->commit_reserved_attachment(request, committed_at, clock_sample);
}

Result<ReservationReceipt> WorldStateStore::commit_reserved_detachment(
  const ReservedDetachmentRequest & request, const rclcpp::Time & committed_at)
{
  return impl_->commit_reserved_detachment(request, committed_at);
}

Result<MutationReceipt> WorldStateStore::release_task_reservation(
  const ReleaseReservationRequest & request, const rclcpp::Time & released_at)
{
  return impl_->release_task_reservation(request, released_at);
}

WorldStateSnapshot WorldStateStore::snapshot() const {return impl_->snapshot();}

std::vector<JournalRetentionSnapshot> WorldStateStore::retention_snapshot() const
{
  return impl_->retention_snapshot();
}

std::string to_string(WorldStateErrorCode code)
{
  switch (code) {
    case WorldStateErrorCode::InvalidArgument:
      return "invalid_argument";
    case WorldStateErrorCode::ClockMismatch:
      return "clock_mismatch";
    case WorldStateErrorCode::FrameMismatch:
      return "frame_mismatch";
    case WorldStateErrorCode::StaleObservation:
      return "stale_observation";
    case WorldStateErrorCode::FutureObservation:
      return "future_observation";
    case WorldStateErrorCode::OutOfOrder:
      return "out_of_order";
    case WorldStateErrorCode::IdentityConflict:
      return "identity_conflict";
    case WorldStateErrorCode::NotFound:
      return "not_found";
    case WorldStateErrorCode::Removed:
      return "removed";
    case WorldStateErrorCode::RevisionConflict:
      return "revision_conflict";
    case WorldStateErrorCode::ReservationConflict:
      return "reservation_conflict";
    case WorldStateErrorCode::TokenMismatch:
      return "token_mismatch";
    case WorldStateErrorCode::PredicateFailed:
      return "predicate_failed";
    case WorldStateErrorCode::InvalidTransition:
      return "invalid_transition";
    case WorldStateErrorCode::IdempotencyConflict:
      return "idempotency_conflict";
    case WorldStateErrorCode::ResourceExhausted:
      return "resource_exhausted";
    case WorldStateErrorCode::InvariantViolation:
      return "invariant_violation";
    case WorldStateErrorCode::AttachmentClockNotReady:
      return "attachment_clock_not_ready";
    case WorldStateErrorCode::ClockAuthorityInhibited:
      return "clock_authority_inhibited";
  }
  throw std::logic_error("unhandled world-state error code");
}

std::optional<ProductLaneProfile> resolve_product_lane_profile(
  const std::vector<ProductLaneProfile> & profiles, ProductClass product_class,
  const std::optional<std::string> & sku)
{
  if (product_class == ProductClass::Unknown) {
    return std::nullopt;
  }
  const ProductLaneProfile * fallback = nullptr;
  for (const ProductLaneProfile & profile : profiles) {
    if (profile.product_class != product_class) {
      continue;
    }
    if (profile.sku && sku && *profile.sku == *sku) {
      return profile;
    }
    if (!profile.sku && fallback == nullptr) {
      fallback = &profile;
    }
  }
  if (fallback == nullptr) {
    return std::nullopt;
  }
  return *fallback;
}

bool policy_accepts_object(
  ProductClass expected_product_class,
  const std::optional<std::string> & expected_sku,
  const TrackedObject & object)
{
  if (expected_product_class != ProductClass::Unknown &&
    expected_product_class != object.product_class)
  {
    return false;
  }
  if (expected_sku && *expected_sku != object.sku) {
    return false;
  }
  return true;
}

}  // namespace restocker_world_state
