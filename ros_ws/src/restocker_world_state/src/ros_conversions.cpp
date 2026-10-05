// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_world_state/ros_conversions.hpp"

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <restocker_interfaces/msg/robot_execution_state.hpp>
#include <restocker_interfaces/msg/shelf_lane.hpp>
#include <restocker_interfaces/msg/tracked_object.hpp>
#include <restocker_interfaces/msg/world_state_event.hpp>

namespace restocker_world_state
{
namespace
{

using ObservationMessage = restocker_interfaces::msg::ObjectObservation;
using LaneObservationMessage = restocker_interfaces::msg::LaneObservation;
using RobotTelemetryMessage = restocker_interfaces::msg::RobotTelemetry;
using OperationStatus = restocker_interfaces::msg::WorldStateOperationStatus;

[[nodiscard]] Result<ObjectObservation> invalid_observation(std::string detail)
{
  return Result<ObjectObservation>::failure(
    WorldStateError{WorldStateErrorCode::InvalidArgument, std::move(detail)});
}

[[nodiscard]] Result<LaneObservation> invalid_lane_observation(std::string detail)
{
  return Result<LaneObservation>::failure(
    WorldStateError{WorldStateErrorCode::InvalidArgument, std::move(detail)});
}

// `confidence` is a float32 on the wire but floors are double parameters, so a plain comparison
// promotes the message value and a producer emitting the floor literal (e.g. 0.70, 0.90, 0.95)
// can round below the double and be refused. Comparing at float precision admits a value equal to
// the floor for every floor; floors are honoured to float precision, all the field carries.
[[nodiscard]] bool confidence_meets_floor(float confidence, double minimum_confidence) noexcept
{
  return std::isfinite(confidence) && confidence >= static_cast<float>(minimum_confidence) &&
         confidence <= 1.0F;
}

template<typename Value>
[[nodiscard]] Result<Value> invalid_request(std::string detail)
{
  return Result<Value>::failure(
    WorldStateError{WorldStateErrorCode::InvalidArgument, std::move(detail)});
}

[[nodiscard]] bool ascii_alphanumeric(const char character) noexcept
{
  return (character >= 'a' && character <= 'z') ||
         (character >= 'A' && character <= 'Z') ||
         (character >= '0' && character <= '9');
}

[[nodiscard]] bool valid_source_character(const char character) noexcept
{
  return ascii_alphanumeric(character) || character == '_' || character == '-' ||
         character == '.' || character == ':' || character == '/';
}

[[nodiscard]] std::optional<TaskPhase> task_phase_from_message(std::uint8_t value)
{
  using RobotMessage = restocker_interfaces::msg::RobotExecutionState;
  switch (value) {
    case RobotMessage::TASK_IDLE:
      return TaskPhase::Idle;
    case RobotMessage::TASK_VALIDATING_SCENE:
      return TaskPhase::ValidatingScene;
    case RobotMessage::TASK_EXECUTING:
      return TaskPhase::Executing;
    case RobotMessage::TASK_RECOVERING:
      return TaskPhase::Recovering;
    case RobotMessage::TASK_FAULT:
      return TaskPhase::Fault;
    case RobotMessage::TASK_REQUESTING_OPERATOR:
      return TaskPhase::RequestingOperator;
    default:
      return std::nullopt;
  }
}

[[nodiscard]] std::optional<FaultState> fault_state_from_message(std::uint8_t value)
{
  using RobotMessage = restocker_interfaces::msg::RobotExecutionState;
  switch (value) {
    case RobotMessage::FAULT_NONE:
      return FaultState::None;
    case RobotMessage::FAULT_RECOVERABLE:
      return FaultState::Recoverable;
    case RobotMessage::FAULT_NON_RECOVERABLE:
      return FaultState::NonRecoverable;
    case RobotMessage::FAULT_EXTERNAL_INCONSISTENCY:
      return FaultState::ExternalInconsistency;
    default:
      return std::nullopt;
  }
}

[[nodiscard]] std::optional<ProductClass> product_class_from_message(std::uint8_t value)
{
  switch (value) {
    case ObservationMessage::PRODUCT_CLASS_UNKNOWN:
      return ProductClass::Unknown;
    case ObservationMessage::PRODUCT_CLASS_CAN:
      return ProductClass::Can;
    case ObservationMessage::PRODUCT_CLASS_SMALL_BOTTLE:
      return ProductClass::SmallBottle;
    case ObservationMessage::PRODUCT_CLASS_LARGE_BOTTLE:
      return ProductClass::LargeBottle;
    default:
      return std::nullopt;
  }
}

[[nodiscard]] std::optional<ObjectOrientation> orientation_from_message(std::uint8_t value)
{
  switch (value) {
    case ObservationMessage::ORIENTATION_UNKNOWN:
      return ObjectOrientation::Unknown;
    case ObservationMessage::ORIENTATION_UPRIGHT:
      return ObjectOrientation::Upright;
    case ObservationMessage::ORIENTATION_HORIZONTAL:
      return ObjectOrientation::Horizontal;
    case ObservationMessage::ORIENTATION_TILTED:
      return ObjectOrientation::Tilted;
    default:
      return std::nullopt;
  }
}

[[nodiscard]] builtin_interfaces::msg::Time time_to_message(const rclcpp::Time & time)
{
  const std::int64_t nanoseconds = time.nanoseconds();
  std::int64_t seconds = nanoseconds / 1'000'000'000LL;
  std::int64_t remainder = nanoseconds % 1'000'000'000LL;
  if (remainder < 0) {
    --seconds;
    remainder += 1'000'000'000LL;
  }
  builtin_interfaces::msg::Time message;
  message.sec = static_cast<std::int32_t>(seconds);
  message.nanosec = static_cast<std::uint32_t>(remainder);
  return message;
}

[[nodiscard]] std::optional<rclcpp::Time> time_from_message(
  const builtin_interfaces::msg::Time & message)
{
  if (message.sec < 0 || message.nanosec >= 1'000'000'000U) {
    return std::nullopt;
  }
  const std::int64_t nanoseconds =
    static_cast<std::int64_t>(message.sec) * 1'000'000'000LL + message.nanosec;
  return rclcpp::Time(nanoseconds, RCL_ROS_TIME);
}

template<typename Enum>
[[nodiscard]] std::optional<Enum> bounded_enum(std::uint8_t value, std::uint8_t maximum)
{
  if (value > maximum) {
    return std::nullopt;
  }
  return static_cast<Enum>(value);
}

[[nodiscard]] std::optional<Eigen::Isometry3d> pose_from_message(
  const geometry_msgs::msg::Pose & message)
{
  const Eigen::Quaterniond rotation(message.orientation.w, message.orientation.x,
    message.orientation.y, message.orientation.z);
  if (!std::isfinite(message.position.x) || !std::isfinite(message.position.y) ||
    !std::isfinite(message.position.z) || !rotation.coeffs().allFinite() ||
    std::abs(rotation.norm() - 1.0) > 1.0e-6)
  {
    return std::nullopt;
  }
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() =
    Eigen::Vector3d(message.position.x, message.position.y, message.position.z);
  result.linear() = rotation.normalized().toRotationMatrix();
  return result;
}

[[nodiscard]] geometry_msgs::msg::Pose pose_to_message(const Eigen::Isometry3d & pose)
{
  geometry_msgs::msg::Pose message;
  message.position.x = pose.translation().x();
  message.position.y = pose.translation().y();
  message.position.z = pose.translation().z();
  const Eigen::Quaterniond rotation(pose.linear());
  message.orientation.x = rotation.x();
  message.orientation.y = rotation.y();
  message.orientation.z = rotation.z();
  message.orientation.w = rotation.w();
  return message;
}

[[nodiscard]] bool valid_optional_string(bool present, const std::string & value)
{
  return present ? !value.empty() : value.empty();
}

[[nodiscard]] Result<WorldStateSnapshot> invalid_snapshot(
  std::string detail, WorldStateErrorCode code = WorldStateErrorCode::InvalidArgument)
{
  return Result<WorldStateSnapshot>::failure(WorldStateError{code, std::move(detail)});
}

[[nodiscard]] bool valid_covariance(const PoseCovariance & covariance)
{
  if (!covariance.allFinite() || !covariance.isApprox(covariance.transpose(), 1.0e-10)) {
    return false;
  }
  const Eigen::SelfAdjointEigenSolver<PoseCovariance> solver(covariance, Eigen::EigenvaluesOnly);
  return solver.info() == Eigen::Success && solver.eigenvalues().minCoeff() >= -1.0e-12;
}

[[nodiscard]] bool has_reservation_payload(
  const restocker_interfaces::msg::TaskReservation & message)
{
  return message.reservation_id != 0 || !message.request_id.empty() || message.object_id != 0 ||
         !message.object_source_id.empty() || message.product_class != 0 || message.has_sku ||
         !message.sku.empty() || message.has_source_lane || !message.source_lane_id.empty() ||
         !message.destination_lane_id.empty() || message.stage != 0 ||
         message.placed_in_destination || message.created_at.sec != 0 ||
         message.created_at.nanosec != 0 || message.created_revision != 0 ||
         message.admitted_robot_telemetry_revision != 0 || message.revision != 0 ||
         message.destination_expected_product_class != 0 ||
         message.has_destination_expected_sku || !message.destination_expected_sku.empty();
}

[[nodiscard]] std::optional<std::string> snapshot_invariant_error(
  const WorldStateSnapshot & snapshot)
{
  std::map<ObjectId, LaneId> memberships;
  for (const auto &[lane_id, lane] : snapshot.lanes) {
    for (const ObjectId object_id : lane.contents) {
      const auto object = snapshot.objects.find(object_id);
      if (object == snapshot.objects.end()) {
        return "lane membership references an object absent from the snapshot";
      }
      if (object->second.tracking_state == TrackingState::Removed ||
        object->second.grasp_state != GraspState::Free)
      {
        return "removed or attached object appears in lane membership";
      }
      if (!memberships.emplace(object_id, lane_id).second) {
        return "object appears in more than one lane";
      }
    }
  }

  std::optional<ObjectId> attached_object;
  for (const auto &[object_id, object] : snapshot.objects) {
    if (object.grasp_state != GraspState::Attached) {
      continue;
    }
    if (attached_object) {
      return "snapshot contains more than one attached object";
    }
    attached_object = object_id;
  }
  if (snapshot.robot.held_object != attached_object) {
    return "robot held-object state and object grasp state disagree";
  }

  if (!snapshot.active_reservation) {
    return std::nullopt;
  }
  const TaskReservation & reservation = *snapshot.active_reservation;
  if (reservation.admitted_robot_telemetry_revision == 0 ||
    reservation.admitted_robot_telemetry_revision > reservation.created_revision ||
    reservation.admitted_robot_telemetry_revision > snapshot.robot.telemetry_revision)
  {
    return "reservation robot telemetry lineage is invalid";
  }
  const auto object = snapshot.objects.find(reservation.object_id);
  const auto destination = snapshot.lanes.find(reservation.destination_lane);
  if (object == snapshot.objects.end() || destination == snapshot.lanes.end()) {
    return "reservation references a missing object or destination lane";
  }
  if (reservation.source_lane && !snapshot.lanes.contains(*reservation.source_lane)) {
    return "reservation references a missing source lane";
  }
  if (reservation.source_lane == std::optional<LaneId>(reservation.destination_lane)) {
    return "reservation source and destination lanes are identical";
  }
  if (object->second.source_object_id != reservation.object_source_id ||
    object->second.product_class != reservation.product_class ||
    object->second.sku != reservation.sku)
  {
    return "reservation identity differs from its object";
  }
  if (!policy_accepts_object(
      reservation.destination_expected_product_class,
      reservation.destination_expected_sku, object->second))
  {
    return "reservation captured destination policy does not accept its object";
  }

  const auto membership = memberships.find(reservation.object_id);
  const std::optional<LaneId> containing_lane =
    membership == memberships.end() ? std::nullopt : std::optional<LaneId>(membership->second);
  switch (reservation.stage) {
    case ReservationStage::Reserved:
      if (reservation.placed_in_destination || object->second.grasp_state != GraspState::Free ||
        snapshot.robot.held_object || containing_lane != reservation.source_lane)
      {
        return "reserved-stage attachment or membership state is inconsistent";
      }
      break;
    case ReservationStage::Attached:
      if (reservation.placed_in_destination || object->second.grasp_state != GraspState::Attached ||
        snapshot.robot.held_object != reservation.object_id || containing_lane)
      {
        return "attached-stage attachment or membership state is inconsistent";
      }
      break;
    case ReservationStage::Detached: {
        const std::optional<LaneId> expected_lane =
          reservation.placed_in_destination ? std::optional<LaneId>(reservation.destination_lane) :
          std::nullopt;
        if (object->second.grasp_state != GraspState::Free || snapshot.robot.held_object ||
          containing_lane != expected_lane)
        {
          return "detached-stage attachment or membership state is inconsistent";
        }
        break;
      }
  }
  return std::nullopt;
}

}  // namespace

Result<ObjectObservation> object_observation_from_message(
  const ObservationMessage & message,
  double minimum_confidence)
{
  if (!std::isfinite(minimum_confidence) || minimum_confidence < 0.0 || minimum_confidence > 1.0) {
    return invalid_observation("minimum confidence must be finite and in [0, 1]");
  }
  if (message.status != ObservationMessage::STATUS_OK) {
    return invalid_observation("observation status is not OK: " + message.status_detail);
  }
  if (message.source_object_id.empty() || message.header.frame_id.empty()) {
    return invalid_observation("source object ID and frame must not be empty");
  }
  if (message.backend_name.empty() || message.backend_version.empty()) {
    return invalid_observation("backend name and version must not be empty");
  }
  if (!confidence_meets_floor(message.confidence, minimum_confidence)) {
    return invalid_observation("observation confidence is invalid or below threshold");
  }
  const auto product_class = product_class_from_message(message.product_class);
  const auto orientation = orientation_from_message(message.orientation);
  if (!product_class || !orientation) {
    return invalid_observation("observation contains an unknown enum value");
  }
  if (message.has_sku && message.sku.empty()) {
    return invalid_observation("has_sku is true but SKU is empty");
  }

  const auto & position = message.pose.pose.position;
  const auto & rotation = message.pose.pose.orientation;
  if (!std::isfinite(position.x) || !std::isfinite(position.y) || !std::isfinite(position.z) ||
    !std::isfinite(rotation.x) || !std::isfinite(rotation.y) || !std::isfinite(rotation.z) ||
    !std::isfinite(rotation.w))
  {
    return invalid_observation("observation pose contains a non-finite value");
  }
  const Eigen::Quaterniond quaternion(rotation.w, rotation.x, rotation.y, rotation.z);
  if (std::abs(quaternion.norm() - 1.0) > 1.0e-3) {
    return invalid_observation("observation quaternion is not unit length");
  }

  ObjectObservation observation;
  observation.source_object_id = message.source_object_id;
  observation.frame_id = message.header.frame_id;
  observation.product_class = *product_class;
  if (message.has_sku) {
    observation.sku = message.sku;
  }
  observation.pose_in_world = Eigen::Isometry3d::Identity();
  observation.pose_in_world.linear() = quaternion.normalized().toRotationMatrix();
  observation.pose_in_world.translation() = Eigen::Vector3d(position.x, position.y, position.z);
  for (std::size_t row = 0; row < 6; ++row) {
    for (std::size_t column = 0; column < 6; ++column) {
      observation.pose_covariance(row, column) = message.pose.covariance[row * 6 + column];
    }
  }
  observation.orientation = *orientation;
  observation.observation_time = rclcpp::Time(message.header.stamp, RCL_ROS_TIME);
  return Result<ObjectObservation>::success(std::move(observation));
}

Result<LaneObservation> lane_observation_from_message(
  const LaneObservationMessage & message,
  double minimum_confidence)
{
  if (!std::isfinite(minimum_confidence) || minimum_confidence < 0.0 || minimum_confidence > 1.0) {
    return invalid_lane_observation("minimum confidence must be finite and in [0, 1]");
  }
  if (message.status != LaneObservationMessage::STATUS_OK) {
    return invalid_lane_observation("lane observation status is not OK: " + message.status_detail);
  }
  if (message.lane_id.empty() || message.header.frame_id.empty()) {
    return invalid_lane_observation("lane ID and source frame must not be empty");
  }
  if (message.header.frame_id != message.lane_id) {
    return invalid_lane_observation("lane observation source frame must match its lane ID");
  }
  if (message.backend_name.empty() || message.backend_version.empty()) {
    return invalid_lane_observation("backend name and version must not be empty");
  }
  if (!confidence_meets_floor(message.confidence, minimum_confidence)) {
    // Lane, reported value and threshold go in the detail because this rejection is logged and
    // discarded. The leading sentence is unchanged so anything matching on it keeps matching.
    std::ostringstream detail;
    detail << "lane observation confidence is invalid or below threshold: lane " <<
      message.lane_id << " reported " << message.confidence << ", minimum is " <<
      minimum_confidence;
    return invalid_lane_observation(detail.str());
  }
  if (!std::isfinite(message.available_depth_m) || message.available_depth_m < 0.0) {
    return invalid_lane_observation("available lane depth must be finite and non-negative");
  }
  const auto & source_ids = message.observed_source_object_ids;
  if (std::any_of(
      source_ids.begin(), source_ids.end(),
      [](const std::string & source_id) {return source_id.empty();}) ||
    !std::is_sorted(source_ids.begin(), source_ids.end()) ||
    std::adjacent_find(source_ids.begin(), source_ids.end()) != source_ids.end())
  {
    return invalid_lane_observation(
      "observed lane source IDs must be non-empty, sorted, and unique");
  }

  LaneObservation observation;
  observation.id = LaneId{message.lane_id};
  observation.observed_source_object_ids = source_ids;
  observation.available_depth_m = message.available_depth_m;
  observation.obstructed = message.obstructed;
  observation.observation_time = rclcpp::Time(message.header.stamp, RCL_ROS_TIME);
  return Result<LaneObservation>::success(std::move(observation));
}

Result<RobotTelemetryObservation> robot_telemetry_from_message(
  const RobotTelemetryMessage & message)
{
  constexpr std::array<std::string_view, kArmJointCount> kCanonicalJointNames{
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint",
    "wrist_3_joint"};
  constexpr std::array<std::string_view, kGripperJointCount> kCanonicalGripperJointNames{
    "left_finger_joint", "right_finger_joint"};
  if (message.source_id.empty() || message.source_id.size() > 128U ||
    !ascii_alphanumeric(message.source_id.front()) ||
    !std::all_of(message.source_id.begin(), message.source_id.end(), valid_source_character))
  {
    return invalid_request<RobotTelemetryObservation>(
      "robot telemetry source ID must be a normalized 1-128 character identifier");
  }
  const auto observation_time = time_from_message(message.stamp);
  if (!observation_time || observation_time->nanoseconds() <= 0) {
    return invalid_request<RobotTelemetryObservation>(
      "robot telemetry timestamp must be positive and valid");
  }

  RobotTelemetryObservation observation;
  for (std::size_t index = 0; index < kArmJointCount; ++index) {
    if (message.joint_names[index] != kCanonicalJointNames[index]) {
      return invalid_request<RobotTelemetryObservation>(
        "robot telemetry joint names must use canonical order shoulder_pan_joint through "
        "wrist_3_joint");
    }
    if (!std::isfinite(message.joint_positions[index]) ||
      !std::isfinite(message.joint_velocities[index]))
    {
      return invalid_request<RobotTelemetryObservation>(
        "robot telemetry joint positions and velocities must be finite");
    }
    observation.joint_positions[index] = message.joint_positions[index];
    observation.joint_velocities[index] = message.joint_velocities[index];
  }
  if (!std::isfinite(message.rail_position) || !std::isfinite(message.rail_velocity)) {
    return invalid_request<RobotTelemetryObservation>(
      "robot telemetry rail position and velocity must be finite");
  }
  for (std::size_t index = 0; index < kGripperJointCount; ++index) {
    if (message.gripper_joint_names[index] != kCanonicalGripperJointNames[index]) {
      return invalid_request<RobotTelemetryObservation>(
        "robot telemetry gripper joint names must use canonical left/right order");
    }
    if (!std::isfinite(message.gripper_joint_positions[index]) ||
      !std::isfinite(message.gripper_joint_velocities[index]))
    {
      return invalid_request<RobotTelemetryObservation>(
        "robot telemetry gripper joint positions and velocities must be finite");
    }
    observation.gripper_joint_positions[index] = message.gripper_joint_positions[index];
    observation.gripper_joint_velocities[index] = message.gripper_joint_velocities[index];
  }
  observation.rail_position = message.rail_position;
  observation.rail_velocity = message.rail_velocity;
  observation.observation_time = *observation_time;
  observation.source_id = message.source_id;
  return Result<RobotTelemetryObservation>::success(std::move(observation));
}

Result<TaskReservation> task_reservation_from_message(
  const restocker_interfaces::msg::TaskReservation & message, Revision maximum_revision)
{
  using ReservationMessage = restocker_interfaces::msg::TaskReservation;
  const auto product_class = bounded_enum<ProductClass>(
    message.product_class, ReservationMessage::PRODUCT_CLASS_LARGE_BOTTLE);
  // Captured at grant from the destination lane's policy; Unknown is a wildcard policy, exactly
  // as it is on a lane.
  const auto destination_expected_product_class = bounded_enum<ProductClass>(
    message.destination_expected_product_class,
    ReservationMessage::PRODUCT_CLASS_LARGE_BOTTLE);
  const auto stage =
    bounded_enum<ReservationStage>(message.stage, ReservationMessage::STAGE_DETACHED);
  const auto created_at = time_from_message(message.created_at);
  if (message.reservation_id == 0 || message.request_id.empty() || message.object_id == 0 ||
    message.object_source_id.empty() || !product_class || !stage || !created_at ||
    !valid_optional_string(message.has_sku, message.sku) ||
    !valid_optional_string(message.has_source_lane, message.source_lane_id) ||
    !destination_expected_product_class ||
    !valid_optional_string(
      message.has_destination_expected_sku, message.destination_expected_sku) ||
    message.destination_lane_id.empty() ||
    (message.has_source_lane && message.source_lane_id == message.destination_lane_id) ||
    (*stage != ReservationStage::Detached && message.placed_in_destination) ||
    message.created_revision == 0 || message.admitted_robot_telemetry_revision == 0 ||
    message.admitted_robot_telemetry_revision > message.created_revision ||
    message.created_revision > message.revision ||
    message.revision > maximum_revision)
  {
    return invalid_request<TaskReservation>("task reservation message is invalid");
  }
  TaskReservation reservation;
  reservation.reservation_id = message.reservation_id;
  reservation.request_id = message.request_id;
  reservation.object_id = ObjectId{message.object_id};
  reservation.object_source_id = message.object_source_id;
  reservation.product_class = *product_class;
  if (message.has_sku) {
    reservation.sku = message.sku;
  }
  if (message.has_source_lane) {
    reservation.source_lane = LaneId{message.source_lane_id};
  }
  reservation.destination_lane = LaneId{message.destination_lane_id};
  reservation.stage = *stage;
  reservation.placed_in_destination = message.placed_in_destination;
  reservation.created_at = *created_at;
  reservation.created_revision = message.created_revision;
  reservation.admitted_robot_telemetry_revision = message.admitted_robot_telemetry_revision;
  reservation.revision = message.revision;
  reservation.destination_expected_product_class = *destination_expected_product_class;
  if (message.has_destination_expected_sku) {
    reservation.destination_expected_sku = message.destination_expected_sku;
  }
  return Result<TaskReservation>::success(std::move(reservation));
}

Result<TrackedObject> tracked_object_from_message(
  const restocker_interfaces::msg::TrackedObject & message, Revision maximum_revision)
{
  using ObjectMessage = restocker_interfaces::msg::TrackedObject;
  const auto product_class =
    bounded_enum<ProductClass>(message.product_class, ObjectMessage::PRODUCT_CLASS_LARGE_BOTTLE);
  const auto orientation =
    bounded_enum<ObjectOrientation>(message.orientation, ObjectMessage::ORIENTATION_TILTED);
  const auto tracking =
    bounded_enum<TrackingState>(message.tracking_state, ObjectMessage::TRACKING_REMOVED);
  const auto grasp = bounded_enum<GraspState>(message.grasp_state, ObjectMessage::GRASP_ATTACHED);
  const auto pose = pose_from_message(message.pose.pose);
  const auto observation_time = time_from_message(message.observation_time);
  const auto transition_time = time_from_message(message.transition_time);
  if (message.id == 0 || message.source_object_id.empty() || !product_class || !orientation ||
    !tracking || !grasp || !pose || !observation_time || !transition_time ||
    !valid_optional_string(message.has_sku, message.sku) || message.revision == 0 ||
    message.revision > maximum_revision ||
    transition_time->nanoseconds() < observation_time->nanoseconds())
  {
    return invalid_request<TrackedObject>("tracked object message is invalid");
  }

  TrackedObject object;
  object.id = ObjectId{message.id};
  object.source_object_id = message.source_object_id;
  object.product_class = *product_class;
  if (message.has_sku) {
    object.sku = message.sku;
  }
  object.pose_in_world = *pose;
  for (std::size_t row = 0; row < 6; ++row) {
    for (std::size_t column = 0; column < 6; ++column) {
      object.pose_covariance(row, column) = message.pose.covariance[row * 6 + column];
    }
  }
  if (!valid_covariance(object.pose_covariance)) {
    return invalid_request<TrackedObject>("tracked object pose covariance is invalid");
  }
  object.orientation = *orientation;
  object.tracking_state = *tracking;
  object.grasp_state = *grasp;
  object.observation_time = *observation_time;
  object.transition_time = *transition_time;
  object.revision = message.revision;
  return Result<TrackedObject>::success(std::move(object));
}

Result<ShelfLane> shelf_lane_from_message(
  const restocker_interfaces::msg::ShelfLane & message, Revision maximum_revision)
{
  using LaneMessage = restocker_interfaces::msg::ShelfLane;
  const auto product_class = bounded_enum<ProductClass>(
    message.expected_product_class, LaneMessage::PRODUCT_CLASS_LARGE_BOTTLE);
  const auto verified_at = time_from_message(message.last_verified);
  if (message.id.empty() || !product_class || !verified_at ||
    !valid_optional_string(message.has_expected_sku, message.expected_sku) ||
    !std::isfinite(message.depth_m) || message.depth_m <= 0.0 ||
    !std::isfinite(message.available_depth_m) || message.available_depth_m < 0.0 ||
    message.available_depth_m > message.depth_m || message.revision == 0 ||
    message.revision > maximum_revision || message.evidence_revision > message.revision)
  {
    return invalid_request<ShelfLane>("shelf lane geometry, identity, or revision is invalid");
  }

  ShelfLane lane;
  lane.id = LaneId{message.id};
  lane.expected_product_class = *product_class;
  if (message.has_expected_sku) {
    lane.expected_sku = message.expected_sku;
  }
  lane.target_count = message.target_count;
  std::set<ObjectId> content_ids;
  for (const std::uint64_t id : message.contents) {
    if (id == 0 || !content_ids.insert(ObjectId{id}).second) {
      return invalid_request<ShelfLane>(
        "shelf lane contents contain an invalid or duplicate object ID");
    }
    lane.contents.push_back(ObjectId{id});
  }
  if (std::ranges::any_of(
      message.observed_source_object_ids,
      [](const std::string & value) {return value.empty();}) ||
    !std::ranges::is_sorted(message.observed_source_object_ids) ||
    std::adjacent_find(
      message.observed_source_object_ids.begin(),
      message.observed_source_object_ids.end()) != message.observed_source_object_ids.end())
  {
    return invalid_request<ShelfLane>(
      "shelf lane evidence must be non-empty, sorted, and unique");
  }
  lane.observed_source_object_ids = message.observed_source_object_ids;
  lane.depth_m = message.depth_m;
  lane.available_depth_m = message.available_depth_m;
  lane.obstructed = message.obstructed;
  lane.last_verified = *verified_at;
  lane.evidence_invalidated = message.evidence_invalidated;
  if (message.evidence_invalidated) {
    const auto invalidated_at = time_from_message(message.evidence_invalidated_at);
    if (!invalidated_at) {
      return invalid_request<ShelfLane>("invalidated lane evidence needs a finite timestamp");
    }
    lane.evidence_invalidated_at = *invalidated_at;
  } else {
    lane.evidence_invalidated_at = rclcpp::Time{std::int64_t{0}, verified_at->get_clock_type()};
  }
  lane.evidence_revision = message.evidence_revision;
  lane.ledger_unreliable = message.ledger_unreliable;
  lane.revision = message.revision;
  return Result<ShelfLane>::success(std::move(lane));
}

Result<RobotExecutionState> robot_execution_state_from_message(
  const restocker_interfaces::msg::RobotExecutionState & message, Revision maximum_revision)
{
  const auto task_phase = task_phase_from_message(message.task_phase);
  const auto fault_state = fault_state_from_message(message.fault_state);
  const auto robot_time = time_from_message(message.telemetry_time);
  const bool has_telemetry_source = !message.telemetry_source_id.empty();
  const bool valid_telemetry_source = has_telemetry_source &&
    message.telemetry_source_id.size() <= 128U &&
    ascii_alphanumeric(message.telemetry_source_id.front()) &&
    std::all_of(
    message.telemetry_source_id.begin(), message.telemetry_source_id.end(),
    valid_source_character);
  if (!task_phase || !fault_state || !robot_time || !std::isfinite(message.rail_position) ||
    !std::isfinite(message.rail_velocity) ||
    !std::ranges::all_of(
      message.joint_positions, [](double value) {return std::isfinite(value);}) ||
    !std::ranges::all_of(
      message.joint_velocities, [](double value) {return std::isfinite(value);}) ||
    !std::ranges::all_of(
      message.gripper_joint_positions, [](double value) {return std::isfinite(value);}) ||
    !std::ranges::all_of(
      message.gripper_joint_velocities, [](double value) {return std::isfinite(value);}) ||
    message.telemetry_revision > message.revision || message.revision > maximum_revision ||
    (message.telemetry_revision == 0) != (robot_time->nanoseconds() == 0) ||
    (message.telemetry_revision == 0 ? has_telemetry_source : !valid_telemetry_source) ||
    (message.has_held_object ? message.held_object == 0 : message.held_object != 0))
  {
    return invalid_request<RobotExecutionState>("robot execution state message is invalid");
  }
  // The grasp transform derives a held product's pose, so a malformed one is rejected even when
  // nothing is held (identity is legitimate only alongside has_held_object == false).
  const auto grasp = pose_from_message(message.grasp_center_from_held_object);
  if (!grasp) {
    return invalid_request<RobotExecutionState>(
      "held-product grasp transform is not a finite rigid transform");
  }

  RobotExecutionState robot;
  robot.joint_positions = message.joint_positions;
  robot.joint_velocities = message.joint_velocities;
  robot.rail_position = message.rail_position;
  robot.rail_velocity = message.rail_velocity;
  robot.gripper_joint_positions = message.gripper_joint_positions;
  robot.gripper_joint_velocities = message.gripper_joint_velocities;
  if (message.has_held_object) {
    robot.held_object = ObjectId{message.held_object};
  }
  robot.grasp_center_from_held_object = *grasp;
  robot.task_phase = *task_phase;
  robot.fault_state = *fault_state;
  robot.telemetry_time = *robot_time;
  robot.telemetry_source_id = message.telemetry_source_id;
  robot.telemetry_revision = message.telemetry_revision;
  robot.revision = message.revision;
  return Result<RobotExecutionState>::success(std::move(robot));
}

Result<WorldStateSnapshot> snapshot_from_message(
  const restocker_interfaces::msg::WorldStateSnapshot & message,
  const std::string & expected_planning_frame)
{
  using EventMessage = restocker_interfaces::msg::WorldStateEvent;

  if (expected_planning_frame.empty()) {
    return invalid_snapshot("expected planning frame is empty");
  }
  if (message.header.frame_id != expected_planning_frame) {
    return invalid_snapshot(
      "snapshot planning frame does not match the configured planning frame",
      WorldStateErrorCode::FrameMismatch);
  }
  if (!time_from_message(message.header.stamp)) {
    return invalid_snapshot("snapshot header contains an invalid timestamp");
  }

  WorldStateSnapshot snapshot;
  snapshot.revision = message.revision;
  std::set<std::string> source_ids;
  for (const auto & item : message.objects) {
    auto converted = tracked_object_from_message(item, message.revision);
    if (!converted || !source_ids.insert(item.source_object_id).second) {
      return invalid_snapshot("snapshot contains an invalid or duplicate tracked object");
    }
    auto object = std::move(converted.value());
    if (!snapshot.objects.emplace(object.id, std::move(object)).second) {
      return invalid_snapshot("snapshot contains a duplicate tracked-object ID");
    }
  }

  for (const auto & item : message.lanes) {
    auto converted = shelf_lane_from_message(item, message.revision);
    if (!converted) {
      return invalid_snapshot("snapshot contains invalid lane geometry, identity, or revision");
    }
    auto lane = std::move(converted.value());
    if (!snapshot.lanes.emplace(lane.id, std::move(lane)).second) {
      return invalid_snapshot("snapshot contains a duplicate lane ID");
    }
  }

  auto robot = robot_execution_state_from_message(message.robot, message.revision);
  if (!robot) {
    return invalid_snapshot("snapshot contains invalid robot state");
  }
  snapshot.robot = std::move(robot.value());

  if (message.has_active_reservation) {
    auto reservation = task_reservation_from_message(message.active_reservation, message.revision);
    if (!reservation) {
      return invalid_snapshot(
        "snapshot contains an invalid active reservation: " +
        reservation.error().detail);
    }
    snapshot.active_reservation = std::move(reservation.value());
  } else if (has_reservation_payload(message.active_reservation)) {
    return invalid_snapshot("inactive reservation contains a diagnostic payload");
  }

  Revision previous_event_revision = 0;
  snapshot.events.reserve(message.events.size());
  for (const auto & item : message.events) {
    const auto kind = bounded_enum<EventKind>(item.kind, EventMessage::LANE_LEDGER_UNRELIABLE);
    const auto event_time = time_from_message(item.event_time);
    if (!kind || !event_time || item.revision == 0 || item.revision > message.revision ||
      item.revision < previous_event_revision ||
      (item.has_object_id ? item.object_id == 0 : item.object_id != 0) ||
      !valid_optional_string(item.has_lane_id, item.lane_id))
    {
      return invalid_snapshot("snapshot contains an invalid or unordered event");
    }
    WorldStateEvent event;
    event.revision = item.revision;
    event.event_time = *event_time;
    event.kind = *kind;
    if (item.has_object_id) {
      event.object_id = ObjectId{item.object_id};
    }
    if (item.has_lane_id) {
      event.lane_id = LaneId{item.lane_id};
    }
    event.detail = item.detail;
    snapshot.events.push_back(std::move(event));
    previous_event_revision = item.revision;
  }
  if (const auto error = snapshot_invariant_error(snapshot)) {
    return invalid_snapshot(*error, WorldStateErrorCode::InvariantViolation);
  }
  return Result<WorldStateSnapshot>::success(std::move(snapshot));
}

Result<ReserveTaskRequest> reserve_task_request_from_message(
  const restocker_interfaces::srv::ReserveTask::Request & message)
{
  if (message.has_source_lane &&
    (message.source_lane_id.empty() || message.source_lane_revision == 0))
  {
    return invalid_request<ReserveTaskRequest>(
      "source lane identity and revision are required when has_source_lane is true");
  }
  if (!message.has_source_lane &&
    (!message.source_lane_id.empty() || message.source_lane_revision != 0))
  {
    return invalid_request<ReserveTaskRequest>(
      "source lane identity and revision must be empty when has_source_lane is false");
  }

  ReserveTaskRequest request;
  request.request_id = message.request_id;
  request.selected_snapshot_revision = message.selected_snapshot_revision;
  request.object_id = ObjectId{message.object_id};
  request.object_revision = message.object_revision;
  if (message.has_source_lane) {
    request.source_lane = LaneId{message.source_lane_id};
    request.source_lane_revision = message.source_lane_revision;
  }
  request.destination_lane = LaneId{message.destination_lane_id};
  request.destination_lane_revision = message.destination_lane_revision;
  return Result<ReserveTaskRequest>::success(std::move(request));
}

Result<ReservedTaskCheckpoint> checkpoint_request_from_message(
  const restocker_interfaces::srv::CheckpointTaskState::Request & message,
  const rclcpp::Time & accepted_at)
{
  const auto task_phase = task_phase_from_message(message.task_phase);
  const auto fault_state = fault_state_from_message(message.fault_state);
  const auto stage = bounded_enum<ReservationStage>(
    message.expected_reservation_stage,
    restocker_interfaces::srv::CheckpointTaskState::Request::STAGE_DETACHED);
  if (!task_phase || !fault_state || !stage) {
    return invalid_request<ReservedTaskCheckpoint>(
      "checkpoint contains an unknown stage, task-phase, or fault-state value");
  }
  return Result<ReservedTaskCheckpoint>::success(
    ReservedTaskCheckpoint{
      message.token, message.operation_id, message.expected_reservation_id, *stage,
      message.expected_reservation_revision, *task_phase, *fault_state, accepted_at});
}

restocker_interfaces::srv::CommitReservedAttachment::Response attachment_result_to_message(
  const ReservedAttachmentResult & result, Revision current_revision)
{
  restocker_interfaces::srv::CommitReservedAttachment::Response response;
  response.status = result ? operation_status_ok() : operation_status_from_error(result.error());
  const ReservationReceipt * receipt = result ? &result.value() :
    (result.historical_receipt() ? &*result.historical_receipt() : nullptr);
  response.world_revision = receipt ? receipt->revision : current_revision;
  if (receipt) {
    response.has_reservation = true;
    response.reservation = task_reservation_to_message(receipt->reservation);
  }
  return response;
}

Result<ReservedAttachmentRequest> attachment_request_from_message(
  const restocker_interfaces::srv::CommitReservedAttachment::Request & message)
{
  const auto grasp = pose_from_message(message.grasp_center_from_held_object);
  if (!grasp) {
    return invalid_request<ReservedAttachmentRequest>(
      "attachment grasp transform is not a finite rigid transform");
  }
  return Result<ReservedAttachmentRequest>::success(
    ReservedAttachmentRequest{message.token, message.operation_id, *grasp});
}

Result<ReservedDetachmentRequest> detachment_request_from_message(
  const restocker_interfaces::srv::CommitReservedDetachment::Request & message)
{
  using Request = restocker_interfaces::srv::CommitReservedDetachment::Request;
  DetachmentDisposition disposition;
  switch (message.disposition) {
    case Request::PLACE_IN_RESERVED_DESTINATION:
      disposition = DetachmentDisposition::PlaceInReservedDestination;
      break;
    case Request::RELEASE_WITHOUT_MEMBERSHIP:
      disposition = DetachmentDisposition::ReleaseWithoutMembership;
      break;
    default:
      return invalid_request<ReservedDetachmentRequest>(
        "detachment contains an unknown disposition value");
  }
  const auto released_at = time_from_message(message.released_at);
  if (!released_at) {
    return invalid_request<ReservedDetachmentRequest>(
      "detachment release time is not a valid timestamp");
  }
  return Result<ReservedDetachmentRequest>::success(
    ReservedDetachmentRequest{message.token, message.operation_id, disposition, *released_at});
}

Result<ReleaseReservationRequest> release_request_from_message(
  const restocker_interfaces::srv::ReleaseTaskReservation::Request & message)
{
  using Request = restocker_interfaces::srv::ReleaseTaskReservation::Request;
  ReservationStage expected_stage;
  switch (message.expected_reservation_stage) {
    case Request::EXPECTED_STAGE_RESERVED:
      expected_stage = ReservationStage::Reserved;
      break;
    case Request::EXPECTED_STAGE_ATTACHED:
      expected_stage = ReservationStage::Attached;
      break;
    case Request::EXPECTED_STAGE_DETACHED:
      expected_stage = ReservationStage::Detached;
      break;
    default:
      return invalid_request<ReleaseReservationRequest>(
        "release contains an unknown expected reservation stage");
  }
  ReservationOutcome outcome;
  switch (message.outcome) {
    case Request::OUTCOME_SUCCEEDED:
      outcome = ReservationOutcome::Succeeded;
      break;
    case Request::OUTCOME_CANCELED:
      outcome = ReservationOutcome::Canceled;
      break;
    case Request::OUTCOME_FAILED_SAFE:
      outcome = ReservationOutcome::FailedSafe;
      break;
    default:
      return invalid_request<ReleaseReservationRequest>(
        "release contains an unknown outcome value");
  }
  const auto task_phase = task_phase_from_message(message.terminal_task_phase);
  const auto fault_state = fault_state_from_message(message.terminal_fault_state);
  if (message.expected_reservation_id == 0 || message.expected_reservation_revision == 0 ||
    !task_phase || !fault_state)
  {
    return invalid_request<ReleaseReservationRequest>(
      "release contains invalid expected reservation or terminal state fields");
  }
  return Result<ReleaseReservationRequest>::success(
    ReleaseReservationRequest{
      message.token, message.operation_id, message.expected_reservation_id, expected_stage,
      message.expected_reservation_revision, outcome, *task_phase, *fault_state});
}

OperationStatus operation_status_ok()
{
  OperationStatus status;
  status.code = OperationStatus::OK;
  return status;
}

OperationStatus operation_status_from_error(const WorldStateError & error)
{
  OperationStatus status;
  status.detail = error.detail;
  switch (error.code) {
    case WorldStateErrorCode::InvalidArgument:
    case WorldStateErrorCode::ClockMismatch:
    case WorldStateErrorCode::StaleObservation:
    case WorldStateErrorCode::FutureObservation:
    case WorldStateErrorCode::OutOfOrder:
    case WorldStateErrorCode::FrameMismatch:
    case WorldStateErrorCode::IdentityConflict:
      status.code = OperationStatus::INVALID_ARGUMENT;
      break;
    case WorldStateErrorCode::AttachmentClockNotReady:
      status.code = OperationStatus::ATTACHMENT_CLOCK_NOT_READY;
      break;
    case WorldStateErrorCode::ClockAuthorityInhibited:
      status.code = OperationStatus::CLOCK_AUTHORITY_INHIBITED;
      break;
    case WorldStateErrorCode::NotFound:
    case WorldStateErrorCode::Removed:
      status.code = OperationStatus::NOT_FOUND;
      break;
    case WorldStateErrorCode::RevisionConflict:
      status.code = OperationStatus::REVISION_CONFLICT;
      break;
    case WorldStateErrorCode::ReservationConflict:
      status.code = OperationStatus::RESERVATION_CONFLICT;
      break;
    case WorldStateErrorCode::TokenMismatch:
      status.code = OperationStatus::TOKEN_MISMATCH;
      break;
    case WorldStateErrorCode::PredicateFailed:
      status.code = OperationStatus::PREDICATE_FAILED;
      break;
    case WorldStateErrorCode::InvalidTransition:
      status.code = OperationStatus::INVALID_TRANSITION;
      break;
    case WorldStateErrorCode::IdempotencyConflict:
      status.code = OperationStatus::IDEMPOTENCY_CONFLICT;
      break;
    case WorldStateErrorCode::ResourceExhausted:
      status.code = OperationStatus::RESOURCE_EXHAUSTED;
      break;
    case WorldStateErrorCode::InvariantViolation:
      status.code = OperationStatus::INTERNAL_ERROR;
      break;
  }
  return status;
}

restocker_interfaces::msg::TaskReservation task_reservation_to_message(
  const TaskReservation & reservation)
{
  restocker_interfaces::msg::TaskReservation message;
  message.reservation_id = reservation.reservation_id;
  message.request_id = reservation.request_id;
  message.object_id = reservation.object_id.value;
  message.object_source_id = reservation.object_source_id;
  message.product_class = static_cast<std::uint8_t>(reservation.product_class);
  message.has_sku = reservation.sku.has_value();
  message.sku = reservation.sku.value_or("");
  message.has_source_lane = reservation.source_lane.has_value();
  message.source_lane_id = reservation.source_lane ? reservation.source_lane->value : "";
  message.destination_lane_id = reservation.destination_lane.value;
  message.stage = static_cast<std::uint8_t>(reservation.stage);
  message.placed_in_destination = reservation.placed_in_destination;
  message.created_at = time_to_message(reservation.created_at);
  message.created_revision = reservation.created_revision;
  message.admitted_robot_telemetry_revision = reservation.admitted_robot_telemetry_revision;
  message.revision = reservation.revision;
  message.destination_expected_product_class = static_cast<std::uint8_t>(
    reservation.destination_expected_product_class);
  message.has_destination_expected_sku = reservation.destination_expected_sku.has_value();
  message.destination_expected_sku = reservation.destination_expected_sku.value_or("");
  return message;
}

restocker_interfaces::msg::TrackedObject tracked_object_to_message(
  const TrackedObject & object)
{
  restocker_interfaces::msg::TrackedObject message;
  message.id = object.id.value;
  message.source_object_id = object.source_object_id;
  message.product_class = static_cast<std::uint8_t>(object.product_class);
  message.has_sku = object.sku.has_value();
  message.sku = object.sku.value_or("");
  message.pose.pose = pose_to_message(object.pose_in_world);
  for (std::size_t row = 0; row < 6; ++row) {
    for (std::size_t column = 0; column < 6; ++column) {
      message.pose.covariance[row * 6 + column] = object.pose_covariance(row, column);
    }
  }
  message.orientation = static_cast<std::uint8_t>(object.orientation);
  message.tracking_state = static_cast<std::uint8_t>(object.tracking_state);
  message.grasp_state = static_cast<std::uint8_t>(object.grasp_state);
  message.observation_time = time_to_message(object.observation_time);
  message.transition_time = time_to_message(object.transition_time);
  message.revision = object.revision;
  return message;
}

restocker_interfaces::msg::ShelfLane shelf_lane_to_message(const ShelfLane & lane)
{
  restocker_interfaces::msg::ShelfLane message;
  message.id = lane.id.value;
  message.expected_product_class = static_cast<std::uint8_t>(lane.expected_product_class);
  message.has_expected_sku = lane.expected_sku.has_value();
  message.expected_sku = lane.expected_sku.value_or("");
  message.target_count = lane.target_count;
  message.contents.reserve(lane.contents.size());
  for (const ObjectId object_id : lane.contents) {
    message.contents.push_back(object_id.value);
  }
  message.observed_source_object_ids = lane.observed_source_object_ids;
  message.depth_m = lane.depth_m;
  message.available_depth_m = lane.available_depth_m;
  message.obstructed = lane.obstructed;
  message.last_verified = time_to_message(lane.last_verified);
  message.evidence_invalidated = lane.evidence_invalidated;
  message.evidence_invalidated_at = time_to_message(lane.evidence_invalidated_at);
  message.evidence_revision = lane.evidence_revision;
  message.revision = lane.revision;
  message.ledger_unreliable = lane.ledger_unreliable;
  return message;
}

restocker_interfaces::msg::RobotExecutionState robot_execution_state_to_message(
  const RobotExecutionState & robot)
{
  restocker_interfaces::msg::RobotExecutionState message;
  message.joint_positions = robot.joint_positions;
  message.joint_velocities = robot.joint_velocities;
  message.rail_position = robot.rail_position;
  message.rail_velocity = robot.rail_velocity;
  message.gripper_joint_positions = robot.gripper_joint_positions;
  message.gripper_joint_velocities = robot.gripper_joint_velocities;
  message.has_held_object = robot.held_object.has_value();
  message.held_object = robot.held_object.value_or(ObjectId{}).value;
  message.grasp_center_from_held_object = pose_to_message(robot.grasp_center_from_held_object);
  message.task_phase = static_cast<std::uint8_t>(robot.task_phase);
  message.fault_state = static_cast<std::uint8_t>(robot.fault_state);
  message.telemetry_time = time_to_message(robot.telemetry_time);
  message.telemetry_source_id = robot.telemetry_source_id;
  message.telemetry_revision = robot.telemetry_revision;
  message.revision = robot.revision;
  return message;
}

restocker_interfaces::msg::WorldStateSnapshot snapshot_to_message(
  const WorldStateSnapshot & snapshot, const SnapshotMessageOptions & options)
{
  restocker_interfaces::msg::WorldStateSnapshot message;
  message.header.frame_id = options.planning_frame;
  message.header.stamp = time_to_message(options.snapshot_time);
  message.revision = snapshot.revision;
  message.objects.reserve(snapshot.objects.size());
  for (const auto & entry : snapshot.objects) {
    const auto & object = entry.second;
    if (!options.include_removed && object.tracking_state == TrackingState::Removed) {
      continue;
    }
    message.objects.push_back(tracked_object_to_message(object));
  }

  message.lanes.reserve(snapshot.lanes.size());
  for (const auto & entry : snapshot.lanes) {
    const auto & lane = entry.second;
    message.lanes.push_back(shelf_lane_to_message(lane));
  }

  message.robot = robot_execution_state_to_message(snapshot.robot);
  message.has_active_reservation = snapshot.active_reservation.has_value();
  if (snapshot.active_reservation) {
    message.active_reservation = task_reservation_to_message(*snapshot.active_reservation);
  }

  if (options.include_events) {
    message.events.reserve(snapshot.events.size());
    for (const WorldStateEvent & event : snapshot.events) {
      restocker_interfaces::msg::WorldStateEvent item;
      item.revision = event.revision;
      item.event_time = time_to_message(event.event_time);
      item.kind = static_cast<std::uint8_t>(event.kind);
      item.has_object_id = event.object_id.has_value();
      item.object_id = event.object_id.value_or(ObjectId{}).value;
      item.has_lane_id = event.lane_id.has_value();
      item.lane_id = event.lane_id ? event.lane_id->value : "";
      item.detail = event.detail;
      message.events.push_back(std::move(item));
    }
  }
  return message;
}

}  // namespace restocker_world_state
