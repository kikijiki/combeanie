// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <compare>

#include <rcl/time.h>

#include <chrono>
#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

#include <rclcpp/time.hpp>

namespace restocker_world_state
{

using Revision = std::uint64_t;
using PoseCovariance = Eigen::Matrix<double, 6, 6>;
inline constexpr std::size_t kArmJointCount = 6U;
inline constexpr std::size_t kGripperJointCount = 2U;

struct ObjectId
{
  std::uint64_t value{0};

  [[nodiscard]] explicit operator bool() const noexcept {return value != 0;}
  auto operator<=>(const ObjectId &) const = default;
};

struct LaneId
{
  std::string value;

  auto operator<=>(const LaneId &) const = default;
};

// Captured while the managed clock update mutex is held through store admission. Retained
// samples keep their original lineage; callers cannot restamp them after a clock change.
struct AuthorityClockSample
{
  rclcpp::Time time{std::int64_t{0}, RCL_ROS_TIME};
  std::uint64_t lineage{0};
  bool managed_ros_started{false};
};

enum class ProductClass : std::uint8_t
{
  Unknown,
  Can,
  SmallBottle,
  LargeBottle,
};

enum class ObjectOrientation : std::uint8_t
{
  Unknown,
  Upright,
  Horizontal,
  Tilted,
};

enum class TrackingState : std::uint8_t
{
  Tracked,
  Occluded,
  Lost,
  Removed,
};

enum class GraspState : std::uint8_t
{
  Free,
  Attached,
};

enum class TaskPhase : std::uint8_t
{
  Idle,
  ValidatingScene,
  Executing,
  Recovering,
  Fault,
  RequestingOperator,
};

enum class FaultState : std::uint8_t
{
  None,
  Recoverable,
  NonRecoverable,
  ExternalInconsistency,
};

// Semantic states permitted while an authoritative task reservation is active.
// Kept public so every authority adapter validates the same closed matrix.
[[nodiscard]] constexpr bool valid_active_task_semantics(
  TaskPhase phase, FaultState fault) noexcept
{
  switch (phase) {
    case TaskPhase::Executing:
      return fault == FaultState::None;
    case TaskPhase::Recovering:
      return fault == FaultState::Recoverable;
    case TaskPhase::Fault:
    case TaskPhase::RequestingOperator:
      return fault != FaultState::None;
    case TaskPhase::Idle:
    case TaskPhase::ValidatingScene:
      return false;
  }
  return false;
}

enum class WorldStateErrorCode : std::uint8_t
{
  InvalidArgument,
  ClockMismatch,
  FrameMismatch,
  StaleObservation,
  FutureObservation,
  OutOfOrder,
  IdentityConflict,
  NotFound,
  Removed,
  RevisionConflict,
  ReservationConflict,
  TokenMismatch,
  PredicateFailed,
  InvalidTransition,
  IdempotencyConflict,
  ResourceExhausted,
  InvariantViolation,
  AttachmentClockNotReady,
  ClockAuthorityInhibited,
};

// Exact operation IDs fit the existing 128-byte admission bound. Source identities have no
// such bound: retain a prefix and original byte count, alongside the record's canonical object
// ID. Prefixes can collide and are never authorization or unique identity. Hex prevents embedded
// control/NUL bytes from injecting or truncating log fields. Neither function allocates.
struct AttachmentDiagnosticIdentity
{
  static constexpr std::size_t capacity = 128;
  std::array<std::uint8_t, capacity> prefix{};
  std::uint64_t original_size{0};

  [[nodiscard]] static AttachmentDiagnosticIdentity capture(std::string_view value) noexcept;
  [[nodiscard]] std::array<char, capacity * 2 + 1> hex_prefix() const noexcept;
  [[nodiscard]] bool truncated() const noexcept {return original_size > capacity;}
};

// Fixed-size diagnostic evidence copied at the attachment refusal's locked decision boundary.
// No capabilities or borrowed object storage: this remains valid after later state mutations.
struct AttachmentTemporalRefusal
{
  std::int64_t committed_at_ns{0};
  std::int64_t object_observation_ns{0};
  std::int64_t object_transition_ns{0};
  std::int64_t robot_telemetry_ns{0};
  Revision world_revision{0};
  ObjectId object_id;
  Revision object_revision{0};
  Revision robot_revision{0};
  Revision robot_telemetry_revision{0};
  std::uint64_t reservation_id{0};
  Revision reservation_revision{0};
  std::uint8_t reservation_stage{0};
  bool object_postdates_commit{false};
  bool robot_postdates_commit{false};
  std::optional<std::int64_t> object_received_at_ns;
  std::optional<std::uint64_t> object_clock_lineage;
  std::optional<std::uint64_t> commit_clock_lineage;
  bool object_managed_ros_started{false};
  bool commit_managed_ros_started{false};
  WorldStateErrorCode decision_kind{WorldStateErrorCode::OutOfOrder};
  bool clock_inhibited{false};
  std::uint64_t authority_clock_lineage{0};
  AttachmentDiagnosticIdentity operation_id{};
  AttachmentDiagnosticIdentity source_id{};
};

struct WorldStateError
{
  WorldStateErrorCode code;
  std::string detail;
  std::optional<AttachmentTemporalRefusal> attachment_temporal_refusal{};
};

template<typename T>
class [[nodiscard]] Result
{
public:
  [[nodiscard]] static Result success(T value) {return Result(std::move(value));}

  [[nodiscard]] static Result failure(WorldStateError error) {return Result(std::move(error));}

  [[nodiscard]] bool has_value() const noexcept {return std::holds_alternative<T>(storage_);}
  [[nodiscard]] explicit operator bool() const noexcept {return has_value();}

  [[nodiscard]] const T & value() const {return std::get<T>(storage_);}
  [[nodiscard]] T & value() {return std::get<T>(storage_);}
  [[nodiscard]] const WorldStateError & error() const {return std::get<WorldStateError>(storage_);}

private:
  explicit Result(T value)
  : storage_(std::move(value)) {}
  explicit Result(WorldStateError error)
  : storage_(std::move(error)) {}

  std::variant<T, WorldStateError> storage_;
};

struct ObjectObservation
{
  std::string source_object_id;
  std::string frame_id;
  ProductClass product_class{ProductClass::Unknown};
  std::optional<std::string> sku;
  Eigen::Isometry3d pose_in_world{Eigen::Isometry3d::Identity()};
  PoseCovariance pose_covariance{PoseCovariance::Zero()};
  ObjectOrientation orientation{ObjectOrientation::Unknown};
  rclcpp::Time observation_time{std::int64_t{0}, RCL_ROS_TIME};
};

// One admitted measurement as an indivisible unit: the pose, covariance and orientation exactly
// as observed, stamped with the observation time that describes them (CMB-SPEC-11 I-2).
struct PoseEstimate
{
  Eigen::Isometry3d pose{Eigen::Isometry3d::Identity()};
  PoseCovariance covariance{PoseCovariance::Zero()};
  ObjectOrientation orientation{ObjectOrientation::Unknown};
  rclcpp::Time observation_time{std::int64_t{0}, RCL_ROS_TIME};
};

struct TrackedObject
{
  ObjectId id;
  std::string source_object_id;
  ProductClass product_class{ProductClass::Unknown};
  std::optional<std::string> sku;
  Eigen::Isometry3d pose_in_world{Eigen::Isometry3d::Identity()};
  PoseCovariance pose_covariance{PoseCovariance::Zero()};
  ObjectOrientation orientation{ObjectOrientation::Unknown};
  TrackingState tracking_state{TrackingState::Tracked};
  GraspState grasp_state{GraspState::Free};
  rclcpp::Time observation_time{std::int64_t{0}, RCL_ROS_TIME};
  rclcpp::Time transition_time{std::int64_t{0}, RCL_ROS_TIME};
  Revision revision{0};
  // Newest admitted measurement. Always written by observation admission, including admissions
  // whose act-on fields (`pose_in_world`, `pose_covariance`, `orientation`) the reservation hold
  // retains. Nothing reads it yet (CMB-SPEC-11 stage 1).
  PoseEstimate latest_measured{};
};

// What one more product of a kind costs a gravity-fed lane, in depth at its rear entrance.
//
// A lane used to hold at most one product, and every placement predicate asked whether it was
// empty. Under gravity feed a lane holds a column packed against the front rail, so the question
// becomes how much free depth is left behind that column, and the answer is per product: a can,
// a small bottle and a large bottle are different widths and consume different depths.
//
// Both numbers are derived from the catalogued product envelope and the surveyed lane by
// `load_product_lane_profiles`; neither is a tuned constant.
struct ProductLaneProfile
{
  ProductClass product_class{ProductClass::Unknown};
  // Absent for the entry that stands for every SKU of this class. An exact SKU match wins over
  // the class fallback, exactly as the collision catalog resolves geometry.
  std::optional<std::string> sku;
  // Free depth the lane must still have at its rear for one more of these to be released inside
  // the usable volume: the product's own footprint plus the clearance the release leaves behind
  // it. This is what replaces "the destination must be empty".
  double required_rear_depth_m{0.0};
  // Depth one more of these adds to the settled column. Products pack against each other on the
  // incline, so consecutive axes stand one diameter apart along the bed and the lane's depth axis
  // sees that foreshortened by the incline. It is what a placement must be seen to consume before
  // it may be called a placement.
  double column_pitch_m{0.0};
};

// Resolves the profile for an observed product: an exact SKU match if the catalog carries one,
// otherwise the class fallback. Empty when the class has no catalogued envelope, which is a
// refusal rather than a default: a product whose depth is unknown cannot be shown to fit.
[[nodiscard]] std::optional<ProductLaneProfile> resolve_product_lane_profile(
  const std::vector<ProductLaneProfile> & profiles, ProductClass product_class,
  const std::optional<std::string> & sku);

struct LaneDefinition
{
  LaneId id;
  ProductClass expected_product_class{ProductClass::Unknown};
  std::optional<std::string> expected_sku;
  double depth_m{0.0};
  std::uint32_t target_count{0};
};

// One row of the desired-stocking table: what the owner wants in one lane. The table is the
// world state's own lanes; this is the unit a policy change writes and a restart reloads.
struct LanePolicy
{
  LaneId id;
  ProductClass expected_product_class{ProductClass::Unknown};
  std::optional<std::string> expected_sku;
  std::uint32_t target_count{0};

  auto operator<=>(const LanePolicy &) const = default;
};

struct LaneObservation
{
  LaneId id;
  std::vector<std::string> observed_source_object_ids;
  double available_depth_m{0.0};
  bool obstructed{false};
  rclcpp::Time observation_time{std::int64_t{0}, RCL_ROS_TIME};
};

struct ShelfLane
{
  LaneId id;
  ProductClass expected_product_class{ProductClass::Unknown};
  std::optional<std::string> expected_sku;
  // Desired stocking count paired with the expected class/SKU above. Declared by the owner
  // (baseline file at start-up, SetLanePolicy afterwards), never derived from evidence.
  std::uint32_t target_count{0};
  std::vector<ObjectId> contents;
  std::vector<std::string> observed_source_object_ids;
  double depth_m{0.0};
  double available_depth_m{0.0};
  bool obstructed{false};
  rclcpp::Time last_verified{std::int64_t{0}, RCL_ROS_TIME};
  // Placement (and later policy / operator events) invalidate depletion evidence until a survey
  // refreshes it. Validity is this flag plus the configured horizon on last_verified, not a
  // continuous 500 ms age gate.
  bool evidence_invalidated{false};
  rclcpp::Time evidence_invalidated_at{std::int64_t{0}, RCL_ROS_TIME};
  Revision evidence_revision{0};
  // Identity trust for the contents FIFO. Set (latched) when a survey's measured column length
  // diverges beyond half a product pitch from what the settled contents can explain (an
  // in-flight Attached placement counts toward the growth bound only): unexplained growth, or a
  // shortfall that cannot be popped as a whole number of front-entry sales. Geometry stays
  // authoritative — evidence, the occupied-volume projection, and geometry-safe placements
  // continue — but reported contents are withdrawn from the deficit calculation and the
  // withdrawal is surfaced as an EventKind::LaneLedgerUnreliable event. Never cleared: the
  // camera cannot re-establish the identities it never read. A ledger whose entries the current
  // policy does not accept is not length-reconciled at all (selection reports the conflict).
  bool ledger_unreliable{false};
  Revision revision{0};
};

struct RobotTelemetryObservation
{
  std::array<double, kArmJointCount> joint_positions{};
  std::array<double, kArmJointCount> joint_velocities{};
  double rail_position{0.0};
  double rail_velocity{0.0};
  rclcpp::Time observation_time{std::int64_t{0}, RCL_ROS_TIME};
  std::array<double, kGripperJointCount> gripper_joint_positions{};
  std::array<double, kGripperJointCount> gripper_joint_velocities{};
  std::string source_id;
};

struct RobotExecutionState
{
  std::array<double, kArmJointCount> joint_positions{};
  std::array<double, kArmJointCount> joint_velocities{};
  double rail_position{0.0};
  double rail_velocity{0.0};
  std::array<double, kGripperJointCount> gripper_joint_positions{};
  std::array<double, kGripperJointCount> gripper_joint_velocities{};
  std::optional<ObjectId> held_object;
  // Pose of the held product's body frame in the gripper's `grasp_center` frame, recorded when
  // the grasp was committed and left untouched until it is released. A gripped product is rigidly
  // attached to the gripper, so this transform is constant while it is held: composed with the
  // robot's forward kinematics it gives the product's pose at any instant, and no observation of
  // the product is needed, or possible, once the jaws are closed around it. Identity, and
  // meaningless, when `held_object` is empty.
  Eigen::Isometry3d grasp_center_from_held_object{Eigen::Isometry3d::Identity()};
  TaskPhase task_phase{TaskPhase::Idle};
  FaultState fault_state{FaultState::None};
  rclcpp::Time telemetry_time{std::int64_t{0}, RCL_ROS_TIME};
  std::string telemetry_source_id;
  Revision telemetry_revision{0};
  Revision revision{0};
};

enum class EventKind : std::uint8_t
{
  ObjectObserved,
  ObjectTrackingChanged,
  ObjectRemoved,
  LaneConfigured,
  LaneVerified,
  RobotSemanticStateChanged,
  ObjectAttached,
  ObjectDetached,
  TaskReserved,
  TaskCheckpointed,
  TaskReservationReleased,
  LanePolicySet,
  // Identity-trust withdrawal on one lane: measured geometry and the contents ledger diverged
  // beyond half a product pitch. Appended after LaneVerified on the same revision, never
  // renumbered against the WorldStateEvent message constants.
  LaneLedgerUnreliable,
};

enum class ReservationStage : std::uint8_t
{
  Reserved,
  Attached,
  Detached,
};

enum class DetachmentDisposition : std::uint8_t
{
  PlaceInReservedDestination,
  ReleaseWithoutMembership,
};

enum class ReservationOutcome : std::uint8_t
{
  Succeeded,
  Canceled,
  FailedSafe,
};

struct TaskReservation
{
  std::uint64_t reservation_id{0};
  std::string request_id;
  ObjectId object_id;
  std::string object_source_id;
  ProductClass product_class{ProductClass::Unknown};
  std::optional<std::string> sku;
  std::optional<LaneId> source_lane;
  LaneId destination_lane;
  ReservationStage stage{ReservationStage::Reserved};
  bool placed_in_destination{false};
  rclcpp::Time created_at{std::int64_t{0}, RCL_ROS_TIME};
  Revision created_revision{0};
  Revision admitted_robot_telemetry_revision{0};
  // The destination's free rear depth when the reservation was granted, and the products its
  // evidence named then. Together they are the baseline the placement proof is measured against:
  // a placement is proven when the column has grown by one pitch *since this*, not since some
  // instant chosen after the fact. Recording them at the grant is what makes the growth a
  // difference between two stated observations rather than an inference.
  double destination_available_depth_m{0.0};
  std::vector<std::string> destination_source_ids;
  Revision revision{0};
  // The destination lane's policy at grant. Intent may change while this reservation is in
  // flight; every revalidation compares the reserved object against this captured policy rather
  // than the lane's live one, so a granted reservation completes under the policy it was granted
  // under and a policy change alone never strands the reservation.
  ProductClass destination_expected_product_class{ProductClass::Unknown};
  std::optional<std::string> destination_expected_sku;
};

struct WorldStateEvent
{
  Revision revision{0};
  rclcpp::Time event_time{std::int64_t{0}, RCL_ROS_TIME};
  EventKind kind{EventKind::ObjectObserved};
  std::optional<ObjectId> object_id;
  std::optional<LaneId> lane_id;
  std::string detail;
};

struct WorldStateSnapshot
{
  Revision revision{0};
  std::map<ObjectId, TrackedObject> objects;
  std::map<LaneId, ShelfLane> lanes;
  RobotExecutionState robot;
  std::optional<TaskReservation> active_reservation;
  std::vector<WorldStateEvent> events;
};

// Upper bound on WorldStateConfig::joint_limit_tolerance (rad, Card 060 review N2).
inline constexpr double kMaximumJointLimitTolerance = 0.01;

struct WorldStateConfig
{
  std::string planning_frame{"world"};
  rcl_clock_type_t clock_type{RCL_ROS_TIME};
  // Transport / admission skew for object and lane observation *ingest*, and the age bound for
  // object and robot evidence at reservation. Lane *selection and destination* freshness uses
  // lane_evidence_validity plus explicit invalidation instead.
  std::chrono::nanoseconds maximum_observation_age{std::chrono::seconds(2)};
  // How long a surveyed lane's available_depth_m may be acted on without a re-survey, unless an
  // event invalidates it sooner. Nominal 60 s: a moving eye cannot keep every lane inside 500 ms.
  std::chrono::nanoseconds lane_evidence_validity{std::chrono::seconds(60)};
  std::chrono::nanoseconds maximum_future_skew{std::chrono::milliseconds(100)};
  // How far a fresh observation may place the reserved product from its held pose while a task
  // reservation is at stage Reserved before the pose is replaced (ADR-0016, Milestone 10 §5).
  //
  // Inside the bound the pose is held — freshness stamp, tracking state, identity and revision
  // still advance — so a grasp staged from the reservation and the planning-scene projection of
  // the same snapshot describe one pose and confirm-frame noise cannot rewrite the obstacle
  // under a frozen approach. Beyond the bound the observation's pose is applied and becomes the
  // new held pose: a real movement is surfaced, never hidden, and the existing scene-content
  // fence, collision checks, and the attachment boundary's physical proof fail the goal closed.
  // The default 0.020 m sits above the worst confirm-frame error measured on this cell
  // (13.69 mm against ground truth, Card 009 attempt 11) plus the mean-band offset between two
  // frames of the same static product. The first live cut at 0.015 m false-tripped once on a
  // static tray product (Card 029 attempt 3: a >15 mm pin-to-frame step re-projected the
  // obstacle under the staged grasp and the approach refused closed), so the bound must clear
  // the pin-to-frame noise tail, not the single-frame error alone. Beyond it the observation's
  // pose is applied and becomes the new held pose (ADR-0016).
  double reserved_pose_divergence_bound_m{0.020};
  std::size_t event_capacity{256};
  std::size_t operation_journal_capacity{4096};
  // Keep telemetry admission aligned with the one-revolution MoveIt bounds. The former custom
  // arm limits rejected valid UR10e shoulder/elbow states, leaving otherwise healthy snapshots
  // stale during long manipulation runs.
  std::array<double, kArmJointCount> joint_lower_limits{
    -3.121593, -3.121593, -3.121593, -3.121593, -3.121593, -3.121593};
  std::array<double, kArmJointCount> joint_upper_limits{
    3.121593, 3.121593, 3.121593, 3.121593, 3.121593, 3.121593};
  // How far past those limits a measured arm joint position is still admitted (rad). A joint
  // the controller parks at its bound can read a float excess past it; the motion port already
  // absorbs up to 0.001 rad of such excess as a start-state correction, and a stricter admission
  // here froze the robot block for two minutes (Milestone 10 §6, Card 060). Equal to the port's
  // maximum_start_state_bounds_correction so both consumers of one joint state agree. The measured
  // value is stored unchanged.
  double joint_limit_tolerance{0.001};
  double rail_lower_limit{-1.7};
  double rail_upper_limit{1.7};
  std::array<double, kGripperJointCount> gripper_joint_lower_limits{0.0, 0.0};
  std::array<double, kGripperJointCount> gripper_joint_upper_limits{0.035, 0.035};
  // Depth a lane must have free at its rear for one more of each catalogued product, and the
  // depth each adds to the column. Loaded from the product catalog and the shelf survey by
  // `load_product_lane_profiles`. A world state configured without them admits no placement at
  // all: every destination predicate needs the product's own depth, and guessing one is how a
  // lane comes to be reserved for a product that does not fit it.
  std::vector<ProductLaneProfile> product_lane_profiles;
  // A measured final slot may fall this far short of the preferred rear entry clearance while a
  // complete product still fits. Selection and placement then derive a shallower release pose;
  // this tolerance must remain well below the smallest product pitch.
  double placement_entry_depth_tolerance_m{0.010};
  // How far short of a full pitch a settled column may read and still prove a placement.
  //
  // It covers one measured effect and the contact penetration between packed products, and
  // nothing else. The effect: a product's depth footprint on an inclined bed carries a term set
  // by its own height, so a *shorter* product packing behind a taller one gives back
  // `(taller - shorter) / 2 * sin(incline)` of the lane's free depth. Across the shipped
  // catalogue the worst pair is a can behind a large bottle, at 5.9 mm
  // (restocker_gazebo/test/test_lane_evidence.cpp measures it). A column of one class (which
  // is what lane policy admits) has no such term and grows by exactly one pitch.
  //
  // It stays far below the smallest catalogued pitch, 66 mm for the can, so no tolerance stack
  // lets a placement of nothing pass for a placement of something.
  double placement_growth_tolerance_m{0.010};
  // Selects the placement proof at commit and at Detached revalidation (Milestone 10 §3).
  //
  // true (default): the destination's free rear depth must have shrunk against the grant-time
  // baseline (`TaskReservation::destination_available_depth_m`) by at least one catalogued
  // `column_pitch_m` less `placement_growth_tolerance_m`, in evidence recorded strictly after
  // the release. Geometry only, so a wrist-camera lane producer — which measures depth and never
  // names identities in a lane — can prove a placement. This is the proof; naming is not
  // required alongside it.
  //
  // false: post-release evidence must name the target instead (exact source identity). The
  // simulator opt-out the dense demo keeps: near-full lanes can grow the rear gap by less than a
  // pitch on a real placement (Card 019 measured 9.7 mm and 2 mm against 67.8 mm), so the growth
  // proof would refuse there — a fail-closed availability limit, not a reason to weaken the
  // threshold. Ground-truth producers fill `observed_source_object_ids` and satisfy this mode.
  bool placement_require_column_growth{true};
};

struct ObservationReceipt
{
  ObjectId object_id;
  Revision revision{0};
  bool created{false};
};

struct MutationReceipt
{
  Revision revision{0};
};

struct ReserveTaskRequest
{
  std::string request_id;
  Revision selected_snapshot_revision{0};
  ObjectId object_id;
  Revision object_revision{0};
  std::optional<LaneId> source_lane;
  std::optional<Revision> source_lane_revision;
  LaneId destination_lane;
  Revision destination_lane_revision{0};
};

struct ReservationReceipt
{
  Revision revision{0};
  std::string token;
  TaskReservation reservation;
};

// Historical success under clock inhibition is evidence, never a successful current result.
// Keeping it outside value() prevents a generic success branch from authorizing continuation.
class [[nodiscard]] ReservedAttachmentResult
{
public:
  [[nodiscard]] static ReservedAttachmentResult success(ReservationReceipt receipt)
  {
    return ReservedAttachmentResult(Result<ReservationReceipt>::success(std::move(receipt)));
  }

  [[nodiscard]] static ReservedAttachmentResult failure(WorldStateError error)
  {
    return ReservedAttachmentResult(Result<ReservationReceipt>::failure(std::move(error)));
  }

  [[nodiscard]] static ReservedAttachmentResult inhibited_history(ReservationReceipt receipt)
  {
    auto result = failure(
      {WorldStateErrorCode::ClockAuthorityInhibited,
        "historical attachment committed; current clock authority is inhibited"});
    result.historical_receipt_.emplace(std::move(receipt));
    return result;
  }

  [[nodiscard]] bool has_value() const noexcept {return result_.has_value();}
  [[nodiscard]] explicit operator bool() const noexcept {return has_value();}
  [[nodiscard]] const ReservationReceipt & value() const {return result_.value();}
  [[nodiscard]] const WorldStateError & error() const {return result_.error();}
  [[nodiscard]] const std::optional<ReservationReceipt> & historical_receipt() const noexcept
  {
    return historical_receipt_;
  }

private:
  explicit ReservedAttachmentResult(Result<ReservationReceipt> result)
  : result_(std::move(result)) {}

  Result<ReservationReceipt> result_;
  std::optional<ReservationReceipt> historical_receipt_;
};

struct ReservationValidation
{
  Revision revision{0};
  TaskReservation reservation;
};

// Capability-authorized execution evidence copied under one store lock. The token is deliberately
// excluded so this proof can cross validation layers without duplicating the reservation secret.
struct ExecutionWorldAuthorityProof
{
  Revision revision{0};
  std::string planning_frame;
  TaskReservation reservation;
  TrackedObject object;
  ShelfLane destination_lane;
  std::optional<ShelfLane> source_lane;
  RobotExecutionState robot;
};

struct ExecutionWorldAuthorityExpectation
{
  std::uint64_t reservation_id{0U};
  Revision reservation_revision{0U};
  ObjectId object_id;
  LaneId destination_lane;
};

struct ReservedTaskCheckpoint
{
  std::string token;
  std::string operation_id;
  std::uint64_t expected_reservation_id{0};
  ReservationStage expected_reservation_stage{ReservationStage::Reserved};
  Revision expected_reservation_revision{0};
  TaskPhase task_phase{TaskPhase::Idle};
  FaultState fault_state{FaultState::None};
  rclcpp::Time update_time{std::int64_t{0}, RCL_ROS_TIME};
};

struct ReservedAttachmentRequest
{
  std::string token;
  std::string operation_id;
  // The grasp the attachment boundary verified, in the gripper's `grasp_center` frame. Stored on
  // the robot state so a held product's pose stays derivable from kinematics alone.
  Eigen::Isometry3d grasp_center_from_held_object{Eigen::Isometry3d::Identity()};
};

struct ReservedDetachmentRequest
{
  std::string token;
  std::string operation_id;
  DetachmentDisposition disposition{DetachmentDisposition::PlaceInReservedDestination};
  // When the physical boundary verified that the gripper had let go. A placement claim is only
  // accepted on destination evidence recorded after this instant, because a gripped product
  // already occupies the destination lane volume during the pre-insert. Ignored by a release
  // that claims no membership.
  rclcpp::Time released_at{std::int64_t{0}, RCL_ROS_TIME};
};

struct ReleaseReservationRequest
{
  std::string token;
  std::string operation_id;
  std::uint64_t expected_reservation_id{0};
  ReservationStage expected_reservation_stage{ReservationStage::Reserved};
  Revision expected_reservation_revision{0};
  ReservationOutcome outcome{ReservationOutcome::Succeeded};
  TaskPhase terminal_task_phase{TaskPhase::Idle};
  FaultState terminal_fault_state{FaultState::None};
};

// Durably writes the full desired-stocking table as the store's set_lane_policy mutation commits.
// Invoked under the store's write lock after the revision fence passes and before anything in
// memory changes: returning false refuses the mutation, leaving both the lane and the policy
// file untouched. The writer itself must be all-or-nothing (atomic replace), so a failed write
// never leaves a half-updated document.
using LanePolicyPersist = std::function<bool (const std::vector<LanePolicy> &)>;

struct ObjectContextPreconditions
{
  Revision object_revision{0};
  std::optional<Revision> containing_lane_revision;
};

struct AttachPreconditions
{
  Revision object_revision{0};
  Revision robot_revision{0};
  std::optional<Revision> source_lane_revision;
};

struct DetachPreconditions
{
  Revision object_revision{0};
  Revision robot_revision{0};
  std::optional<Revision> destination_lane_revision;
};

// Read-only description of one journal's retention bound. It carries counts and identifiers only:
// never an operation ID, token or payload. `open_obligations` counts journal RECORDS only, and
// `open_obligations + terminal_receipts == size`; `reserved_credits` (entries owed to the active
// reservation's remaining lifecycle) are not part of `size` and are the explicit signal that
// something is still owed. An open record is an unsettled side effect, a reserved credit's owner,
// or an unknown outcome; a terminal receipt is a settled record kept only so an exact retry
// replays. `inhibited` is the single store-wide clock-authority latch, identical for both
// journals. `evicting` is true only for the FIFO audit journal; operation journals never evict,
// they refuse admission when full. `epoch_id` names the incarnation that owns the journal and is
// diagnostic, not a wire fence.
struct JournalRetentionSnapshot
{
  std::string journal;
  std::string epoch_id;
  std::size_t capacity{0};
  std::size_t size{0};
  std::size_t open_obligations{0};
  std::size_t terminal_receipts{0};
  std::size_t reserved_credits{0};
  bool inhibited{false};
  bool evicting{false};
};

class WorldStateStore
{
public:
  explicit WorldStateStore(WorldStateConfig config = {});
  ~WorldStateStore();

  WorldStateStore(const WorldStateStore &) = delete;
  WorldStateStore & operator=(const WorldStateStore &) = delete;
  WorldStateStore(WorldStateStore &&) = delete;
  WorldStateStore & operator=(WorldStateStore &&) = delete;

  // Caller holds the managed clock update mutex across sampling and the ensuing store call.
  [[nodiscard]] AuthorityClockSample capture_authority_clock(
    const rclcpp::Time & sampled_at, bool managed_ros_started);
  // The managed pre-jump callback already holds its clock update mutex.
  void notify_clock_discontinuity() noexcept;

  [[nodiscard]] Result<ObservationReceipt> observe_object(
    const ObjectObservation & observation,
    const rclcpp::Time & received_at,
    const std::optional<AuthorityClockSample> & clock_sample = std::nullopt);

  [[nodiscard]] Result<MutationReceipt> set_tracking_state(
    ObjectId object_id, TrackingState state, const rclcpp::Time & transition_time,
    const ObjectContextPreconditions & preconditions);

  [[nodiscard]] Result<MutationReceipt> configure_lane(
    const LaneDefinition & definition,
    const rclcpp::Time & configured_at);

  [[nodiscard]] Result<MutationReceipt> update_lane(
    const LaneObservation & observation,
    const rclcpp::Time & received_at,
    Revision expected_lane_revision,
    const std::optional<AuthorityClockSample> & clock_sample = std::nullopt);

  // Marks a lane's depletion evidence unusable until the next accepted update_lane. Used when a
  // placement (or later a policy change) means the last survey no longer describes the lane.
  [[nodiscard]] Result<MutationReceipt> invalidate_lane_evidence(
    LaneId lane_id,
    const rclcpp::Time & invalidated_at,
    Revision expected_lane_revision);

  // Replaces one lane's desired policy under a caller-captured lane revision, writing the full
  // table through `persist` before anything in memory changes. A stale expected_lane_revision,
  // an invalid policy, or a failed persist each refuse without mutating the lane; an unchanged
  // policy succeeds without bumping the revision or rewriting the table. A change bumps the lane
  // revision, invalidates that lane's evidence until re-surveyed, and appends one event. The
  // persist callback is required: a policy change that cannot be made durable is not made.
  [[nodiscard]] Result<MutationReceipt> set_lane_policy(
    const LanePolicy & policy,
    Revision expected_lane_revision,
    const rclcpp::Time & changed_at,
    const LanePolicyPersist & persist);

  [[nodiscard]] Result<MutationReceipt> observe_robot_telemetry(
    const RobotTelemetryObservation & observation, const rclcpp::Time & received_at,
    const std::optional<AuthorityClockSample> & clock_sample = std::nullopt);

  [[nodiscard]] Result<MutationReceipt> commit_attachment(
    ObjectId object_id,
    const Eigen::Isometry3d & grasp_center_from_object,
    const rclcpp::Time & committed_at,
    const AttachPreconditions & preconditions);

  [[nodiscard]] Result<MutationReceipt> commit_detachment(
    ObjectId object_id,
    std::optional<LaneId> destination_lane,
    const rclcpp::Time & committed_at,
    const DetachPreconditions & preconditions);

  [[nodiscard]] Result<ReservationReceipt> reserve_task(
    const ReserveTaskRequest & request,
    const rclcpp::Time & reserved_at,
    const std::optional<AuthorityClockSample> & clock_sample = std::nullopt);

  [[nodiscard]] Result<ReservationValidation> validate_task_reservation(
    const std::string & token) const;

  [[nodiscard]] Result<ExecutionWorldAuthorityProof> validate_execution_world_authority(
    const std::string & token,
    const ExecutionWorldAuthorityExpectation & expected) const;

  [[nodiscard]] Result<ReservationReceipt> checkpoint_reserved_task(
    const ReservedTaskCheckpoint & checkpoint);

  [[nodiscard]] ReservedAttachmentResult commit_reserved_attachment(
    const ReservedAttachmentRequest & request, const rclcpp::Time & committed_at,
    const std::optional<AuthorityClockSample> & clock_sample = std::nullopt);

  [[nodiscard]] Result<ReservationReceipt> commit_reserved_detachment(
    const ReservedDetachmentRequest & request, const rclcpp::Time & committed_at);

  [[nodiscard]] Result<MutationReceipt> release_task_reservation(
    const ReleaseReservationRequest & request, const rclcpp::Time & released_at);

  [[nodiscard]] WorldStateSnapshot snapshot() const;

  // Operation journal ("world_state.operations") and event deque ("world_state.events"), in
  // that order. Observability only: no admission, eviction or replay decision reads it.
  [[nodiscard]] std::vector<JournalRetentionSnapshot> retention_snapshot() const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

[[nodiscard]] std::string to_string(WorldStateErrorCode code);

// Whether a lane policy admits an object: class wildcard-free equality (Unknown accepts any
// class, as a deliberately unassigned lane does) plus exact SKU when the policy names one.
// Shared by the store's reserve-time live check and by every revalidation that compares against
// a policy captured at grant.
[[nodiscard]] bool policy_accepts_object(
  ProductClass expected_product_class,
  const std::optional<std::string> & expected_sku,
  const TrackedObject & object);

}  // namespace restocker_world_state
