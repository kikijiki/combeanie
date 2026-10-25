// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <chrono>
#include <deque>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <memory>
#include <optional>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/common/Event.hh>
#include <gz/physics/ContactProperties.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/physics/Events.hh>
#include <gz/sim/components/ContactSensorData.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/DetachableJoint.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Static.hh>
#include <gz/transport/Node.hh>

#include "restocker_gazebo/attachment_config.hpp"
#include "restocker_gazebo/attachment_mailbox.hpp"
#include "restocker_gazebo/attachment_physics.hpp"
#include "restocker_gazebo/attachment_verifier.hpp"

namespace restocker_gazebo
{

namespace
{

constexpr char kOwnedJointName[] = "restocker_attachment_joint";

[[nodiscard]] std::optional<std::filesystem::path> environment_path(const char * name)
{
  const char * value = std::getenv(name);
  if (value == nullptr || *value == '\0') {
    return std::nullopt;
  }
  return std::filesystem::path(value);
}

[[nodiscard]] std::string random_epoch()
{
  std::random_device source;
  std::ostringstream stream;
  stream << std::hex << std::setfill('0');
  for (std::size_t index = 0; index < 4; ++index) {
    stream << std::setw(8) << static_cast<std::uint32_t>(source());
  }
  return stream.str();
}

[[nodiscard]] std::string next_random_epoch(std::string_view previous = {})
{
  for (std::size_t attempt = 0; attempt < 8; ++attempt) {
    std::string candidate = random_epoch();
    if (candidate != previous) {
      return candidate;
    }
  }
  throw std::runtime_error("failed to generate a distinct simulator epoch");
}

[[nodiscard]] std::int64_t simulation_time_ns(const gz::sim::UpdateInfo & info)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
}

[[nodiscard]] Eigen::Isometry3d eigen_pose(const gz::math::Pose3d & pose)
{
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = Eigen::Vector3d(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z());
  result.linear() =
    Eigen::Quaterniond(pose.Rot().W(), pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z())
    .toRotationMatrix();
  return result;
}

struct ResolvedEntities
{
  gz::sim::Entity robot_model{gz::sim::kNullEntity};
  gz::sim::Entity parent_link{gz::sim::kNullEntity};
  gz::sim::Entity left_finger_joint{gz::sim::kNullEntity};
  gz::sim::Entity right_finger_joint{gz::sim::kNullEntity};
  gz::sim::Entity child_model{gz::sim::kNullEntity};
  gz::sim::Entity child_link{gz::sim::kNullEntity};
};

struct ResolutionResult
{
  AttachmentStatus status;
  std::optional<ResolvedEntities> entities;
};

struct ActiveMutation
{
  DecodedAttachmentCommand command;
  ResolvedEntities entities;
  gz::sim::Entity joint_entity{gz::sim::kNullEntity};
  Eigen::Isometry3d applied_parent_from_child{Eigen::Isometry3d::Identity()};
};

// The product can keep moving for a few ticks after the gripper reaches its joint target. Keep the
// authorized command pending until the validate_physical_attach velocity predicate holds on
// consecutive ticks; the fixed joint is not created before then.
struct PendingAttachValidation
{
  DecodedAttachmentCommand command;
  ResolvedEntities entities;
  std::uint64_t first_iteration{0U};
  std::size_t consecutive_stable_ticks{0U};
  // Worst tick seen while waiting for the velocity gate: the refusal names the peak, not just the
  // final sample, because a settled pose can hide intermittent contact jitter.
  double peak_relative_linear_mps{0.0};
  double peak_relative_angular_radps{0.0};
  // Independent check on the reported velocity streams: pose finite differences over the
  // pending window. If WorldLinearVelocity and the pose disagree, one of them is wrong and the
  // refusal must say so rather than gate on a phantom.
  std::optional<Eigen::Isometry3d> first_parent_world;
  std::optional<Eigen::Isometry3d> first_child_world;
  std::int64_t first_time_ns{0};
  std::optional<Eigen::Isometry3d> previous_parent_world;
  std::optional<Eigen::Isometry3d> previous_child_world;
  std::int64_t previous_time_ns{0};
  double peak_parent_fd_mps{0.0};
  double peak_child_fd_mps{0.0};
};

// Where one pad touched the product when the physical hold was verified (spec M10 section 11 D5
// amendment, as corrected), and the pad's slip accumulated since.
struct HeldPadReference
{
  gz::sim::Entity finger_link{gz::sim::kNullEntity};
  Eigen::Vector3d contact_in_finger{Eigen::Vector3d::Zero()};
  Eigen::Vector3d contact_in_child{Eigen::Vector3d::Zero()};
  PadSlipAccumulator slip;
};

// How many ticks the watchdog waits for both pads to report a contact before it refuses the hold.
constexpr int kPadCaptureRetryTicks = 200;

// A pad can only touch a product whose centre is within this distance of the gripper link: the
// fingers reach 0.16 m past the link (gripper_geometry.yaml) and the tallest catalogued product
// has a half-diagonal of about 0.12 m, so this is their sum with a margin.
constexpr double kPadReachM = 0.3;

// One tick of the held-grip flight recorder: the last kRecorderTicks of these are printed when the
// watchdog latches, so a lost grip arrives with the 3 s that led to it (what the hand was doing,
// how hard each pad pressed, how the product moved against it).
struct HeldSample
{
  double time_s{0.0};
  Eigen::Vector3d drift{Eigen::Vector3d::Zero()};
  Eigen::Vector3d hand_linear{Eigen::Vector3d::Zero()};
  Eigen::Vector3d relative_angular{Eigen::Vector3d::Zero()};
  double left_force_n{0.0};
  double right_force_n{0.0};
  double aperture_m{0.0};
  double swing_m{0.0};
  double tilt_rad{0.0};
};
constexpr std::size_t kRecorderTicks = 3000;
constexpr std::size_t kRecorderPrintStride = 20;

// The robot's entities the per-tick passes touch and the pad collisions of its two fingers.
struct CachedRobot
{
  gz::sim::Entity model{gz::sim::kNullEntity};
  gz::sim::Entity parent{gz::sim::kNullEntity};
  gz::sim::Entity left{gz::sim::kNullEntity};
  gz::sim::Entity right{gz::sim::kNullEntity};
  std::vector<gz::sim::Entity> pad_collisions;
  bool observed{false};
};

// One catalogued product's entities and the collisions the contact-surface map holds for it.
struct CachedProduct
{
  gz::sim::Entity model{gz::sim::kNullEntity};
  gz::sim::Entity link{gz::sim::kNullEntity};
  std::vector<gz::sim::Entity> collisions;
  bool contact_mapped{false};
  bool observed{false};
};

struct HeldAttachment
{
  AttachmentIdentity identity;
  ResolvedEntities entities;
  gz::sim::Entity joint_entity{gz::sim::kNullEntity};
  Eigen::Isometry3d applied_parent_from_child{Eigen::Isometry3d::Identity()};
  // Physical hold only: both pads, left then right. Absent if a pad had no contact at verification.
  std::optional<std::array<HeldPadReference, 2>> pads;
  // Worst case over the whole carry, so watchdog margin is measurable from passing runs.
  double peak_translation_drift_m{0.0};
  double peak_relative_linear_mps{0.0};
  double peak_relative_angular_radps{0.0};
  double peak_pad_slip_m{0.0};
  // D5 amendment point 5: q_left + q_right when the grip was verified, and whether the jaws have
  // since parted. A release is not a slip, and a slip is not a release: only the jaws moving
  // apart disarms the watchdog.
  double aperture_reference{0.0};
  // Mean pad force (N) once the grip has settled, so a release can be told from a product that
  // merely pushes the jaws apart while both pads stay loaded. Zero until first measured.
  double pad_force_reference_n{0.0};
  double peak_swing_m{0.0};
  double peak_tilt_rad{0.0};
  bool release_started{false};
  // Ticks spent waiting for a contact report from both pads after the hold verified.
  int pad_capture_retries{0};
  std::deque<HeldSample> recorder;
};

[[nodiscard]] AttachmentStatus status(AttachmentStatusCode code, std::string detail)
{
  return AttachmentStatus{code, std::move(detail)};
}

template<typename Marker>
[[nodiscard]] std::optional<gz::sim::Entity> unique_child(
  gz::sim::Entity parent, const std::string & name, const gz::sim::EntityComponentManager & ecm,
  AttachmentStatus & error)
{
  const auto matches = ecm.ChildrenByComponents(parent, gz::sim::components::Name(name), Marker());
  if (matches.empty()) {
    error = status(
      AttachmentStatusCode::kObjectNotFound,
      "configured entity is not available: " + name);
    return std::nullopt;
  }
  if (matches.size() != 1) {
    error =
      status(AttachmentStatusCode::kEntityAmbiguous, "configured entity is ambiguous: " + name);
    return std::nullopt;
  }
  return matches.front();
}

[[nodiscard]] bool named_entity_matches(
  gz::sim::Entity entity, gz::sim::Entity parent,
  const std::string & expected_name,
  const gz::sim::EntityComponentManager & ecm)
{
  if (!ecm.HasEntity(entity) || ecm.ParentEntity(entity) != parent) {
    return false;
  }
  const auto name = ecm.ComponentData<gz::sim::components::Name>(entity);
  return name && *name == expected_name;
}

[[nodiscard]] std::size_t child_constraint_count(
  gz::sim::Entity child,
  const gz::sim::EntityComponentManager & ecm)
{
  std::size_t result = 0;
  ecm.Each<gz::sim::components::DetachableJoint>(
    [&result, child](const gz::sim::Entity &, const gz::sim::components::DetachableJoint * joint) {
      if (joint->Data().childLink == child) {
        ++result;
      }
      return true;
    });
  return result;
}

[[nodiscard]] bool exact_joint(
  gz::sim::Entity joint_entity, gz::sim::Entity parent,
  gz::sim::Entity child, const gz::sim::EntityComponentManager & ecm)
{
  if (joint_entity == gz::sim::kNullEntity || !ecm.HasEntity(joint_entity)) {
    return false;
  }
  const auto name = ecm.ComponentData<gz::sim::components::Name>(joint_entity);
  const auto * joint = ecm.Component<gz::sim::components::DetachableJoint>(joint_entity);
  return name && *name == kOwnedJointName && joint && joint->Data().parentLink == parent &&
         joint->Data().childLink == child && joint->Data().jointType == "fixed";
}

[[nodiscard]] std::vector<gz::sim::Entity> exact_owned_joints(
  gz::sim::Entity parent, gz::sim::Entity child, const gz::sim::EntityComponentManager & ecm)
{
  std::vector<gz::sim::Entity> result;
  ecm.Each<gz::sim::components::Name, gz::sim::components::DetachableJoint>(
    [&result, parent, child](const gz::sim::Entity & entity, const gz::sim::components::Name * name,
    const gz::sim::components::DetachableJoint * joint) {
      if (name->Data() == kOwnedJointName && joint->Data().parentLink == parent &&
      joint->Data().childLink == child && joint->Data().jointType == "fixed")
      {
        result.push_back(entity);
      }
      return true;
    });
  return result;
}

[[nodiscard]] std::size_t project_owned_joint_marker_count(
  const gz::sim::EntityComponentManager & ecm)
{
  std::size_t result = 0;
  ecm.Each<gz::sim::components::Name>(
    [&result](const gz::sim::Entity &, const gz::sim::components::Name * name) {
      if (name->Data() == kOwnedJointName) {
        ++result;
      }
      return true;
    });
  return result;
}

// A finger's link, found from its joint: the entity the pad contacts belong to.
[[nodiscard]] gz::sim::Entity finger_link(
  gz::sim::Entity robot_model, gz::sim::Entity finger_joint,
  const gz::sim::EntityComponentManager & ecm)
{
  const auto name = gz::sim::Joint(finger_joint).ChildLinkName(ecm);
  if (!name) {
    return gz::sim::kNullEntity;
  }
  return gz::sim::Model(robot_model).LinkByName(ecm, *name);
}

// The pad is the finger link's own collisions plus its pad-row links (spec M10 section 11 D10
// item 3); the older single-box fixtures put every pad contact on the finger link itself, and
// both count. Note the rows are found by name through the *model*: in the ECM a link's parent
// entity is its model even when a joint hangs it off another link, so ChildrenByComponents on the
// finger link finds nothing.
[[nodiscard]] std::vector<gz::sim::Entity> pad_collisions_of(
  gz::sim::Entity robot_model, gz::sim::Entity finger,
  const gz::sim::EntityComponentManager & ecm)
{
  std::vector<gz::sim::Entity> result;
  if (finger == gz::sim::kNullEntity) {
    return result;
  }
  for (const auto collision : gz::sim::Link(finger).Collisions(ecm)) {
    result.push_back(collision);
  }
  const auto finger_name = ecm.ComponentData<gz::sim::components::Name>(finger);
  if (!finger_name) {
    return result;
  }
  const std::string pad_prefix = *finger_name + "_pad_row_";
  for (const auto & link : gz::sim::Model(robot_model).Links(ecm)) {
    const auto name = ecm.ComponentData<gz::sim::components::Name>(link);
    if (!name || name->rfind(pad_prefix, 0) != 0) {
      continue;
    }
    for (const auto collision : gz::sim::Link(link).Collisions(ecm)) {
      result.push_back(collision);
    }
  }
  return result;
}

// The world point where one finger holds the product: the force-weighted centroid of the contact
// points between the finger's collisions and the product link (the plain centroid if Physics
// reported no force). Physics fills these only for collisions carrying ContactSensorData, which
// enable_robot_observability creates.
[[nodiscard]] std::optional<Eigen::Vector3d> finger_contact_in_world(
  gz::sim::Entity robot_model, gz::sim::Entity finger, gz::sim::Entity product_link,
  const gz::sim::EntityComponentManager & ecm)
{
  Eigen::Vector3d weighted = Eigen::Vector3d::Zero();
  Eigen::Vector3d plain = Eigen::Vector3d::Zero();
  double total_force = 0.0;
  std::size_t points = 0U;
  for (const auto collision : pad_collisions_of(robot_model, finger, ecm)) {
    const auto * contacts = ecm.Component<gz::sim::components::ContactSensorData>(collision);
    if (!contacts) {
      continue;
    }
    for (const auto & contact : contacts->Data().contact()) {
      const bool first = contact.collision1().id() == collision;
      const gz::sim::Entity other = first ? contact.collision2().id() : contact.collision1().id();
      if (ecm.ParentEntity(other) != product_link) {
        continue;
      }
      for (int index = 0; index < contact.position_size(); ++index) {
        const Eigen::Vector3d point(
          contact.position(index).x(), contact.position(index).y(), contact.position(index).z());
        plain += point;
        ++points;
        if (index < contact.wrench_size()) {
          const auto & force = first ? contact.wrench(index).body_1_wrench().force() :
            contact.wrench(index).body_2_wrench().force();
          const double magnitude = Eigen::Vector3d(force.x(), force.y(), force.z()).norm();
          weighted += magnitude * point;
          total_force += magnitude;
        }
      }
    }
  }
  if (points == 0U) {
    return std::nullopt;
  }
  return total_force > 1.0e-9 ? Eigen::Vector3d(weighted / total_force) :
         Eigen::Vector3d(plain / static_cast<double>(points));
}


// Total contact force magnitude (N) on one finger's pads from the product link.
[[nodiscard]] double pad_force(
  gz::sim::Entity robot_model, gz::sim::Entity finger, gz::sim::Entity product_link,
  const gz::sim::EntityComponentManager & ecm)
{
  double total = 0.0;
  for (const auto collision : pad_collisions_of(robot_model, finger, ecm)) {
    const auto * contacts = ecm.Component<gz::sim::components::ContactSensorData>(collision);
    if (!contacts) {
      continue;
    }
    for (const auto & contact : contacts->Data().contact()) {
      const bool first = contact.collision1().id() == collision;
      const gz::sim::Entity other = first ? contact.collision2().id() : contact.collision1().id();
      if (ecm.ParentEntity(other) != product_link) {
        continue;
      }
      for (int index = 0; index < contact.wrench_size(); ++index) {
        const auto & force = first ? contact.wrench(index).body_1_wrench().force() :
          contact.wrench(index).body_2_wrench().force();
        total += Eigen::Vector3d(force.x(), force.y(), force.z()).norm();
      }
    }
  }
  return total;
}

// The distance between the two pad faces is zero_gap + q_left + q_right, so the change of that
// sum is the change of the aperture (spec M10 section 11 D5 amendment point 5). Reading it from
// the two joint positions keeps it invariant under a common-mode shift of both fingers along the
// jaw axis, which a slip can produce and a release cannot.
[[nodiscard]] std::optional<double> finger_aperture(
  const ResolvedEntities & entities, const gz::sim::EntityComponentManager & ecm)
{
  const auto left = gz::sim::Joint(entities.left_finger_joint).Position(ecm);
  const auto right = gz::sim::Joint(entities.right_finger_joint).Position(ecm);
  if (!left || left->empty() || !right || right->empty()) {
    return std::nullopt;
  }
  return left->front() + right->front();
}

// The last world pose the plugin sampled for a link, and the twist measured from the sample
// before it. The gates read this instead of the engine's velocity components (see
// finite_difference_world_twist for why those cannot be trusted for a link at rest).
struct LinkMotionSample
{
  Eigen::Isometry3d world_pose{Eigen::Isometry3d::Identity()};
  std::int64_t time_ns{0};
  std::optional<WorldTwist> twist;
  std::uint64_t generation{0};
};

using LinkMotion = std::unordered_map<gz::sim::Entity, LinkMotionSample>;

[[nodiscard]] std::optional<WorldTwist> sampled_twist(
  const LinkMotion & motion, gz::sim::Entity link)
{
  const auto found = motion.find(link);
  if (found == motion.end()) {
    return std::nullopt;
  }
  return found->second.twist;
}

[[nodiscard]] std::optional<AttachmentVerificationObservation> physical_observation(
  const ResolvedEntities & entities, gz::sim::Entity joint_entity, const gz::sim::UpdateInfo & info,
  const LinkMotion & motion, const gz::sim::EntityComponentManager & ecm)
{
  const gz::sim::Link parent(entities.parent_link);
  const gz::sim::Link child(entities.child_link);
  const auto world_from_parent_gz = parent.WorldPose(ecm);
  const auto world_from_child_gz = child.WorldPose(ecm);
  const auto parent_twist = sampled_twist(motion, entities.parent_link);
  const auto child_twist = sampled_twist(motion, entities.child_link);
  if (!world_from_parent_gz || !world_from_child_gz || !parent_twist || !child_twist) {
    return std::nullopt;
  }

  const Eigen::Isometry3d world_from_parent = eigen_pose(*world_from_parent_gz);
  const Eigen::Isometry3d world_from_child = eigen_pose(*world_from_child_gz);
  const Eigen::Vector3d offset_world =
    world_from_child.translation() - world_from_parent.translation();
  const Eigen::Vector3d & parent_linear = parent_twist->linear;
  const Eigen::Vector3d & child_linear = child_twist->linear;
  const Eigen::Vector3d & parent_angular = parent_twist->angular;
  const Eigen::Vector3d & child_angular = child_twist->angular;
  const Eigen::Matrix3d parent_from_world = world_from_parent.linear().transpose();

  AttachmentVerificationObservation result;
  result.exact_joint_observed =
    exact_joint(joint_entity, entities.parent_link, entities.child_link, ecm);
  result.entity_identity_matches =
    ecm.HasEntity(entities.robot_model) && ecm.HasEntity(entities.parent_link) &&
    ecm.HasEntity(entities.child_model) && ecm.HasEntity(entities.child_link);
  result.parent_from_child = world_from_parent.inverse() * world_from_child;
  result.world_from_child = world_from_child;
  result.relative_linear_velocity_in_parent =
    parent_from_world * (child_linear - parent_linear - parent_angular.cross(offset_world));
  result.relative_angular_velocity_in_parent = parent_from_world * (child_angular - parent_angular);
  result.simulator_iteration = info.iterations;
  result.simulation_time_ns = simulation_time_ns(info);
  return result;
}

}  // namespace

class AttachmentSystem final : public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemReset,
  public gz::sim::ISystemPreUpdate,
  public gz::sim::ISystemPostUpdate
{
public:
  void Configure(
    const gz::sim::Entity & entity, const std::shared_ptr<const sdf::Element> &,
    gz::sim::EntityComponentManager &, gz::sim::EventManager & event_manager) override
  {
    world_entity_ = entity;
    // Spec M10 section 11 D10 item 2: pair-specific pad-to-product friction. Physics only emits
    // CollectContactSurfaceProperties for collisions carrying
    // EnableContactSurfaceCustomization, and only the dartsim plugin implements the callback the
    // event sits on, so this is the one place a contact pair's mu can be set to a value the two
    // bodies' own coefficients (combined with min, section 11.2 F5) would not produce.
    using CollectContactSurface = gz::sim::events::CollectContactSurfaceProperties;
    contact_surface_connection_ = event_manager.Connect<CollectContactSurface>(
      [this](
        const gz::sim::Entity & collision1, const gz::sim::Entity & collision2,
        const gz::math::Vector3d &, const std::optional<gz::math::Vector3d> &,
        const std::optional<gz::math::Vector3d> &, const std::optional<double> &, const std::size_t,
        gz::physics::SetContactPropertiesCallbackFeature::ContactSurfaceParams<
          gz::physics::FeaturePolicy3d> & params)
      {
        this->apply_pad_friction(collision1, collision2, params);
      });
    try {
      const auto boundary = environment_path("RESTOCKER_ATTACHMENT_BOUNDARY_CONFIG");
      const auto gripper = environment_path("RESTOCKER_GRIPPER_GEOMETRY_CONFIG");
      const auto catalog = environment_path("RESTOCKER_PRODUCT_CATALOG_CONFIG");
      const auto scenario = environment_path("RESTOCKER_SCENARIO_CONFIG");
      if (!boundary || !gripper || !catalog || !scenario) {
        throw std::runtime_error("the four RESTOCKER_* attachment config paths are required");
      }
      auto loaded = load_attachment_boundary_config(*boundary, *gripper, *catalog, *scenario);
      if (!loaded) {
        throw std::runtime_error(loaded.error().detail);
      }
      config_ = std::move(loaded.value());
      mailbox_ = std::make_unique<AttachmentMailbox>(
        next_random_epoch(), config_->journal_capacity,
        std::nullopt, config_->detach_attempt_reserve,
        std::string(hold_mode_name(config_->hold_mode)));
      verifier_ = std::make_unique<AttachmentMutationVerifier>(
        verification_config(), config_->hold_mode);
      // Receipt (spec 11.3 D1): every process states which physics it is running, so a log can
      // never be read as evidence for the other mode.
      const char * hold_detail =
        config_->hold_mode == AttachmentHoldMode::kPhysical ?
        " (finger contact and friction hold the product; no joint is created)" :
        " (the restocker_attachment_joint weld holds the product; non-physical and excluded "
        "from evidence)";
      gzmsg << "AttachmentSystem hold_mode=" << hold_mode_name(config_->hold_mode)
            << hold_detail << '\n';
      // Receipt: which coefficient each product class grips at, and that nothing else changes.
      {
        std::ostringstream friction;
        for (const auto & [product_class, mu] : config_->pad_friction_by_product_class) {
          friction << (friction.tellp() > 0 ? ", " : "") << product_class << "=" << mu;
        }
        gzmsg << "AttachmentSystem pad-to-product friction " << friction.str()
              << " (pad contacts only; product-to-shelf, product-to-tray and product-to-product "
          "keep their catalogued values)\n";
      }
    } catch (const std::exception & error) {
      gzerr << "AttachmentSystem configuration rejected: " << error.what() << '\n';
      config_.reset();
      mailbox_.reset();
      verifier_.reset();
    }
  }

  void Reset(const gz::sim::UpdateInfo & info, gz::sim::EntityComponentManager & ecm) override
  {
    rotate_epoch(ecm);
    last_iteration_ = info.iterations;
    last_simulation_time_ns_ = simulation_time_ns(info);
  }

  void PreUpdate(const gz::sim::UpdateInfo & info, gz::sim::EntityComponentManager & ecm) override
  {
    if (!ready()) {
      return;
    }
    if (!services_advertised_ && !reconcile_startup_and_advertise(ecm)) {
      return;
    }
    const std::int64_t time_ns = simulation_time_ns(info);
    if ((last_iteration_ && info.iterations < *last_iteration_) ||
      (last_simulation_time_ns_ && time_ns < *last_simulation_time_ns_))
    {
      rotate_epoch(ecm);
    }
    last_iteration_ = info.iterations;
    last_simulation_time_ns_ = time_ns;
    enable_robot_observability(ecm);
    refresh_contact_surface_pairs(ecm);
    if (info.paused || active_) {
      return;
    }
    if (pending_attach_) {
      continue_pending_attach(info, ecm);
      return;
    }
    auto command = mailbox_->take_pending();
    if (!command) {
      return;
    }
    apply_command(*command, info, ecm);
  }

  void PostUpdate(
    const gz::sim::UpdateInfo & info,
    const gz::sim::EntityComponentManager & ecm) override
  {
    if (!ready() || info.paused) {
      return;
    }
    sample_link_motion(info, ecm);
    if (active_ && verifier_->active()) {
      auto observation = physical_observation(
        active_->entities, active_->joint_entity, info,
        link_motion_, ecm);
      if (!observation) {
        fail_active("post-physics attachment evidence is unavailable");
        return;
      }
      observation->entity_identity_matches =
        entities_match(active_->entities, active_->command.request.identity, ecm);
      const auto verified = verifier_->observe(*observation);
      if (verified.disposition == AttachmentVerificationDisposition::kSucceeded) {
        if (!verified.evidence) {
          fail_active("successful verification omitted terminal physics evidence");
          return;
        }
        const auto reply = mailbox_->mark_verification_succeeded(
          active_->command.operation_id,
          *verified.evidence);
        if (reply.status.code == AttachmentStatusCode::kAttached) {
          held_ = HeldAttachment{active_->command.request.identity, active_->entities,
            active_->joint_entity, active_->applied_parent_from_child, std::nullopt};
          if (config_->hold_mode == AttachmentHoldMode::kPhysical) {
            held_->pads = capture_pads(held_->entities, ecm);
            if (!held_->pads) {
              gzwarn << "AttachmentSystem physical hold verified with a pad out of contact; the "
                "held-object watchdog will refuse it\n";
            }
            const auto aperture = finger_aperture(held_->entities, ecm);
            if (aperture) {
              held_->aperture_reference = *aperture;
            }
          }
        } else if (reply.status.code == AttachmentStatusCode::kDetached) {
          if (held_) {
            gzmsg << "AttachmentSystem carry ended within the watchdog limits (peak drift "
                  << held_->peak_translation_drift_m << " of "
                  << translation_bound_m() << " m, peak relative linear "
                  << held_->peak_relative_linear_mps << " of "
                  << config_->tolerances.relative_linear_velocity_mps
                  << " m/s, peak relative angular " << held_->peak_relative_angular_radps
                  << " of " << config_->tolerances.relative_angular_velocity_radps
                  << " rad/s, net pad excursion "
                  << (held_->pads ? (*held_->pads)[0].slip.peak_excursion_m : 0.0) << " m left, "
                  << (held_->pads ? (*held_->pads)[1].slip.peak_excursion_m : 0.0)
                  << " m right, cumulative pad slip " << held_->peak_pad_slip_m << " of "
                  << translation_bound_m() << " m, peak swing " << held_->peak_swing_m << " of "
                  << config_->held_swing_allowance_m << " m, peak tilt " << held_->peak_tilt_rad
                  << " of " << config_->held_tilt_allowance_rad << " rad)\n";
          }
          held_.reset();
        } else {
          static_cast<void>(mailbox_->mark_external_inconsistency(
            "journal rejected a verified physical mutation terminal state"));
        }
        verifier_->reset();
        active_.reset();
      } else if (verified.disposition == AttachmentVerificationDisposition::kOutcomeUnknown) {
        fail_active(verified.status.detail);
      }
      return;
    }
    if (held_ && mailbox_->state().phase == AttachmentPhase::kAttached) {
      // D5 amendment point 5: the release starts when the jaws part, and the watchdog stops
      // there with it. Everything below is the hold; the release is measured by the probe.
      if (config_->hold_mode == AttachmentHoldMode::kPhysical && !held_->release_started &&
        release_started(*held_, ecm))
      {
        held_->release_started = true;
      }
      if (held_->release_started) {
        return;
      }
      auto observation = physical_observation(
        held_->entities, held_->joint_entity, info,
        link_motion_, ecm);
      if (!observation) {
        gzwarn << "AttachmentSystem held-object watchdog lost its physical evidence\n";
        static_cast<void>(
          mailbox_->mark_external_inconsistency("held-object watchdog evidence is unavailable"));
        return;
      }
      observation->entity_identity_matches = entities_match(held_->entities, held_->identity, ecm);
      if (config_->hold_mode == AttachmentHoldMode::kPhysical) {
        // A turn about the product's own axis is no pose change for an upright cylinder (DART has
        // no torsional friction, so the product turns freely there and the pads never slip).
        observation->parent_from_child = without_spin_about_product_axis(
          held_->applied_parent_from_child, observation->parent_from_child);
        if (!held_->pads && held_->pad_capture_retries < kPadCaptureRetryTicks) {
          // The hold verified on a tick whose contact report was empty for a pad: take the
          // reference points from the next tick that reports both.
          ++held_->pad_capture_retries;
          held_->pads = capture_pads(held_->entities, ecm);
        }
        observation->pads = observe_pads(*held_, *observation, ecm);
      }
      held_->peak_translation_drift_m = std::max(
        held_->peak_translation_drift_m,
        (observation->parent_from_child.translation() -
        held_->applied_parent_from_child.translation()).norm());
      held_->peak_relative_linear_mps = std::max(
        held_->peak_relative_linear_mps, observation->relative_linear_velocity_in_parent.norm());
      held_->peak_relative_angular_radps = std::max(
        held_->peak_relative_angular_radps,
        observation->relative_angular_velocity_in_parent.norm());
      if (observation->pads) {
        const HeldGripMeasure grip = measure_held_grip(held_->applied_parent_from_child, *observation);
        held_->peak_swing_m = std::max(held_->peak_swing_m, grip.swing_m);
        held_->peak_tilt_rad = std::max(held_->peak_tilt_rad, grip.tilt_rad);
        record_held_sample(info, ecm, *observation, grip);
        held_->peak_pad_slip_m = std::max(
          held_->peak_pad_slip_m,
          std::max(
            (*observation->pads)[0].cumulative_slip_m, (*observation->pads)[1].cumulative_slip_m));
      }
      if (config_->hold_mode == AttachmentHoldMode::kPhysical && !observation->pads &&
        held_->pad_capture_retries < kPadCaptureRetryTicks)
      {
        return;  // still waiting for a tick that reports both pads in contact
      }
      const auto watchdog = validate_attached_watchdog(
        held_->applied_parent_from_child,
        verification_config(), *observation, config_->hold_mode);
      if (watchdog.code != AttachmentStatusCode::kAttached) {
        // The latch blocks later mutations and the coordinator sees it only on the next refusal,
        // so log the tripping measurement here.
        gzwarn << "AttachmentSystem held-object watchdog latched an inconsistency: "
               << watchdog.detail << " (drift " << held_->peak_translation_drift_m << " of "
               << translation_bound_m() << " m, relative linear "
               << held_->peak_relative_linear_mps << " of "
               << config_->tolerances.relative_linear_velocity_mps << " m/s, relative angular "
               << held_->peak_relative_angular_radps << " of "
               << config_->tolerances.relative_angular_velocity_radps
               << " rad/s, cumulative pad slip " << held_->peak_pad_slip_m << " of "
               << translation_bound_m() << " m)\n";
        print_held_recorder();
        static_cast<void>(mailbox_->mark_external_inconsistency(watchdog.detail));
      }
    }
  }

private:
  [[nodiscard]] bool ready() const noexcept {return config_.has_value() && mailbox_ && verifier_;}

  void record_held_sample(
    const gz::sim::UpdateInfo & info, const gz::sim::EntityComponentManager & ecm,
    const AttachmentVerificationObservation & observation, const HeldGripMeasure & grip)
  {
    HeldSample sample;
    sample.time_s = std::chrono::duration<double>(info.simTime).count();
    sample.drift = observation.parent_from_child.translation() -
      held_->applied_parent_from_child.translation();
    if (const auto hand = sampled_twist(link_motion_, held_->entities.parent_link)) {
      sample.hand_linear = hand->linear;
    }
    sample.relative_angular = observation.relative_angular_velocity_in_parent;
    const auto & entities = held_->entities;
    sample.left_force_n = pad_force(
      entities.robot_model, finger_link(entities.robot_model, entities.left_finger_joint, ecm),
      entities.child_link, ecm);
    sample.right_force_n = pad_force(
      entities.robot_model, finger_link(entities.robot_model, entities.right_finger_joint, ecm),
      entities.child_link, ecm);
    if (const auto aperture = finger_aperture(entities, ecm)) {
      sample.aperture_m = *aperture;
    }
    sample.swing_m = grip.swing_m;
    sample.tilt_rad = grip.tilt_rad;
    held_->recorder.push_back(sample);
    if (held_->recorder.size() > kRecorderTicks) {
      held_->recorder.pop_front();
    }
  }

  void print_held_recorder() const
  {
    for (std::size_t index = 0; index < held_->recorder.size(); index += kRecorderPrintStride) {
      const HeldSample & sample = held_->recorder[index];
      gzwarn << "HELDSAMPLE t=" << sample.time_s << " drift_mm=["
             << 1000.0 * sample.drift.transpose() << "] hand_mps=[" << sample.hand_linear.transpose()
             << "] rel_ang_radps=[" << sample.relative_angular.transpose() << "] force_n="
             << sample.left_force_n << "," << sample.right_force_n << " aperture_mm="
             << 1000.0 * sample.aperture_m << " swing_mm=" << 1000.0 * sample.swing_m
             << " tilt_rad=" << sample.tilt_rad << "\n";
    }
  }

  // The translation bound the held-object watchdog judges against: a pad's cumulative slip for a
  // physical hold, the product's drift for a weld.
  [[nodiscard]] double translation_bound_m() const
  {
    return config_->hold_mode == AttachmentHoldMode::kPhysical ?
           config_->held_slip_allowance_m : config_->post_attach_translation_drift_m;
  }

  [[nodiscard]] AttachmentVerificationConfig verification_config() const
  {
    const bool physical = config_->hold_mode == AttachmentHoldMode::kPhysical;
    return AttachmentVerificationConfig{config_->required_verification_ticks,
      config_->maximum_pending_ticks,
      translation_bound_m(),
      config_->post_attach_rotation_drift_rad,
      config_->tolerances.relative_linear_velocity_mps,
      config_->tolerances.relative_angular_velocity_radps,
      config_->held_swing_allowance_m,
      physical ? config_->held_tilt_allowance_rad : config_->tolerances.expected_rotation_rad};
  }

  bool reconcile_startup_and_advertise(const gz::sim::EntityComponentManager & ecm)
  {
    const std::size_t marker_count = project_owned_joint_marker_count(ecm);
    if (marker_count != 0) {
      static_cast<void>(mailbox_->mark_external_inconsistency(
        "startup found " + std::to_string(marker_count) +
        " pre-existing project-owned attachment joint marker(s)"));
    }

    const bool set_advertised =
      node_.Advertise(config_->set_service, &AttachmentSystem::on_set, this);
    const bool query_advertised =
      node_.Advertise(config_->query_service, &AttachmentSystem::on_query, this);
    if (!set_advertised || !query_advertised) {
      gzerr << "AttachmentSystem failed to advertise transport services\n";
      if (set_advertised) {
        static_cast<void>(node_.UnadvertiseSrv(config_->set_service));
      }
      if (query_advertised) {
        static_cast<void>(node_.UnadvertiseSrv(config_->query_service));
      }
      return false;
    }
    services_advertised_ = true;
    gzmsg << "AttachmentSystem ready at epoch " << mailbox_->simulator_epoch() << '\n';
    return true;
  }

  bool on_set(const msgs::SetAttachmentRequest & request, msgs::AttachmentReply & reply)
  {
    if (!mailbox_) {
      return false;
    }
    reply = mailbox_->handle_set(request);
    return true;
  }

  bool on_query(const msgs::QueryAttachmentRequest & request, msgs::AttachmentReply & reply)
  {
    if (!mailbox_) {
      return false;
    }
    reply = mailbox_->handle_query(request);
    return true;
  }

  [[nodiscard]] ResolutionResult resolve(
    const AttachmentIdentity & identity,
    const gz::sim::EntityComponentManager & ecm) const
  {
    const auto product = config_->products_by_source_id.find(identity.object_source_id);
    if (product == config_->products_by_source_id.end() ||
      identity.parent_model != config_->robot_model_name ||
      identity.parent_link != config_->parent_link ||
      (product != config_->products_by_source_id.end() &&
      (identity.child_model != product->second.model_name ||
      identity.child_link != product->second.child_link)))
    {
      return ResolutionResult{
        status(
          AttachmentStatusCode::kAuthorizationFailed,
          "attachment identity is outside the immutable scenario allowlist"),
        std::nullopt};
    }

    AttachmentStatus error;
    auto robot = unique_child<gz::sim::components::Model>(
      world_entity_, config_->robot_model_name,
      ecm, error);
    if (!robot) {
      return ResolutionResult{error, std::nullopt};
    }
    auto parent = unique_child<gz::sim::components::Link>(*robot, config_->parent_link, ecm, error);
    if (!parent) {
      return ResolutionResult{error, std::nullopt};
    }
    auto left_joint =
      unique_child<gz::sim::components::Joint>(*robot, config_->left_finger_joint, ecm, error);
    if (!left_joint) {
      return ResolutionResult{error, std::nullopt};
    }
    auto right_joint =
      unique_child<gz::sim::components::Joint>(*robot, config_->right_finger_joint, ecm, error);
    if (!right_joint) {
      return ResolutionResult{error, std::nullopt};
    }
    auto child_model = unique_child<gz::sim::components::Model>(
      world_entity_, product->second.model_name, ecm, error);
    if (!child_model) {
      return ResolutionResult{error, std::nullopt};
    }
    auto child_link = unique_child<gz::sim::components::Link>(
      *child_model, product->second.child_link, ecm, error);
    if (!child_link) {
      return ResolutionResult{error, std::nullopt};
    }
    return ResolutionResult{
      status(AttachmentStatusCode::kPending, "configured entities resolved uniquely"),
      ResolvedEntities{*robot, * parent, * left_joint, * right_joint, * child_model, * child_link}};
  }

  [[nodiscard]] bool entities_match(
    const ResolvedEntities & entities,
    const AttachmentIdentity & identity,
    const gz::sim::EntityComponentManager & ecm) const
  {
    return named_entity_matches(
      entities.robot_model, world_entity_, config_->robot_model_name,
      ecm) &&
           ecm.Component<gz::sim::components::Model>(entities.robot_model) != nullptr &&
           named_entity_matches(
      entities.parent_link, entities.robot_model, config_->parent_link,
      ecm) &&
           ecm.Component<gz::sim::components::Link>(entities.parent_link) != nullptr &&
           named_entity_matches(
      entities.left_finger_joint, entities.robot_model,
      config_->left_finger_joint, ecm) &&
           ecm.Component<gz::sim::components::Joint>(entities.left_finger_joint) != nullptr &&
           named_entity_matches(
      entities.right_finger_joint, entities.robot_model,
      config_->right_finger_joint, ecm) &&
           ecm.Component<gz::sim::components::Joint>(entities.right_finger_joint) != nullptr &&
           named_entity_matches(entities.child_model, world_entity_, identity.child_model, ecm) &&
           ecm.Component<gz::sim::components::Model>(entities.child_model) != nullptr &&
           named_entity_matches(
      entities.child_link, entities.child_model, identity.child_link,
      ecm) &&
           ecm.Component<gz::sim::components::Link>(entities.child_link) != nullptr;
  }

  // The robot's entities, found once and used while they all exist; any missing one, including
  // one the configuration never matched, sends the next tick back to the scan.
  [[nodiscard]] const CachedRobot * robot_entities(const gz::sim::EntityComponentManager & ecm)
  {
    const auto present = [&ecm](gz::sim::Entity entity) {
        return entity != gz::sim::kNullEntity && ecm.HasEntity(entity);
      };
    if (robot_cache_ && present(robot_cache_->model) && present(robot_cache_->parent) &&
      present(robot_cache_->left) && present(robot_cache_->right))
    {
      return &*robot_cache_;
    }
    robot_cache_.reset();
    AttachmentStatus error;
    const auto robot = unique_child<gz::sim::components::Model>(
      world_entity_, config_->robot_model_name, ecm, error);
    if (!robot) {
      return nullptr;
    }
    CachedRobot found;
    found.model = *robot;
    const auto parent =
      unique_child<gz::sim::components::Link>(*robot, config_->parent_link, ecm, error);
    const auto left =
      unique_child<gz::sim::components::Joint>(*robot, config_->left_finger_joint, ecm, error);
    const auto right =
      unique_child<gz::sim::components::Joint>(*robot, config_->right_finger_joint, ecm, error);
    found.parent = parent.value_or(gz::sim::kNullEntity);
    found.left = left.value_or(gz::sim::kNullEntity);
    found.right = right.value_or(gz::sim::kNullEntity);
    for (const auto joint : {found.left, found.right}) {
      if (joint == gz::sim::kNullEntity) {
        continue;
      }
      const gz::sim::Entity finger = finger_link(*robot, joint, ecm);
      for (const auto collision : pad_collisions_of(*robot, finger, ecm)) {
        found.pad_collisions.push_back(collision);
      }
    }
    robot_cache_ = std::move(found);
    return &*robot_cache_;
  }

  // Whether any catalogued product stands within the pads' reach of the gripper, judged from the
  // poses sampled at the end of the previous step. An attachment in progress or a hold always
  // counts, and so does a gripper whose pose has not been sampled yet.
  [[nodiscard]] bool pads_within_reach_of_a_product(const CachedRobot & robot) const
  {
    if (held_ || active_ || pending_attach_) {
      return true;
    }
    const auto hand = link_motion_.find(robot.parent);
    if (hand == link_motion_.end()) {
      return true;
    }
    const Eigen::Vector3d hand_position = hand->second.world_pose.translation();
    for (const auto & [source_id, cached] : product_cache_) {
      static_cast<void>(source_id);
      const auto product = link_motion_.find(cached.link);
      if (product != link_motion_.end() &&
        (product->second.world_pose.translation() - hand_position).norm() < kPadReachM)
      {
        return true;
      }
    }
    return false;
  }

  // The product's link, or none while it is not in the world. A cached pair is used while both
  // entities exist; once either is gone the full scan runs again, so a product that is removed or
  // respawned is never read through a stale entity.
  [[nodiscard]] std::optional<gz::sim::Entity> product_link(
    const std::string & source_id, const AttachmentProductConfig & product,
    const gz::sim::EntityComponentManager & ecm)
  {
    auto cached = product_cache_.find(source_id);
    // Entity ids are never reused and a name never changes, so an id that is still in the world is
    // the entity that was found.
    if (cached != product_cache_.end() && ecm.HasEntity(cached->second.model) &&
      ecm.HasEntity(cached->second.link))
    {
      return cached->second.link;
    }
    AttachmentStatus error;
    const auto model =
      unique_child<gz::sim::components::Model>(world_entity_, product.model_name, ecm, error);
    const auto link = model ?
      unique_child<gz::sim::components::Link>(*model, product.child_link, ecm, error) :
      std::nullopt;
    if (!link) {
      if (cached != product_cache_.end()) {
        for (const auto collision : cached->second.collisions) {
          product_mu_by_collision_.erase(collision);
        }
        product_cache_.erase(cached);
      }
      return std::nullopt;
    }
    if (cached != product_cache_.end()) {
      for (const auto collision : cached->second.collisions) {
        product_mu_by_collision_.erase(collision);
      }
    }
    product_cache_[source_id] = CachedProduct{*model, *link, {}, false, false};
    return link;
  }

  // Records this tick's post-physics world pose of the gripper link and every catalogued product
  // link, measuring each one's twist against its previous sample. PreUpdate's pending gate reads
  // the twist measured at the end of the previous step, which is the state it is gating on.
  void sample_link_motion(
    const gz::sim::UpdateInfo & info, const gz::sim::EntityComponentManager & ecm)
  {
    std::vector<gz::sim::Entity> links;
    AttachmentStatus error;
    const auto robot = unique_child<gz::sim::components::Model>(
      world_entity_, config_->robot_model_name, ecm, error);
    if (robot) {
      const auto parent =
        unique_child<gz::sim::components::Link>(*robot, config_->parent_link, ecm, error);
      if (parent) {
        links.push_back(*parent);
      }
    }
    for (const auto &[source_id, product] : config_->products_by_source_id) {
      const auto link = product_link(source_id, product, ecm);
      if (link) {
        links.push_back(*link);
      }
    }

    const std::int64_t now_ns = simulation_time_ns(info);
    // Updated in place: a map built afresh every step for every product was a measurable share of
    // the step. A link not sampled this step (gone, or without a pose) is dropped afterwards.
    ++motion_generation_;
    for (const auto link : links) {
      const auto pose = gz::sim::Link(link).WorldPose(ecm);
      if (!pose) {
        continue;
      }
      auto [slot, inserted] = link_motion_.try_emplace(link);
      LinkMotionSample & sample = slot->second;
      const LinkMotionSample previous = sample;
      sample.world_pose = eigen_pose(*pose);
      sample.time_ns = now_ns;
      sample.generation = motion_generation_;
      if (inserted) {
        sample.twist = std::nullopt;
      } else if (now_ns == previous.time_ns) {
        // No step since the last sample: the previous measurement still describes this state.
        sample.twist = previous.twist;
      } else if (now_ns > previous.time_ns) {
        sample.twist = finite_difference_world_twist(
          previous.world_pose, sample.world_pose,
          static_cast<double>(now_ns - previous.time_ns) * 1.0e-9);
      } else {
        sample.twist = std::nullopt;
      }
    }
    for (auto it = link_motion_.begin(); it != link_motion_.end();) {
      it = it->second.generation == motion_generation_ ? std::next(it) : link_motion_.erase(it);
    }
  }

  // Each finger's contact with the product at the moment the physical hold is verified, as a
  // point in the finger's frame and the same material point in the product's frame.
  [[nodiscard]] std::optional<std::array<HeldPadReference, 2>> capture_pads(
    const ResolvedEntities & entities, const gz::sim::EntityComponentManager & ecm) const
  {
    const auto world_from_child = gz::sim::Link(entities.child_link).WorldPose(ecm);
    if (!world_from_child) {
      return std::nullopt;
    }
    std::array<HeldPadReference, 2> result;
    const std::array<gz::sim::Entity, 2> joints{
      entities.left_finger_joint, entities.right_finger_joint};
    for (std::size_t index = 0; index < 2; ++index) {
      const gz::sim::Entity finger = finger_link(entities.robot_model, joints[index], ecm);
      const auto world_from_finger = gz::sim::Link(finger).WorldPose(ecm);
      const auto contact =
        finger_contact_in_world(entities.robot_model, finger, entities.child_link, ecm);
      if (finger == gz::sim::kNullEntity || !world_from_finger || !contact) {
        return std::nullopt;
      }
      result[index].finger_link = finger;
      result[index].contact_in_finger = eigen_pose(*world_from_finger).inverse() * *contact;
      result[index].contact_in_child = eigen_pose(*world_from_child).inverse() * *contact;
    }
    return result;
  }

  // Both pads now: the finger's pose in the gripper frame, and each pad's slip accumulated over the
  // hold in steps of the settle rule's 0.1 mm (spec M10 section 11 D5 amendment).
  [[nodiscard]] std::optional<std::array<HeldPadObservation, 2>> observe_pads(
    HeldAttachment & held, const AttachmentVerificationObservation & observation,
    const gz::sim::EntityComponentManager & ecm) const
  {
    constexpr double kSlipCountStepM = 1.0e-4;
    const auto world_from_parent = gz::sim::Link(held.entities.parent_link).WorldPose(ecm);
    if (!held.pads || !world_from_parent) {
      return std::nullopt;
    }
    std::array<HeldPadObservation, 2> result;
    for (std::size_t index = 0; index < 2; ++index) {
      HeldPadReference & reference = (*held.pads)[index];
      const auto world_from_finger = gz::sim::Link(reference.finger_link).WorldPose(ecm);
      if (!world_from_finger) {
        return std::nullopt;
      }
      HeldPadObservation & pad = result[index];
      pad.contact_in_finger = reference.contact_in_finger;
      pad.contact_in_child = reference.contact_in_child;
      pad.parent_from_finger =
        eigen_pose(*world_from_parent).inverse() * eigen_pose(*world_from_finger);
      // Slip is motion in the pad plane (see pad_plane_slip): the compliant pad row's deflection
      // along the jaw axis is not sliding.
      accumulate_pad_slip(
        reference.slip,
        pad_plane_slip(
          observation.parent_from_child * reference.contact_in_child -
          pad.parent_from_finger * reference.contact_in_finger, pad.parent_from_finger),
        kSlipCountStepM);
      pad.cumulative_slip_m = cumulative_pad_slip(reference.slip);
    }
    return result;
  }

  // The two maps the contact-surface callback reads, rebuilt each tick: which collisions are pad
  // collisions (the finger link's own plus its pad-row children), and which coefficient each
  // product collision grips at. Nothing else in the world is listed, so the override cannot reach
  // a product-to-shelf, product-to-tray or product-to-product contact.
  void refresh_contact_surface_pairs(gz::sim::EntityComponentManager & ecm)
  {
    pad_collisions_.clear();
    if (const CachedRobot * robot = robot_entities(ecm)) {
      // Asking Physics to customize contacts is not free: with the request standing on the pads
      // every step, the engine ran at a fraction of the speed it has without it, for as long as
      // the arm was anywhere. The request is made only while a product is within the pads' reach.
      const bool engaged = pads_within_reach_of_a_product(*robot);
      for (const auto collision : robot->pad_collisions) {
        pad_collisions_.insert(collision);
        if (engaged) {
          ecm.SetComponentData<gz::sim::components::EnableContactSurfaceCustomization>(
            collision, true);
        }
      }
    }
    for (const auto & [source_id, product] : config_->products_by_source_id) {
      const auto coefficient = config_->pad_friction_by_product_class.find(product.product_class);
      if (coefficient == config_->pad_friction_by_product_class.end()) {
        continue;
      }
      const auto link = product_link(source_id, product, ecm);
      if (!link) {
        continue;
      }
      auto & entry = product_cache_[source_id];
      if (entry.contact_mapped) {
        continue;
      }
      entry.contact_mapped = true;
      entry.collisions = gz::sim::Link(*link).Collisions(ecm);
      for (const auto collision : entry.collisions) {
        product_mu_by_collision_.emplace(collision, coefficient->second);
      }
    }
  }

  // Runs from the physics engine for every contact point of a collision that asked for surface
  // customization. Only a pad-to-product pair is touched; everything else returns untouched, so
  // the catalogued friction keeps its own min() (section 11.2 F5/F6).
  void apply_pad_friction(
    gz::sim::Entity collision1, gz::sim::Entity collision2,
    gz::physics::SetContactPropertiesCallbackFeature::ContactSurfaceParams<
      gz::physics::FeaturePolicy3d> & params) const
  {
    const bool first_is_pad = pad_collisions_.contains(collision1);
    const bool second_is_pad = pad_collisions_.contains(collision2);
    if (first_is_pad == second_is_pad) {
      return;
    }
    const auto coefficient =
      product_mu_by_collision_.find(first_is_pad ? collision2 : collision1);
    if (coefficient == product_mu_by_collision_.end()) {
      return;
    }
    params.frictionCoeff = coefficient->second;
    params.secondaryFrictionCoeff = coefficient->second;
  }

  // D5 amendment point 5 (refined 2026-10-17): true once the jaws have parted by more than the
  // contact tolerance since the grip was verified AND the pads have let go. A release unloads the
  // pads; a product that swings or squeezes out can still push the jaws apart by a fraction of a
  // millimetre while both pads stay loaded (measured: +0.13 mm at 13-25 N per pad, mid-carry),
  // and that must keep the watchdog armed.
  [[nodiscard]] bool release_started(
    HeldAttachment & held, const gz::sim::EntityComponentManager & ecm) const
  {
    constexpr double kUnloadedFraction = 0.25;
    constexpr double kMinimumReferenceN = 1.0;
    const auto left = finger_link(held.entities.robot_model, held.entities.left_finger_joint, ecm);
    const auto right =
      finger_link(held.entities.robot_model, held.entities.right_finger_joint, ecm);
    const double force_n = 0.5 * (
      pad_force(held.entities.robot_model, left, held.entities.child_link, ecm) +
      pad_force(held.entities.robot_model, right, held.entities.child_link, ecm));
    if (held.pad_force_reference_n <= 0.0) {
      if (force_n >= kMinimumReferenceN) {
        held.pad_force_reference_n = force_n;
      }
      return false;
    }
    const auto aperture = finger_aperture(held.entities, ecm);
    if (!aperture) {
      return false;
    }
    const double growth = *aperture - held.aperture_reference;
    if (growth <= config_->gripper.contact_tolerance_m ||
      force_n > kUnloadedFraction * held.pad_force_reference_n)
    {
      return false;
    }
    gzmsg << "AttachmentSystem held-object watchdog disarmed: release started (aperture +"
          << growth << " m of the " << config_->gripper.contact_tolerance_m
          << " m contact tolerance, pad force " << force_n << " N of the " 
          << held.pad_force_reference_n
          << " N hold; slip after this point is the release, not a lost grip)\n";
    return true;
  }

  void enable_robot_observability(gz::sim::EntityComponentManager & ecm)
  {
    const CachedRobot * robot = robot_entities(ecm);
    if (robot == nullptr) {
      return;
    }
    if (!robot_cache_->observed) {
      robot_cache_->observed = true;
      if (robot->parent != gz::sim::kNullEntity) {
        gz::sim::Link(robot->parent).EnableVelocityChecks(ecm);
      }
      for (const auto joint : {robot->left, robot->right}) {
        if (joint == gz::sim::kNullEntity) {
          continue;
        }
        gz::sim::Joint(joint).EnablePositionCheck(ecm);
        // Physical mode's stall predicate reads these: a jaw stopped against the product has a
        // position short of the command and a speed of about zero, and only the second of those
        // distinguishes it from a jaw that has not started closing.
        gz::sim::Joint(joint).EnableVelocityCheck(ecm);
      }
    }
    // The physical hold's pad contacts (spec M10 section 11 D5 amendment): Physics reports
    // contacts only for collisions that carry this component.
    for (const auto collision : robot->pad_collisions) {
      if (!ecm.Component<gz::sim::components::ContactSensorData>(collision)) {
        ecm.CreateComponent(collision, gz::sim::components::ContactSensorData());
      }
    }
    for (const auto &[source_id, product] : config_->products_by_source_id) {
      const auto link = product_link(source_id, product, ecm);
      if (link && !product_cache_[source_id].observed) {
        product_cache_[source_id].observed = true;
        gz::sim::Link(*link).EnableVelocityChecks(ecm);
      }
    }
  }

  [[nodiscard]] std::optional<AttachPhysicalObservation> attach_observation(
    const ResolvedEntities & entities, const gz::sim::EntityComponentManager & ecm) const
  {
    const auto common =
      physical_observation(
      entities, gz::sim::kNullEntity, gz::sim::UpdateInfo{}, link_motion_,
      ecm);
    const auto left_position = gz::sim::Joint(entities.left_finger_joint).Position(ecm);
    const auto right_position = gz::sim::Joint(entities.right_finger_joint).Position(ecm);
    const auto left_velocity = gz::sim::Joint(entities.left_finger_joint).Velocity(ecm);
    const auto right_velocity = gz::sim::Joint(entities.right_finger_joint).Velocity(ecm);
    if (!common || !left_position || left_position->size() != 1 || !right_position ||
      right_position->size() != 1 || !left_velocity || left_velocity->size() != 1 ||
      !right_velocity || right_velocity->size() != 1)
    {
      return std::nullopt;
    }
    const auto static_value = ecm.ComponentData<gz::sim::components::Static>(entities.child_model);
    AttachPhysicalObservation result;
    result.grasp_center_from_child =
      config_->gripper_from_grasp_center.inverse() * common->parent_from_child;
    result.relative_linear_velocity_in_parent = common->relative_linear_velocity_in_parent;
    result.relative_angular_velocity_in_parent = common->relative_angular_velocity_in_parent;
    result.left_finger_position_m = left_position->front();
    result.right_finger_position_m = right_position->front();
    result.left_finger_velocity_mps = left_velocity->front();
    result.right_finger_velocity_mps = right_velocity->front();
    result.child_dynamic = !static_value.value_or(false);
    result.parent_and_child_share_rigid_body = entities.robot_model == entities.child_model;
    const auto owned = exact_owned_joints(entities.parent_link, entities.child_link, ecm);
    result.matching_owned_joint_count = owned.size();
    const std::size_t constraints = child_constraint_count(entities.child_link, ecm);
    result.other_child_constraint_count =
      constraints >= owned.size() ? constraints - owned.size() : constraints;
    return result;
  }

  [[nodiscard]] std::optional<DetachPhysicalObservation> detach_observation(
    const ResolvedEntities & entities, gz::sim::Entity joint_entity,
    const gz::sim::EntityComponentManager & ecm) const
  {
    const auto common = physical_observation(
      entities, joint_entity, gz::sim::UpdateInfo{},
      link_motion_, ecm);
    const auto left_position = gz::sim::Joint(entities.left_finger_joint).Position(ecm);
    const auto right_position = gz::sim::Joint(entities.right_finger_joint).Position(ecm);
    if (!common || !left_position || left_position->size() != 1 || !right_position ||
      right_position->size() != 1)
    {
      return std::nullopt;
    }
    const auto owned = exact_owned_joints(entities.parent_link, entities.child_link, ecm);
    DetachPhysicalObservation result;
    result.parent_from_child = common->parent_from_child;
    result.world_from_child = common->world_from_child;
    result.left_finger_position_m = left_position->front();
    result.right_finger_position_m = right_position->front();
    result.matching_owned_joint_count =
      exact_joint(joint_entity, entities.parent_link, entities.child_link, ecm) ? 1U : 0U;
    result.other_owned_joint_count = owned.size() >= result.matching_owned_joint_count ?
      owned.size() - result.matching_owned_joint_count :
      owned.size();
    return result;
  }

  void apply_command(
    const DecodedAttachmentCommand & command, const gz::sim::UpdateInfo & info,
    gz::sim::EntityComponentManager & ecm)
  {
    const auto resolution = resolve(command.request.identity, ecm);
    if (!resolution.entities) {
      static_cast<void>(mailbox_->reject_before_mutation(command.operation_id, resolution.status));
      return;
    }
    const auto & entities = *resolution.entities;
    if (command.request.command == AttachmentCommand::kAttach) {
      apply_attach(command, entities, info, ecm);
    } else {
      apply_detach(command, entities, info, ecm);
    }
  }

  void apply_attach(
    const DecodedAttachmentCommand & command, const ResolvedEntities & entities,
    const gz::sim::UpdateInfo & info, gz::sim::EntityComponentManager & ecm)
  {
    pending_attach_.emplace();
    pending_attach_->command = command;
    pending_attach_->entities = entities;
    pending_attach_->first_iteration = info.iterations;
    continue_pending_attach(info, ecm);
  }

  void reject_pending_attach(
    AttachmentStatus rejection, std::optional<AttachmentFidelity> fidelity = std::nullopt)
  {
    static_cast<void>(mailbox_->reject_before_mutation(
      pending_attach_->command.operation_id, std::move(rejection), std::move(fidelity)));
    pending_attach_.reset();
  }

  void continue_pending_attach(
    const gz::sim::UpdateInfo & info, gz::sim::EntityComponentManager & ecm)
  {
    if (!pending_attach_) {
      return;
    }
    const auto & command = pending_attach_->command;
    const auto & entities = pending_attach_->entities;
    const auto product =
      config_->products_by_source_id.find(command.request.identity.object_source_id);
    const auto observation = attach_observation(entities, ecm);
    if (!observation || product == config_->products_by_source_id.end()) {
      reject_pending_attach(
        status(
          AttachmentStatusCode::kOutOfTolerance,
          "required physical attachment observations are unavailable"));
      return;
    }

    const bool finite_velocity = observation->relative_linear_velocity_in_parent.allFinite() &&
      observation->relative_angular_velocity_in_parent.allFinite();
    if (finite_velocity) {
      pending_attach_->peak_relative_linear_mps = std::max(
        pending_attach_->peak_relative_linear_mps,
        observation->relative_linear_velocity_in_parent.norm());
      pending_attach_->peak_relative_angular_radps = std::max(
        pending_attach_->peak_relative_angular_radps,
        observation->relative_angular_velocity_in_parent.norm());
    }
    // Pose finite differences over the pending window: an independent measurement of the same
    // quantity the velocity components report, so a refusal can name a disagreement instead of
    // gating on one stream's word.
    {
      const auto parent_world = gz::sim::Link(entities.parent_link).WorldPose(ecm);
      const auto child_world = gz::sim::Link(entities.child_link).WorldPose(ecm);
      const std::int64_t now_ns = simulation_time_ns(info);
      if (parent_world && child_world) {
        const Eigen::Isometry3d parent_pose = eigen_pose(*parent_world);
        const Eigen::Isometry3d child_pose = eigen_pose(*child_world);
        if (!pending_attach_->first_parent_world) {
          pending_attach_->first_parent_world = parent_pose;
          pending_attach_->first_child_world = child_pose;
          pending_attach_->first_time_ns = now_ns;
        }
        if (pending_attach_->previous_parent_world &&
          now_ns > pending_attach_->previous_time_ns)
        {
          const double dt =
            static_cast<double>(now_ns - pending_attach_->previous_time_ns) * 1.0e-9;
          const double parent_fd = (parent_pose.translation() -
            pending_attach_->previous_parent_world->translation()).norm() / dt;
          const double child_fd = (child_pose.translation() -
            pending_attach_->previous_child_world->translation()).norm() / dt;
          pending_attach_->peak_parent_fd_mps = std::max(
            pending_attach_->peak_parent_fd_mps, parent_fd);
          pending_attach_->peak_child_fd_mps = std::max(
            pending_attach_->peak_child_fd_mps, child_fd);
        }
        pending_attach_->previous_parent_world = parent_pose;
        pending_attach_->previous_child_world = child_pose;
        pending_attach_->previous_time_ns = now_ns;
      }
    }
    const bool velocity_stable = finite_velocity &&
      observation->relative_linear_velocity_in_parent.norm() <=
      config_->tolerances.relative_linear_velocity_mps &&
      observation->relative_angular_velocity_in_parent.norm() <=
      config_->tolerances.relative_angular_velocity_radps;
    // Physical mode additionally requires the jaws themselves to have stopped: the product can
    // be motionless while a finger is still travelling toward it, and only a stalled finger has
    // taken the load. Counted in the same consecutive-tick budget so both must hold for
    // required_verification_ticks, not just the product's twist.
    const bool fingers_stalled =
      config_->hold_mode != AttachmentHoldMode::kPhysical ||
      (std::isfinite(observation->left_finger_velocity_mps) &&
      std::isfinite(observation->right_finger_velocity_mps) &&
      std::abs(observation->left_finger_velocity_mps) <=
      config_->tolerances.relative_linear_velocity_mps &&
      std::abs(observation->right_finger_velocity_mps) <=
      config_->tolerances.relative_linear_velocity_mps);
    if (!velocity_stable || !fingers_stalled) {
      pending_attach_->consecutive_stable_ticks = 0U;
      const std::uint64_t elapsed = info.iterations >= pending_attach_->first_iteration ?
        info.iterations - pending_attach_->first_iteration : 0U;
      if (!finite_velocity || elapsed >= config_->maximum_pending_ticks) {
        auto validation = validate_physical_attach(
          command.request, config_->gripper, product->second.geometry,
          config_->tolerances, *observation, config_->hold_mode);
        // Decomposition for the refusal: the norm alone cannot say which body carries the
        // velocity (product, gripper, or the cross term), so print the parent-frame vectors,
        // both world linear velocities and the jaw speeds on the same line as the peak.
        const auto parent_linear =
          gz::sim::Link(entities.parent_link).WorldLinearVelocity(ecm);
        const auto child_linear = gz::sim::Link(entities.child_link).WorldLinearVelocity(ecm);
        const auto & relative_linear = observation->relative_linear_velocity_in_parent;
        const auto & relative_angular = observation->relative_angular_velocity_in_parent;
        std::ostringstream peak_receipt;
        peak_receipt << " [pending window peak: linear "
                     << pending_attach_->peak_relative_linear_mps << " m/s, angular "
                     << pending_attach_->peak_relative_angular_radps << " rad/s over "
                     << (elapsed + 1) << " ticks; relative linear xyz " << relative_linear.x()
                     << " " << relative_linear.y() << " " << relative_linear.z()
                     << "; relative angular xyz " << relative_angular.x() << " "
                     << relative_angular.y() << " " << relative_angular.z()
                     << "; engine (ungated) parent world linear "
                     << (parent_linear ? std::to_string(parent_linear->X()) + " " +
        std::to_string(parent_linear->Y()) + " " +
        std::to_string(parent_linear->Z()) : std::string("unavailable"))
                     << "; engine (ungated) child world linear "
                     << (child_linear ? std::to_string(child_linear->X()) + " " +
        std::to_string(child_linear->Y()) + " " +
        std::to_string(child_linear->Z()) : std::string("unavailable"))
                     << "; fingers left " << observation->left_finger_velocity_mps << " right "
                     << observation->right_finger_velocity_mps << " m/s";
        if (pending_attach_->previous_parent_world && pending_attach_->first_parent_world &&
          pending_attach_->previous_time_ns > pending_attach_->first_time_ns)
        {
          const double span = static_cast<double>(
            pending_attach_->previous_time_ns - pending_attach_->first_time_ns) * 1.0e-9;
          const Eigen::Vector3d parent_drift =
            pending_attach_->previous_parent_world->translation() -
            pending_attach_->first_parent_world->translation();
          const Eigen::Vector3d child_drift =
            pending_attach_->previous_child_world->translation() -
            pending_attach_->first_child_world->translation();
          peak_receipt << "; window pose fd over " << span << " s: parent drift xyz "
                       << parent_drift.x() << " " << parent_drift.y() << " " << parent_drift.z()
                       << " (mean " << parent_drift.norm() / span << " m/s), child drift xyz "
                       << child_drift.x() << " " << child_drift.y() << " " << child_drift.z()
                       << " (mean " << child_drift.norm() / span
                       << " m/s), per-tick peaks parent " << pending_attach_->peak_parent_fd_mps
                       << " child " << pending_attach_->peak_child_fd_mps << " m/s]";
        }
        validation.status.detail += peak_receipt.str();
        gzmsg << "AttachmentSystem pre-attach refusal for "
              << command.request.identity.object_source_id << ":"
              << validation.status.detail << '\n';
        reject_pending_attach(validation.status);
      }
      return;
    }
    ++pending_attach_->consecutive_stable_ticks;
    if (pending_attach_->consecutive_stable_ticks < config_->required_verification_ticks) {
      return;
    }

    const auto validation =
      validate_physical_attach(
      command.request, config_->gripper, product->second.geometry,
      config_->tolerances, *observation, config_->hold_mode);
    // Grasp fidelity (authorized vs observed) is recorded on rejected attaches too, so residuals
    // are not sampled from successes only. Goes to the journal, its mirrored state and the
    // adapter state topic.
    std::optional<AttachmentFidelity> fidelity;
    if (validation.evidence) {
      fidelity = AttachmentFidelity{
        validation.evidence->translation_error_m,
        validation.evidence->rotation_error_rad,
        validation.evidence->translation_budget_m,
        validation.evidence->claimed_translation_sigma_m,
        validation.evidence->translation_ceiling_m};
    }
    if (!validation) {
      reject_pending_attach(validation.status, fidelity);
      return;
    }
    static_cast<void>(mailbox_->mark_validation_succeeded(command.operation_id, fidelity));
    const Eigen::Isometry3d applied_parent_from_child =
      config_->gripper_from_grasp_center * observation->grasp_center_from_child;
    // The gate (spec 11.3 D1/D2): physical mode creates no entity at all — the grip the jaws
    // just established is the hold — while weld mode builds the DetachableJoint it always did.
    // Everything below (journal phases, verifier, receipts) is shared; only the mutation differs.
    gz::sim::Entity joint_entity = gz::sim::kNullEntity;
    if (config_->hold_mode == AttachmentHoldMode::kWeld) {
      joint_entity = ecm.CreateEntity();
      auto * name = ecm.CreateComponent(
        joint_entity, gz::sim::components::Name(kOwnedJointName));
      if (!name) {
        ecm.RequestRemoveEntity(joint_entity, false);
        static_cast<void>(mailbox_->reject_before_mutation(
          command.operation_id,
          status(AttachmentStatusCode::kInternalError, "failed to create owned joint marker")));
        pending_attach_.reset();
        return;
      }
      auto * joint = ecm.CreateComponent(
        joint_entity,
        gz::sim::components::DetachableJoint(
          {entities.parent_link, entities.child_link, "fixed"}));
      if (!joint) {
        ecm.RequestRemoveEntity(joint_entity, false);
        static_cast<void>(mailbox_->reject_before_mutation(
          command.operation_id,
          status(AttachmentStatusCode::kInternalError, "failed to create detachable joint")));
        pending_attach_.reset();
        return;
      }
    } else {
      gzmsg << "AttachmentSystem physical grip verified for "
            << command.request.identity.object_source_id
            << ": no joint created, finger contact and friction hold the product\n";
    }
    static_cast<void>(mailbox_->mark_mutation_started(command.operation_id));
    active_ = ActiveMutation{command, entities, joint_entity, applied_parent_from_child};
    verifier_->begin(
      command.operation_id, AttachmentCommand::kAttach, applied_parent_from_child,
      info.iterations, simulation_time_ns(info));
    pending_attach_.reset();
  }

  void apply_detach(
    const DecodedAttachmentCommand & command, const ResolvedEntities & entities,
    const gz::sim::UpdateInfo & info, gz::sim::EntityComponentManager & ecm)
  {
    const auto joints = exact_owned_joints(entities.parent_link, entities.child_link, ecm);
    const std::size_t expected_owned_joints =
      config_->hold_mode == AttachmentHoldMode::kPhysical ? 0U : 1U;
    if (joints.size() != expected_owned_joints) {
      static_cast<void>(mailbox_->reject_before_mutation(
        command.operation_id, status(
          AttachmentStatusCode::kExternalInconsistency,
          config_->hold_mode == AttachmentHoldMode::kPhysical ?
          "detach in physical mode found an owned joint it must not create" :
          "detach did not resolve exactly one owned joint")));
      return;
    }
    const auto observation = detach_observation(
      entities, joints.empty() ? gz::sim::kNullEntity : joints.front(), ecm);
    if (!observation) {
      static_cast<void>(mailbox_->reject_before_mutation(
        command.operation_id,
        status(
          AttachmentStatusCode::kOutOfTolerance,
          "required physical detachment observations are unavailable")));
      return;
    }
    const auto validation =
      validate_physical_detach(
      config_->gripper, config_->tolerances, *observation, config_->hold_mode);
    if (validation.code != AttachmentStatusCode::kPending) {
      static_cast<void>(mailbox_->reject_before_mutation(command.operation_id, validation));
      return;
    }
    static_cast<void>(mailbox_->mark_validation_succeeded(command.operation_id));
    // Weld removes the joint it created; physical has nothing to remove, and the verifier's
    // terminal evidence for that mode is precisely that no owned joint exists (spec 11.4).
    const gz::sim::Entity joint_entity =
      joints.empty() ? gz::sim::kNullEntity : joints.front();
    if (joint_entity != gz::sim::kNullEntity) {
      ecm.RequestRemoveEntity(joint_entity, false);
    }
    static_cast<void>(mailbox_->mark_mutation_started(command.operation_id));
    active_ = ActiveMutation{command, entities, joint_entity, observation->parent_from_child};
    verifier_->begin(
      command.operation_id, AttachmentCommand::kDetach,
      observation->parent_from_child, info.iterations, simulation_time_ns(info));
  }

  void fail_active(std::string detail)
  {
    gzwarn << "AttachmentSystem could not prove the outcome of operation "
           << active_->command.operation_id << ": " << detail << '\n';
    static_cast<void>(
      mailbox_->mark_outcome_unknown(active_->command.operation_id, std::move(detail)));
    verifier_->reset();
    pending_attach_.reset();
    active_.reset();
  }

  void rotate_epoch(const gz::sim::EntityComponentManager & ecm)
  {
    if (!ready()) {
      return;
    }
    std::optional<AttachmentIdentity> recovered;
    if (held_ && exact_joint(
        held_->joint_entity, held_->entities.parent_link,
        held_->entities.child_link, ecm))
    {
      recovered = held_->identity;
    } else {
      if (active_ && active_->command.request.command == AttachmentCommand::kAttach &&
        exact_joint(
          active_->joint_entity, active_->entities.parent_link,
          active_->entities.child_link, ecm))
      {
        recovered = active_->command.request.identity;
      }
    }
    const std::string previous_epoch = mailbox_->simulator_epoch();
    mailbox_->rotate_epoch(next_random_epoch(previous_epoch), recovered);
    if (!recovered && project_owned_joint_marker_count(ecm) != 0) {
      static_cast<void>(mailbox_->mark_external_inconsistency(
        "epoch transition found an attachment joint without recoverable identity"));
    }
    verifier_->reset();
    active_.reset();
    held_.reset();
    link_motion_.clear();
    product_cache_.clear();
    robot_cache_.reset();
    product_mu_by_collision_.clear();
  }

  gz::sim::Entity world_entity_{gz::sim::kNullEntity};
  std::optional<AttachmentBoundaryConfig> config_;
  std::unique_ptr<AttachmentMailbox> mailbox_;
  std::unique_ptr<AttachmentMutationVerifier> verifier_;
  std::optional<PendingAttachValidation> pending_attach_;
  std::optional<ActiveMutation> active_;
  std::optional<HeldAttachment> held_;
  std::optional<std::uint64_t> last_iteration_;
  std::optional<std::int64_t> last_simulation_time_ns_;
  LinkMotion link_motion_;
  std::uint64_t motion_generation_{0};
  bool services_advertised_{false};
  gz::transport::Node node_;
  // Spec M10 section 11 D10 item 2. Refreshed every PreUpdate because Physics removes
  // EnableContactSurfaceCustomization at the end of every one of its own updates.
  std::unordered_set<gz::sim::Entity> pad_collisions_;
  std::unordered_map<gz::sim::Entity, double> product_mu_by_collision_;
  // The catalogued products' model and link entities, found once and re-checked by identity on
  // every tick. Finding one is a scan of the world's children, and three per-tick passes made it
  // for every product on every step: with 84 products that scan was the whole cost of a step.
  std::unordered_map<std::string, CachedProduct> product_cache_;
  std::optional<CachedRobot> robot_cache_;
  gz::common::ConnectionPtr contact_surface_connection_;
};

}  // namespace restocker_gazebo

GZ_ADD_PLUGIN(
  restocker_gazebo::AttachmentSystem, gz::sim::System, gz::sim::ISystemConfigure,
  gz::sim::ISystemReset, gz::sim::ISystemPreUpdate, gz::sim::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(restocker_gazebo::AttachmentSystem, "restocker_gazebo::AttachmentSystem")
