// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <chrono>
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
#include <utility>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/System.hh>
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

[[nodiscard]] Eigen::Vector3d eigen_vector(const gz::math::Vector3d & vector)
{
  return Eigen::Vector3d(vector.X(), vector.Y(), vector.Z());
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
};

struct HeldAttachment
{
  AttachmentIdentity identity;
  ResolvedEntities entities;
  gz::sim::Entity joint_entity{gz::sim::kNullEntity};
  Eigen::Isometry3d applied_parent_from_child{Eigen::Isometry3d::Identity()};
  // Worst case over the whole carry, so watchdog margin is measurable from passing runs.
  double peak_translation_drift_m{0.0};
  double peak_relative_linear_mps{0.0};
  double peak_relative_angular_radps{0.0};
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

[[nodiscard]] std::optional<AttachmentVerificationObservation> physical_observation(
  const ResolvedEntities & entities, gz::sim::Entity joint_entity, const gz::sim::UpdateInfo & info,
  const gz::sim::EntityComponentManager & ecm)
{
  const gz::sim::Link parent(entities.parent_link);
  const gz::sim::Link child(entities.child_link);
  const auto world_from_parent_gz = parent.WorldPose(ecm);
  const auto world_from_child_gz = child.WorldPose(ecm);
  const auto parent_linear_gz = parent.WorldLinearVelocity(ecm);
  const auto child_linear_gz = child.WorldLinearVelocity(ecm);
  const auto parent_angular_gz = parent.WorldAngularVelocity(ecm);
  const auto child_angular_gz = child.WorldAngularVelocity(ecm);
  if (!world_from_parent_gz || !world_from_child_gz || !parent_linear_gz || !child_linear_gz ||
    !parent_angular_gz || !child_angular_gz)
  {
    return std::nullopt;
  }

  const Eigen::Isometry3d world_from_parent = eigen_pose(*world_from_parent_gz);
  const Eigen::Isometry3d world_from_child = eigen_pose(*world_from_child_gz);
  const Eigen::Vector3d offset_world =
    world_from_child.translation() - world_from_parent.translation();
  const Eigen::Vector3d parent_linear = eigen_vector(*parent_linear_gz);
  const Eigen::Vector3d child_linear = eigen_vector(*child_linear_gz);
  const Eigen::Vector3d parent_angular = eigen_vector(*parent_angular_gz);
  const Eigen::Vector3d child_angular = eigen_vector(*child_angular_gz);
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
    gz::sim::EntityComponentManager &, gz::sim::EventManager &) override
  {
    world_entity_ = entity;
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
        std::nullopt, config_->detach_attempt_reserve);
      verifier_ = std::make_unique<AttachmentMutationVerifier>(verification_config());
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
    if (active_ && verifier_->active()) {
      auto observation = physical_observation(active_->entities, active_->joint_entity, info, ecm);
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
            active_->joint_entity, active_->applied_parent_from_child};
        } else if (reply.status.code == AttachmentStatusCode::kDetached) {
          if (held_) {
            gzmsg << "AttachmentSystem carry ended within the watchdog limits (peak drift "
                  << held_->peak_translation_drift_m << " of "
                  << config_->post_attach_translation_drift_m << " m, peak relative linear "
                  << held_->peak_relative_linear_mps << " of "
                  << config_->tolerances.relative_linear_velocity_mps
                  << " m/s, peak relative angular " << held_->peak_relative_angular_radps
                  << " of " << config_->tolerances.relative_angular_velocity_radps << " rad/s)\n";
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
      auto observation = physical_observation(held_->entities, held_->joint_entity, info, ecm);
      if (!observation) {
        gzwarn << "AttachmentSystem held-object watchdog lost its physical evidence\n";
        static_cast<void>(
          mailbox_->mark_external_inconsistency("held-object watchdog evidence is unavailable"));
        return;
      }
      observation->entity_identity_matches = entities_match(held_->entities, held_->identity, ecm);
      held_->peak_translation_drift_m = std::max(
        held_->peak_translation_drift_m,
        (observation->parent_from_child.translation() -
        held_->applied_parent_from_child.translation()).norm());
      held_->peak_relative_linear_mps = std::max(
        held_->peak_relative_linear_mps, observation->relative_linear_velocity_in_parent.norm());
      held_->peak_relative_angular_radps = std::max(
        held_->peak_relative_angular_radps,
        observation->relative_angular_velocity_in_parent.norm());
      const auto watchdog = validate_attached_watchdog(
        held_->applied_parent_from_child,
        verification_config(), *observation);
      if (watchdog.code != AttachmentStatusCode::kAttached) {
        // The latch blocks later mutations and the coordinator sees it only on the next refusal,
        // so log the tripping measurement here.
        gzwarn << "AttachmentSystem held-object watchdog latched an inconsistency: "
               << watchdog.detail << " (drift " << held_->peak_translation_drift_m << " of "
               << config_->post_attach_translation_drift_m << " m, relative linear "
               << held_->peak_relative_linear_mps << " of "
               << config_->tolerances.relative_linear_velocity_mps << " m/s, relative angular "
               << held_->peak_relative_angular_radps << " of "
               << config_->tolerances.relative_angular_velocity_radps << " rad/s)\n";
        static_cast<void>(mailbox_->mark_external_inconsistency(watchdog.detail));
      }
    }
  }

private:
  [[nodiscard]] bool ready() const noexcept {return config_.has_value() && mailbox_ && verifier_;}

  [[nodiscard]] AttachmentVerificationConfig verification_config() const
  {
    return AttachmentVerificationConfig{config_->required_verification_ticks,
      config_->maximum_pending_ticks,
      config_->post_attach_translation_drift_m,
      config_->post_attach_rotation_drift_rad,
      config_->tolerances.relative_linear_velocity_mps,
      config_->tolerances.relative_angular_velocity_radps};
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

  void enable_robot_observability(gz::sim::EntityComponentManager & ecm) const
  {
    AttachmentStatus error;
    const auto robot = unique_child<gz::sim::components::Model>(
      world_entity_, config_->robot_model_name, ecm, error);
    if (!robot) {
      return;
    }
    const auto parent =
      unique_child<gz::sim::components::Link>(*robot, config_->parent_link, ecm, error);
    const auto left =
      unique_child<gz::sim::components::Joint>(*robot, config_->left_finger_joint, ecm, error);
    const auto right =
      unique_child<gz::sim::components::Joint>(*robot, config_->right_finger_joint, ecm, error);
    if (parent) {
      gz::sim::Link(*parent).EnableVelocityChecks(ecm);
    }
    if (left) {
      gz::sim::Joint(*left).EnablePositionCheck(ecm);
    }
    if (right) {
      gz::sim::Joint(*right).EnablePositionCheck(ecm);
    }
    for (const auto &[source_id, product] : config_->products_by_source_id) {
      static_cast<void>(source_id);
      const auto model =
        unique_child<gz::sim::components::Model>(world_entity_, product.model_name, ecm, error);
      if (!model) {
        continue;
      }
      const auto link =
        unique_child<gz::sim::components::Link>(*model, product.child_link, ecm, error);
      if (link) {
        gz::sim::Link(*link).EnableVelocityChecks(ecm);
      }
    }
  }

  [[nodiscard]] std::optional<AttachPhysicalObservation> attach_observation(
    const ResolvedEntities & entities, const gz::sim::EntityComponentManager & ecm) const
  {
    const auto common =
      physical_observation(entities, gz::sim::kNullEntity, gz::sim::UpdateInfo{}, ecm);
    const auto left_position = gz::sim::Joint(entities.left_finger_joint).Position(ecm);
    const auto right_position = gz::sim::Joint(entities.right_finger_joint).Position(ecm);
    if (!common || !left_position || left_position->size() != 1 || !right_position ||
      right_position->size() != 1)
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
    const auto common = physical_observation(entities, joint_entity, gz::sim::UpdateInfo{}, ecm);
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
    pending_attach_ = PendingAttachValidation{command, entities, info.iterations, 0U};
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
    const bool velocity_stable = finite_velocity &&
      observation->relative_linear_velocity_in_parent.norm() <=
      config_->tolerances.relative_linear_velocity_mps &&
      observation->relative_angular_velocity_in_parent.norm() <=
      config_->tolerances.relative_angular_velocity_radps;
    if (!velocity_stable) {
      pending_attach_->consecutive_stable_ticks = 0U;
      const std::uint64_t elapsed = info.iterations >= pending_attach_->first_iteration ?
        info.iterations - pending_attach_->first_iteration : 0U;
      if (!finite_velocity || elapsed >= config_->maximum_pending_ticks) {
        const auto validation = validate_physical_attach(
          command.request, config_->gripper, product->second.geometry,
          config_->tolerances, *observation);
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
      config_->tolerances, *observation);
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
    const gz::sim::Entity joint_entity = ecm.CreateEntity();
    auto * name = ecm.CreateComponent(joint_entity, gz::sim::components::Name(kOwnedJointName));
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
      gz::sim::components::DetachableJoint({entities.parent_link, entities.child_link, "fixed"}));
    if (!joint) {
      ecm.RequestRemoveEntity(joint_entity, false);
      static_cast<void>(mailbox_->reject_before_mutation(
        command.operation_id,
        status(AttachmentStatusCode::kInternalError, "failed to create detachable joint")));
      pending_attach_.reset();
      return;
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
    if (joints.size() != 1) {
      static_cast<void>(mailbox_->reject_before_mutation(
        command.operation_id, status(
          AttachmentStatusCode::kExternalInconsistency,
          "detach did not resolve exactly one owned joint")));
      return;
    }
    const auto observation = detach_observation(entities, joints.front(), ecm);
    if (!observation) {
      static_cast<void>(mailbox_->reject_before_mutation(
        command.operation_id,
        status(
          AttachmentStatusCode::kOutOfTolerance,
          "required physical detachment observations are unavailable")));
      return;
    }
    const auto validation =
      validate_physical_detach(config_->gripper, config_->tolerances, *observation);
    if (validation.code != AttachmentStatusCode::kPending) {
      static_cast<void>(mailbox_->reject_before_mutation(command.operation_id, validation));
      return;
    }
    static_cast<void>(mailbox_->mark_validation_succeeded(command.operation_id));
    ecm.RequestRemoveEntity(joints.front(), false);
    static_cast<void>(mailbox_->mark_mutation_started(command.operation_id));
    active_ = ActiveMutation{command, entities, joints.front(), observation->parent_from_child};
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
  bool services_advertised_{false};
  gz::transport::Node node_;
};

}  // namespace restocker_gazebo

GZ_ADD_PLUGIN(
  restocker_gazebo::AttachmentSystem, gz::sim::System, gz::sim::ISystemConfigure,
  gz::sim::ISystemReset, gz::sim::ISystemPreUpdate, gz::sim::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(restocker_gazebo::AttachmentSystem, "restocker_gazebo::AttachmentSystem")
