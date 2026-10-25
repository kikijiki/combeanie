// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// The pad-row spring-damper (Milestone 10 section 11 D10 item 3, Card 057).
//
// The description carries the rubber — every pad joint declares spring_stiffness 111 kN/m,
// spring_reference 0 and damping 11.5 N*s/m, and gz-sim's components::JointAxis holds exactly
// those values (Card 057 run B) — but the engine never acts on them: at 10, 20 and 40 N the loaded
// row parks at its 0.002 m travel limit instead of deflecting 90.1 / 180.2 / 360.4 um, and the
// unloaded rows drift instead of returning. The loss is inside gz-physics' joint construction,
// a pinned dependency, so this system supplies the force the engine will not:
//
//   F = -k * (x - x_ref)      (the damping c * ẋ is the engine's own, implicit)
//   as an equal-and-opposite ExternalWorldWrenchCmd pair
//
// on the two links: gz-sim applies the wrench at each link's origin, and on these joints the
// origin sits on the parent frame (the xacro's <origin> and the fixtures' <pose> are both the
// identity), so the pair is the joint force and its reaction exactly — the reaction reaches the
// finger, which is the physically right path. The direct joint path was measured first and is not
// used: as components::JointForceCmd the same value reaches the pad at only ~7 % (Card 057 bench
// run 1: a 10 N drive holds the pad 1.23 mm off the finger while the contact wrench reads 10 N,
// i.e. 8.1 kN/m of the commanded 111 kN/m), so the spring reads as a loose slider. The command's
// own bound is k * 0.002 m + c * 0.12 m/s = 223.4 N; the pad joints' effort is 500 N (spec M10
// section 11 D10 item 3) so a joint force command could never be the thing that clips it either,
// and the finger drive keeps effort_n = 40 N.
//
// x is taken from the child link's pose relative to the parent link along the axis (Physics
// leaves components::JointPosition empty on passive joints, Card 057 run A, so the joint reading
// is not there to take) and ẋ by finite difference over the step. The joint frame sits on the
// parent link frame at rest, so the axis as declared is the axis in the parent link frame and x
// is zero at rest, where spring_reference 0 says the uncompressed pad is.

#include <gz/msgs/wrench.pb.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/ContactSensorData.hh>
#include <gz/sim/components/ExternalWorldWrenchCmd.hh>
#include <gz/sim/components/Factory.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointAxis.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointPositionReset.hh>
#include <gz/sim/components/JointType.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/JointVelocityReset.hh>
#include <gz/sim/components/Name.hh>

namespace restocker_gazebo
{
namespace
{

// The pad rows are the only passive joints in the world: every other joint is driven, and a driven
// joint already gets its force from its controller.
constexpr char kPadRowMarker[] = "_pad_row_";
constexpr char kJointSuffix[] = "_joint";

[[nodiscard]] bool is_pad_row_joint(const std::string & name)
{
  constexpr std::size_t suffix_length = sizeof(kJointSuffix) - 1;
  if (name.size() < suffix_length) {
    return false;
  }
  const std::size_t suffix = name.size() - suffix_length;
  const bool ends_with_joint = name.compare(suffix, suffix_length, kJointSuffix) == 0;
  return ends_with_joint && name.find(kPadRowMarker) != std::string::npos;
}

// Opt-in trace of every quantity the force is computed from (Card 057's bench). Off by default so
// a production run prints only the per-joint receipt at resolve.
[[nodiscard]] bool trace_enabled()
{
  const char * value = std::getenv("RESTOCKER_PAD_SPRING_DEBUG");
  return value != nullptr && value[0] != '\0' && std::string(value) != "0";
}

// The minimal-repro bisect: read and print everything, but write no force, so the same world can
// be run with only the engine's own joint spring (and then with ours on top).
[[nodiscard]] bool force_output_disabled()
{
  const char * value = std::getenv("RESTOCKER_PAD_SPRING_OFF");
  return value != nullptr && value[0] != '\0' && std::string(value) != "0";
}

template<typename ComponentT>
[[nodiscard]] std::string front_or(const ComponentT * component)
{
  if (component == nullptr || component->Data().empty()) {
    return "none";
  }
  return std::to_string(component->Data().front());
}

// "absent" vs the value: a command component that does not exist and one holding zero are
// different facts for this bisect.
template<typename ComponentT>
[[nodiscard]] std::string command_state(
  const gz::sim::EntityComponentManager & ecm, const gz::sim::Entity entity)
{
  const auto * component = ecm.Component<ComponentT>(entity);
  if (component == nullptr) {
    return "absent";
  }
  return front_or(component);
}

// A wrench Physics consumed is cleared to zero; one still holding our y component was never read.
template<typename ComponentT>
[[nodiscard]] std::string wrench_state(const ComponentT * component)
{
  if (component == nullptr) {
    return "absent";
  }
  return std::to_string(component->Data().force().y());
}

struct PadSpringState
{
  gz::sim::Entity parent{gz::sim::kNullEntity};
  gz::sim::Entity child{gz::sim::kNullEntity};
  std::vector<gz::sim::Entity> collisions;
  double previous_m{0.0};
  bool primed{false};
};

}  // namespace

class PadSpringSystem final : public gz::sim::System,
  public gz::sim::ISystemPreUpdate,
  public gz::sim::ISystemPostUpdate,
  public gz::sim::ISystemReset
{
public:
  void PreUpdate(const gz::sim::UpdateInfo & info, gz::sim::EntityComponentManager & ecm) override
  {
    if (info.paused || info.dt.count() <= 0) {
      return;
    }
    const double dt = std::chrono::duration<double>(info.dt).count();
    ++calls_;
    // The reactions on a shared parent (four rows hang off one finger) are summed before they are
    // written: one wrench per link, so the last row must not overwrite the other three.
    reactions_.clear();
    bool trace = false;
    if (trace_enabled()) {
      // The first 20 steps are always traced: a minimal repro runs only a couple of hundred
      // iterations and the settle happens inside the first few milliseconds.
      trace = info.iterations <= 20;
      const auto now_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
      if (!trace && now_ns >= next_trace_ns_) {
        trace = true;
        next_trace_ns_ = now_ns + 100'000'000;
      }
    }

    // Resolved every step rather than once: the robot is spawned into a running world, so the pad
    // joints may appear after the world starts. `seen_` also drops the joints a reset deleted.
    seen_.clear();
    for (const auto entity : ecm.EntitiesByComponents(gz::sim::components::Joint())) {
      const auto * name = ecm.Component<gz::sim::components::Name>(entity);
      const auto * type = ecm.Component<gz::sim::components::JointType>(entity);
      if (name == nullptr || type == nullptr || !is_pad_row_joint(name->Data()) ||
        type->Data() != sdf::JointType::PRISMATIC)
      {
        continue;
      }
      seen_.insert(entity);
      if (springs_.find(entity) != springs_.end()) {
        continue;
      }
      PadSpringState state;
      if (resolve(ecm, entity, state)) {
        springs_.emplace(entity, state);
        const auto * axis = ecm.Component<gz::sim::components::JointAxis>(entity);
        std::cout << "PadSpringSystem: " << name->Data() << " -> "
                  << gz::sim::Link(state.child).Name(ecm).value_or("?") << " on "
                  << gz::sim::Link(state.parent).Name(ecm).value_or("?") << " k "
                  << (axis == nullptr ? 0.0 : axis->Data().SpringStiffness()) << " c "
                  << (axis == nullptr ? 0.0 : axis->Data().Damping()) << " ref "
                  << (axis == nullptr ? 0.0 : axis->Data().SpringReference()) << " effort "
                  << (axis == nullptr ? 0.0 : axis->Data().Effort()) << "\n";
        // The bisect asks which components the joint actually carries: a JointVelocityCmd of 0
        // would keep DART's actuator in SERVO at zero velocity (gz-sim #2729 / PR #2728 /
        // #2785), which holds a passive pad row against everything up to the effort limit.
        if (trace_enabled()) {
          std::cout << "PadSpringSystem components on " << name->Data() << ":";
          const auto types = ecm.ComponentTypes(entity);
          std::vector<std::string> names;
          for (const auto type : types) {
            names.push_back(gz::sim::components::Factory::Instance()->Name(type));
          }
          std::sort(names.begin(), names.end());
          for (const auto & component_name : names) {
            std::cout << " " << component_name;
          }
          std::cout << "\n" << "PadSpringSystem commands on " << name->Data()
                    << ": velocity_cmd " << command_state<gz::sim::components::JointVelocityCmd>(
            ecm, entity) << " position_reset "
                    << command_state<gz::sim::components::JointPositionReset>(ecm, entity)
                    << " velocity_reset "
                    << command_state<gz::sim::components::JointVelocityReset>(ecm, entity)
                    << " force_cmd "
                    << command_state<gz::sim::components::JointForceCmd>(ecm, entity) << "\n";
        }
        // Physics only fills a joint's position/velocity components when they already exist, so
        // create them here: the trace then reports the coordinate the ENGINE holds next to the one
        // taken from the link poses (Card 057's bisect: does DART's q equal the pose difference?).
        if (!ecm.Component<gz::sim::components::JointPosition>(entity)) {
          ecm.CreateComponent(entity, gz::sim::components::JointPosition());
        }
        if (!ecm.Component<gz::sim::components::JointVelocity>(entity)) {
          ecm.CreateComponent(entity, gz::sim::components::JointVelocity());
        }
        // The trace also lists every contact partner of the pad's own collision, so the load path
        // is visible: the block, or the finger body if the two collide.
        if (trace_enabled()) {
          for (const auto collision : state.collisions) {
            if (!ecm.Component<gz::sim::components::ContactSensorData>(collision)) {
              ecm.CreateComponent(collision, gz::sim::components::ContactSensorData());
            }
          }
        }
      }
      // Unresolved joints are retried next step: the links may not exist yet.
    }
    for (auto entry = springs_.begin(); entry != springs_.end(); ) {
      if (seen_.find(entry->first) == seen_.end()) {
        entry = springs_.erase(entry);
      } else {
        ++entry;
      }
    }

    for (auto & [entity, state] : springs_) {
      const auto * component = ecm.Component<gz::sim::components::JointAxis>(entity);
      const auto parent_pose = gz::sim::Link(state.parent).WorldPose(ecm);
      const auto child_pose = gz::sim::Link(state.child).WorldPose(ecm);
      if (component == nullptr || !parent_pose || !child_pose) {
        continue;
      }
      const sdf::JointAxis & axis = component->Data();
      // x: the child frame's displacement from the parent frame along the axis. The joint frame
      // sits on the parent link frame at rest, so the declared axis is the axis here.
      const gz::math::Vector3d axis_world = parent_pose->Rot().RotateVector(axis.Xyz());
      const double position_m = (child_pose->Pos() - parent_pose->Pos()).Dot(axis_world);
      // ẋ by finite difference: the first observation only establishes the reference sample.
      const double velocity_mps = state.primed ? (position_m - state.previous_m) / dt : 0.0;
      state.previous_m = position_m;
      state.primed = true;
      // The spring only. Damping is the joint's own <dynamics damping>, which the engine
      // integrates implicitly: applying it here from a one-step finite-difference velocity is
      // explicit and, at the damping the explicit spring needs (about critical), unstable.
      const double force = -axis.SpringStiffness() * (position_m - axis.SpringReference());
      // Equal-and-opposite pair on the two links: the child is pushed along the axis by the
      // spring and the parent — the finger — takes the reaction. Four rows share one parent, so
      // the reactions accumulate and are written once per link after this loop. With the output
      // disabled only the engine's own joint spring is left in the loop (the minimal repro).
      if (!force_output_disabled()) {
        const gz::math::Vector3d on_child = force * axis_world;
        set_wrench(ecm, state.child, on_child);
        reactions_[state.parent] -= on_child;
      }
      if (trace) {
        const auto * position = ecm.Component<gz::sim::components::JointPosition>(entity);
        const auto * velocity = ecm.Component<gz::sim::components::JointVelocity>(entity);
        const auto * joint_name = ecm.Component<gz::sim::components::Name>(entity);
        std::cout << "pad spring trace: "
                  << (joint_name == nullptr ? "?" : joint_name->Data()) << " iter "
                  << info.iterations << " k " << axis.SpringStiffness() << " x_pose "
                  << position_m << " v_pose " << velocity_mps << " F_cmd " << force
                  << " clipped " << (std::abs(force) > axis.Effort() ? "yes" : "no")
                  << " q_engine " << front_or(position) << " v_engine " << front_or(velocity)
                  << " y_parent " << parent_pose->Pos().Y() << " y_child "
                  << child_pose->Pos().Y() << " effort " << axis.Effort() << " vel_cmd "
                  << command_state<gz::sim::components::JointVelocityCmd>(ecm, entity) << "\n";
        const std::string label =
          joint_name == nullptr ? std::string("?") : joint_name->Data();
        for (const auto collision : state.collisions) {
          const auto * contacts =
            ecm.Component<gz::sim::components::ContactSensorData>(collision);
          if (contacts == nullptr) {
            continue;
          }
          for (const auto & contact : contacts->Data().contact()) {
            const bool first = contact.collision1().id() == collision;
            const auto other_entity =
              first ? contact.collision2().id() : contact.collision1().id();
            const auto * other = ecm.Component<gz::sim::components::Name>(other_entity);
            double force_n = 0.0;
            for (const auto & wrench : contact.wrench()) {
              const auto & body = first ? wrench.body_1_wrench() : wrench.body_2_wrench();
              force_n += gz::math::Vector3d(body.force().x(), body.force().y(), body.force().z())
                .Length();
            }
            std::cout << "pad spring contact: " << label << " with "
                      << (other == nullptr ? "?" : other->Data()) << " force " << force_n << "\n";
          }
        }
      }
    }
    for (const auto & [link, reaction] : reactions_) {
      set_wrench(ecm, link, reaction);
    }
  }

  // Physics zeroes JointForceCmd and clears every ExternalWorldWrenchCmd at the end of its own
  // Update, having consumed them. Whatever still holds our value here was therefore never read —
  // that is the bisect between "written" and "consumed" (gz-sim Physics.cc, Clear pending
  // commands).
  void PostUpdate(
    const gz::sim::UpdateInfo & info, const gz::sim::EntityComponentManager & ecm) override
  {
    if (info.paused || !trace_enabled() || info.iterations > 20) {
      return;
    }
    for (const auto & [entity, state] : springs_) {
      const auto * joint_name = ecm.Component<gz::sim::components::Name>(entity);
      const auto * force = ecm.Component<gz::sim::components::JointForceCmd>(entity);
      const auto * child_wrench =
        ecm.Component<gz::sim::components::ExternalWorldWrenchCmd>(state.child);
      const auto * parent_wrench =
        ecm.Component<gz::sim::components::ExternalWorldWrenchCmd>(state.parent);
      std::cout << "pad spring readback: "
                << (joint_name == nullptr ? "?" : joint_name->Data()) << " force_cmd "
                << (force == nullptr ? std::string("absent") : front_or(force))
                << " child_wrench_y " << wrench_state(child_wrench) << " parent_wrench_y "
                << wrench_state(parent_wrench) << "\n";
    }
  }

  void Reset(const gz::sim::UpdateInfo &, gz::sim::EntityComponentManager &) override
  {
    // Entities are recreated by a reset, so the cached links and samples are stale.
    springs_.clear();
    seen_.clear();
    reactions_.clear();
    calls_ = 0;
    next_trace_ns_ = 0;
  }

private:
  // gz-sim applies an ExternalWorldWrenchCmd at the link origin (force_offset is ignored), which
  // on these joints is the joint's own anchor, for one simulation step.
  static void set_wrench(
    gz::sim::EntityComponentManager & ecm, gz::sim::Entity link,
    const gz::math::Vector3d & force)
  {
    gz::msgs::Wrench wrench;
    wrench.mutable_force()->set_x(force.X());
    wrench.mutable_force()->set_y(force.Y());
    wrench.mutable_force()->set_z(force.Z());
    ecm.SetComponentData<gz::sim::components::ExternalWorldWrenchCmd>(link, wrench);
  }

  static bool resolve(
    const gz::sim::EntityComponentManager & ecm, gz::sim::Entity joint, PadSpringState & state)
  {
    const gz::sim::Joint padded_joint(joint);
    const auto model = padded_joint.ParentModel(ecm);
    const auto parent_name = padded_joint.ParentLinkName(ecm);
    const auto child_name = padded_joint.ChildLinkName(ecm);
    if (!model || !parent_name || !child_name) {
      return false;
    }
    const auto parent = model->LinkByName(ecm, *parent_name);
    const auto child = model->LinkByName(ecm, *child_name);
    if (parent == gz::sim::kNullEntity || child == gz::sim::kNullEntity) {
      return false;
    }
    state.parent = parent;
    state.child = child;
    state.collisions = gz::sim::Link(child).Collisions(ecm);
    return true;
  }

  std::unordered_map<gz::sim::Entity, PadSpringState> springs_;
  std::unordered_set<gz::sim::Entity> seen_;
  std::unordered_map<gz::sim::Entity, gz::math::Vector3d> reactions_;
  std::int64_t calls_{0};
  std::int64_t next_trace_ns_{0};
};

}  // namespace restocker_gazebo

GZ_ADD_PLUGIN(
  restocker_gazebo::PadSpringSystem, gz::sim::System, gz::sim::ISystemPreUpdate,
  gz::sim::ISystemPostUpdate, gz::sim::ISystemReset)

GZ_ADD_PLUGIN_ALIAS(restocker_gazebo::PadSpringSystem, "restocker_gazebo::PadSpringSystem")
