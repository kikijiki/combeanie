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

// Test-only bench driver (Milestone 10 section 11 D10 item 3, Card 057): press one finger's pad
// row against the fixture's fixed block under a KNOWN constant effort, then compare the pad
// row's prismatic deflection and the contact wrench against the rubber this design claims.
//
// Three one-second phases (10, 20, 40 N) of constant effort on the finger joint, sampled at the
// end of each. With exactly one row loaded, the expected deflection is effort/111000 and the
// contact wrench should read the same effort. If the deflection is off by the factor the full
// grasp disagreed by, the springs are not engaging as specified; if the row sits at its limit the
// measurement is masked and that says so. Prints PAD_SPRING_PASS or PAD_SPRING_FAIL once, which
// is what the launch test waits for.

#include <gz/msgs/contacts.pb.h>
#include <gz/msgs/wrench.pb.h>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/ContactSensorData.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/JointAxis.hh>
#include <gz/sim/components/ParentEntity.hh>

namespace restocker_gazebo
{
namespace
{

constexpr char kModelName[] = "restocker";
constexpr char kFingerJoint[] = "left_finger_joint";
// Closing is -y and the joint axis is +y, so the commanded generalized force is negative.
constexpr std::array<double, 3> kEffortsN{10.0, 20.0, 40.0};
constexpr std::int64_t kStartNs = 500'000'000;
constexpr std::int64_t kPhaseNs = 1'000'000'000;
// Sample the last 150 ms of each phase, by which the finger/pad ring (26 ms to settle) has gone.
constexpr std::int64_t kSampleOffsetNs = 850'000'000;
constexpr double kExpectedStiffness = 111000.0;
constexpr double kStiffnessTolerance = 0.05;
constexpr double kWrenchTolerance = 0.05;
// Rows the block does not reach must not move at all: anything above this is load they carry.
constexpr double kUnloadedToleranceM = 1.0e-6;
// A row parked at its limit has masked the measurement: F/k is beyond the +/-2 mm travel.
constexpr double kLimitM = 0.002;

struct Sample
{
  double effort_n{0.0};
  double deflection_m{0.0};
  double contact_n{0.0};
  double stiffness{0.0};
  std::array<double, 4> row_deflection_m{};
  bool at_limit{false};
  bool loaded_elsewhere{false};
};

}  // namespace

class PadSpringProbe final : public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate,
  public gz::sim::ISystemPostUpdate
{
public:
  void Configure(
    const gz::sim::Entity & world, const std::shared_ptr<const sdf::Element> &,
    gz::sim::EntityComponentManager &, gz::sim::EventManager &) override
  {
    world_ = world;
  }

  void PreUpdate(const gz::sim::UpdateInfo & info, gz::sim::EntityComponentManager & ecm) override
  {
    if (!resolved_ && !resolve(ecm)) {
      return;
    }
    // Physics fills these only on entities that carry them, and only for the step after they
    // appear, so create them once, before the first force is applied.
    for (const auto collision : row0_collisions_) {
      if (!ecm.Component<gz::sim::components::ContactSensorData>(collision)) {
        ecm.CreateComponent(collision, gz::sim::components::ContactSensorData());
      }
    }
    ecm.SetComponentData<gz::sim::components::JointForceCmd>(
      finger_joint_, std::vector<double>{effort_at(info)});
  }

  void PostUpdate(
    const gz::sim::UpdateInfo & info, const gz::sim::EntityComponentManager & ecm) override
  {
    if (!resolved_ || info.paused || !std::isfinite(info.simTime.count())) {
      return;
    }
    const auto now_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
    const std::int64_t since = now_ns - kStartNs;
    if (since < 0) {
      return;
    }
    const std::size_t phase = static_cast<std::size_t>(since / kPhaseNs);
    if (phase >= kEffortsN.size() || (since % kPhaseNs) < kSampleOffsetNs) {
      return;
    }
    if (sampled_[phase]) {
      return;
    }
    sampled_[phase] = true;
    samples_[phase] = sample(ecm, kEffortsN[phase]);
    print_receipt(samples_[phase]);
    if (sampled_[0] && sampled_[1] && sampled_[2]) {
      verdict();
    }
  }

private:
  [[nodiscard]] double effort_at(const gz::sim::UpdateInfo & info) const
  {
    const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
    const std::int64_t since = now_ns - kStartNs;
    if (since < 0) {
      return 0.0;
    }
    const std::size_t phase = static_cast<std::size_t>(since / kPhaseNs);
    if (phase >= kEffortsN.size()) {
      return 0.0;
    }
    return -kEffortsN[phase];
  }

  bool resolve(const gz::sim::EntityComponentManager & ecm)
  {
    const auto model = ecm.EntityByComponents(
      gz::sim::components::Name(kModelName), gz::sim::components::ParentEntity(world_));
    if (model == gz::sim::kNullEntity) {
      return false;
    }
    const gz::sim::Model robot(model);
    finger_joint_ = robot.JointByName(ecm, kFingerJoint);
    finger_link_ = robot.LinkByName(ecm, "left_finger");
    if (finger_joint_ == gz::sim::kNullEntity || finger_link_ == gz::sim::kNullEntity) {
      return false;
    }
    for (std::size_t index = 0; index < row_links_.size(); ++index) {
      const auto suffix = std::to_string(index);
      row_links_[index] = robot.LinkByName(ecm, "left_finger_pad_row_" + suffix);
      row_joints_[index] = robot.JointByName(ecm, "left_finger_pad_row_" + suffix + "_joint");
      if (row_links_[index] == gz::sim::kNullEntity || row_joints_[index] == gz::sim::kNullEntity) {
        return false;
      }
    }
    // The bisect: what gz-sim itself holds for the pad joints' <dynamics>, read straight out of
    // the ECM component SdfEntityCreator wrote. If this says 111000/11.5/0 while the deflection
    // still sits on the limit, the value never reached DART; if it says 0, sdformat or gz-sim
    // dropped it before the physics engine was ever asked.
    for (std::size_t index = 0; index < row_joints_.size(); ++index) {
      const auto axis = ecm.Component<gz::sim::components::JointAxis>(row_joints_[index]);
      if (!axis) {
        std::cout << "pad spring axis row " << index << ": MISSING JointAxis component\n";
        continue;
      }
      std::cout << "pad spring axis row " << index << ": stiffness "
                << axis->Data().SpringStiffness() << " damping " << axis->Data().Damping()
                << " reference " << axis->Data().SpringReference() << "\n";
    }
    row0_collisions_ = gz::sim::Link(row_links_[0]).Collisions(ecm);
    resolved_ = !row0_collisions_.empty();
    return resolved_;
  }

  [[nodiscard]] Sample sample(const gz::sim::EntityComponentManager & ecm, double effort_n) const
  {
    Sample result;
    result.effort_n = std::abs(effort_n);
    // The deflection is the pad link's displacement relative to the finger link along the pad
    // normal. Read it from the link poses rather than the joint's position component: Physics
    // fills that component only for joints it drives, and a passive spring joint reads empty.
    const auto finger = gz::sim::Link(finger_link_).WorldPose(ecm);
    if (!finger) {
      return result;
    }
    for (std::size_t index = 0; index < row_links_.size(); ++index) {
      const auto row = gz::sim::Link(row_links_[index]).WorldPose(ecm);
      result.row_deflection_m[index] =
        row ? row->Pos().Y() - finger->Pos().Y() : std::numeric_limits<double>::quiet_NaN();
    }
    result.deflection_m = result.row_deflection_m[0];
    result.at_limit = std::abs(result.deflection_m) >= kLimitM - 1.0e-6;
    for (std::size_t index = 1; index < result.row_deflection_m.size(); ++index) {
      if (std::abs(result.row_deflection_m[index]) > kUnloadedToleranceM) {
        result.loaded_elsewhere = true;
      }
    }
    double total = 0.0;
    for (const auto collision : row0_collisions_) {
      const auto * contacts = ecm.Component<gz::sim::components::ContactSensorData>(collision);
      if (!contacts) {
        continue;
      }
      for (const auto & contact : contacts->Data().contact()) {
        const bool first = contact.collision1().id() == collision;
        for (const auto & wrench : contact.wrench()) {
          const auto & body = first ? wrench.body_1_wrench() : wrench.body_2_wrench();
          total += gz::math::Vector3d(body.force().x(), body.force().y(), body.force().z())
            .Length();
        }
      }
    }
    result.contact_n = total;
    result.stiffness =
      std::abs(result.deflection_m) > 0.0 ? result.effort_n / std::abs(result.deflection_m) : 0.0;
    return result;
  }

  void print_receipt(const Sample & measured) const
  {
    std::cout << "pad spring receipt: effort " << measured.effort_n << " N deflection "
              << measured.deflection_m << " m contact " << measured.contact_n << " N stiffness "
              << measured.stiffness << " N/m rows";
    for (const double row : measured.row_deflection_m) {
      std::cout << " " << row;
    }
    std::cout << (measured.at_limit ? " (row at its limit: measurement masked)" : "") << "\n";
  }

  void verdict() const
  {
    std::string reason;
    for (std::size_t index = 0; index < samples_.size(); ++index) {
      const Sample & measured = samples_[index];
      if (measured.at_limit) {
        reason += " row " + std::to_string(index) + " parked at its limit";
      }
      if (measured.loaded_elsewhere) {
        reason += " a row the block does not reach also moved";
      }
      if (std::abs(measured.stiffness / kExpectedStiffness - 1.0) > kStiffnessTolerance) {
        reason += " row 0 stiffness " + std::to_string(measured.stiffness) + " N/m vs " +
          std::to_string(kExpectedStiffness);
      }
      if (std::abs(measured.contact_n / measured.effort_n - 1.0) > kWrenchTolerance) {
        reason += " contact wrench " + std::to_string(measured.contact_n) + " N vs effort " +
          std::to_string(measured.effort_n);
      }
    }
    if (reason.empty()) {
      std::cout << "PAD_SPRING_PASS: deflection matches " << kExpectedStiffness
                << " N/m over one loaded row at 10, 20 and 40 N, and the contact wrench reads "
        "the applied effort\n";
      return;
    }
    std::cout << "PAD_SPRING_FAIL:" << reason << "\n";
  }

  gz::sim::Entity world_{gz::sim::kNullEntity};
  gz::sim::Entity finger_joint_{gz::sim::kNullEntity};
  gz::sim::Entity finger_link_{gz::sim::kNullEntity};
  std::array<gz::sim::Entity, 4> row_joints_{
    gz::sim::kNullEntity, gz::sim::kNullEntity, gz::sim::kNullEntity, gz::sim::kNullEntity};
  std::array<gz::sim::Entity, 4> row_links_{
    gz::sim::kNullEntity, gz::sim::kNullEntity, gz::sim::kNullEntity, gz::sim::kNullEntity};
  std::vector<gz::sim::Entity> row0_collisions_;
  std::array<bool, 3> sampled_{false, false, false};
  std::array<Sample, 3> samples_;
  bool resolved_{false};
};

}  // namespace restocker_gazebo

GZ_ADD_PLUGIN(
  restocker_gazebo::PadSpringProbe, gz::sim::System, gz::sim::ISystemConfigure,
  gz::sim::ISystemPreUpdate, gz::sim::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(restocker_gazebo::PadSpringProbe, "restocker_gazebo::PadSpringProbe")
