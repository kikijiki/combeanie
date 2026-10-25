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

// Test-only stand-in for the arm's time-parameterized straight-line insert. The grasp fixtures
// used gz-sim's JointPositionController in velocity mode, whose law is dead-beat
// (target velocity = -error / dt, capped): it starts and stops the carry within one 1 ms step,
// about 50 m/s^2 at 0.05 m/s. Production retimes every Cartesian path with acceleration scaling
// 0.2 on joint limits of 0.5-2.0 rad/s^2, so its tool never sees that. Under a weld the step was
// harmless; a friction grip is a pendulum about the jaw axis and swings (measured, Card 057 run
// M2). This drives one prismatic joint along a trapezoidal velocity profile in simulated time
// toward the last position published on `topic`, with `max_velocity` and `max_acceleration`
// from the fixture.

#include <gz/msgs/double.pb.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

namespace restocker_gazebo
{

class TrapezoidalCarry final : public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  void Configure(
    const gz::sim::Entity & entity, const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager & ecm, gz::sim::EventManager &) override
  {
    const gz::sim::Model model(entity);
    joint_ = model.JointByName(ecm, sdf->Get<std::string>("joint_name"));
    max_velocity_ = sdf->Get<double>("max_velocity");
    max_acceleration_ = sdf->Get<double>("max_acceleration");
    target_ = sdf->Get<double>("initial_position", 0.0).first;
    node_.Subscribe(
      sdf->Get<std::string>("topic"), &TrapezoidalCarry::OnTarget, this);
  }

  void PreUpdate(const gz::sim::UpdateInfo & info, gz::sim::EntityComponentManager & ecm) override
  {
    if (joint_ == gz::sim::kNullEntity || info.paused) {
      return;
    }
    gz::sim::Joint joint(joint_);
    joint.EnablePositionCheck(ecm);
    joint.EnableVelocityCheck(ecm);
    const auto position = joint.Position(ecm);
    const double dt = std::chrono::duration<double>(info.dt).count();
    if (!position || position->empty() || dt <= 0.0) {
      return;
    }
    double target = 0.0;
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      target = target_;
    }
    const double remaining = target - position->front();
    // The fastest speed from which the remaining distance can still be stopped in, capped at
    // cruise; the commanded speed then moves toward it by at most max_acceleration * dt.
    const double stoppable = std::copysign(
      std::min(max_velocity_, std::sqrt(2.0 * max_acceleration_ * std::abs(remaining))),
      remaining);
    // Inside the last 10 um a proportional settle (time constant 20 ms) replaces the square-root
    // law, whose gain diverges at zero and chattered at the acceleration step (run M3).
    const double wanted = std::abs(remaining) < 1.0e-5 ? remaining / 0.02 : stoppable;
    const double step = max_acceleration_ * dt;
    velocity_ = std::clamp(wanted, velocity_ - step, velocity_ + step);
    ecm.SetComponentData<gz::sim::components::JointVelocityCmd>(joint_, {velocity_});
    // Every 50 ms of simulated time: the drive's own state beside the joint it drives, so a
    // tracking error between command and joint is visible in the run log.
    const std::int64_t now_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
    if (now_ns - last_print_ns_ >= 50'000'000) {
      last_print_ns_ = now_ns;
      const auto measured = joint.Velocity(ecm);
      std::cout << "carry drive receipt: t " << now_ns * 1.0e-9 << " target " << target
                << " q " << position->front() << " qd "
                << (measured && !measured->empty() ? measured->front() : std::nan(""))
                << " cmd_v " << velocity_ << std::endl;
    }
  }

private:
  void OnTarget(const gz::msgs::Double & message)
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    target_ = message.data();
  }

  gz::sim::Entity joint_{gz::sim::kNullEntity};
  double max_velocity_{0.0};
  double max_acceleration_{0.0};
  double velocity_{0.0};
  std::int64_t last_print_ns_{-50'000'000};
  std::mutex mutex_;
  double target_{0.0};
  gz::transport::Node node_;
};

}  // namespace restocker_gazebo

GZ_ADD_PLUGIN(
  restocker_gazebo::TrapezoidalCarry, gz::sim::System, gz::sim::ISystemConfigure,
  gz::sim::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(restocker_gazebo::TrapezoidalCarry, "restocker_gazebo::TrapezoidalCarry")
