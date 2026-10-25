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

// Test-only instrument for the physical grip (Milestone 10 section 11 D7): how hard the jaws
// actually squeeze. Every 50 ms of simulated time it prints, per finger, the joint position,
// velocity and velocity command, the wrench the joint transmits (for a stalled finger, the servo
// force holding it against the product), and the summed contact force between that finger's
// collision and the carried product, plus the product's height. Loaded only by the grasp fixtures;
// it reads what Physics fills and changes nothing the attachment boundary sees.

#include <gz/msgs/contacts.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/wrench.pb.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/ContactSensorData.hh>
#include <gz/sim/components/JointTransmittedWrench.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/transport/Node.hh>

namespace restocker_gazebo
{
namespace
{

constexpr std::int64_t kPeriodNs = 50'000'000;
constexpr std::int64_t kPadPeriodNs = 10'000'000;
// Each finger's contact point with the carried product, for the SC-002 probe's own slip measure
// (spec M10 section 11 D5 amendment): independent of the attachment boundary's implementation.
constexpr char kPadTopic[] = "/release_contact/pad_contacts";

struct Finger
{
  std::string label;
  std::string joint_name;
  std::string link_name;
  gz::sim::Entity joint{gz::sim::kNullEntity};
  gz::sim::Entity link{gz::sim::kNullEntity};
  std::vector<gz::sim::Entity> collisions;
};

}  // namespace

class GripForceProbe final : public gz::sim::System,
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

  void PreUpdate(const gz::sim::UpdateInfo &, gz::sim::EntityComponentManager & ecm) override
  {
    if (!resolved_ && !resolve(ecm)) {
      return;
    }
    // Physics fills these only on entities that carry them, so create them once and keep them.
    for (auto & finger : fingers_) {
      if (!ecm.Component<gz::sim::components::JointTransmittedWrench>(finger.joint)) {
        ecm.CreateComponent(finger.joint, gz::sim::components::JointTransmittedWrench());
      }
      for (const auto collision : finger.collisions) {
        if (!ecm.Component<gz::sim::components::ContactSensorData>(collision)) {
          ecm.CreateComponent(collision, gz::sim::components::ContactSensorData());
        }
      }
      gz::sim::Joint(finger.joint).EnablePositionCheck(ecm);
      gz::sim::Joint(finger.joint).EnableVelocityCheck(ecm);
    }
  }

  void PostUpdate(
    const gz::sim::UpdateInfo & info, const gz::sim::EntityComponentManager & ecm) override
  {
    if (!resolved_ || info.paused) {
      return;
    }
    const std::int64_t now_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
    if (!last_pad_ns_ || now_ns - *last_pad_ns_ >= kPadPeriodNs) {
      last_pad_ns_ = now_ns;
      publish_pads(ecm);
    }
    if (last_print_ns_ && now_ns - *last_print_ns_ < kPeriodNs) {
      return;
    }
    last_print_ns_ = now_ns;
    std::ostringstream line;
    line << std::setprecision(6) << "grip force receipt: t " << now_ns * 1.0e-9;
    for (const auto & finger : fingers_) {
      const gz::sim::Joint joint(finger.joint);
      const auto position = joint.Position(ecm);
      const auto velocity = joint.Velocity(ecm);
      const auto * command = ecm.Component<gz::sim::components::JointVelocityCmd>(finger.joint);
      const auto * wrench =
        ecm.Component<gz::sim::components::JointTransmittedWrench>(finger.joint);
      line << " | " << finger.label << " q "
           << (position && !position->empty() ? position->front() : std::nan(""))
           << " qd " << (velocity && !velocity->empty() ? velocity->front() : std::nan(""))
           << " cmd_v "
           << (command && !command->Data().empty() ? command->Data().front() : std::nan(""));
      if (wrench) {
        const auto & force = wrench->Data().force();
        line << " joint_F " << force.x() << " " << force.y() << " " << force.z();
      } else {
        line << " joint_F none";
      }
      gz::math::Vector3d contact_force;
      // The squeeze itself: the sum of the contact forces on this finger, taken as magnitudes so
      // that body frames cannot be added together (spec M10 section 11 D10 item 1 — the grip has
      // to be read as 40 N, the design clamp, and not as the servo's position error).
      double grip_force = 0.0;
      std::size_t contact_points = 0U;
      // Per point: world height, world position along the approach, and the force along the jaw
      // axis (world x in the side-grasp fixture). Separated heights with shared load are what
      // resist the product spinning about the jaw axis (spec M10 section 11 D9).
      std::ostringstream points;
      for (const auto collision : finger.collisions) {
        const auto * contacts = ecm.Component<gz::sim::components::ContactSensorData>(collision);
        if (!contacts) {
          continue;
        }
        for (const auto & contact : contacts->Data().contact()) {
          const bool first = contact.collision1().id() == collision;
          for (const auto & joint_wrench : contact.wrench()) {
            const auto & body = first ? joint_wrench.body_1_wrench() : joint_wrench.body_2_wrench();
            const gz::math::Vector3d vector(
              body.force().x(), body.force().y(), body.force().z());
            contact_force += vector;
            grip_force += vector.Length();
          }
          contact_points += static_cast<std::size_t>(contact.position_size());
          for (int index = 0; index < contact.position_size(); ++index) {
            const auto & position = contact.position(index);
            double jaw_force = std::nan("");
            if (index < contact.wrench_size()) {
              const auto & body = first ? contact.wrench(index).body_1_wrench() :
                contact.wrench(index).body_2_wrench();
              jaw_force = body.force().x();
            }
            points << " [" << position.z() << " " << position.y() << " " << jaw_force << "]";
          }
        }
      }
      line << " contact_F " << contact_force.X() << " " << contact_force.Y() << " "
           << contact_force.Z() << " |" << contact_force.Length() << "| grip " << grip_force
           << " N points " << contact_points << " at" << points.str();
    }
    if (product_link_) {
      if (const auto pose = gz::sim::Link(*product_link_).WorldPose(ecm)) {
        line << " | product z " << pose->Pos().Z() << " y " << pose->Pos().Y();
      }
    }
    std::cout << line.str() << std::endl;
  }

private:
  // Force-weighted centroid of one finger's contact points with the carried product (the plain
  // centroid if no force was reported), published per finger by name; a finger out of contact is
  // left out of the message.
  void publish_pads(const gz::sim::EntityComponentManager & ecm)
  {
    if (!product_link_) {
      return;
    }
    gz::msgs::Pose_V message;
    for (const auto & finger : fingers_) {
      gz::math::Vector3d weighted;
      gz::math::Vector3d plain;
      double total = 0.0;
      std::size_t points = 0U;
      for (const auto collision : finger.collisions) {
        const auto * contacts = ecm.Component<gz::sim::components::ContactSensorData>(collision);
        if (!contacts) {
          continue;
        }
        for (const auto & contact : contacts->Data().contact()) {
          const bool first = contact.collision1().id() == collision;
          const auto other = first ? contact.collision2().id() : contact.collision1().id();
          if (ecm.ParentEntity(other) != *product_link_) {
            continue;
          }
          for (int index = 0; index < contact.position_size(); ++index) {
            const gz::math::Vector3d point(
              contact.position(index).x(), contact.position(index).y(),
              contact.position(index).z());
            plain += point;
            ++points;
            if (index < contact.wrench_size()) {
              const auto & force = first ? contact.wrench(index).body_1_wrench().force() :
                contact.wrench(index).body_2_wrench().force();
              const double magnitude =
                gz::math::Vector3d(force.x(), force.y(), force.z()).Length();
              weighted += magnitude * point;
              total += magnitude;
            }
          }
        }
      }
      if (points == 0U) {
        continue;
      }
      const gz::math::Vector3d centroid =
        total > 1.0e-9 ? weighted / total : plain / static_cast<double>(points);
      auto * pose = message.add_pose();
      pose->set_name(finger.label);
      pose->mutable_position()->set_x(centroid.X());
      pose->mutable_position()->set_y(centroid.Y());
      pose->mutable_position()->set_z(centroid.Z());
    }
    if (!pad_publisher_) {
      pad_publisher_ = node_.Advertise<gz::msgs::Pose_V>(kPadTopic);
    }
    pad_publisher_->Publish(message);
  }

  bool resolve(const gz::sim::EntityComponentManager & ecm)
  {
    const auto robot = ecm.EntityByComponents(
      gz::sim::components::Name("restocker"), gz::sim::components::ParentEntity(world_));
    if (robot == gz::sim::kNullEntity) {
      return false;
    }
    const gz::sim::Model model(robot);
    for (auto & finger : fingers_) {
      finger.joint = model.JointByName(ecm, finger.joint_name);
      finger.link = model.LinkByName(ecm, finger.link_name);
      if (finger.joint == gz::sim::kNullEntity || finger.link == gz::sim::kNullEntity) {
        return false;
      }
      // The pad is the finger link's own collisions plus its pad-row links (spec M10 section 11
      // D10 item 3): with compliant rows the product only ever touches the rows. The rows are
      // found by name through the model -- a link's parent entity is its model even when a joint
      // hangs it off another link.
      finger.collisions = gz::sim::Link(finger.link).Collisions(ecm);
      const auto finger_name = ecm.ComponentData<gz::sim::components::Name>(finger.link);
      if (finger_name) {
        const std::string pad_prefix = *finger_name + "_pad_row_";
        for (const auto & link : model.Links(ecm)) {
          const auto name = ecm.ComponentData<gz::sim::components::Name>(link);
          if (!name || name->rfind(pad_prefix, 0) != 0) {
            continue;
          }
          const auto pad_collisions = gz::sim::Link(link).Collisions(ecm);
          finger.collisions.insert(
            finger.collisions.end(), pad_collisions.begin(), pad_collisions.end());
        }
      }
    }
    const auto product = ecm.EntityByComponents(
      gz::sim::components::Name("stock_can_01"), gz::sim::components::ParentEntity(world_));
    if (product != gz::sim::kNullEntity) {
      const auto link = gz::sim::Model(product).LinkByName(ecm, "product_body");
      if (link != gz::sim::kNullEntity) {
        product_link_ = link;
      }
    }
    resolved_ = true;
    return true;
  }

  gz::sim::Entity world_{gz::sim::kNullEntity};
  bool resolved_{false};
  std::optional<gz::sim::Entity> product_link_;
  std::optional<std::int64_t> last_print_ns_;
  std::optional<std::int64_t> last_pad_ns_;
  gz::transport::Node node_;
  std::optional<gz::transport::Node::Publisher> pad_publisher_;
  std::vector<Finger> fingers_{
    Finger{"left", "left_finger_joint", "left_finger", gz::sim::kNullEntity, gz::sim::kNullEntity,
      {}},
    Finger{"right", "right_finger_joint", "right_finger", gz::sim::kNullEntity,
      gz::sim::kNullEntity, {}}};
};

}  // namespace restocker_gazebo

GZ_ADD_PLUGIN(
  restocker_gazebo::GripForceProbe, gz::sim::System, gz::sim::ISystemConfigure,
  gz::sim::ISystemPreUpdate, gz::sim::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(restocker_gazebo::GripForceProbe, "restocker_gazebo::GripForceProbe")
