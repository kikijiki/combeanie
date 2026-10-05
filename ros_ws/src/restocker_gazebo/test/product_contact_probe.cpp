// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

// Measure whether one product is solid to another product, in the real world file, using the
// real product description and nothing else (no arm, no attachment, no controllers).
//
//   * `stack`  - one product released above another. It must come to rest one product height
//                up. Regression signature: it fell through and settled at the same pose as the
//                product below.
//   * `column` - two upright products spawned closer together than their diameter, as the arm
//                creates when releasing into a lane that already holds one. They must push apart
//                to at least touching distance. Regression signature: they stayed put.
//   * `pinned` - the same overlap with the first product spawned `static`, as every stocked
//                product in the dense demo fixture is (restocker_gazebo/lane_columns.py). A
//                static body is a different narrowphase case, so it is measured separately: the
//                pinned product must not move and the placed one must be pushed clear.
//
// The failure cause was that DART's default narrowphase is ODE's, which has no cylinder-cylinder
// collider, and every product here is a cylinder. Products rest on the floor box correctly, which
// this probe also asserts so that a missing collision is told apart from an undispatched shape
// pair.

#include <gz/msgs/pose_v.pb.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <gz/transport/Node.hh>

namespace restocker_gazebo
{
namespace
{

using namespace std::chrono_literals;

constexpr char kPoseTopic[] = "/world/restocking/pose/info";

// The large bottle of restocker_description/config/product_collision_catalog.yaml, the product
// this probe spawns. Restated rather than read so the probe stays a single translation unit;
// test_product_contact_runtime.py spawns the same numbers and test_simulation_description.py
// keeps the catalog and the spawner in agreement.
constexpr double kProductHeightM = 0.290;
constexpr double kProductDiameterM = 0.090;
// A resting contact is allowed to sink into its neighbour by the solver's penetration allowance.
// DART's default is 1 mm; 4 mm allows the settling transient and stays well away from the
// 0.0 m and 0.05 m separations of the unfixed simulation.
constexpr double kPenetrationAllowanceM = 0.004;
constexpr double kRestingHeightM = kProductHeightM / 2.0;

struct Sample
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

class PoseWatcher
{
public:
  void Ingest(const gz::msgs::Pose_V & message)
  {
    const std::lock_guard<std::mutex> guard(mutex_);
    for (const auto & pose : message.pose()) {
      auto found = poses_.find(pose.name());
      if (found == poses_.end() && !Tracked(pose.name())) {
        continue;
      }
      poses_[pose.name()] = Sample{
        pose.position().x(), pose.position().y(), pose.position().z()};
    }
  }

  [[nodiscard]] std::optional<std::map<std::string, Sample>> Snapshot() const
  {
    const std::lock_guard<std::mutex> guard(mutex_);
    if (poses_.size() != kTracked.size()) {
      return std::nullopt;
    }
    return poses_;
  }

  [[nodiscard]] static const std::vector<std::string> & Names() {return kTracked;}

private:
  [[nodiscard]] static bool Tracked(const std::string & name)
  {
    return std::find(kTracked.begin(), kTracked.end(), name) != kTracked.end();
  }

  static const std::vector<std::string> kTracked;
  mutable std::mutex mutex_;
  std::map<std::string, Sample> poses_;
};

const std::vector<std::string> PoseWatcher::kTracked{
  "contact_stack_lower", "contact_stack_upper", "contact_column_near", "contact_column_far",
  "contact_pinned_static", "contact_pinned_placed"};

[[nodiscard]] double LargestMove(
  const std::map<std::string, Sample> & before, const std::map<std::string, Sample> & after)
{
  double largest = 0.0;
  for (const auto & [name, sample] : after) {
    const auto previous = before.find(name);
    if (previous == before.end()) {
      return std::numeric_limits<double>::infinity();
    }
    const double dx = sample.x - previous->second.x;
    const double dy = sample.y - previous->second.y;
    const double dz = sample.z - previous->second.z;
    largest = std::max(largest, std::sqrt(dx * dx + dy * dy + dz * dz));
  }
  return largest;
}

// Wait for all six products to exist and stop moving. Mid-fall, a product that never collides
// and one that has not landed yet look the same.
[[nodiscard]] std::optional<std::map<std::string, Sample>> AwaitSettled(const PoseWatcher & watcher)
{
  constexpr double kStillnessM = 1.0e-4;
  const auto deadline = std::chrono::steady_clock::now() + 60s;
  std::optional<std::map<std::string, Sample>> previous;
  int still = 0;
  while (std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(250ms);
    auto current = watcher.Snapshot();
    if (!current) {
      previous.reset();
      still = 0;
      continue;
    }
    if (previous && LargestMove(*previous, *current) < kStillnessM) {
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

int Fail(const std::string & detail)
{
  std::cerr << "product contact probe failed: " << detail << '\n';
  return 1;
}

int Run()
{
  PoseWatcher watcher;
  gz::transport::Node node;
  if (!node.Subscribe<gz::msgs::Pose_V>(
      kPoseTopic, [&watcher](const gz::msgs::Pose_V & message) {watcher.Ingest(message);}))
  {
    return Fail(std::string("could not subscribe to ") + kPoseTopic);
  }

  const auto settled = AwaitSettled(watcher);
  if (!settled) {
    return Fail("the six spawned products never all appeared and came to rest");
  }
  for (const auto & name : PoseWatcher::Names()) {
    const auto & sample = settled->at(name);
    std::cout << "settled " << name << " xyz " << sample.x << ' ' << sample.y << ' ' << sample.z
              << '\n';
  }

  int failures = 0;

  // Control: box-cylinder is a pair ODE does dispatch. A product resting at its own half-height
  // is solid to the world, which separates "collision never reached the engine" from "engine has
  // no collider for this shape pair".
  const double floor_rest = settled->at("contact_stack_lower").z;
  if (std::abs(floor_rest - kRestingHeightM) > kPenetrationAllowanceM) {
    failures += Fail(
      "the lower product is not resting on the floor: z " + std::to_string(floor_rest) +
      " against an expected " + std::to_string(kRestingHeightM) +
      ". The product collision is missing from the physics engine entirely, which is a "
      "different defect from the shape-pair one this probe measures.");
  }

  const double stack_gap =
    settled->at("contact_stack_upper").z - settled->at("contact_stack_lower").z;
  if (stack_gap < kProductHeightM - kPenetrationAllowanceM) {
    failures += Fail(
      "a product released above another did not come to rest on it: centres " +
      std::to_string(stack_gap) + " m apart against one product height of " +
      std::to_string(kProductHeightM) + " m. At a gap near zero the upper product fell through "
      "the lower one and settled at the identical pose.");
  }

  const double column_gap = std::abs(
    settled->at("contact_column_far").y - settled->at("contact_column_near").y);
  if (column_gap < kProductDiameterM - kPenetrationAllowanceM) {
    failures += Fail(
      "two products spawned inside one another did not push apart: centres " +
      std::to_string(column_gap) + " m apart against one product diameter of " +
      std::to_string(kProductDiameterM) + " m. A lane column of two consumes the lane depth of "
      "one while this holds.");
  }

  // A static body never moves, so the placed product must supply the whole separation. Spawn is
  // 0.050 m away and products are 0.090 m wide, so 0.040 m of overlap must be resolved.
  const double pinned_spawn_gap_m = 0.050;
  const double pinned_gap = std::abs(
    settled->at("contact_pinned_placed").y - settled->at("contact_pinned_static").y);
  const double pinned_static_shift = std::abs(
    settled->at("contact_pinned_static").y - 1.500);
  if (pinned_static_shift > kPenetrationAllowanceM) {
    failures += Fail(
      "the pinned product moved: y " + std::to_string(settled->at("contact_pinned_static").y) +
      " against the 1.500 it was pinned at. A static body cannot be pushed, so this measurement "
      "is not the one this pair exists to make.");
  }
  if (pinned_gap < kProductDiameterM - kPenetrationAllowanceM) {
    failures += Fail(
      "a product placed into a pinned one did not push clear of it: centres " +
      std::to_string(pinned_gap) + " m apart against one product diameter of " +
      std::to_string(kProductDiameterM) + " m, from a spawn separation of " +
      std::to_string(pinned_spawn_gap_m) + " m. While this holds, a product placed into a lane "
      "that a pinned column already occupies passes through it, and measured column growth is "
      "not a placement proof in any scenario built from pinned products.");
  }

  if (failures > 0) {
    return 1;
  }
  std::cout << "product contact probe succeeded: stack gap " << stack_gap << " m, column gap "
            << column_gap << " m, pinned gap " << pinned_gap << " m\n";
  return 0;
}

}  // namespace
}  // namespace restocker_gazebo

int main()
{
  return restocker_gazebo::Run();
}
