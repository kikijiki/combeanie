// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_perception/observation_identity.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace restocker_perception
{
namespace
{

using ObservationMessage = restocker_interfaces::msg::ObjectObservation;

[[nodiscard]] Eigen::Vector3d position_of(const ObservationMessage & observation)
{
  return {
    observation.pose.pose.position.x, observation.pose.pose.position.y,
    observation.pose.pose.position.z};
}

}  // namespace

ObservationIdentityAssigner::ObservationIdentityAssigner(ObservationIdentityConfig config)
: config_(std::move(config))
{
  if (!std::isfinite(config_.association_radius_m) || config_.association_radius_m <= 0.0 ||
    !std::isfinite(config_.ambiguity_margin_m) || config_.ambiguity_margin_m <= 0.0)
  {
    throw std::invalid_argument(
            "identity association radius and ambiguity margin must be finite and positive");
  }
  require_declared_positions_where_named_by_pose(config_.manifest, "object manifest");
  for (const auto & entry : config_.manifest) {
    if (stocked_count(entry.product_class) > 1) {
      instances_.push_back(
        Instance{entry.source_object_id, entry.product_class, *entry.declared_position, false});
    }
  }
}

std::size_t ObservationIdentityAssigner::stocked_count(std::uint8_t product_class) const
{
  return static_cast<std::size_t>(
    std::ranges::count_if(
      config_.manifest, [product_class](const ManifestEntry & candidate) {
        return candidate.product_class == product_class;
      }));
}

std::size_t ObservationIdentityAssigner::track_count() const noexcept
{
  return tracks_.size() +
         static_cast<std::size_t>(std::ranges::count_if(instances_, &Instance::bound));
}

ObservationIdentityAssigner::Track * ObservationIdentityAssigner::track_for(
  std::uint8_t product_class)
{
  const auto existing = std::ranges::find_if(
    tracks_, [product_class](const Track & candidate) {
      return candidate.product_class == product_class;
    });
  if (existing != tracks_.end()) {
    return &*existing;
  }
  // Only a category the inventory stocks exactly one of can be identified from appearance.
  if (stocked_count(product_class) != 1) {
    return nullptr;
  }
  const auto entry = std::ranges::find_if(
    config_.manifest, [product_class](const ManifestEntry & candidate) {
      return candidate.product_class == product_class;
    });
  tracks_.push_back(Track{entry->source_object_id, product_class, Eigen::Vector3d::Zero()});
  return &tracks_.back();
}

void ObservationIdentityAssigner::assign_by_pose(
  std::vector<ObservationMessage> & observations, std::vector<bool> & keep)
{
  constexpr std::size_t kNone = std::numeric_limits<std::size_t>::max();
  // The instance each detection proposes, decided against the references as they stood at the
  // start of this acquisition, so the order detections arrive in cannot change any name.
  std::vector<std::size_t> proposal(observations.size(), kNone);
  const auto refuse = [this](std::size_t & counter, const std::string & detail) {
    ++counter;
    if (refusals_.first_detail.empty()) {
      refusals_.first_detail = detail;
    }
  };
  for (std::size_t index = 0; index < observations.size(); ++index) {
    const std::uint8_t product_class = observations[index].product_class;
    if (stocked_count(product_class) < 2) {
      continue;
    }
    const Eigen::Vector3d position = position_of(observations[index]);
    std::size_t nearest = kNone;
    double nearest_distance = std::numeric_limits<double>::infinity();
    double runner_up_distance = std::numeric_limits<double>::infinity();
    for (std::size_t instance = 0; instance < instances_.size(); ++instance) {
      if (instances_[instance].product_class != product_class) {
        continue;
      }
      const double distance = (position - instances_[instance].reference).norm();
      if (distance < nearest_distance) {
        runner_up_distance = nearest_distance;
        nearest_distance = distance;
        nearest = instance;
      } else if (distance < runner_up_distance) {
        runner_up_distance = distance;
      }
    }
    char detail[160];
    std::snprintf(
      detail, sizeof(detail), "nearest '%s' at %.4f m, runner-up at %.4f m",
      nearest == kNone ? "" : instances_[nearest].source_object_id.c_str(), nearest_distance,
      runner_up_distance);
    if (nearest == kNone || !(nearest_distance <= config_.association_radius_m)) {
      refuse(refusals_.outside_every_gate, std::string("outside every gate: ") + detail);
      continue;
    }
    if (runner_up_distance - nearest_distance < config_.ambiguity_margin_m) {
      refuse(refusals_.ambiguous, std::string("ambiguous between instances: ") + detail);
      continue;
    }
    proposal[index] = nearest;
  }

  for (std::size_t index = 0; index < observations.size(); ++index) {
    if (proposal[index] == kNone) {
      continue;
    }
    const auto claims = std::ranges::count(proposal, proposal[index]);
    if (claims > 1) {
      refuse(
        refusals_.contested,
        "contested: " + std::to_string(claims) + " detections picked '" +
        instances_[proposal[index]].source_object_id + "'");
      continue;
    }
    Instance & instance = instances_[proposal[index]];
    const Eigen::Vector3d position = position_of(observations[index]);
    if (!instance.bound) {
      first_bindings_.push_back(
        FirstBinding{instance.source_object_id, (position - instance.reference).norm()});
    }
    instance.reference = position;
    instance.bound = true;
    observations[index].source_object_id = instance.source_object_id;
    keep[index] = true;
  }
}

std::size_t ObservationIdentityAssigner::assign(std::vector<ObservationMessage> & observations)
{
  refusals_ = IdentityRefusals{};
  first_bindings_.clear();
  std::vector<bool> keep(observations.size(), false);
  std::vector<bool> decided(observations.size(), false);

  assign_by_pose(observations, keep);

  for (std::size_t index = 0; index < observations.size(); ++index) {
    const std::uint8_t product_class = observations[index].product_class;
    const std::size_t stocked = stocked_count(product_class);
    if (stocked > 1 || decided[index]) {
      continue;
    }
    if (stocked == 0) {
      ++refusals_.not_stocked;
      continue;
    }
    Track * track = track_for(product_class);

    // The inventory has one object in this category: keep the observation nearest its last
    // position and drop the rest as surplus.
    std::size_t best = index;
    double best_distance = std::numeric_limits<double>::max();
    for (std::size_t candidate = index; candidate < observations.size(); ++candidate) {
      if (decided[candidate] || observations[candidate].product_class != product_class) {
        continue;
      }
      decided[candidate] = true;
      const double distance = (position_of(observations[candidate]) - track->position).norm();
      if (distance < best_distance) {
        best_distance = distance;
        best = candidate;
      }
    }
    keep[best] = true;
    track->position = position_of(observations[best]);
    observations[best].source_object_id = track->source_object_id;
  }
  for (std::size_t index = 0; index < observations.size(); ++index) {
    if (decided[index] && !keep[index]) {
      ++refusals_.surplus;
    }
  }

  std::size_t written = 0;
  std::size_t dropped = 0;
  for (std::size_t index = 0; index < observations.size(); ++index) {
    if (!keep[index]) {
      ++dropped;
      continue;
    }
    // Guarded: self-move-assignment need not leave the value intact.
    if (written != index) {
      observations[written] = std::move(observations[index]);
    }
    ++written;
  }
  observations.resize(written);
  return dropped;
}

}  // namespace restocker_perception
