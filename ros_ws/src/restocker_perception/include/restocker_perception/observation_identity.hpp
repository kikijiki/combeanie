// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <restocker_interfaces/msg/object_observation.hpp>

#include "restocker_perception/frame_geometry.hpp"
#include "restocker_perception/object_manifest.hpp"

namespace restocker_perception
{

struct ObservationIdentityConfig
{
  // The inventory this cell is stocked from. It says how many of each category exist, which makes
  // identity decidable; see object_manifest.hpp for why identity comes from here and pose never
  // does.
  std::vector<ManifestEntry> manifest;
  // Card 063 (Milestone 10 section 6), categories stocked more than once only. A detection is
  // bound to a stocked instance only within this 3-D distance of the instance's reference
  // position. The shipped fixtures' closest same-category pitch is 0.085 m.
  double association_radius_m{0.035};
  // ...and only when every other instance of the category is at least this much further away.
  double ambiguity_margin_m{0.020};
};

// Why the detections dropped by the last assign() call were dropped.
struct IdentityRefusals
{
  // The inventory stocks nothing of this category.
  std::size_t not_stocked{0};
  // One-of-a-kind category: a second blob beside the one kept.
  std::size_t surplus{0};
  // Multi-instance category: no stocked instance within the association radius.
  std::size_t outside_every_gate{0};
  // Multi-instance category: the runner-up is not ambiguity_margin_m further than the nearest.
  std::size_t ambiguous{0};
  // Multi-instance category: two or more detections of one acquisition picked the same instance.
  std::size_t contested{0};
  // The first multi-instance refusal of the call, with its distances, for the log.
  std::string first_detail;

  [[nodiscard]] std::size_t total() const noexcept
  {
    return not_stocked + surplus + outside_every_gate + ambiguous + contested;
  }
};

// A multi-instance name bound for the first time by the last assign() call.
struct FirstBinding
{
  std::string source_object_id;
  double distance_m{0.0};
};

// Turns per-acquisition pose estimates into the identities the rest of the system knows.
//
// The rule is the inventory's, not a tracker's: a category the inventory stocks exactly one of has
// exactly one such object in the cell, so any well-formed detection of that category is that
// object, wherever it is. There is deliberately no association radius for such a category.
//
// An earlier version gated association on distance, and a product carried across the cell outran
// the gate between two 6 Hz acquisitions. Each time, a new identity was minted; the world state
// has no reaper, so each became a permanent object that was never observed again and went stale.
// The planning-scene projection fails as a whole when any tracked object is stale, so the entire
// MoveIt scene froze with a product still drawn where it no longer was, and the next approach
// planned into a product that had already moved. A distance gate answers "did this move too far to
// be the same thing" when the inventory already says "there is only one of these".
//
// A category the inventory stocks more than one of is not decidable from appearance, and the
// simulated attachment adapter resolves the chosen identity straight to a Gazebo model, so it is
// never guessed at. It is decided by pose instead, and the gate is safe there because nothing
// named that way is ever minted: a detection that leaves the gate is dropped, not renamed.
//
// Surplus detections (a second blob of a category already accounted for) are dropped, and no
// identity is ever minted: an object nothing will observe again is worse than a missing
// observation.
//
// Card 063 (Milestone 10 section 6) spells out the pose rule. Each stocked instance has a
// reference position: its declared stocking position until it is first bound, then the measured
// position last published under its name, so a product nudged a centimetre at a time by a
// neighbour's grasp stays itself. A detection takes an instance's name only when:
//   - that reference is within association_radius_m (else: outside every gate);
//   - every other instance of the category is at least ambiguity_margin_m further away, bound or
//     not (else: ambiguous);
//   - no other detection of the same acquisition picked that instance (else: every one of them is
//     contested).
// A refused detection is dropped, never given the nearest name. The published pose is always the
// measured one. The attach predicate still measures the named Gazebo model against the fingers,
// so a wrong name could only ever end as a refused attach, never as a latched neighbour.
class ObservationIdentityAssigner
{
public:
  explicit ObservationIdentityAssigner(ObservationIdentityConfig config);

  // Fills in source_object_id on the observations that have an identity, erases those that do not,
  // and returns how many were erased. Observations must all carry poses in one common frame, which
  // the caller has guaranteed by construction: they came out of one estimate() call against one
  // transform.
  std::size_t assign(std::vector<restocker_interfaces::msg::ObjectObservation> & observations);

  // Names that have been published at least once.
  [[nodiscard]] std::size_t track_count() const noexcept;

  [[nodiscard]] const IdentityRefusals & last_refusals() const noexcept {return refusals_;}
  [[nodiscard]] const std::vector<FirstBinding> & last_first_bindings() const noexcept
  {
    return first_bindings_;
  }

private:
  struct Track
  {
    std::string source_object_id;
    std::uint8_t product_class{0};
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  };

  struct Instance
  {
    std::string source_object_id;
    std::uint8_t product_class{0};
    Eigen::Vector3d reference{Eigen::Vector3d::Zero()};
    bool bound{false};
  };

  [[nodiscard]] Track * track_for(std::uint8_t product_class);
  [[nodiscard]] std::size_t stocked_count(std::uint8_t product_class) const;
  void assign_by_pose(
    std::vector<restocker_interfaces::msg::ObjectObservation> & observations,
    std::vector<bool> & keep);

  ObservationIdentityConfig config_;
  std::vector<Track> tracks_;
  std::vector<Instance> instances_;
  IdentityRefusals refusals_;
  std::vector<FirstBinding> first_bindings_;
};

}  // namespace restocker_perception
