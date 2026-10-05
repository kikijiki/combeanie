// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <Eigen/Core>
#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "restocker_perception/object_manifest.hpp"
#include "restocker_perception/observation_identity.hpp"

namespace
{

using restocker_perception::ManifestEntry;
using restocker_perception::ObservationIdentityAssigner;
using restocker_perception::ObservationIdentityConfig;
using restocker_perception::load_object_manifest;
using ObservationMessage = restocker_interfaces::msg::ObjectObservation;

[[nodiscard]] std::string baseline_scenario()
{
  const char * path = std::getenv("RESTOCKER_TEST_BASELINE_SCENARIO");
  EXPECT_NE(path, nullptr);
  return path == nullptr ? std::string{} : std::string{path};
}

[[nodiscard]] std::string dense_scenario()
{
  const char * path = std::getenv("RESTOCKER_TEST_DENSE_SCENARIO");
  EXPECT_NE(path, nullptr);
  return path == nullptr ? std::string{} : std::string{path};
}

[[nodiscard]] std::string write_temporary(const std::string & name, const std::string & content)
{
  const std::string path = std::string(testing::TempDir()) + "/" + name;
  std::ofstream file(path);
  file << content;
  return path;
}

TEST(ObjectManifest, ReadsTheStockedInventoryOutOfTheBaselineScenario)
{
  const auto manifest = load_object_manifest(baseline_scenario());
  ASSERT_EQ(manifest.size(), 3U);
  EXPECT_EQ(manifest[0].source_object_id, "sim:stock_can_01");
  EXPECT_EQ(manifest[0].product_class, ObservationMessage::PRODUCT_CLASS_CAN);
  EXPECT_EQ(manifest[1].source_object_id, "sim:stock_small_bottle_01");
  EXPECT_EQ(manifest[1].product_class, ObservationMessage::PRODUCT_CLASS_SMALL_BOTTLE);
  EXPECT_EQ(manifest[2].source_object_id, "sim:stock_large_bottle_01");
  EXPECT_EQ(manifest[2].product_class, ObservationMessage::PRODUCT_CLASS_LARGE_BOTTLE);
}

TEST(ObjectManifest, AOneOfAKindEntryNeverDependsOnItsDeclaredPose)
{
  // The scenario this reads also declares where every product was spawned. For a category stocked
  // exactly once that pose is never needed: any detection of the category is that object. This
  // asserts a document whose poses are absurd still loads and still names a can seen anywhere,
  // so nothing about one-of-a-kind identity has come to depend on the declared pose.
  const std::string path = write_temporary(
    "manifest_without_usable_poses.yaml",
    "products:\n"
    "  - source_object_id: inventory:can:A\n"
    "    product_class: can\n"
    "    spawn_pose: [9999.0, 9999.0, 9999.0, 0.0, 0.0, 0.0]\n");
  const auto manifest = load_object_manifest(path);
  ASSERT_EQ(manifest.size(), 1U);
  EXPECT_EQ(manifest[0].source_object_id, "inventory:can:A");
  EXPECT_EQ(manifest[0].product_class, ObservationMessage::PRODUCT_CLASS_CAN);

  ObservationIdentityConfig config;
  config.manifest = manifest;
  ObservationIdentityAssigner assigner(config);
  ObservationMessage can;
  can.product_class = ObservationMessage::PRODUCT_CLASS_CAN;
  can.pose.pose.position.x = 0.3;
  std::vector<ObservationMessage> observations{can};
  EXPECT_EQ(assigner.assign(observations), 0U);
  ASSERT_EQ(observations.size(), 1U);
  EXPECT_EQ(observations[0].source_object_id, "inventory:can:A");
}

TEST(ObjectManifest, ReadsTheDeclaredStockingPositionOfEveryProduct)
{
  // Card 063: the dense fixture stocks several products per category, and the declared stocking
  // position is what chooses between their names. It is read for naming only (see
  // observation_identity.hpp); no observation ever carries it.
  const auto manifest = load_object_manifest(dense_scenario());
  ASSERT_EQ(manifest.size(), 62U);
  const auto can = std::ranges::find_if(
    manifest, [](const ManifestEntry & entry) {
      return entry.source_object_id == "sim:stock_can_01";
    });
  ASSERT_NE(can, manifest.end());
  ASSERT_TRUE(can->declared_position.has_value());
  EXPECT_DOUBLE_EQ(can->declared_position->x(), -0.15);
  EXPECT_DOUBLE_EQ(can->declared_position->y(), -0.68);
  EXPECT_DOUBLE_EQ(can->declared_position->z(), 0.631);
}

TEST(ObjectManifest, RefusesAMultiInstanceEntryWithoutAUsableDeclaredPose)
{
  // Two cans and no pose for one of them: nothing could choose between their names, so the
  // document is refused at start-up rather than silently identifying nothing.
  EXPECT_THROW(
    {
      [[maybe_unused]] const auto loaded = load_object_manifest(
        write_temporary(
          "two_cans_one_pose.yaml",
          "products:\n"
          "  - source_object_id: a\n    product_class: can\n"
          "    spawn_pose: [0.0, 0.0, 0.6, 0.0, 0.0, 0.0]\n"
          "  - source_object_id: b\n    product_class: can\n"));
    },
    std::invalid_argument);
  EXPECT_THROW(
    {
      [[maybe_unused]] const auto loaded = load_object_manifest(
        write_temporary(
          "two_cans_nan_pose.yaml",
          "products:\n"
          "  - source_object_id: a\n    product_class: can\n"
          "    spawn_pose: [0.0, 0.0, 0.6, 0.0, 0.0, 0.0]\n"
          "  - source_object_id: b\n    product_class: can\n"
          "    spawn_pose: [.nan, 0.0, 0.6, 0.0, 0.0, 0.0]\n"));
    },
    std::invalid_argument);
  // The assigner holds the same line for a manifest built in code.
  ObservationIdentityConfig config;
  config.manifest = {
    ManifestEntry{ObservationMessage::PRODUCT_CLASS_CAN, "a", Eigen::Vector3d(0.0, 0.0, 0.6)},
    ManifestEntry{ObservationMessage::PRODUCT_CLASS_CAN, "b", std::nullopt}};
  EXPECT_THROW(ObservationIdentityAssigner{config}, std::invalid_argument);
}

TEST(ObjectManifest, RefusesADocumentItCannotTrust)
{
  EXPECT_THROW(
    {[[maybe_unused]] const auto loaded = load_object_manifest("");},
    std::invalid_argument);
  EXPECT_THROW(
    {
      [[maybe_unused]] const auto loaded =
      load_object_manifest("/nonexistent/scenario.yaml");
    },
    std::invalid_argument);
  EXPECT_THROW(
    {
      [[maybe_unused]] const auto loaded =
      load_object_manifest(write_temporary("no_products.yaml", "products: []\n"));
    },
    std::invalid_argument);
  EXPECT_THROW(
    {
      [[maybe_unused]] const auto loaded = load_object_manifest(
        write_temporary(
          "unknown_class.yaml",
          "products:\n  - source_object_id: a\n    product_class: crate\n"));
    },
    std::invalid_argument);
  EXPECT_THROW(
    {
      [[maybe_unused]] const auto loaded = load_object_manifest(
        write_temporary(
          "duplicate_identity.yaml",
          "products:\n  - source_object_id: a\n    product_class: can\n"
          "  - source_object_id: a\n    product_class: can\n"));
    },
    std::invalid_argument);
}

TEST(ObservationIdentityAssigner, GivesEachProductTheIdentityTheInventoryKnowsItBy)
{
  ObservationIdentityConfig config;
  config.manifest = load_object_manifest(baseline_scenario());
  ObservationIdentityAssigner assigner(config);

  ObservationMessage can;
  can.product_class = ObservationMessage::PRODUCT_CLASS_CAN;
  ObservationMessage bottle;
  bottle.product_class = ObservationMessage::PRODUCT_CLASS_LARGE_BOTTLE;
  bottle.pose.pose.position.x = 0.4;
  std::vector<ObservationMessage> observations{can, bottle};
  EXPECT_EQ(assigner.assign(observations), 0U);
  ASSERT_EQ(observations.size(), 2U);
  EXPECT_EQ(observations[0].source_object_id, "sim:stock_can_01");
  EXPECT_EQ(observations[1].source_object_id, "sim:stock_large_bottle_01");
}

TEST(ObservationIdentityAssigner, KeepsAnIdentityAcrossAnyDistanceTheProductIsCarried)
{
  // The regression this exists for. A gated association mints a new identity when a carried
  // product outruns the gate between two acquisitions, and because the world state has no reaper
  // that identity becomes an object nothing ever observes again, which goes stale, fails the
  // whole planning-scene projection, and leaves MoveIt planning against a product drawn where it
  // no longer is. The inventory stocks one can, so a can seen anywhere is that can.
  ObservationIdentityConfig config;
  config.manifest = load_object_manifest(baseline_scenario());
  ObservationIdentityAssigner assigner(config);

  ObservationMessage can;
  can.product_class = ObservationMessage::PRODUCT_CLASS_CAN;
  can.pose.pose.position.x = -0.42;
  can.pose.pose.position.y = -0.80;
  std::vector<ObservationMessage> tray{can};
  assigner.assign(tray);
  ASSERT_EQ(tray.size(), 1U);
  EXPECT_EQ(tray[0].source_object_id, "sim:stock_can_01");

  // The far end of the cell, two metres away and well past any plausible per-frame motion.
  ObservationMessage carried = can;
  carried.pose.pose.position.x = -1.0;
  carried.pose.pose.position.y = 1.33;
  carried.pose.pose.position.z = 0.90;
  std::vector<ObservationMessage> lane{carried};
  EXPECT_EQ(assigner.assign(lane), 0U);
  ASSERT_EQ(lane.size(), 1U);
  EXPECT_EQ(lane[0].source_object_id, "sim:stock_can_01");
  EXPECT_EQ(assigner.track_count(), 1U);
}

TEST(ObservationIdentityAssigner, DropsASurplusDetectionRatherThanInventOneMoreObject)
{
  // One can is stocked; two are seen. The nearest to where the can was last seen is kept and the
  // other is dropped, because an object nothing will observe again is worse than an observation
  // that was never made.
  ObservationIdentityConfig config;
  config.manifest = {
    ManifestEntry{ObservationMessage::PRODUCT_CLASS_CAN, "sim:stock_can_01"}};
  ObservationIdentityAssigner assigner(config);

  ObservationMessage near;
  near.product_class = ObservationMessage::PRODUCT_CLASS_CAN;
  near.pose.pose.position.x = 0.05;
  ObservationMessage far = near;
  far.pose.pose.position.x = 0.9;
  std::vector<ObservationMessage> observations{far, near};
  EXPECT_EQ(assigner.assign(observations), 1U);
  ASSERT_EQ(observations.size(), 1U);
  EXPECT_EQ(observations[0].source_object_id, "sim:stock_can_01");
  EXPECT_DOUBLE_EQ(observations[0].pose.pose.position.x, 0.05);
  EXPECT_EQ(assigner.track_count(), 1U);
}

TEST(ObservationIdentityAssigner, KeepsOnlyTheNearestOfTwoOneOfAKindDetectionsInEitherOrder)
{
  // Card 063 review (N1): before Card 063 a surplus blob that came *after* the nearest one was
  // also published under the same name, because the loop skipped only kept indices. Either
  // order now keeps exactly the nearest and drops the other.
  ObservationIdentityConfig config;
  config.manifest = {ManifestEntry{ObservationMessage::PRODUCT_CLASS_CAN, "sim:stock_can_01"}};
  for (const bool nearest_first : {true, false}) {
    ObservationIdentityAssigner assigner(config);
    ObservationMessage near;
    near.product_class = ObservationMessage::PRODUCT_CLASS_CAN;
    near.pose.pose.position.x = 0.05;
    ObservationMessage far = near;
    far.pose.pose.position.x = 0.9;
    std::vector<ObservationMessage> observations =
      nearest_first ? std::vector<ObservationMessage>{near, far} :
    std::vector<ObservationMessage>{far, near};
    SCOPED_TRACE(nearest_first ? "nearest first" : "nearest last");
    EXPECT_EQ(assigner.assign(observations), 1U);
    ASSERT_EQ(observations.size(), 1U);
    EXPECT_EQ(observations[0].source_object_id, "sim:stock_can_01");
    EXPECT_DOUBLE_EQ(observations[0].pose.pose.position.x, 0.05);
    EXPECT_EQ(assigner.last_refusals().surplus, 1U);
  }
}

// Card 063 (Milestone 10 section 6): a category stocked more than once is named by pose. Each
// stocked instance has a reference position, its declared stocking position until first bound and
// its last measured position after, and a detection is bound only inside the gate, unambiguously
// and uncontested.

[[nodiscard]] ObservationMessage detection_at(
  std::uint8_t product_class, const Eigen::Vector3d & position)
{
  ObservationMessage observation;
  observation.product_class = product_class;
  observation.pose.pose.position.x = position.x();
  observation.pose.pose.position.y = position.y();
  observation.pose.pose.position.z = position.z();
  return observation;
}

[[nodiscard]] ObservationIdentityConfig two_cans(double separation_m)
{
  ObservationIdentityConfig config;
  config.manifest = {
    ManifestEntry{ObservationMessage::PRODUCT_CLASS_CAN, "can:A", Eigen::Vector3d(0.0, 0.0, 0.6)},
    ManifestEntry{
      ObservationMessage::PRODUCT_CLASS_CAN, "can:B", Eigen::Vector3d(separation_m, 0.0, 0.6)},
    ManifestEntry{
      ObservationMessage::PRODUCT_CLASS_LARGE_BOTTLE, "bottle:only",
      Eigen::Vector3d(0.5, 0.5, 0.7)}};
  return config;
}

TEST(ObservationIdentityAssigner, NamesEveryDenseTrayInstanceByItsOwnStockingPosition)
{
  // The case this card exists for: 9 cans, 9 small bottles and 7 large bottles on the dense tray,
  // all seen in one acquisition with a few millimetres of measurement error, each named as itself.
  ObservationIdentityConfig config;
  config.manifest = load_object_manifest(dense_scenario());
  ObservationIdentityAssigner assigner(config);

  const Eigen::Vector3d error(0.004, -0.003, 0.002);
  std::vector<ObservationMessage> observations;
  std::vector<std::string> expected;
  for (const auto & entry : config.manifest) {
    if (entry.source_object_id.rfind("sim:stock_", 0) != 0) {
      continue;
    }
    observations.push_back(detection_at(entry.product_class, *entry.declared_position + error));
    expected.push_back(entry.source_object_id);
  }
  ASSERT_EQ(observations.size(), 25U);
  EXPECT_EQ(assigner.assign(observations), 0U);
  ASSERT_EQ(observations.size(), expected.size());
  for (std::size_t index = 0; index < expected.size(); ++index) {
    EXPECT_EQ(observations[index].source_object_id, expected[index]);
  }
  EXPECT_EQ(assigner.track_count(), 25U);
}

TEST(ObservationIdentityAssigner, FollowsANudgedProductPastItsDeclaredPosition)
{
  // A neighbour's grasp can nudge a product a centimetre at a time. The reference follows each
  // binding, so a product 5 cm from where it was stocked is still itself.
  ObservationIdentityAssigner assigner(two_cans(0.20));
  for (const double x : {0.01, 0.03, 0.05}) {
    std::vector<ObservationMessage> observations{
      detection_at(ObservationMessage::PRODUCT_CLASS_CAN, Eigen::Vector3d(x, 0.0, 0.6))};
    EXPECT_EQ(assigner.assign(observations), 0U);
    ASSERT_EQ(observations.size(), 1U);
    EXPECT_EQ(observations[0].source_object_id, "can:A");
    EXPECT_DOUBLE_EQ(observations[0].pose.pose.position.x, x);
  }
}

TEST(ObservationIdentityAssigner, RefusesADetectionOutsideEveryGate)
{
  // Nothing stocked within the gate: dropped, never minted, never given the nearest name.
  ObservationIdentityAssigner assigner(two_cans(0.20));
  std::vector<ObservationMessage> observations{
    detection_at(ObservationMessage::PRODUCT_CLASS_CAN, Eigen::Vector3d(0.0, 0.06, 0.6))};
  EXPECT_EQ(assigner.assign(observations), 1U);
  EXPECT_TRUE(observations.empty());
  EXPECT_EQ(assigner.last_refusals().outside_every_gate, 1U);
  EXPECT_EQ(assigner.track_count(), 0U);
}

TEST(ObservationIdentityAssigner, RefusesADetectionAmbiguousBetweenTwoInstances)
{
  // Two cans 6 cm apart and a detection nearly between them: inside A's gate, but B is not
  // clearly further, so neither name is claimed.
  ObservationIdentityAssigner assigner(two_cans(0.06));
  std::vector<ObservationMessage> observations{
    detection_at(ObservationMessage::PRODUCT_CLASS_CAN, Eigen::Vector3d(0.028, 0.0, 0.6))};
  EXPECT_EQ(assigner.assign(observations), 1U);
  EXPECT_TRUE(observations.empty());
  EXPECT_EQ(assigner.last_refusals().ambiguous, 1U);

  // Clearly nearer A: bound.
  std::vector<ObservationMessage> clear{
    detection_at(ObservationMessage::PRODUCT_CLASS_CAN, Eigen::Vector3d(0.005, 0.0, 0.6))};
  EXPECT_EQ(assigner.assign(clear), 0U);
  ASSERT_EQ(clear.size(), 1U);
  EXPECT_EQ(clear[0].source_object_id, "can:A");
}

TEST(ObservationIdentityAssigner, RefusesEveryDetectionContestingOneInstance)
{
  // Two blobs inside A's gate in one acquisition: one product split in two, or two products where
  // one was stocked. Neither is kept as "nearest"; both are refused.
  ObservationIdentityAssigner assigner(two_cans(0.20));
  std::vector<ObservationMessage> observations{
    detection_at(ObservationMessage::PRODUCT_CLASS_CAN, Eigen::Vector3d(0.004, 0.0, 0.6)),
    detection_at(ObservationMessage::PRODUCT_CLASS_CAN, Eigen::Vector3d(-0.01, 0.0, 0.6)),
    detection_at(ObservationMessage::PRODUCT_CLASS_CAN, Eigen::Vector3d(0.20, 0.0, 0.6))};
  EXPECT_EQ(assigner.assign(observations), 2U);
  ASSERT_EQ(observations.size(), 1U);
  EXPECT_EQ(observations[0].source_object_id, "can:B");
  EXPECT_EQ(assigner.last_refusals().contested, 2U);
}

TEST(ObservationIdentityAssigner, RefusesALiftedProductPassingOverItsNeighbour)
{
  // A product lifted in the jaws over the neighbour's slot is not the neighbour: 15 cm above
  // B's reference is outside every gate.
  ObservationIdentityAssigner assigner(two_cans(0.085));
  std::vector<ObservationMessage> lifted{
    detection_at(ObservationMessage::PRODUCT_CLASS_CAN, Eigen::Vector3d(0.085, 0.0, 0.75))};
  EXPECT_EQ(assigner.assign(lifted), 1U);
  EXPECT_TRUE(lifted.empty());
  EXPECT_EQ(assigner.last_refusals().outside_every_gate, 1U);
}

TEST(ObservationIdentityAssigner, LeavesTheOneOfAKindRuleUntouchedBesideMultiInstanceClasses)
{
  // The large bottle is stocked once: seen two metres from its declared pose, it is still itself.
  ObservationIdentityAssigner assigner(two_cans(0.20));
  std::vector<ObservationMessage> observations{
    detection_at(ObservationMessage::PRODUCT_CLASS_LARGE_BOTTLE, Eigen::Vector3d(-1.5, 1.3, 0.9)),
    detection_at(ObservationMessage::PRODUCT_CLASS_CAN, Eigen::Vector3d(0.2, 0.001, 0.6))};
  EXPECT_EQ(assigner.assign(observations), 0U);
  ASSERT_EQ(observations.size(), 2U);
  EXPECT_EQ(observations[0].source_object_id, "bottle:only");
  EXPECT_EQ(observations[1].source_object_id, "can:B");
}

TEST(ObservationIdentityAssigner, RefusesAClassTheInventoryDoesNotStock)
{
  ObservationIdentityAssigner assigner(two_cans(0.20));
  std::vector<ObservationMessage> observations{
    detection_at(ObservationMessage::PRODUCT_CLASS_SMALL_BOTTLE, Eigen::Vector3d::Zero())};
  EXPECT_EQ(assigner.assign(observations), 1U);
  EXPECT_EQ(assigner.last_refusals().not_stocked, 1U);
}

TEST(ObservationIdentityAssigner, RefusesAnUnusableGate)
{
  auto config = two_cans(0.20);
  config.association_radius_m = 0.0;
  EXPECT_THROW(ObservationIdentityAssigner{config}, std::invalid_argument);
  config = two_cans(0.20);
  config.ambiguity_margin_m = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(ObservationIdentityAssigner{config}, std::invalid_argument);
}

}  // namespace
