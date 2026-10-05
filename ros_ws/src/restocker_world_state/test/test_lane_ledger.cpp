// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include "restocker_world_state/lane_config.hpp"
#include "restocker_world_state/world_state.hpp"

namespace restocker_world_state
{
namespace
{
using namespace std::chrono_literals;

[[nodiscard]] rclcpp::Time ros_time(std::int64_t nanoseconds)
{
  return rclcpp::Time(nanoseconds, RCL_ROS_TIME);
}

// The catalogued can, same arithmetic as `load_product_lane_profiles` on the shipped catalog and
// survey: one diameter foreshortened by the 4-degree bed (cos(4 deg) = 0.99756405025982420).
inline constexpr double kCanPitchM = 2.0 * 0.033 * 0.99756405025982420;
inline constexpr double kCanRearDepthM = 2.0 * 0.033 + 0.045;
inline constexpr double kLaneDepthM = 0.85;

[[nodiscard]] WorldStateConfig config()
{
  WorldStateConfig value;
  value.maximum_observation_age = 2s;
  value.lane_evidence_validity = 60s;
  value.maximum_future_skew = 100ms;
  value.event_capacity = 64;
  value.product_lane_profiles = {
    ProductLaneProfile{ProductClass::Can, std::nullopt, kCanRearDepthM, kCanPitchM}};
  return value;
}

[[nodiscard]] ObjectObservation can_observation(
  const std::string & source_id,
  std::int64_t timestamp_ns)
{
  ObjectObservation value;
  value.source_object_id = source_id;
  value.frame_id = "world";
  value.product_class = ProductClass::Can;
  value.sku = "SIM-CAN-STD";
  value.pose_in_world = Eigen::Isometry3d::Identity();
  value.pose_covariance = PoseCovariance::Identity() * 1.0e-6;
  value.orientation = ObjectOrientation::Upright;
  value.observation_time = ros_time(timestamp_ns);
  return value;
}

[[nodiscard]] Eigen::Isometry3d test_grasp()
{
  Eigen::Isometry3d grasp = Eigen::Isometry3d::Identity();
  grasp.translation() = Eigen::Vector3d(0.004, -0.002, 0.031);
  return grasp;
}

// Configures lane_01 (can, target 6) and surveys it empty so the first placement starts from a
// reconciled, trusted, empty ledger.
[[nodiscard]] bool configure_and_survey_empty(WorldStateStore & store)
{
  const auto configured = store.configure_lane(
    LaneDefinition{LaneId{"lane_01"}, ProductClass::Can, "SIM-CAN-STD", kLaneDepthM, 6},
    ros_time(9'000'000'000));
  EXPECT_TRUE(configured) << (configured ? "" : configured.error().detail);
  if (!configured) {
    return false;
  }
  const auto surveyed = store.update_lane(
    LaneObservation{LaneId{"lane_01"}, {}, kLaneDepthM, false, ros_time(9'100'000'000)},
    ros_time(10'000'000'000), configured.value().revision);
  EXPECT_TRUE(surveyed) << (surveyed ? "" : surveyed.error().detail);
  return static_cast<bool>(surveyed);
}

// One tray product through the legacy attach/detach pair (no reservation): it comes to rest in
// lane_01's contents, exactly as a reserved placement leaves it.
[[nodiscard]] std::optional<ObjectId> place_can(
  WorldStateStore & store,
  const std::string & source_id, std::int64_t when_ns)
{
  const auto observed = store.observe_object(
    can_observation(source_id, when_ns - 100'000'000), ros_time(10'000'000'000));
  EXPECT_TRUE(observed) << (observed ? "" : observed.error().detail);
  if (!observed) {
    return std::nullopt;
  }
  auto snapshot = store.snapshot();
  const auto attached = store.commit_attachment(
    observed.value().object_id, test_grasp(), ros_time(when_ns),
    AttachPreconditions{observed.value().revision, snapshot.robot.revision, std::nullopt});
  EXPECT_TRUE(attached) << (attached ? "" : attached.error().detail);
  if (!attached) {
    return std::nullopt;
  }
  snapshot = store.snapshot();
  const auto detached = store.commit_detachment(
    observed.value().object_id, LaneId{"lane_01"}, ros_time(when_ns + 10'000'000),
    DetachPreconditions{
        snapshot.objects.at(observed.value().object_id).revision, snapshot.robot.revision,
        snapshot.lanes.at(LaneId{"lane_01"}).revision});
  EXPECT_TRUE(detached) << (detached ? "" : detached.error().detail);
  if (!detached) {
    return std::nullopt;
  }
  return observed.value().object_id;
}

// One accepted survey at a stated column length (measured from the lane's rear), i.e. available
// depth = depth - column_length.
[[nodiscard]] Result<MutationReceipt> survey_column(
  WorldStateStore & store,
  double column_length_m, std::int64_t observation_ns,
  std::vector<std::string> observed_sources = {})
{
  const auto snapshot = store.snapshot();
  return store.update_lane(
    LaneObservation{
        LaneId{"lane_01"}, std::move(observed_sources), kLaneDepthM - column_length_m, false,
        ros_time(observation_ns)},
    ros_time(10'000'000'000), snapshot.lanes.at(LaneId{"lane_01"}).revision);
}

[[nodiscard]] const ShelfLane & lane_01(const WorldStateSnapshot & snapshot)
{
  return snapshot.lanes.at(LaneId{"lane_01"});
}

// Reservations refuse without admitted telemetry; the ledger tests that reserve take it first.
[[nodiscard]] bool observe_telemetry(WorldStateStore & store)
{
  const auto telemetry = store.observe_robot_telemetry(
    RobotTelemetryObservation{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {}, 0.0, 0.0,
      ros_time(9'200'000'000), {}, {}, "test/lane-ledger"},
    ros_time(10'000'000'000));
  EXPECT_TRUE(telemetry) << (telemetry ? "" : telemetry.error().detail);
  return static_cast<bool>(telemetry);
}

[[nodiscard]] std::size_t count_events(const WorldStateSnapshot & snapshot, EventKind kind)
{
  std::size_t count = 0U;
  for (const auto & event : snapshot.events) {
    if (event.kind == kind) {
      ++count;
    }
  }
  return count;
}

// SC-001: a measured loss of one or more whole pitches pops that many front ledger entries, the
// popped objects leave the world as sales, and the accepted observation still lands.
TEST(LaneLedgerReconciliation, PopsFrontEntriesForWholePitchSales)
{
  WorldStateStore store(config());
  ASSERT_TRUE(configure_and_survey_empty(store));
  const auto first = place_can(store, "sim:can_a", 9'200'000'000);
  const auto second = place_can(store, "sim:can_b", 9'300'000'000);
  const auto third = place_can(store, "sim:can_c", 9'400'000'000);
  ASSERT_TRUE(first && second && third);

  auto snapshot = store.snapshot();
  ASSERT_EQ(lane_01(snapshot).contents.size(), 3U);
  EXPECT_FALSE(lane_01(snapshot).ledger_unreliable);

  // Agreement at three pitches: nothing pops, nothing diverges.
  ASSERT_TRUE(survey_column(store, 3.0 * kCanPitchM, 9'500'000'000));
  snapshot = store.snapshot();
  EXPECT_EQ(lane_01(snapshot).contents.size(), 3U);
  EXPECT_FALSE(lane_01(snapshot).ledger_unreliable);
  EXPECT_EQ(count_events(snapshot, EventKind::LaneLedgerUnreliable), 0U);

  // One pitch sold: pop exactly the front entry (first placed), keep the other two.
  ASSERT_TRUE(survey_column(store, 2.0 * kCanPitchM, 9'600'000'000));
  snapshot = store.snapshot();
  ASSERT_EQ(lane_01(snapshot).contents.size(), 2U);
  EXPECT_EQ(lane_01(snapshot).contents.front(), second.value());
  EXPECT_EQ(lane_01(snapshot).contents.back(), third.value());
  EXPECT_FALSE(lane_01(snapshot).ledger_unreliable);
  EXPECT_EQ(
    snapshot.objects.at(first.value()).tracking_state, TrackingState::Removed);
  EXPECT_EQ(snapshot.objects.at(second.value()).tracking_state, TrackingState::Tracked);
  EXPECT_EQ(count_events(snapshot, EventKind::LaneLedgerUnreliable), 0U);
  bool saw_sale_event = false;
  for (const auto & event : snapshot.events) {
    if (event.kind == EventKind::ObjectRemoved && event.object_id == first) {
      saw_sale_event = true;
      EXPECT_EQ(event.lane_id, LaneId{"lane_01"});
      EXPECT_NE(event.detail.find("sold"), std::string::npos);
    }
  }
  EXPECT_TRUE(saw_sale_event);

  // Two more pitches in one acquisition: pop both remaining entries in FIFO order.
  ASSERT_TRUE(survey_column(store, 0.0, 9'700'000'000));
  snapshot = store.snapshot();
  EXPECT_TRUE(lane_01(snapshot).contents.empty());
  EXPECT_FALSE(lane_01(snapshot).ledger_unreliable);
  EXPECT_EQ(snapshot.objects.at(second.value()).tracking_state, TrackingState::Removed);
  EXPECT_EQ(snapshot.objects.at(third.value()).tracking_state, TrackingState::Removed);
}

// SC-002: growth beyond half a pitch withdraws identity trust, surfaces a typed event, and keeps
// the measured geometry accepted — the run does not stop and later surveys still land.
TEST(LaneLedgerReconciliation, GrowthMarksLedgerUnreliableAndKeepsGeometry)
{
  WorldStateStore store(config());
  ASSERT_TRUE(configure_and_survey_empty(store));

  const auto grown = survey_column(store, 2.0 * kCanPitchM, 9'500'000'000);
  ASSERT_TRUE(grown) << (grown ? "" : grown.error().detail);
  auto snapshot = store.snapshot();
  EXPECT_TRUE(lane_01(snapshot).ledger_unreliable);
  EXPECT_DOUBLE_EQ(lane_01(snapshot).available_depth_m, kLaneDepthM - 2.0 * kCanPitchM);
  EXPECT_EQ(lane_01(snapshot).evidence_revision, grown.value().revision);
  EXPECT_TRUE(lane_01(snapshot).contents.empty());
  EXPECT_EQ(count_events(snapshot, EventKind::LaneLedgerUnreliable), 1U);
  bool described = false;
  for (const auto & event : snapshot.events) {
    if (event.kind == EventKind::LaneLedgerUnreliable) {
      described = true;
      EXPECT_EQ(event.lane_id, LaneId{"lane_01"});
      EXPECT_NE(event.detail.find("geometry retained"), std::string::npos);
      EXPECT_NE(event.detail.find("measured column"), std::string::npos);
    }
  }
  EXPECT_TRUE(described);

  // Geometry-safe operation: the next survey is still accepted and the flag stays latched even
  // when the measurement comes back into agreement with the (empty) ledger.
  const auto agreeing = survey_column(store, 0.0, 9'600'000'000);
  ASSERT_TRUE(agreeing) << (agreeing ? "" : agreeing.error().detail);
  snapshot = store.snapshot();
  EXPECT_TRUE(lane_01(snapshot).ledger_unreliable);
  EXPECT_EQ(count_events(snapshot, EventKind::LaneLedgerUnreliable), 1U);
}

// Pre-insert surveys land while the reservation is still Attached and before the column has
// grown: the default continuous lane-evidence topic publishes them every acquisition. They must
// neither invent a front sale out of an empty ledger nor latch identity withdrawal — the
// shortfall bound is the settled contents, so the in-flight product is simply not due yet.
TEST(LaneLedgerReconciliation, PreInsertSurveysWhileAttachedNeitherPopNorWithdraw)
{
  WorldStateStore store(config());
  ASSERT_TRUE(configure_and_survey_empty(store));
  ASSERT_TRUE(observe_telemetry(store));
  const auto observed = store.observe_object(
    can_observation("sim:can_01", 9'200'000'000), ros_time(10'000'000'000));
  ASSERT_TRUE(observed);
  auto snapshot = store.snapshot();
  const auto reserved = store.reserve_task(
    ReserveTaskRequest{"ledger-0001", snapshot.revision, observed.value().object_id,
      observed.value().revision, std::nullopt, std::nullopt, LaneId{"lane_01"},
      snapshot.lanes.at(LaneId{"lane_01"}).revision},
    ros_time(9'300'000'000));
  ASSERT_TRUE(reserved) << (reserved ? "" : reserved.error().detail);
  ASSERT_TRUE(
    store.commit_reserved_attachment(
      ReservedAttachmentRequest{reserved.value().token, "attach-0001"},
      ros_time(9'400'000'000)));

  // Empty destination: the column has not grown yet, and that is the normal pre-insert state.
  const auto pre_insert = survey_column(store, 0.0, 9'500'000'000);
  ASSERT_TRUE(pre_insert) << (pre_insert ? "" : pre_insert.error().detail);
  snapshot = store.snapshot();
  EXPECT_FALSE(lane_01(snapshot).ledger_unreliable);
  EXPECT_TRUE(lane_01(snapshot).contents.empty());
  EXPECT_EQ(count_events(snapshot, EventKind::LaneLedgerUnreliable), 0U);

  // A second acquisition at the same length (continuous evidence) still agrees.
  const auto still_pre_insert = survey_column(store, 0.0, 9'550'000'000);
  ASSERT_TRUE(still_pre_insert) << (still_pre_insert ? "" : still_pre_insert.error().detail);
  snapshot = store.snapshot();
  EXPECT_FALSE(lane_01(snapshot).ledger_unreliable);
  EXPECT_EQ(count_events(snapshot, EventKind::LaneLedgerUnreliable), 0U);

  // Retreat growth while still Attached is the in-flight product; trust survives.
  const auto retreat = survey_column(store, kCanPitchM, 9'600'000'000, {"sim:can_01"});
  ASSERT_TRUE(retreat) << (retreat ? "" : retreat.error().detail);
  snapshot = store.snapshot();
  EXPECT_FALSE(lane_01(snapshot).ledger_unreliable);
  EXPECT_TRUE(lane_01(snapshot).contents.empty());

  const auto detached = store.commit_reserved_detachment(
    ReservedDetachmentRequest{
        reserved.value().token, "detach-0001",
        DetachmentDisposition::PlaceInReservedDestination, ros_time(9'450'000'000)},
    ros_time(9'700'000'000));
  ASSERT_TRUE(detached) << (detached ? "" : detached.error().detail);
  snapshot = store.snapshot();
  ASSERT_EQ(lane_01(snapshot).contents.size(), 1U);
  EXPECT_FALSE(lane_01(snapshot).ledger_unreliable);
}

// The stocked-lane half of the same boundary: a pre-insert survey at the old column length must
// not pop a front entry as a phantom sale while the in-flight product is still in the gripper.
TEST(LaneLedgerReconciliation, PreInsertSurveyOnAStockedLaneDoesNotPhantomPop)
{
  WorldStateStore store(config());
  ASSERT_TRUE(configure_and_survey_empty(store));
  ASSERT_TRUE(place_can(store, "sim:can_a", 9'200'000'000));
  ASSERT_TRUE(place_can(store, "sim:can_b", 9'300'000'000));
  ASSERT_TRUE(place_can(store, "sim:can_c", 9'400'000'000));
  ASSERT_TRUE(survey_column(store, 3.0 * kCanPitchM, 9'450'000'000));
  ASSERT_TRUE(observe_telemetry(store));

  const auto observed = store.observe_object(
    can_observation("sim:can_d", 9'460'000'000), ros_time(10'000'000'000));
  ASSERT_TRUE(observed);
  auto snapshot = store.snapshot();
  const auto reserved = store.reserve_task(
    ReserveTaskRequest{"ledger-0002", snapshot.revision, observed.value().object_id,
      observed.value().revision, std::nullopt, std::nullopt, LaneId{"lane_01"},
      snapshot.lanes.at(LaneId{"lane_01"}).revision},
    ros_time(9'470'000'000));
  ASSERT_TRUE(reserved) << (reserved ? "" : reserved.error().detail);
  ASSERT_TRUE(
    store.commit_reserved_attachment(
      ReservedAttachmentRequest{reserved.value().token, "attach-0002"},
      ros_time(9'480'000'000)));

  // Column still at three pitches: contents stay at three, nothing is marked Removed.
  const auto pre_insert = survey_column(store, 3.0 * kCanPitchM, 9'490'000'000);
  ASSERT_TRUE(pre_insert) << (pre_insert ? "" : pre_insert.error().detail);
  snapshot = store.snapshot();
  ASSERT_EQ(lane_01(snapshot).contents.size(), 3U);
  EXPECT_FALSE(lane_01(snapshot).ledger_unreliable);
  EXPECT_EQ(count_events(snapshot, EventKind::LaneLedgerUnreliable), 0U);
  for (const ObjectId content_id : lane_01(snapshot).contents) {
    EXPECT_EQ(snapshot.objects.at(content_id).tracking_state, TrackingState::Tracked)
      << content_id.value;
  }

  // A real customer sale during the carry still pops: shortfall is judged against contents.
  const auto sold_one = survey_column(store, 2.0 * kCanPitchM, 9'495'000'000);
  ASSERT_TRUE(sold_one) << (sold_one ? "" : sold_one.error().detail);
  snapshot = store.snapshot();
  ASSERT_EQ(lane_01(snapshot).contents.size(), 2U);
  EXPECT_FALSE(lane_01(snapshot).ledger_unreliable);
}

// An identity policy change leaves the old column in place. The next survey must not
// length-reconcile that column at the new product's pitch (it would pop real entries as
// phantom sales and destroy the wrong-product conflict selection has to report).
TEST(LaneLedgerReconciliation, IdentityPolicyChangeLeavesTheOldColumnLedgerIntact)
{
  WorldStateConfig value = config();
  value.product_lane_profiles.push_back(
    ProductLaneProfile{ProductClass::LargeBottle, std::nullopt, 2.0 * 0.045 + 0.045,
      2.0 * 0.045 * 0.99756405025982420});
  WorldStateStore store(value);
  ASSERT_TRUE(configure_and_survey_empty(store));
  ASSERT_TRUE(place_can(store, "sim:can_a", 9'200'000'000));
  ASSERT_TRUE(place_can(store, "sim:can_b", 9'300'000'000));
  ASSERT_TRUE(place_can(store, "sim:can_c", 9'400'000'000));
  ASSERT_TRUE(survey_column(store, 3.0 * kCanPitchM, 9'450'000'000));
  auto snapshot = store.snapshot();
  const auto before = snapshot.lanes.at(LaneId{"lane_01"}).revision;

  const auto path = std::filesystem::temp_directory_path() / "lane_policy_ledger_pitch.yaml";
  std::filesystem::remove(path);
  const auto policy = store.set_lane_policy(
    LanePolicy{LaneId{"lane_01"}, ProductClass::LargeBottle, std::nullopt, 6}, before,
    ros_time(9'460'000'000),
    [&path](const std::vector<LanePolicy> & table) {
      save_lane_policy_state(path, table);
      return true;
    });
  ASSERT_TRUE(policy) << (policy ? "" : policy.error().detail);

  // The can column is measured at the can pitch; the new large-bottle pitch is much longer, so a
  // naive shortfall at the new pitch would pop. The ledger must survive intact and trusted so
  // selection can report the wrong-product conflict.
  const auto after_policy = survey_column(store, 3.0 * kCanPitchM, 9'500'000'000);
  ASSERT_TRUE(after_policy) << (after_policy ? "" : after_policy.error().detail);
  snapshot = store.snapshot();
  const auto & lane = lane_01(snapshot);
  ASSERT_EQ(lane.contents.size(), 3U);
  EXPECT_FALSE(lane.ledger_unreliable);
  EXPECT_EQ(lane.expected_product_class, ProductClass::LargeBottle);
  EXPECT_EQ(count_events(snapshot, EventKind::LaneLedgerUnreliable), 0U);
  EXPECT_EQ(count_events(snapshot, EventKind::ObjectRemoved), 0U);
  for (const ObjectId content_id : lane.contents) {
    EXPECT_EQ(snapshot.objects.at(content_id).tracking_state, TrackingState::Tracked)
      << content_id.value;
  }
  std::filesystem::remove(path);
}

// The retreat survey that proves a placement runs while the reservation is still Attached, one
// pitch longer than the not-yet-pushed ledger. That growth is the in-flight product, not a
// divergence: the ledger stays trusted through the whole transfer.
TEST(LaneLedgerReconciliation, InFlightPlacementGrowthIsReconciledNotWithdrawn)
{
  WorldStateStore store(config());
  ASSERT_TRUE(configure_and_survey_empty(store));
  ASSERT_TRUE(observe_telemetry(store));
  const auto observed = store.observe_object(
    can_observation("sim:can_01", 9'200'000'000), ros_time(10'000'000'000));
  ASSERT_TRUE(observed);
  auto snapshot = store.snapshot();
  const auto reserved = store.reserve_task(
    ReserveTaskRequest{"ledger-0002", snapshot.revision, observed.value().object_id,
      observed.value().revision, std::nullopt, std::nullopt, LaneId{"lane_01"},
      snapshot.lanes.at(LaneId{"lane_01"}).revision},
    ros_time(9'300'000'000));
  ASSERT_TRUE(reserved) << (reserved ? "" : reserved.error().detail);
  ASSERT_TRUE(
    store.commit_reserved_attachment(
      ReservedAttachmentRequest{reserved.value().token, "attach-0002"},
      ros_time(9'400'000'000)));

  const auto retreat = survey_column(
    store, kCanPitchM, 9'500'000'000, {"sim:can_01"});
  ASSERT_TRUE(retreat) << (retreat ? "" : retreat.error().detail);
  snapshot = store.snapshot();
  EXPECT_FALSE(lane_01(snapshot).ledger_unreliable);
  EXPECT_TRUE(lane_01(snapshot).contents.empty());

  const auto detached = store.commit_reserved_detachment(
    ReservedDetachmentRequest{
        reserved.value().token, "detach-0002",
        DetachmentDisposition::PlaceInReservedDestination, ros_time(9'450'000'000)},
    ros_time(9'600'000'000));
  ASSERT_TRUE(detached) << (detached ? "" : detached.error().detail);
  snapshot = store.snapshot();
  ASSERT_EQ(lane_01(snapshot).contents.size(), 1U);
  EXPECT_FALSE(lane_01(snapshot).ledger_unreliable);

  const auto settled = survey_column(store, kCanPitchM, 9'700'000'000, {"sim:can_01"});
  ASSERT_TRUE(settled) << (settled ? "" : settled.error().detail);
  snapshot = store.snapshot();
  EXPECT_FALSE(lane_01(snapshot).ledger_unreliable);
  EXPECT_EQ(lane_01(snapshot).contents.size(), 1U);
  EXPECT_EQ(count_events(snapshot, EventKind::LaneLedgerUnreliable), 0U);
}

// Property: for shortfalls of k whole pitches within half a pitch, exactly k front entries pop;
// growth of any tested size withdraws trust; the tolerance boundary itself agrees.
TEST(LaneLedgerReconciliation, ShortfallPopsRoundToTheNearestWholePitchCount)
{
  for (int k = 1; k <= 3; ++k) {
    for (const double fractional : {0.4, -0.4}) {
      WorldStateStore store(config());
      ASSERT_TRUE(configure_and_survey_empty(store));
      for (int index = 0; index < 4; ++index) {
        ASSERT_TRUE(
          place_can(
            store, "sim:can_" + std::to_string(index),
            9'200'000'000 + index * 100'000'000));
      }
      const double shortfall_pitches =
        static_cast<double>(k) + fractional;
      const double column_length_m = (4.0 - shortfall_pitches) * kCanPitchM;
      const auto surveyed = survey_column(
        store, column_length_m, 9'700'000'000);
      ASSERT_TRUE(surveyed) << "k=" << k << " fractional=" << fractional << ": " <<
        (surveyed ? "" : surveyed.error().detail);
      const auto snapshot = store.snapshot();
      EXPECT_EQ(lane_01(snapshot).contents.size(), 4U - static_cast<std::size_t>(k))
        << "k=" << k << " fractional=" << fractional;
      EXPECT_FALSE(lane_01(snapshot).ledger_unreliable)
        << "k=" << k << " fractional=" << fractional;
    }
  }

  // Growth beyond the half-pitch tolerance at every tested size: identity withdrawn, once.
  for (const double growth_pitches : {0.6, 1.0, 2.5}) {
    WorldStateStore store(config());
    ASSERT_TRUE(configure_and_survey_empty(store));
    ASSERT_TRUE(place_can(store, "sim:can_0", 9'200'000'000));
    const auto surveyed = survey_column(
      store, (1.0 + growth_pitches) * kCanPitchM, 9'700'000'000);
    ASSERT_TRUE(surveyed) << (surveyed ? "" : surveyed.error().detail);
    const auto snapshot = store.snapshot();
    EXPECT_TRUE(lane_01(snapshot).ledger_unreliable) << "growth=" << growth_pitches;
    EXPECT_EQ(lane_01(snapshot).contents.size(), 1U) << "growth=" << growth_pitches;
  }
}

// A lane with no catalogued pitch for its expected class cannot be reconciled at all: the
// ledger is left untouched rather than judged against a guessed pitch.
TEST(LaneLedgerReconciliation, MissingPitchLeavesTheLedgerUntouched)
{
  WorldStateStore store(config());
  const auto configured = store.configure_lane(
    LaneDefinition{LaneId{"lane_01"}, ProductClass::Unknown, std::nullopt, kLaneDepthM, 0},
    ros_time(9'000'000'000));
  ASSERT_TRUE(configured);
  const auto surveyed = store.update_lane(
    LaneObservation{LaneId{"lane_01"}, {}, kLaneDepthM - kCanPitchM, false,
      ros_time(9'500'000'000)},
    ros_time(10'000'000'000), configured.value().revision);
  ASSERT_TRUE(surveyed) << (surveyed ? "" : surveyed.error().detail);
  const auto snapshot = store.snapshot();
  EXPECT_FALSE(lane_01(snapshot).ledger_unreliable);
  EXPECT_EQ(count_events(snapshot, EventKind::LaneLedgerUnreliable), 0U);
}

}  // namespace
}  // namespace restocker_world_state
