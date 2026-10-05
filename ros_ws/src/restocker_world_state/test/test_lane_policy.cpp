// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <optional>
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

[[nodiscard]] WorldStateConfig config()
{
  WorldStateConfig value;
  value.maximum_observation_age = 2s;
  value.lane_evidence_validity = 60s;
  value.maximum_future_skew = 100ms;
  value.event_capacity = 32;
  return value;
}

[[nodiscard]] LaneDefinition baseline_lane(std::uint32_t target_count = 6)
{
  return LaneDefinition{LaneId{"lane_01"}, ProductClass::Can, "SIM-CAN-STD", 0.85, target_count};
}

// Configures lane_01 and gives it one accepted survey, so policy changes have evidence to
// invalidate. Returns the lane's revision after the survey.
[[nodiscard]] Revision configured_with_evidence(WorldStateStore & store)
{
  const auto configured = store.configure_lane(baseline_lane(), ros_time(9'000'000'000));
  EXPECT_TRUE(configured) << (configured ? "" : configured.error().detail);
  if (!configured) {
    return 0;
  }
  const auto evidence = store.update_lane(
    LaneObservation{LaneId{"lane_01"}, {}, 0.85, false, ros_time(9'400'000'000)},
    ros_time(10'000'000'000), configured.value().revision);
  EXPECT_TRUE(evidence) << (evidence ? "" : evidence.error().detail);
  return evidence ? evidence.value().revision : 0;
}

[[nodiscard]] std::filesystem::path fresh_state_path(const std::string & name)
{
  const auto path = std::filesystem::path(::testing::TempDir()) / name;
  std::filesystem::remove(path);
  std::filesystem::remove(path.string() + ".tmp");
  return path;
}

TEST(LanePolicyService, AppliesRevisionCheckedChangeAndPersistsTheWholeTable)
{
  WorldStateStore store(config());
  const Revision surveyed = configured_with_evidence(store);
  ASSERT_NE(surveyed, 0U);
  const auto path = fresh_state_path("lane_policy_applies.yaml");

  const auto receipt = store.set_lane_policy(
    LanePolicy{LaneId{"lane_01"}, ProductClass::Can, "SIM-CAN-STD", 9},
    surveyed, ros_time(10'100'000'000),
    [&path](const std::vector<LanePolicy> & table) {
      save_lane_policy_state(path, table);
      return true;
    });
  ASSERT_TRUE(receipt) << (receipt ? "" : receipt.error().detail);

  const auto snapshot = store.snapshot();
  const auto & lane = snapshot.lanes.at(LaneId{"lane_01"});
  // SC-001, live half: the next snapshot carries the new intent, and the change bumped the
  // lane revision so selections and reservations made against the old revision conflict.
  EXPECT_EQ(lane.target_count, 9U);
  EXPECT_EQ(lane.expected_product_class, ProductClass::Can);
  EXPECT_EQ(lane.expected_sku, "SIM-CAN-STD");
  EXPECT_GT(lane.revision, surveyed);
  EXPECT_EQ(receipt.value().revision, snapshot.revision);
  // A policy change is an owner event: the survey that produced the current evidence was taken
  // under the old intent and must be re-taken before anything acts on this lane again.
  EXPECT_TRUE(lane.evidence_invalidated);
  ASSERT_FALSE(snapshot.events.empty());
  EXPECT_EQ(snapshot.events.back().kind, EventKind::LanePolicySet);
  EXPECT_EQ(snapshot.events.back().lane_id, LaneId{"lane_01"});

  // SC-001, disk half: the durable document is the full table in the baseline schema.
  const auto persisted = load_lane_policy_state(path);
  ASSERT_EQ(persisted.size(), 1U);
  EXPECT_EQ(persisted[0], (LanePolicy{LaneId{"lane_01"}, ProductClass::Can, "SIM-CAN-STD", 9}));
}

// SC-002: a stale expected revision is refused before anything is written or mutated.
TEST(LanePolicyService, RefusesStaleRevisionWithoutTouchingLaneOrFile)
{
  WorldStateStore store(config());
  const auto configured = store.configure_lane(
    baseline_lane(), ros_time(9'000'000'000));
  ASSERT_TRUE(configured) << (configured ? "" : configured.error().detail);
  const auto surveyed = store.update_lane(
    LaneObservation{LaneId{"lane_01"}, {}, 0.85, false, ros_time(9'400'000'000)},
    ros_time(10'000'000'000), configured.value().revision);
  ASSERT_TRUE(surveyed) << (surveyed ? "" : surveyed.error().detail);
  const auto path = fresh_state_path("lane_policy_stale.yaml");

  bool persist_called = false;
  const auto stale = store.set_lane_policy(
    LanePolicy{LaneId{"lane_01"}, ProductClass::SmallBottle, "SIM-BOTTLE-SMALL", 4},
    configured.value().revision, ros_time(10'100'000'000),
    [&persist_called, &path](const std::vector<LanePolicy> & table) {
      persist_called = true;
      save_lane_policy_state(path, table);
      return true;
    });
  ASSERT_FALSE(stale);
  EXPECT_EQ(stale.error().code, WorldStateErrorCode::RevisionConflict);
  EXPECT_FALSE(persist_called);
  EXPECT_FALSE(std::filesystem::exists(path));

  const auto snapshot = store.snapshot();
  const auto & lane = snapshot.lanes.at(LaneId{"lane_01"});
  EXPECT_EQ(lane.expected_product_class, ProductClass::Can);
  EXPECT_EQ(lane.expected_sku, "SIM-CAN-STD");
  EXPECT_EQ(lane.target_count, 6U);
  EXPECT_EQ(lane.revision, surveyed.value().revision);
  EXPECT_FALSE(lane.evidence_invalidated);
  for (const auto & event : snapshot.events) {
    EXPECT_NE(event.kind, EventKind::LanePolicySet);
  }
}

TEST(LanePolicyService, RefusesWhenPersistenceFailsWithoutChangingTheLane)
{
  WorldStateStore store(config());
  const auto surveyed = configured_with_evidence(store);
  ASSERT_NE(surveyed, 0U);
  // The target's parent path is a regular file, so the durable writer fails at open(2) with
  // nothing written; the store must refuse the mutation that could not be made durable.
  const auto blocker = fresh_state_path("lane_policy_write_blocker");
  {
    std::ofstream stream(blocker);
    stream << "not a directory\n";
  }
  const auto receipt = store.set_lane_policy(
    LanePolicy{LaneId{"lane_01"}, ProductClass::Can, "SIM-CAN-STD", 8},
    surveyed, ros_time(10'100'000'000),
    [&blocker](const std::vector<LanePolicy> & table) {
      try {
        save_lane_policy_state(blocker / "state.yaml", table);
        return true;
      } catch (const std::exception &) {
        return false;
      }
    });
  ASSERT_FALSE(receipt);
  EXPECT_EQ(receipt.error().code, WorldStateErrorCode::InvariantViolation);

  const auto snapshot = store.snapshot();
  const auto & lane = snapshot.lanes.at(LaneId{"lane_01"});
  EXPECT_EQ(lane.target_count, 6U);
  EXPECT_EQ(lane.revision, surveyed);
  EXPECT_FALSE(lane.evidence_invalidated);
  std::filesystem::remove(blocker);
}

TEST(LanePolicyService, UnchangedPolicySucceedsWithoutBumpEventOrRewrite)
{
  WorldStateStore store(config());
  const auto surveyed = configured_with_evidence(store);
  ASSERT_NE(surveyed, 0U);
  const auto before = store.snapshot().revision;

  bool persist_called = false;
  const auto receipt = store.set_lane_policy(
    LanePolicy{LaneId{"lane_01"}, ProductClass::Can, "SIM-CAN-STD", 6},
    surveyed, ros_time(10'100'000'000),
    [&persist_called](const std::vector<LanePolicy> &) {
      persist_called = true;
      return true;
    });
  ASSERT_TRUE(receipt) << (receipt ? "" : receipt.error().detail);
  EXPECT_EQ(receipt.value().revision, before);
  EXPECT_FALSE(persist_called);
  const auto snapshot = store.snapshot();
  EXPECT_EQ(snapshot.lanes.at(LaneId{"lane_01"}).revision, surveyed);
  for (const auto & event : snapshot.events) {
    EXPECT_NE(event.kind, EventKind::LanePolicySet);
  }
}

TEST(LanePolicyService, RejectsInvalidPoliciesAndMissingPersistence)
{
  WorldStateStore store(config());
  const auto surveyed = configured_with_evidence(store);
  ASSERT_NE(surveyed, 0U);
  const auto persist_ok = [](const std::vector<LanePolicy> &) {return true;};

  const auto no_callback = store.set_lane_policy(
    LanePolicy{LaneId{"lane_01"}, ProductClass::Can, "SIM-CAN-STD", 7},
    surveyed, ros_time(10'100'000'000), {});
  ASSERT_FALSE(no_callback);
  EXPECT_EQ(no_callback.error().code, WorldStateErrorCode::InvalidArgument);

  const auto empty_id = store.set_lane_policy(
    LanePolicy{LaneId{""}, ProductClass::Can, "SIM-CAN-STD", 7},
    surveyed, ros_time(10'100'000'000), persist_ok);
  ASSERT_FALSE(empty_id);
  EXPECT_EQ(empty_id.error().code, WorldStateErrorCode::InvalidArgument);

  const auto unknown_class = store.set_lane_policy(
    LanePolicy{LaneId{"lane_01"}, ProductClass::Unknown, std::nullopt, 7},
    surveyed, ros_time(10'100'000'000), persist_ok);
  ASSERT_FALSE(unknown_class);
  EXPECT_EQ(unknown_class.error().code, WorldStateErrorCode::InvalidArgument);

  const auto empty_sku = store.set_lane_policy(
    LanePolicy{LaneId{"lane_01"}, ProductClass::Can, std::string(""), 7},
    surveyed, ros_time(10'100'000'000), persist_ok);
  ASSERT_FALSE(empty_sku);
  EXPECT_EQ(empty_sku.error().code, WorldStateErrorCode::InvalidArgument);

  const auto unknown_lane = store.set_lane_policy(
    LanePolicy{LaneId{"lane_99"}, ProductClass::Can, "SIM-CAN-STD", 7},
    surveyed, ros_time(10'100'000'000), persist_ok);
  ASSERT_FALSE(unknown_lane);
  EXPECT_EQ(unknown_lane.error().code, WorldStateErrorCode::NotFound);

  const auto wrong_clock = store.set_lane_policy(
    LanePolicy{LaneId{"lane_01"}, ProductClass::Can, "SIM-CAN-STD", 7},
    surveyed, rclcpp::Time(10'100'000'000, RCL_SYSTEM_TIME), persist_ok);
  ASSERT_FALSE(wrong_clock);
  EXPECT_EQ(wrong_clock.error().code, WorldStateErrorCode::ClockMismatch);

  // None of the refusals may leave an event or a lane mutation behind.
  const auto snapshot = store.snapshot();
  EXPECT_EQ(snapshot.lanes.at(LaneId{"lane_01"}).revision, surveyed);
  for (const auto & event : snapshot.events) {
    EXPECT_NE(event.kind, EventKind::LanePolicySet);
  }
}

// SC-001 end to end at the store boundary: a policy written through one store's persistence
// resumes in a fresh store built the way world_state_node builds itself at start-up.
TEST(LanePolicyService, APersistedPolicyChangeSurvivesARestartAndReturnsInTheNextSnapshot)
{
  const auto path = fresh_state_path("lane_policy_restart.yaml");
  {
    WorldStateStore store(config());
    const auto surveyed = configured_with_evidence(store);
    ASSERT_NE(surveyed, 0U);
    const auto receipt = store.set_lane_policy(
      LanePolicy{LaneId{"lane_01"}, ProductClass::Can, "SIM-CAN-STD", 11},
      surveyed, ros_time(10'100'000'000),
      [&path](const std::vector<LanePolicy> & table) {
        save_lane_policy_state(path, table);
        return true;
      });
    ASSERT_TRUE(receipt) << (receipt ? "" : receipt.error().detail);
  }

  // The start-up sequence: shipped definitions, persisted overlay, configure, snapshot.
  std::vector<LaneDefinition> definitions{baseline_lane(6)};
  const auto resumed = load_lane_policy_state(path);
  apply_lane_policy_state(definitions, resumed);
  WorldStateStore restarted(config());
  const auto configured = restarted.configure_lane(definitions[0], ros_time(11'000'000'000));
  ASSERT_TRUE(configured) << (configured ? "" : configured.error().detail);
  const auto snapshot = restarted.snapshot();
  const auto & lane = snapshot.lanes.at(LaneId{"lane_01"});
  EXPECT_EQ(lane.target_count, 11U);
  EXPECT_EQ(lane.expected_product_class, ProductClass::Can);
  EXPECT_EQ(lane.expected_sku, "SIM-CAN-STD");
  // The restart also resumes policy changes that altered identity, not just the count.
}

}  // namespace
}  // namespace restocker_world_state
