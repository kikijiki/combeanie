// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include <rcl/time.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <future>
#include <memory>
#include <mutex>
#include <thread>

#include "restocker_world_state/clock_authority.hpp"

namespace restocker_world_state
{
namespace
{
using namespace std::chrono_literals;

ObjectObservation object_at(const rclcpp::Time & time)
{
  ObjectObservation value;
  value.source_object_id = "clock-object";
  value.frame_id = "world";
  value.product_class = ProductClass::Can;
  value.pose_in_world = Eigen::Isometry3d::Identity();
  value.pose_covariance = PoseCovariance::Identity() * 1.0e-6;
  value.orientation = ObjectOrientation::Upright;
  value.observation_time = time;
  return value;
}

// Match the pinned TimeSource's mutex discipline. Calling raw setters without this lock is not
// the managed production path and would not establish the sample/pre-change ordering contract.
void set_managed_time(const rclcpp::Clock::SharedPtr & clock, std::int64_t ns)
{
  std::lock_guard lock(clock->get_clock_mutex());
  EXPECT_EQ(rcl_enable_ros_time_override(clock->get_clock_handle()), RCL_RET_OK);
  EXPECT_EQ(rcl_set_ros_time_override(clock->get_clock_handle(), ns), RCL_RET_OK);
}

Result<ObservationReceipt> admit(ClockAuthority & authority, WorldStateStore & store)
{
  return authority.with_sample(
    [&](const AuthorityClockSample & sample) {
      return store.observe_object(object_at(sample.time), sample.time, sample);
    });
}

TEST(ClockAuthority, InactiveAndZeroTimeAreNotManagedReadinessProvenance)
{
  WorldStateStore store;
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  ClockAuthority authority(store, clock);
  const auto inactive = authority.with_sample([](auto sample) {return sample;});
  EXPECT_FALSE(inactive.managed_ros_started);
  set_managed_time(clock, 0);
  const auto zero = authority.with_sample([](auto sample) {return sample;});
  EXPECT_FALSE(zero.managed_ros_started);
  EXPECT_NE(zero.lineage, inactive.lineage);
  set_managed_time(clock, 1);
  const auto started = authority.with_sample([](auto sample) {return sample;});
  EXPECT_TRUE(started.managed_ros_started);
  EXPECT_EQ(started.lineage, zero.lineage);
}

TEST(ClockAuthority, InitialActivationAfterStaticConfigurationCanBootstrap)
{
  WorldStateStore store;
  ASSERT_TRUE(
    store.configure_lane(
      LaneDefinition{LaneId{"lane"}, ProductClass::Can, std::nullopt, 0.85},
      rclcpp::Time(std::int64_t{100}, RCL_ROS_TIME)));
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  ClockAuthority authority(store, clock);
  const auto old = authority.with_sample([](auto sample) {return sample;});
  set_managed_time(clock, 1'000'000'000);
  const auto stale = store.observe_object(object_at(old.time), old.time, old);
  ASSERT_FALSE(stale);
  EXPECT_EQ(stale.error().code, WorldStateErrorCode::ClockAuthorityInhibited);
  ASSERT_TRUE(admit(authority, store));
}

TEST(ClockAuthority, ConfiguredLaneTimeDoesNotFenceFirstEvidenceInNewBootstrapLineage)
{
  WorldStateStore store;
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  set_managed_time(clock, 10'000'000'000);
  ClockAuthority authority(store, clock);
  const auto old = authority.with_sample([](auto value) {return value;});
  const auto configured = store.configure_lane(
    LaneDefinition{LaneId{"lane"}, ProductClass::Can, std::nullopt, 0.85}, old.time);
  ASSERT_TRUE(configured);
  set_managed_time(clock, 1'000'000'000);
  const LaneObservation first{LaneId{"lane"}, {}, 0.85, false,
    rclcpp::Time(std::int64_t{1'000'000'000}, RCL_ROS_TIME)};
  const auto stale = store.update_lane(first, old.time, configured.value().revision, old);
  ASSERT_FALSE(stale);
  EXPECT_EQ(stale.error().code, WorldStateErrorCode::ClockAuthorityInhibited);
  const auto admitted = authority.with_sample(
    [&](const AuthorityClockSample & sample) {
      return store.update_lane(first, sample.time, configured.value().revision, sample);
    });
  ASSERT_TRUE(admitted) << "configuration time is not prior lane evidence";
  EXPECT_EQ(store.snapshot().lanes.at(LaneId{"lane"}).last_verified, first.observation_time);
  EXPECT_NE(store.snapshot().lanes.at(LaneId{"lane"}).evidence_revision, 0U);
  set_managed_time(clock, 1'100'000'000);
  const auto invalidated = store.invalidate_lane_evidence(
    LaneId{"lane"}, rclcpp::Time(std::int64_t{1'100'000'000}, RCL_ROS_TIME),
    admitted.value().revision);
  ASSERT_TRUE(invalidated);
  const auto repeated = authority.with_sample(
    [&](const AuthorityClockSample & sample) {
      return store.update_lane(first, sample.time, invalidated.value().revision, sample);
    });
  ASSERT_FALSE(repeated);
  EXPECT_EQ(repeated.error().code, WorldStateErrorCode::OutOfOrder);
}

TEST(ClockAuthority, ActivationAfterSystemTimeEvidenceLatchesInsteadOfRelabelingIt)
{
  WorldStateStore store;
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  ClockAuthority authority(store, clock);
  ASSERT_TRUE(admit(authority, store));
  set_managed_time(clock, 1'000'000'000);
  const auto rejected = admit(authority, store);
  ASSERT_FALSE(rejected);
  EXPECT_EQ(rejected.error().code, WorldStateErrorCode::ClockAuthorityInhibited);
}

TEST(ClockAuthority, ForwardChangesDoNotLatchButOneNanosecondBackwardDoes)
{
  WorldStateStore store;
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  set_managed_time(clock, 1'000'000'000);
  ClockAuthority authority(store, clock);
  ASSERT_TRUE(admit(authority, store));
  set_managed_time(clock, 2'000'000'000);
  ASSERT_TRUE(admit(authority, store));
  const auto before = store.snapshot().revision;
  set_managed_time(clock, 1'999'999'999);
  const auto rejected = admit(authority, store);
  ASSERT_FALSE(rejected);
  EXPECT_EQ(rejected.error().code, WorldStateErrorCode::ClockAuthorityInhibited);
  set_managed_time(clock, 3'000'000'000);
  EXPECT_EQ(admit(authority, store).error().code, WorldStateErrorCode::ClockAuthorityInhibited);
  EXPECT_EQ(store.snapshot().revision, before);
}

TEST(ClockAuthority, OverrideDeactivationLatchesBeforeFallbackTimeIsAdmitted)
{
  WorldStateStore store;
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  set_managed_time(clock, 1'000'000'000);
  ClockAuthority authority(store, clock);
  ASSERT_TRUE(admit(authority, store));
  {
    std::lock_guard lock(clock->get_clock_mutex());
    ASSERT_EQ(rcl_disable_ros_time_override(clock->get_clock_handle()), RCL_RET_OK);
  }
  const auto rejected = admit(authority, store);
  ASSERT_FALSE(rejected);
  EXPECT_EQ(rejected.error().code, WorldStateErrorCode::ClockAuthorityInhibited);
}

TEST(ClockAuthority, SampleAndAdmissionFinishBeforeWaitingClockChange)
{
  WorldStateStore store;
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  set_managed_time(clock, 2'000'000'000);
  ClockAuthority authority(store, clock);
  std::promise<void> sample_entered;
  auto entered = sample_entered.get_future();
  std::promise<void> release_sample;
  auto released = release_sample.get_future();
  std::promise<bool> admission_result;
  auto admitted = admission_result.get_future();
  std::jthread caller([&]() {
      authority.with_sample(
        [&](const AuthorityClockSample & sample) {
          sample_entered.set_value();
          if (released.wait_for(2s) != std::future_status::ready) {
            admission_result.set_value(false);
            return;
          }
          admission_result.set_value(
            static_cast<bool>(
              store.observe_object(object_at(sample.time), sample.time, sample)));
        });
    });
  EXPECT_EQ(entered.wait_for(2s), std::future_status::ready);
  std::promise<void> reset_started;
  auto started = reset_started.get_future();
  std::promise<void> reset_finished;
  auto finished = reset_finished.get_future();
  std::jthread writer([&]() {
      reset_started.set_value();
      set_managed_time(clock, 1'000'000'000);
      reset_finished.set_value();
    });
  EXPECT_EQ(started.wait_for(2s), std::future_status::ready);
  EXPECT_EQ(finished.wait_for(20ms), std::future_status::timeout);
  release_sample.set_value();
  caller.join();
  writer.join();
  EXPECT_TRUE(admitted.get());
  const auto rejected = admit(authority, store);
  ASSERT_FALSE(rejected);
  EXPECT_EQ(rejected.error().code, WorldStateErrorCode::ClockAuthorityInhibited);
}

TEST(ClockAuthority, DestructionWaitsForManagedPreHookWhileStoreRemainsAlive)
{
  WorldStateStore store;
  const auto configured = store.configure_lane(
    LaneDefinition{LaneId{"lane"}, ProductClass::Can, std::nullopt, 0.85},
    rclcpp::Time(std::int64_t{1}, RCL_ROS_TIME));
  ASSERT_TRUE(configured);
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  set_managed_time(clock, 2'000'000'000);
  std::promise<void> pre_entered;
  auto entered = pre_entered.get_future();
  rcl_jump_threshold_t threshold{};
  threshold.min_backward.nanoseconds = -1;
  // Registered first: this tells the test that the TimeSource-style writer owns Clock and is
  // entering the synchronous callback sequence, before the authority hook waits on Store.
  auto probe = clock->create_jump_callback([&]() {pre_entered.set_value();}, nullptr, threshold);
  auto authority = std::make_unique<ClockAuthority>(store, clock);
  ASSERT_TRUE(admit(*authority, store));
  std::promise<void> store_locked;
  auto locked = store_locked.get_future();
  std::promise<void> release_store;
  auto released = release_store.get_future();
  std::jthread policy([&]() {
      const auto result = store.set_lane_policy(
        LanePolicy{LaneId{"lane"}, ProductClass::Can, std::nullopt, 1},
        configured.value().revision, rclcpp::Time(std::int64_t{2'000'000'000}, RCL_ROS_TIME),
        [&](const auto &) {
          store_locked.set_value();
          return released.wait_for(2s) == std::future_status::ready;
        });
      EXPECT_TRUE(result);
    });
  EXPECT_EQ(locked.wait_for(2s), std::future_status::ready);
  std::jthread writer([&]() {set_managed_time(clock, 1'000'000'000);});
  EXPECT_EQ(entered.wait_for(2s), std::future_status::ready);
  std::promise<void> removal_started;
  auto started = removal_started.get_future();
  std::promise<void> removal_finished;
  auto finished = removal_finished.get_future();
  std::jthread remover([&]() {
      removal_started.set_value();
      authority.reset();
      removal_finished.set_value();
    });
  EXPECT_EQ(started.wait_for(2s), std::future_status::ready);
  EXPECT_EQ(finished.wait_for(20ms), std::future_status::timeout);
  release_store.set_value();
  policy.join();
  writer.join();
  remover.join();
  EXPECT_EQ(finished.wait_for(0s), std::future_status::ready);
  const auto sample = store.capture_authority_clock(
    rclcpp::Time(std::int64_t{3'000'000'000}, RCL_ROS_TIME),
    true);
  const auto rejected = store.observe_object(object_at(sample.time), sample.time, sample);
  ASSERT_FALSE(rejected);
  EXPECT_EQ(rejected.error().code, WorldStateErrorCode::ClockAuthorityInhibited);
}

TEST(ClockAuthority, SystemClockRetainsOrdinaryAdmissionWithoutManagedEligibility)
{
  WorldStateConfig config;
  config.clock_type = RCL_SYSTEM_TIME;
  WorldStateStore store(config);
  auto clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  ClockAuthority authority(store, clock);
  const auto sample = authority.with_sample([](auto value) {return value;});
  EXPECT_FALSE(sample.managed_ros_started);
  ASSERT_TRUE(admit(authority, store));
}

}  // namespace
}  // namespace restocker_world_state
