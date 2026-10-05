// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <barrier>
#include <chrono>
#include <future>
#include <memory>
#include <type_traits>
#include <utility>

#include "restocker_task_executor/coordinator_async_steady_clock_failure_latch.hpp"

namespace restocker_task_executor
{
namespace
{

using namespace std::chrono_literals;

constexpr CoordinatorSteadyClockSample kFirstFailure{
  SteadyTime{} + 17ms, true, true};
constexpr CoordinatorSteadyClockSample kLaterFailure{
  SteadyTime{} + 29ms, true, false};

TEST(CoordinatorAsyncSteadyClockFailureLatchTest, RetainsTheExactFirstFailureSample)
{
  CoordinatorAsyncSteadyClockFailureLatch latch;

  EXPECT_FALSE(latch.snapshot());
  EXPECT_TRUE(latch.record_failure(kFirstFailure));
  ASSERT_TRUE(latch.snapshot());
  EXPECT_EQ(*latch.snapshot(), kFirstFailure);

  EXPECT_FALSE(latch.record_failure(kLaterFailure));
  ASSERT_TRUE(latch.snapshot());
  EXPECT_EQ(*latch.snapshot(), kFirstFailure);
}

TEST(CoordinatorAsyncSteadyClockFailureLatchTest, NonFailureCannotOccupyTheLatch)
{
  CoordinatorAsyncSteadyClockFailureLatch latch;

  EXPECT_FALSE(
    latch.record_failure(CoordinatorSteadyClockSample{SteadyTime{} + 7ms, false, false}));
  EXPECT_FALSE(latch.snapshot());
  EXPECT_TRUE(latch.record_failure(kFirstFailure));
  ASSERT_TRUE(latch.snapshot());
  EXPECT_EQ(*latch.snapshot(), kFirstFailure);
}

TEST(CoordinatorAsyncSteadyClockFailureLatchTest, StrongCaptureOutlivesExternalOwnership)
{
  auto latch = std::make_shared<CoordinatorAsyncSteadyClockFailureLatch>();
  std::weak_ptr<CoordinatorAsyncSteadyClockFailureLatch> lifetime = latch;
  auto retained_callback = [owned_latch = latch]() {
    return owned_latch->record_failure(kFirstFailure);
  };
  latch.reset();

  ASSERT_FALSE(lifetime.expired());
  EXPECT_TRUE(retained_callback());
  auto retained = lifetime.lock();
  ASSERT_TRUE(retained);
  ASSERT_TRUE(retained->snapshot());
  EXPECT_EQ(*retained->snapshot(), kFirstFailure);
}

TEST(CoordinatorAsyncSteadyClockFailureLatchTest, ConcurrentRecordAndSnapshotHaveOneBoundary)
{
  constexpr std::size_t kIterations = 128U;
  for (std::size_t iteration = 0U; iteration < kIterations; ++iteration) {
    CoordinatorAsyncSteadyClockFailureLatch latch;
    std::barrier start{4};

    auto record_first = std::async(
      std::launch::async, [&]() {
        start.arrive_and_wait();
        return latch.record_failure(kFirstFailure);
      });
    auto record_later = std::async(
      std::launch::async, [&]() {
        start.arrive_and_wait();
        return latch.record_failure(kLaterFailure);
      });
    auto observe = std::async(
      std::launch::async, [&]() {
        start.arrive_and_wait();
        return latch.snapshot();
      });
    start.arrive_and_wait();

    const bool first_won = record_first.get();
    const bool later_won = record_later.get();
    const auto concurrent_snapshot = observe.get();
    ASSERT_NE(first_won, later_won);

    const auto winner = first_won ? kFirstFailure : kLaterFailure;
    const auto final_snapshot = latch.snapshot();
    ASSERT_TRUE(final_snapshot);
    EXPECT_EQ(*final_snapshot, winner);
    if (concurrent_snapshot) {
      EXPECT_EQ(*concurrent_snapshot, winner);
    }
  }
}

static_assert(!std::is_copy_constructible_v<CoordinatorAsyncSteadyClockFailureLatch>);
static_assert(!std::is_copy_assignable_v<CoordinatorAsyncSteadyClockFailureLatch>);
static_assert(
  !noexcept(std::declval<CoordinatorAsyncSteadyClockFailureLatch &>().record_failure(
    std::declval<CoordinatorSteadyClockSample>())));
static_assert(
  !noexcept(std::declval<const CoordinatorAsyncSteadyClockFailureLatch &>().snapshot()));

}  // namespace
}  // namespace restocker_task_executor
