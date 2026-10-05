// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <future>
#include <optional>
#include <stdexcept>
#include <thread>
#include <vector>

#include "restocker_task_executor/coordinator_steady_clock.hpp"

namespace restocker_task_executor
{
namespace
{

using namespace std::chrono_literals;

TEST(CoordinatorSteadyClockTest, RejectsEmptyProvider)
{
  EXPECT_THROW((void)CoordinatorSteadyClock({}), std::invalid_argument);
}

TEST(CoordinatorSteadyClockTest, ReturnsEachInjectedSampleWithoutChangingItsDomain)
{
  auto next = SteadyTime{};
  CoordinatorSteadyClock clock([&next]() {return next += 7ms;});

  EXPECT_EQ(clock.sample(), (CoordinatorSteadyClockSample{SteadyTime{} + 7ms, false, false}));
  EXPECT_EQ(clock.sample(), (CoordinatorSteadyClockSample{SteadyTime{} + 14ms, false, false}));
  EXPECT_FALSE(clock.provider_failed());
}

TEST(CoordinatorSteadyClockTest, ProviderExceptionSelectsOneStickyNonthrowingFallback)
{
  std::atomic<std::size_t> calls{0U};
  CoordinatorSteadyClock clock([&calls]() -> SteadyTime {
      ++calls;
      throw std::runtime_error("injected clock failure");
    });

  const auto failed = clock.sample();
  EXPECT_TRUE(failed.provider_failed);
  EXPECT_TRUE(failed.newly_failed);
  EXPECT_NE(failed.time, SteadyTime::max());
  EXPECT_TRUE(clock.provider_failed());
  for (std::size_t index = 0U; index < 16U; ++index) {
    EXPECT_EQ(clock.sample(), (CoordinatorSteadyClockSample{failed.time, true, false}));
  }
  EXPECT_EQ(calls.load(), 1U);
}

TEST(CoordinatorSteadyClockTest, InvalidSentinelAlsoSelectsTheStickyFallback)
{
  std::atomic<std::size_t> calls{0U};
  CoordinatorSteadyClock clock([&calls]() {
      ++calls;
      return SteadyTime::max();
    });

  const auto failed = clock.sample();
  EXPECT_TRUE(failed.provider_failed);
  EXPECT_TRUE(failed.newly_failed);
  EXPECT_EQ(clock.sample().time, failed.time);
  EXPECT_EQ(calls.load(), 1U);
}

TEST(CoordinatorSteadyClockTest, ConcurrentFailurePublishesOneFallbackAndOneFirstObserver)
{
  std::atomic<std::size_t> calls{0U};
  CoordinatorSteadyClock clock([&calls]() -> SteadyTime {
      ++calls;
      throw std::runtime_error("injected concurrent clock failure");
    });
  std::vector<std::future<CoordinatorSteadyClockSample>> samples;
  for (std::size_t index = 0U; index < 16U; ++index) {
    samples.push_back(std::async(std::launch::async, [&clock]() {return clock.sample();}));
  }

  std::optional<SteadyTime> fallback;
  std::size_t new_failures = 0U;
  for (auto & future : samples) {
    const auto sample = future.get();
    EXPECT_TRUE(sample.provider_failed);
    fallback = fallback.value_or(sample.time);
    EXPECT_EQ(sample.time, *fallback);
    new_failures += sample.newly_failed ? 1U : 0U;
  }
  EXPECT_EQ(calls.load(), 1U);
  EXPECT_EQ(new_failures, 1U);
}

TEST(CoordinatorSteadyClockTest, ProviderCallsAreSerializedUntilFailurePublication)
{
  std::mutex mutex;
  std::condition_variable condition;
  std::atomic<std::size_t> invocations{0U};
  bool first_entered = false;
  bool release_first = false;
  CoordinatorSteadyClock clock([&]() -> SteadyTime {
      std::unique_lock lock(mutex);
      if (++invocations == 1U) {
        first_entered = true;
        condition.notify_all();
        condition.wait(lock, [&release_first]() {return release_first;});
        return SteadyTime{} + 9ms;
      }
      throw std::runtime_error("second provider invocation fails");
    });
  auto first_sample = std::async(std::launch::async, [&clock]() {return clock.sample();});
  bool entered_in_time = false;
  {
    std::unique_lock lock(mutex);
    entered_in_time = condition.wait_for(lock, 1s, [&first_entered]() {return first_entered;});
  }
  if (!entered_in_time) {
    {
      std::lock_guard lock(mutex);
      release_first = true;
    }
    condition.notify_all();
    (void)first_sample.get();
    FAIL() << "first provider invocation did not reach its barrier";
    return;
  }

  std::promise<void> second_started_promise;
  auto second_started = second_started_promise.get_future();
  auto second_sample = std::async(
    std::launch::async, [&clock, &second_started_promise]() {
      second_started_promise.set_value();
      return clock.sample();
    });
  ASSERT_EQ(second_started.wait_for(1s), std::future_status::ready);
  bool second_provider_entered = false;
  {
    std::unique_lock lock(mutex);
    second_provider_entered = condition.wait_for(
      lock, 50ms, [&invocations]() {return invocations.load() > 1U;});
  }
  EXPECT_FALSE(second_provider_entered);
  EXPECT_EQ(invocations.load(), 1U);
  {
    std::lock_guard lock(mutex);
    release_first = true;
  }
  condition.notify_all();
  const auto first = first_sample.get();
  const auto failure = second_sample.get();
  EXPECT_EQ(first, (CoordinatorSteadyClockSample{SteadyTime{} + 9ms, false, false}));
  ASSERT_TRUE(failure.provider_failed);
  ASSERT_TRUE(failure.newly_failed);
  EXPECT_EQ(clock.sample().time, failure.time);
  EXPECT_EQ(invocations.load(), 2U);
}

TEST(CoordinatorSteadyClockTest, SuccessCanCompleteBeforeALaterFailureFreezesTheDomain)
{
  std::atomic<std::size_t> calls{0U};
  CoordinatorSteadyClock clock([&calls]() -> SteadyTime {
      if (++calls == 1U) {
        return SteadyTime{} + 13ms;
      }
      throw std::runtime_error("later provider failure");
    });

  EXPECT_EQ(
    clock.sample(), (CoordinatorSteadyClockSample{SteadyTime{} + 13ms, false, false}));
  const auto failure = clock.sample();
  EXPECT_TRUE(failure.provider_failed);
  EXPECT_TRUE(failure.newly_failed);
  EXPECT_EQ(clock.sample(), (CoordinatorSteadyClockSample{failure.time, true, false}));
}

TEST(CoordinatorSteadyClockTest, BlockedProviderDoesNotBlockFailureStateObservation)
{
  std::promise<void> entered;
  std::promise<void> release;
  auto release_future = release.get_future().share();
  CoordinatorSteadyClock clock([&entered, release_future]() {
      entered.set_value();
      release_future.wait();
      return SteadyTime{} + 17ms;
    });
  auto sample = std::async(std::launch::async, [&clock]() {return clock.sample();});
  const auto entered_status = entered.get_future().wait_for(1s);

  auto observation = std::async(
    std::launch::async, [&clock]() {return clock.provider_failed();});
  const auto observation_status = observation.wait_for(100ms);
  release.set_value();
  const auto observed_failure = observation.get();
  const auto sampled = sample.get();
  ASSERT_EQ(entered_status, std::future_status::ready);
  EXPECT_EQ(observation_status, std::future_status::ready);
  EXPECT_FALSE(observed_failure);
  EXPECT_FALSE(sampled.provider_failed);
}

}  // namespace
}  // namespace restocker_task_executor
