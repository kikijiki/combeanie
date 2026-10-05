// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <cstdlib>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <type_traits>
#include <utility>

#include "detail/accepted_handle_adoption.hpp"

namespace restocker_task_executor::detail
{
namespace
{

using Handle = std::shared_ptr<int>;
using Adoption = BoundedAcceptedHandleAdoption<Handle>;
std::weak_ptr<int> terminating_handle;

CoordinatorGoalId goal_id(std::uint8_t value)
{
  CoordinatorGoalId id{};
  id.front() = value;
  return id;
}

InspectedAcceptedHandle<Handle> inspected(
  Handle handle, CoordinatorGoalId id,
  bool extraction_failed = false)
{
  return InspectedAcceptedHandle<Handle>{std::move(handle), id, extraction_failed};
}

TEST(AcceptedHandleInspection, NullSkipsExtractor)
{
  int calls = 0;
  const auto result = inspect_accepted_handle(
    Handle{}, [&calls](const Handle &) {
      ++calls;
      return goal_id(1U);
    });

  EXPECT_FALSE(result.handle);
  EXPECT_FALSE(result.goal_id);
  EXPECT_FALSE(result.identity_extraction_failed);
  EXPECT_EQ(calls, 0);
}

TEST(AcceptedHandleInspection, ExceptionLeavesHandlePinnedAndRunsOutsideCallerLock)
{
  std::mutex binding_mutex;
  auto handle = std::make_shared<int>(7);
  bool extractor_observed_unlocked = false;

  auto result = inspect_accepted_handle(
    handle, [&binding_mutex, &extractor_observed_unlocked](const Handle &) -> CoordinatorGoalId {
      extractor_observed_unlocked = binding_mutex.try_lock();
      if (extractor_observed_unlocked) {
        binding_mutex.unlock();
      }
      throw std::runtime_error("identity unavailable");
    });

  EXPECT_TRUE(extractor_observed_unlocked);
  EXPECT_EQ(result.handle.get(), handle.get());
  EXPECT_FALSE(result.goal_id);
  EXPECT_TRUE(result.identity_extraction_failed);

  Adoption adoption;
  std::lock_guard lock(binding_mutex);
  EXPECT_EQ(
    adoption.adopt(result, nullptr, nullptr, false),
    AcceptedHandleAdoptionStatus::kAdoptedFirstOrphan);
  EXPECT_EQ(adoption.orphan(0U).get(), handle.get());
}

TEST(AcceptedHandleAdoption, ExactPendingMovesCanonicalHandle)
{
  Adoption adoption;
  const auto expected_id = goal_id(3U);
  auto source = std::make_shared<int>(11);
  const auto identity = source.get();
  auto incoming = inspected(std::move(source), expected_id);
  Handle pending;

  EXPECT_EQ(
    adoption.adopt(incoming, &expected_id, &pending, false),
    AcceptedHandleAdoptionStatus::kAdoptedPending);
  EXPECT_FALSE(incoming.handle);
  EXPECT_EQ(pending.get(), identity);
  EXPECT_FALSE(adoption.fail_stopped());
  EXPECT_EQ(adoption.orphan_count(), 0U);
}

TEST(AcceptedHandleAdoption, DuplicatePendingPreservesCanonicalAndCallerOwnedDuplicate)
{
  Adoption adoption;
  const auto expected_id = goal_id(4U);
  auto pending = std::make_shared<int>(12);
  auto incoming = inspected(pending, expected_id);
  const auto canonical = pending.get();

  EXPECT_EQ(
    adoption.adopt(incoming, &expected_id, &pending, false),
    AcceptedHandleAdoptionStatus::kDuplicatePending);
  EXPECT_EQ(pending.get(), canonical);
  EXPECT_EQ(incoming.handle.get(), canonical);
  EXPECT_EQ(adoption.orphan_count(), 0U);
}

TEST(AcceptedHandleAdoption, NullFailStopsWithoutConsumingCapacity)
{
  Adoption adoption;
  InspectedAcceptedHandle<Handle> incoming;

  EXPECT_EQ(
    adoption.adopt(incoming, nullptr, nullptr, false),
    AcceptedHandleAdoptionStatus::kNullHandleFailStop);
  EXPECT_TRUE(adoption.fail_stopped());
  EXPECT_EQ(adoption.orphan_count(), 0U);
}

TEST(AcceptedHandleAdoption, RetainsTwoDistinctOrphansAndDeduplicatesBoth)
{
  Adoption adoption;
  auto first_handle = std::make_shared<int>(1);
  auto second_handle = std::make_shared<int>(2);
  auto first = inspected(first_handle, goal_id(1U));
  auto second = inspected(second_handle, goal_id(2U));

  EXPECT_EQ(
    adoption.adopt(first, nullptr, nullptr, false),
    AcceptedHandleAdoptionStatus::kAdoptedFirstOrphan);
  EXPECT_EQ(
    adoption.adopt(second, nullptr, nullptr, false),
    AcceptedHandleAdoptionStatus::kAdoptedSecondDistinctOrphanFailStop);
  EXPECT_TRUE(adoption.fail_stopped());
  ASSERT_EQ(adoption.orphan_count(), 2U);

  auto duplicate_first = inspected(first_handle, goal_id(8U));
  auto duplicate_second = inspected(second_handle, goal_id(9U));
  EXPECT_EQ(
    adoption.adopt(duplicate_first, nullptr, nullptr, false),
    AcceptedHandleAdoptionStatus::kDuplicateOrphan);
  EXPECT_EQ(
    adoption.adopt(duplicate_second, nullptr, nullptr, false),
    AcceptedHandleAdoptionStatus::kDuplicateOrphan);
  EXPECT_TRUE(duplicate_first.handle);
  EXPECT_TRUE(duplicate_second.handle);
  EXPECT_EQ(adoption.orphan_count(), 2U);
}

TEST(AcceptedHandleAdoption, ExactIdDistinctHandlePreservesPendingAndRetainsMismatch)
{
  Adoption adoption;
  const auto expected_id = goal_id(5U);
  auto pending = std::make_shared<int>(1);
  auto unexpected = std::make_shared<int>(2);
  const auto canonical = pending.get();
  const auto anomalous = unexpected.get();
  auto incoming = inspected(std::move(unexpected), expected_id);

  EXPECT_EQ(
    adoption.adopt(incoming, &expected_id, &pending, false),
    AcceptedHandleAdoptionStatus::kPendingIdentityMismatchFailStop);
  EXPECT_EQ(pending.get(), canonical);
  EXPECT_EQ(adoption.orphan(0U).get(), anomalous);
  EXPECT_TRUE(adoption.fail_stopped());
}

TEST(AcceptedHandleAdoption, DifferentIdAndActiveBindingUseOrdinaryOrphanPolicy)
{
  const auto expected_id = goal_id(6U);
  auto pending = std::make_shared<int>(1);

  Adoption wrong_id_adoption;
  auto wrong_id = inspected(std::make_shared<int>(2), goal_id(7U));
  EXPECT_EQ(
    wrong_id_adoption.adopt(wrong_id, &expected_id, &pending, false),
    AcceptedHandleAdoptionStatus::kAdoptedFirstOrphan);
  EXPECT_TRUE(wrong_id_adoption.fail_stopped());

  Adoption active_adoption;
  auto active_arrival = inspected(std::make_shared<int>(3), expected_id);
  EXPECT_EQ(
    active_adoption.adopt(active_arrival, &expected_id, &pending, true),
    AcceptedHandleAdoptionStatus::kAdoptedFirstOrphan);
  EXPECT_TRUE(active_adoption.fail_stopped());
}

TEST(AcceptedHandleAdoption, SameHandleWithDifferentIdIsNotABenignPendingDuplicate)
{
  Adoption adoption;
  const auto expected_id = goal_id(6U);
  auto pending = std::make_shared<int>(1);
  auto wrong_id = inspected(pending, goal_id(7U));

  EXPECT_EQ(
    adoption.adopt(wrong_id, &expected_id, &pending, false),
    AcceptedHandleAdoptionStatus::kAdoptedFirstOrphan);
  EXPECT_TRUE(adoption.fail_stopped());
  EXPECT_EQ(adoption.orphan(0U).get(), pending.get());
}

TEST(AcceptedHandleAdoption, ExtractionFailureCannotAliasPending)
{
  Adoption adoption;
  const auto expected_id = goal_id(10U);
  auto pending = std::make_shared<int>(1);
  auto incoming = inspected(pending, expected_id, true);

  EXPECT_EQ(
    adoption.adopt(incoming, &expected_id, &pending, false),
    AcceptedHandleAdoptionStatus::kAdoptedFirstOrphan);
  EXPECT_TRUE(adoption.fail_stopped());
  EXPECT_EQ(adoption.orphan(0U).get(), pending.get());
}

TEST(AcceptedHandleAdoption, ThirdDistinctOrphanTerminatesWhileIncomingIsPinned)
{
  EXPECT_EXIT(
      {
        std::set_terminate([]() {std::_Exit(terminating_handle.expired() ? 90 : 91);});
        Adoption adoption;
        auto first = inspected(std::make_shared<int>(1), goal_id(1U));
        auto second = inspected(std::make_shared<int>(2), goal_id(2U));
        auto third = inspected(std::make_shared<int>(3), goal_id(3U));
        terminating_handle = third.handle;
        (void)adoption.adopt(first, nullptr, nullptr, false);
        (void)adoption.adopt(second, nullptr, nullptr, false);
        (void)adoption.adopt(third, nullptr, nullptr, false);
        std::_Exit(92);
      },
    ::testing::ExitedWithCode(91), "");
}

static_assert(!std::is_copy_constructible_v<Adoption>);
static_assert(std::is_nothrow_destructible_v<Adoption>);
static_assert(std::is_nothrow_move_constructible_v<InspectedAcceptedHandle<Handle>>);

}  // namespace
}  // namespace restocker_task_executor::detail
