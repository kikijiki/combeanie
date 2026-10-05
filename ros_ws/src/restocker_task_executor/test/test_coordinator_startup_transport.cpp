// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <chrono>
#include <memory>

#include "restocker_task_executor/coordinator_startup_transport.hpp"

namespace restocker_task_executor
{
namespace
{

using namespace std::chrono_literals;

[[nodiscard]] WorldStateRequestHandle handle(OperationGeneration attempt, std::int64_t request = 1)
{
  return {
    OperationCorrelation{kStartupGoalGeneration, attempt},
    WorldStateServiceKind::kGetSnapshot, request};
}

[[nodiscard]] StartupSnapshotArrival arrival(OperationGeneration attempt, SteadyTime arrived_at)
{
  return {
    attempt, arrived_at,
    std::make_shared<restocker_interfaces::srv::GetWorldState::Response>(), {}};
}

TEST(CoordinatorStartupMailbox, RejectsInvalidArmingAndHandleIdentity)
{
  CoordinatorStartupMailbox mailbox;

  EXPECT_FALSE(mailbox.arm(0U));
  ASSERT_TRUE(mailbox.arm(1U));
  EXPECT_FALSE(mailbox.arm(2U));
  EXPECT_EQ(
    mailbox.install_handle(1U, handle(2U)), StartupHandleInstall::kInvalidHandle);
  auto wrong_service = handle(1U);
  wrong_service.service = WorldStateServiceKind::kReserveTask;
  EXPECT_EQ(
    mailbox.install_handle(1U, wrong_service), StartupHandleInstall::kInvalidHandle);
  EXPECT_EQ(
    mailbox.install_handle(1U, handle(1U, 9)), StartupHandleInstall::kInstalled);
  EXPECT_EQ(
    mailbox.install_handle(1U, handle(1U, 10)), StartupHandleInstall::kDuplicate);
}

TEST(CoordinatorStartupMailbox, AcceptsImmediateCompletionBeforeHandleInstallation)
{
  CoordinatorStartupMailbox mailbox;
  ASSERT_TRUE(mailbox.arm(1U));
  EXPECT_EQ(
    mailbox.deposit(arrival(1U, SteadyTime{} + 10ms)),
    StartupMailboxDeposit::kAccepted);
  EXPECT_EQ(
    mailbox.install_handle(1U, handle(1U, 7)), StartupHandleInstall::kInstalled);

  const auto taken = mailbox.take_and_retire(1U);
  ASSERT_TRUE(taken);
  EXPECT_EQ(taken->arrival.attempt_generation, 1U);
  ASSERT_TRUE(taken->request_handle);
  EXPECT_EQ(taken->request_handle->request_id, 7);
  EXPECT_FALSE(taken->collision);
  EXPECT_TRUE(mailbox.empty());
}

TEST(CoordinatorStartupMailbox, ShutdownDuringSubmissionRejectsReturnedHandleAndCallback)
{
  auto mailbox = std::make_shared<CoordinatorStartupMailbox>();
  std::weak_ptr<CoordinatorStartupMailbox> weak = mailbox;
  ASSERT_TRUE(mailbox->arm(1U));

  const auto retired = mailbox->retire(1U);
  EXPECT_TRUE(retired.retired);
  EXPECT_EQ(
    mailbox->install_handle(1U, handle(1U)), StartupHandleInstall::kRetired);
  EXPECT_EQ(
    mailbox->deposit(arrival(1U, SteadyTime{})),
    StartupMailboxDeposit::kDiscarded);
  mailbox.reset();
  EXPECT_FALSE(weak.lock());
}

TEST(CoordinatorStartupMailbox, StaleAttemptCannotPoisonNewerCompletion)
{
  CoordinatorStartupMailbox mailbox;
  ASSERT_TRUE(mailbox.arm(1U));
  EXPECT_TRUE(mailbox.retire(1U).retired);
  ASSERT_TRUE(mailbox.arm(2U));
  ASSERT_EQ(
    mailbox.install_handle(2U, handle(2U)), StartupHandleInstall::kInstalled);

  EXPECT_EQ(
    mailbox.deposit(arrival(2U, SteadyTime{} + 20ms)),
    StartupMailboxDeposit::kAccepted);
  EXPECT_EQ(
    mailbox.deposit(arrival(1U, SteadyTime{} + 10ms)),
    StartupMailboxDeposit::kDiscarded);

  const auto taken = mailbox.take_and_retire(2U);
  ASSERT_TRUE(taken);
  EXPECT_EQ(taken->arrival.attempt_generation, 2U);
  EXPECT_FALSE(taken->collision);
}

TEST(CoordinatorStartupMailbox, DuplicateCurrentCompletionLatchesCollision)
{
  CoordinatorStartupMailbox mailbox;
  ASSERT_TRUE(mailbox.arm(3U));

  EXPECT_EQ(
    mailbox.deposit(arrival(3U, SteadyTime{} + 1ms)),
    StartupMailboxDeposit::kAccepted);
  EXPECT_EQ(
    mailbox.deposit(arrival(3U, SteadyTime{} + 2ms)),
    StartupMailboxDeposit::kCollision);

  const auto taken = mailbox.take_and_retire(3U);
  ASSERT_TRUE(taken);
  EXPECT_TRUE(taken->collision);
  EXPECT_EQ(taken->arrival.arrived_at, SteadyTime{} + 1ms);
  EXPECT_TRUE(mailbox.empty());
}

TEST(CoordinatorStartupMailbox, RetirementReturnsInstalledHandleAndClearsArrival)
{
  CoordinatorStartupMailbox mailbox;
  ASSERT_TRUE(mailbox.arm(4U));
  ASSERT_EQ(
    mailbox.install_handle(4U, handle(4U, 44)), StartupHandleInstall::kInstalled);
  ASSERT_EQ(mailbox.deposit(arrival(4U, SteadyTime{})), StartupMailboxDeposit::kAccepted);

  const auto retired = mailbox.retire(4U);
  ASSERT_TRUE(retired.retired);
  ASSERT_TRUE(retired.request_handle);
  EXPECT_EQ(retired.request_handle->request_id, 44);
  EXPECT_TRUE(mailbox.empty());
  EXPECT_FALSE(mailbox.take_and_retire(4U));
}

TEST(CoordinatorStartupMailbox, UnconditionalRetirementDrainsDivergentGeneration)
{
  CoordinatorStartupMailbox mailbox;
  ASSERT_TRUE(mailbox.arm(8U));
  ASSERT_EQ(
    mailbox.install_handle(8U, handle(8U, 88)), StartupHandleInstall::kInstalled);
  ASSERT_EQ(
    mailbox.deposit(arrival(8U, SteadyTime{})), StartupMailboxDeposit::kAccepted);

  const auto retired = mailbox.retire_any();
  ASSERT_TRUE(retired.retired);
  ASSERT_TRUE(retired.request_handle);
  EXPECT_EQ(retired.request_handle->request_id, 88);
  EXPECT_TRUE(mailbox.empty());
  EXPECT_FALSE(mailbox.retire_any().retired);
}

}  // namespace
}  // namespace restocker_task_executor
