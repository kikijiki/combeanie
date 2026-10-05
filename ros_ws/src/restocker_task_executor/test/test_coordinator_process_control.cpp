// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <signal.h>
#include <pthread.h>

#include <chrono>

#include "restocker_task_executor/coordinator_process_control.hpp"
#include "termination_signal_waiter.hpp"

namespace restocker_task_executor
{
namespace
{

using namespace std::chrono_literals;

TEST(CoordinatorTerminationPolicyTest, FirstSignalRequestsDrainAndSecondForcesExit)
{
  CoordinatorTerminationPolicy policy;
  const auto first = policy.observe_signal();
  EXPECT_TRUE(first.request_node_drain);
  EXPECT_FALSE(first.stop_executor);
  EXPECT_FALSE(first.exit_code);

  const auto second = policy.observe_signal();
  EXPECT_FALSE(second.request_node_drain);
  EXPECT_TRUE(second.stop_executor);
  EXPECT_EQ(second.exit_code, CoordinatorProcessExitCode::kForcedBySecondSignal);

  CoordinatorShutdownSnapshot clean;
  clean.status = CoordinatorShutdownStatus::kClean;
  const auto immutable = policy.observe(clean, ExecutorRunState::kRunning, {});
  EXPECT_EQ(immutable.exit_code, CoordinatorProcessExitCode::kForcedBySecondSignal);
}

TEST(CoordinatorTerminationPolicyTest, SelectsStableTerminalExitCodes)
{
  const auto now = CoordinatorTerminationPolicy::SteadyTime{100ms};
  CoordinatorShutdownSnapshot shutdown;

  CoordinatorTerminationPolicy clean;
  shutdown.status = CoordinatorShutdownStatus::kClean;
  EXPECT_EQ(
    clean.observe(shutdown, ExecutorRunState::kRunning, now).exit_code,
    CoordinatorProcessExitCode::kClean);

  CoordinatorTerminationPolicy node_timeout;
  shutdown.status = CoordinatorShutdownStatus::kTimedOut;
  EXPECT_EQ(
    node_timeout.observe(shutdown, ExecutorRunState::kRunning, now).exit_code,
    CoordinatorProcessExitCode::kDrainTimedOut);

  CoordinatorTerminationPolicy executor_failure;
  shutdown.status = CoordinatorShutdownStatus::kRunning;
  shutdown.deadline.reset();
  EXPECT_EQ(
    executor_failure.observe(shutdown, ExecutorRunState::kFailed, now).exit_code,
    CoordinatorProcessExitCode::kExecutorFailure);

  CoordinatorTerminationPolicy unexpected_stop;
  EXPECT_EQ(
    unexpected_stop.observe(shutdown, ExecutorRunState::kStoppedUnexpectedly, now).exit_code,
    CoordinatorProcessExitCode::kExecutorFailure);

  CoordinatorTerminationPolicy owner_stop;
  EXPECT_FALSE(owner_stop.observe(shutdown, ExecutorRunState::kStoppedByOwner, now).exit_code);

  CoordinatorTerminationPolicy process_timeout;
  shutdown.status = CoordinatorShutdownStatus::kDraining;
  shutdown.deadline = now + 50ms;
  EXPECT_FALSE(process_timeout.observe(shutdown, ExecutorRunState::kRunning, now + 49ms).exit_code);
  EXPECT_EQ(
    process_timeout.observe(shutdown, ExecutorRunState::kRunning, now + 50ms).exit_code,
    CoordinatorProcessExitCode::kDrainTimedOut);

  CoordinatorTerminationPolicy overdue;
  EXPECT_EQ(
    overdue.observe(shutdown, ExecutorRunState::kRunning, now + 51ms).exit_code,
    CoordinatorProcessExitCode::kDrainTimedOut);

  EXPECT_EQ(exit_status(CoordinatorProcessExitCode::kClean), 0);
  EXPECT_EQ(exit_status(CoordinatorProcessExitCode::kStartupFailure), 1);
  EXPECT_EQ(exit_status(CoordinatorProcessExitCode::kDrainTimedOut), 2);
  EXPECT_EQ(exit_status(CoordinatorProcessExitCode::kForcedBySecondSignal), 3);
  EXPECT_EQ(exit_status(CoordinatorProcessExitCode::kExecutorFailure), 4);
}

TEST(CoordinatorTerminationPolicyTest, LatchesCleanBeforeLaterExecutorFailure)
{
  CoordinatorTerminationPolicy policy;
  CoordinatorShutdownSnapshot shutdown;
  shutdown.status = CoordinatorShutdownStatus::kClean;
  const auto clean = policy.observe(shutdown, ExecutorRunState::kRunning, {});
  ASSERT_EQ(clean.exit_code, CoordinatorProcessExitCode::kClean);

  const auto later = policy.observe(shutdown, ExecutorRunState::kFailed, {});
  EXPECT_EQ(later.exit_code, CoordinatorProcessExitCode::kClean);
}

TEST(TerminationSignalWaiterTest, BlocksConsumesAndRestoresTerminationSignal)
{
  sigset_t before{};
  ASSERT_EQ(pthread_sigmask(SIG_BLOCK, nullptr, &before), 0);
  const int sigint_before = sigismember(&before, SIGINT);
  const int sigterm_before = sigismember(&before, SIGTERM);
  ASSERT_GE(sigint_before, 0);
  ASSERT_GE(sigterm_before, 0);

  {
    TerminationSignalWaiter waiter(true);
    sigset_t blocked{};
    ASSERT_EQ(pthread_sigmask(SIG_BLOCK, nullptr, &blocked), 0);
    EXPECT_EQ(sigismember(&blocked, SIGINT), 1);
    EXPECT_EQ(sigismember(&blocked, SIGTERM), 1);

    const int raised = raise(SIGTERM);
    EXPECT_EQ(raised, 0);
    const auto consumed = waiter.wait_for(100ms);
    ASSERT_TRUE(consumed);
    EXPECT_EQ(*consumed, SIGTERM);
    EXPECT_FALSE(waiter.wait_for(0ms));
  }

  sigset_t after{};
  ASSERT_EQ(pthread_sigmask(SIG_BLOCK, nullptr, &after), 0);
  EXPECT_EQ(sigismember(&after, SIGINT), sigint_before);
  EXPECT_EQ(sigismember(&after, SIGTERM), sigterm_before);
}

}  // namespace
}  // namespace restocker_task_executor
