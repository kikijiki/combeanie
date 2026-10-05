// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <algorithm>
#include <atomic>
#include <cinttypes>
#include <chrono>
#include <cstdio>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include "restocker_task_executor/coordinator_process_control.hpp"
#include "restocker_task_executor/restock_action_coordinator_node.hpp"
#include "termination_signal_waiter.hpp"

namespace restocker_task_executor
{
namespace
{

using namespace std::chrono_literals;

class ExecutorRunner
{
public:
  explicit ExecutorRunner(rclcpp::Executor & executor)
  : executor_(executor), thread_([this]() {run();})
  {
    while (!executor_.is_spinning() && state() == ExecutorRunState::kRunning) {
      std::this_thread::yield();
    }
  }

  ~ExecutorRunner()
  {
    stop();
  }

  ExecutorRunner(const ExecutorRunner &) = delete;
  ExecutorRunner & operator=(const ExecutorRunner &) = delete;

  void stop() noexcept
  {
    stop_requested_.store(true, std::memory_order_release);
    try {
      executor_.cancel();
    } catch (...) {
      // Joining remains mandatory; the executor owns no externally recoverable capability.
    }
    if (thread_.joinable()) {
      thread_.join();
    }
  }

  [[nodiscard]] ExecutorRunState state() const noexcept
  {
    return state_.load(std::memory_order_acquire);
  }

  [[nodiscard]] std::string failure_detail() const
  {
    std::lock_guard lock(detail_mutex_);
    return failure_detail_;
  }

private:
  void run() noexcept
  {
    try {
      executor_.spin();
      state_.store(
        stop_requested_.load(std::memory_order_acquire) ? ExecutorRunState::kStoppedByOwner :
        ExecutorRunState::kStoppedUnexpectedly,
        std::memory_order_release);
    } catch (const std::exception & error) {
      {
        std::lock_guard lock(detail_mutex_);
        failure_detail_ = error.what();
      }
      state_.store(ExecutorRunState::kFailed, std::memory_order_release);
    } catch (...) {
      {
        std::lock_guard lock(detail_mutex_);
        failure_detail_ = "executor raised an unknown exception";
      }
      state_.store(ExecutorRunState::kFailed, std::memory_order_release);
    }
  }

  rclcpp::Executor & executor_;
  std::atomic<bool> stop_requested_{false};
  std::atomic<ExecutorRunState> state_{ExecutorRunState::kRunning};
  mutable std::mutex detail_mutex_;
  std::string failure_detail_;
  std::thread thread_;
};

[[nodiscard]] std::chrono::milliseconds signal_poll_interval(
  const CoordinatorShutdownSnapshot & shutdown,
  CoordinatorTerminationPolicy::SteadyTime now)
{
  constexpr auto kMaximumPoll = 20ms;
  if (!shutdown.deadline || now >= *shutdown.deadline) {
    return shutdown.deadline ? 0ms : kMaximumPoll;
  }
  const auto remaining =
    std::chrono::duration_cast<std::chrono::milliseconds>(*shutdown.deadline - now);
  return std::min(kMaximumPoll, remaining);
}

void log_terminal_decision(
  CoordinatorProcessExitCode code, const CoordinatorShutdownSnapshot & shutdown,
  const ExecutorRunner & runner, const std::string & process_failure)
{
  const auto logger = rclcpp::get_logger("restock_action_coordinator");
  switch (code) {
    case CoordinatorProcessExitCode::kClean:
      RCLCPP_INFO(logger, "coordinator drain completed cleanly");
      break;
    case CoordinatorProcessExitCode::kDrainTimedOut:
      RCLCPP_ERROR(
        logger,
        "coordinator drain timed out: goal_generation=%" PRIu64 " "
        "reservation_capability_may_remain=%s",
        shutdown.goal_generation,
        shutdown.reservation_capability_may_remain ? "true" : "false");
      break;
    case CoordinatorProcessExitCode::kForcedBySecondSignal:
      RCLCPP_ERROR(logger, "second termination signal forced fail-closed exit");
      break;
    case CoordinatorProcessExitCode::kExecutorFailure:
      {
        const auto executor_failure = runner.failure_detail();
        const auto & detail = process_failure.empty() ? executor_failure : process_failure;
        RCLCPP_ERROR(
          logger, "executor stopped before clean drain: %s",
          detail.empty() ? "unexpected spin return" : detail.c_str());
        break;
      }
    case CoordinatorProcessExitCode::kStartupFailure:
      break;
  }
}

[[nodiscard]] int run_coordinator(TerminationSignalWaiter & signals)
{
  auto node = std::make_shared<RestockActionCoordinatorNode>();
  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions{}, node->executor_threads());
  executor.add_node(node);
  ExecutorRunner runner(executor);
  CoordinatorTerminationPolicy policy;

  CoordinatorShutdownSnapshot last_shutdown = node->shutdown_snapshot();
  std::optional<CoordinatorProcessExitCode> exit_code;
  std::string process_failure;
  while (!exit_code) {
    const auto now = CoordinatorTerminationPolicy::SteadyClock::now();
    last_shutdown = node->shutdown_snapshot();
    const auto observation = policy.observe(last_shutdown, runner.state(), now);
    if (observation.exit_code) {
      exit_code = observation.exit_code;
      break;
    }

    std::optional<int> signal;
    try {
      signal = signals.wait_for(signal_poll_interval(last_shutdown, now));
    } catch (const std::exception & error) {
      process_failure = std::string("synchronous signal intake failed: ") + error.what();
      exit_code = CoordinatorProcessExitCode::kExecutorFailure;
      break;
    }
    while (signal && !exit_code) {
      const auto decision = policy.observe_signal();
      if (decision.request_node_drain) {
        node->request_shutdown();
      }
      if (decision.exit_code) {
        exit_code = decision.exit_code;
        break;
      }
      try {
        signal = signals.wait_for(0ms);
      } catch (const std::exception & error) {
        process_failure = std::string("synchronous signal intake failed: ") + error.what();
        exit_code = CoordinatorProcessExitCode::kExecutorFailure;
      }
    }
  }

  log_terminal_decision(*exit_code, last_shutdown, runner, process_failure);
  runner.stop();
  executor.remove_node(node);
  node.reset();
  return exit_status(*exit_code);
}

}  // namespace
}  // namespace restocker_task_executor

int main(int argc, char ** argv)
{
  bool ros_initialized = false;
  try {
    restocker_task_executor::TerminationSignalWaiter signals;
    rclcpp::init(
      argc, argv, rclcpp::InitOptions{}, rclcpp::SignalHandlerOptions::None);
    ros_initialized = true;
    const int result = restocker_task_executor::run_coordinator(signals);
    rclcpp::shutdown();
    return result;
  } catch (const std::exception & error) {
    if (ros_initialized) {
      RCLCPP_FATAL(rclcpp::get_logger("restock_action_coordinator"), "%s", error.what());
      rclcpp::shutdown();
    } else {
      std::fprintf(stderr, "restock_action_coordinator startup failed: %s\n", error.what());
    }
    return restocker_task_executor::exit_status(
      restocker_task_executor::CoordinatorProcessExitCode::kStartupFailure);
  } catch (...) {
    if (ros_initialized) {
      RCLCPP_FATAL(
        rclcpp::get_logger("restock_action_coordinator"),
        "startup raised an unknown exception");
      rclcpp::shutdown();
    } else {
      std::fprintf(stderr, "restock_action_coordinator startup failed: unknown exception\n");
    }
    return restocker_task_executor::exit_status(
      restocker_task_executor::CoordinatorProcessExitCode::kStartupFailure);
  }
}
