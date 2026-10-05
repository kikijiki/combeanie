// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <execinfo.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <future>
#include <limits>
#include <memory>
#include <mutex>
#include <new>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <action_msgs/msg/goal_status.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <restocker_interfaces/action/restock_product.hpp>
#include <restocker_interfaces/msg/restock_coordinator_status.hpp>
#include <restocker_interfaces/srv/get_world_state.hpp>
#include <restocker_interfaces/srv/release_task_reservation.hpp>
#include <restocker_interfaces/srv/reserve_task.hpp>
#include <restocker_interfaces/srv/validate_execution_world_authority.hpp>
#include <restocker_interfaces/srv/validate_task_reservation.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>

#include "restocker_task_executor/action_result_publisher.hpp"
#include "restocker_task_executor/moveit_motion_port.hpp"
#include "restocker_task_executor/restock_action_coordinator_node.hpp"
#include "restocker_task_executor/restock_action_contract.hpp"
#include "restocker_task_executor/restock_coordinator_inbox.hpp"
#include "restocker_world_state/ros_conversions.hpp"
#include "restocker_world_state/world_state.hpp"
#include "ros_domain_lease_guard.hpp"

namespace
{
thread_local std::optional<std::size_t> allocations_before_failure;
thread_local std::optional<std::size_t> allocation_trigger_size;
thread_local std::size_t allocations_after_trigger{0U};
thread_local std::atomic<std::size_t> * allocation_trigger_arm_source = nullptr;
// The arm source this thread's live countdown fired in. Card 078: the countdown may only throw
// while that source still reads the marker the fire installed; when the test closes or reopens
// the window, a leftover countdown belongs to a region that has already exited and is discarded.
thread_local std::atomic<std::size_t> * allocation_countdown_arm_source = nullptr;
// Callback-local outage injection, closed by after_callback_failure before ROS middleware
// resumes. A finite count tests the ordinary receipt; an unlimited count covers every allocation
// in that receipt without leaking failure into unrelated executor/DDS work.
thread_local std::size_t allocation_failure_budget{0U};
thread_local std::size_t allocation_failure_thrown{0U};

class AllocationCallBlocker final
{
public:
  ~AllocationCallBlocker() {release();}

  AllocationCallBlocker() = default;
  AllocationCallBlocker(const AllocationCallBlocker &) = delete;
  AllocationCallBlocker & operator=(const AllocationCallBlocker &) = delete;

  void intercept()
  {
    std::unique_lock lock(mutex_);
    blocked_ = true;
    condition_.notify_all();
    if (!condition_.wait_for(lock, std::chrono::seconds(5), [this]() {return released_;})) {
      timed_out_ = true;
    }
  }

  [[nodiscard]] bool wait_until_blocked(std::chrono::milliseconds timeout)
  {
    std::unique_lock lock(mutex_);
    return condition_.wait_for(lock, timeout, [this]() {return blocked_;});
  }

  void release()
  {
    {
      std::lock_guard lock(mutex_);
      released_ = true;
    }
    condition_.notify_all();
  }

  [[nodiscard]] bool timed_out() const
  {
    std::lock_guard lock(mutex_);
    return timed_out_;
  }

private:
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  bool blocked_{false};
  bool released_{false};
  bool timed_out_{false};
};

class ExactDeleteInterceptor final
{
public:
  void capture(void * storage, std::size_t size) noexcept
  {
    size_.store(size, std::memory_order_relaxed);
    captured_.store(storage, std::memory_order_relaxed);
    target_.store(storage, std::memory_order_release);
  }

  void intercept_if_target(void * storage) noexcept
  {
    if (!storage || !target_.load(std::memory_order_acquire)) {
      return;
    }
    void * expected = storage;
    if (target_.compare_exchange_strong(
        expected, nullptr, std::memory_order_acq_rel,
        std::memory_order_acquire))
    {
      blocker_.intercept();
    }
  }

  [[nodiscard]] bool wait_until_blocked(std::chrono::milliseconds timeout)
  {
    return blocker_.wait_until_blocked(timeout);
  }

  void release() {blocker_.release();}
  [[nodiscard]] std::size_t captured_size() const noexcept {return size_.load();}
  [[nodiscard]] std::uintptr_t captured_identity() const noexcept
  {
    return reinterpret_cast<std::uintptr_t>(captured_.load(std::memory_order_relaxed));
  }
  [[nodiscard]] bool timed_out() const {return blocker_.timed_out();}

private:
  std::atomic<void *> target_{nullptr};
  std::atomic<void *> captured_{nullptr};
  std::atomic<std::size_t> size_{0U};
  AllocationCallBlocker blocker_;
};

struct AllocationCaptureScript
{
  std::optional<std::size_t> trigger_size;
  bool active{false};
  std::size_t next_ordinal{0U};
  std::size_t fail_ordinal{std::numeric_limits<std::size_t>::max()};
  std::array<std::size_t, 2U> capture_ordinals{std::numeric_limits<std::size_t>::max(),
    std::numeric_limits<std::size_t>::max()};
  std::array<ExactDeleteInterceptor *, 2U> captures{nullptr, nullptr};
  std::size_t stop_after_ordinal{std::numeric_limits<std::size_t>::max()};
  std::atomic<bool> * arm_source{nullptr};
};

std::array<std::atomic<ExactDeleteInterceptor *>, 2U> exact_delete_interceptors{};
thread_local AllocationCaptureScript allocation_capture_script;
thread_local AllocationCallBlocker * allocation_call_blocker = nullptr;
thread_local AllocationCallBlocker * allocation_trigger_blocker = nullptr;

// Card 078: an uncaught std::bad_alloc from this harness's operator new used to reach
// std::terminate with nothing but the exception name, so the throw site was unattributable.
// The throw site is captured here (frame addresses only, no allocation, re-entry guarded) and
// printed by the terminate reporter below; the reporter then chains to libstdc++'s own
// handler, which still prints "terminate called after throwing an instance of ...".
constexpr int kThrowStackCapacity = 64;
struct ThrowStack
{
  void * frames[kThrowStackCapacity]{};
  int depth{0};
};
thread_local ThrowStack throw_stack;
thread_local bool capturing_throw_stack = false;

void write_text(const char * text, std::size_t size) noexcept
{
  const auto written = ::write(2, text, size);
  static_cast<void>(written);
}

void capture_throw_stack() noexcept
{
  if (capturing_throw_stack) {
    return;
  }
  capturing_throw_stack = true;
  throw_stack.depth = ::backtrace(throw_stack.frames, kThrowStackCapacity);
  capturing_throw_stack = false;
}

void report_failed_allocation(std::size_t size) noexcept
{
  char buffer[160];
  std::size_t length = 0U;
  const auto append = [&buffer, &length](const char * text) {
    while (*text != '\0' && length + 1U < sizeof(buffer)) {
      buffer[length++] = *text++;
    }
  };
  append("card078: operator new(");
  char digits[24];
  std::size_t digit_count = 0U;
  std::size_t remaining = size;
  do {
    digits[digit_count++] = static_cast<char>('0' + (remaining % 10U));
    remaining /= 10U;
  } while (remaining != 0U && digit_count < sizeof(digits));
  while (digit_count > 0U && length + 3U < sizeof(buffer)) {
    buffer[length++] = digits[--digit_count];
  }
  append(") failed: std::malloc returned null\n");
  write_text(buffer, length);
}

std::terminate_handler previous_terminate_handler = nullptr;

#if defined(__SANITIZE_ADDRESS__)
extern "C" void __sanitizer_print_stack_trace();
#endif

void report_terminate() noexcept
{
  constexpr const char kTerminateHeader[] = "card078: stack at std::terminate\n";
  write_text(kTerminateHeader, sizeof(kTerminateHeader) - 1U);
  void * frames[kThrowStackCapacity];
  const int depth = ::backtrace(frames, kThrowStackCapacity);
  if (depth > 0) {
    ::backtrace_symbols_fd(frames, depth, 2);
  }
  constexpr const char kThrowHeader[] = "card078: stack where the exception was thrown\n";
  write_text(kThrowHeader, sizeof(kThrowHeader) - 1U);
  if (throw_stack.depth > 0) {
    ::backtrace_symbols_fd(throw_stack.frames, throw_stack.depth, 2);
  }
#if defined(__SANITIZE_ADDRESS__)
  __sanitizer_print_stack_trace();
#endif
  if (previous_terminate_handler) {
    previous_terminate_handler();
  }
  ::_Exit(134);
}

struct TerminateReporter
{
  TerminateReporter() {previous_terminate_handler = std::set_terminate(report_terminate);}
};
TerminateReporter terminate_reporter;

// The injection window the coordinator's steady-clock provider opens for a test. Card 078: a
// thread that read the clock while a window was open kept its trigger after the window closed,
// and the next 4097-byte allocation on that thread threw std::bad_alloc wherever it happened to
// be — outside the coordinator's guarded admission region, where nothing catches it, so the
// process died with std::terminate. Two sentinels bound the window: the test closing it
// (kAllocationDisarmed) and the harness closing it to every other thread when one thread fires
// (kAllocationWindowFired).
constexpr std::size_t kAllocationDisarmed = std::numeric_limits<std::size_t>::max();
constexpr std::size_t kAllocationWindowFired = std::numeric_limits<std::size_t>::max() - 1U;
// The distinctive size the trigger matches: the coordinator test's 4096-char goal lane, copied
// once in handle_goal before the preaccept authority allocations.
constexpr std::size_t kAllocationTriggerLaneLength = 4096U;

[[nodiscard]] bool allocation_window_open(std::size_t value) noexcept
{
  return value != kAllocationDisarmed && value != kAllocationWindowFired;
}

void reset_allocation_injection() noexcept
{
  allocation_failure_budget = 0U;
  allocation_failure_thrown = 0U;
  allocations_before_failure.reset();
  allocation_trigger_size.reset();
  allocations_after_trigger = 0U;
  allocation_trigger_arm_source = nullptr;
  allocation_countdown_arm_source = nullptr;
  allocation_call_blocker = nullptr;
  allocation_trigger_blocker = nullptr;
}

// Arms (window open) or clears (window closed) this thread's allocation-failure injection. One
// implementation, so what the coordinator tests arm and what these harness tests arm cannot drift
// apart.
void arm_allocation_injection_from_clock(
  std::atomic<std::size_t> & arm_source,
  std::size_t trigger_size = kAllocationTriggerLaneLength + 1U,
  AllocationCallBlocker * trigger_blocker = nullptr) noexcept
{
  const auto allocation_countdown = arm_source.load(std::memory_order_acquire);
  if (allocation_window_open(allocation_countdown)) {
    allocation_trigger_size = trigger_size;
    allocations_after_trigger = allocation_countdown;
    allocation_trigger_blocker = trigger_blocker;
    allocation_trigger_arm_source = &arm_source;
  } else {
    reset_allocation_injection();
  }
}

// Card 082: arm the injected failure on *this* thread straight away, from test code that already
// runs on the thread it wants to poison (a dependency hook), or from the clock provider below.
// Storing kAllocationWindowFired is what makes the countdown valid under Card 078's rule and
// closes the window to every other thread in the same move.
void engage_allocation_failure(std::atomic<std::size_t> & arm_source, std::size_t budget) noexcept
{
  arm_source.store(kAllocationWindowFired, std::memory_order_release);
  allocation_countdown_arm_source = &arm_source;
  allocations_before_failure.reset();
  allocation_failure_budget = budget;
  allocation_failure_thrown = 0U;
}

// Card 078: the injected allocation failure must never escape the window it was armed in.
// Outside the coordinator's guarded admission region nothing catches std::bad_alloc, so an
// escaped injection reaches std::terminate — the crash Card 061 recorded and this card fixes.
TEST(AllocationInjectionHarness, DiscardsAStaleArmAfterItsWindowCloses)
{
  std::atomic<std::size_t> arm_source{1U};
  std::atomic<bool> armed{false};
  std::atomic<bool> window_closed{false};
  std::atomic<bool> threw{false};
  std::thread armer([&]() {
      // A coordinator callback that reads the injected clock while the window is open arms this
      // thread; the pump callback is the one that armed the thread behind Card 061's crash.
      arm_allocation_injection_from_clock(arm_source);
      armed.store(true, std::memory_order_release);
      while (!window_closed.load(std::memory_order_acquire)) {
        std::this_thread::yield();
      }
      try {
        std::vector<std::string> allocations;
        allocations.reserve(4U);
        for (std::size_t index = 0U; index < 8U; ++index) {
          allocations.emplace_back(kAllocationTriggerLaneLength, 'x');
        }
      } catch (const std::bad_alloc &) {
        threw.store(true, std::memory_order_release);
      }
      reset_allocation_injection();
    });
  while (!armed.load(std::memory_order_acquire)) {
    std::this_thread::yield();
  }
  arm_source.store(kAllocationDisarmed, std::memory_order_release);
  window_closed.store(true, std::memory_order_release);
  armer.join();
  EXPECT_FALSE(threw.load(std::memory_order_acquire)) <<
    "an armed thread fired the injection after its window had closed";
}

TEST(AllocationInjectionHarness, DiscardsALiveCountdownWhenItsWindowCloses)
{
  std::atomic<std::size_t> arm_source{1U};
  arm_allocation_injection_from_clock(arm_source);
  bool threw = false;
  try {
    {
      const std::string trigger(kAllocationTriggerLaneLength, 'l');
      EXPECT_FALSE(trigger.empty());
    }
    // A test closes the window as soon as the goal response arrives. A countdown still live at
    // that point belongs to a region that has already exited, so it must be discarded rather
    // than thrown into whatever runs next on this thread.
    arm_source.store(kAllocationDisarmed, std::memory_order_release);
    std::vector<std::string> allocations;
    allocations.reserve(4U);
    for (std::size_t index = 0U; index < 8U; ++index) {
      allocations.emplace_back(64U, 'x');
    }
  } catch (const std::bad_alloc &) {
    threw = true;
  }
  reset_allocation_injection();
  EXPECT_FALSE(threw) << "a live countdown outlived the window it was armed in";
}

void * allocate_storage(std::size_t size)
{
  if (allocation_failure_budget > 0U) {
    --allocation_failure_budget;
    const std::size_t index = ++allocation_failure_thrown;
    if (index == 2U && allocation_call_blocker) {
      auto * blocker = allocation_call_blocker;
      allocation_call_blocker = nullptr;
      blocker->intercept();
    }
    char line[96];
    std::size_t length = 0U;
    const char prefix[] = "card082: injected failure #";
    for (std::size_t i = 0U; i < sizeof(prefix) - 1U; ++i) {
      line[length++] = prefix[i];
    }
    char digits[20];
    std::size_t digit_count = 0U;
    std::size_t remaining = index;
    do {
      digits[digit_count++] = static_cast<char>('0' + (remaining % 10U));
      remaining /= 10U;
    } while (remaining != 0U && digit_count < sizeof(digits));
    while (digit_count > 0U) {line[length++] = digits[--digit_count];}
    line[length++] = '\n';
    const auto written = ::write(2, line, length);
    static_cast<void>(written);
    capture_throw_stack();
    throw std::bad_alloc{};
  }
  if (!allocation_capture_script.active && allocation_capture_script.trigger_size &&
    *allocation_capture_script.trigger_size == size)
  {
    allocation_capture_script.trigger_size.reset();
    allocation_capture_script.active = true;
    allocation_capture_script.next_ordinal = 0U;
    if (allocation_capture_script.arm_source) {
      allocation_capture_script.arm_source->store(false, std::memory_order_release);
      allocation_capture_script.arm_source = nullptr;
    }
  }
  const bool capture_script_active = allocation_capture_script.active;
  std::size_t capture_ordinal = 0U;
  if (capture_script_active) {
    capture_ordinal = allocation_capture_script.next_ordinal++;
    if (capture_ordinal == allocation_capture_script.fail_ordinal) {
      allocation_capture_script.active = false;
      capture_throw_stack();
      throw std::bad_alloc{};
    }
  }
  const bool trigger_size_matched = allocation_trigger_size && *allocation_trigger_size == size;
  // Card 078: a trigger fires only while the arm source this thread armed from still holds the
  // exact countdown it armed with. Anything else is stale - the test closed the window, or
  // another thread already fired in it - and a std::bad_alloc thrown from a stale trigger lands
  // wherever this thread next happens to allocate, which is outside the coordinator's guarded
  // admission region and therefore reaches std::terminate.
  const bool trigger_window_matched = allocation_trigger_arm_source &&
    allocation_trigger_arm_source->load(std::memory_order_acquire) == allocations_after_trigger;
  const bool trigger_matched =
    trigger_size_matched && (!allocation_trigger_arm_source || trigger_window_matched);
  if (trigger_matched) {
    allocation_trigger_size.reset();
    if (allocation_trigger_arm_source) {
      // The first fire closes the window to every other thread armed with the same countdown.
      // kAllocationWindowFired is what keeps this thread's own countdown valid: it may throw
      // while that marker is current, and not one allocation after the window is reopened.
      allocation_trigger_arm_source->store(kAllocationWindowFired, std::memory_order_release);
    }
    allocation_countdown_arm_source = allocation_trigger_arm_source;
    allocation_trigger_arm_source = nullptr;
  } else if (trigger_size_matched) {
    allocation_trigger_size.reset();
    allocation_trigger_arm_source = nullptr;
  } else if (allocations_before_failure) {
    if (allocation_countdown_arm_source &&
      allocation_countdown_arm_source->load(std::memory_order_acquire) != kAllocationWindowFired)
    {
      allocations_before_failure.reset();
      allocation_countdown_arm_source = nullptr;
    } else if (*allocations_before_failure == 0U) {
      allocations_before_failure.reset();
      allocation_countdown_arm_source = nullptr;
      auto * blocker = allocation_call_blocker;
      allocation_call_blocker = nullptr;
      if (blocker) {
        blocker->intercept();
      } else {
        capture_throw_stack();
        throw std::bad_alloc{};
      }
    } else {
      --*allocations_before_failure;
    }
  }
  if (void * storage = std::malloc(size)) {
    if (capture_script_active) {
      for (std::size_t index = 0U; index < allocation_capture_script.captures.size(); ++index) {
        if (allocation_capture_script.capture_ordinals[index] == capture_ordinal &&
          allocation_capture_script.captures[index])
        {
          allocation_capture_script.captures[index]->capture(storage, size);
        }
      }
      if (capture_ordinal >= allocation_capture_script.stop_after_ordinal) {
        allocation_capture_script.active = false;
      }
    }
    if (trigger_matched) {
      allocations_before_failure = allocations_after_trigger;
      allocation_call_blocker = allocation_trigger_blocker;
      allocation_trigger_blocker = nullptr;
    }
    return storage;
  }
  report_failed_allocation(size);
  capture_throw_stack();
  throw std::bad_alloc{};
}

void deallocate_storage(void * storage) noexcept
{
  for (auto & interceptor : exact_delete_interceptors) {
    if (auto * value = interceptor.load(std::memory_order_acquire)) {
      value->intercept_if_target(storage);
    }
  }
  std::free(storage);
}
}  // namespace

// Every replaceable allocation form routes through allocate_storage/deallocate_storage.
// AddressSanitizer interposes the array and nothrow forms itself, so leaving them to the
// runtime would (a) bypass the injection the tests above rely on and (b) pair libasan's
// `operator new(size, nothrow)` with this file's `std::free`, which ASan reports as an
// alloc-dealloc mismatch and aborts on before the first test runs.
void * operator new(std::size_t size) {return allocate_storage(size);}

void * operator new[](std::size_t size) {return allocate_storage(size);}

void * operator new(std::size_t size, const std::nothrow_t &) noexcept
{
  try {
    return allocate_storage(size);
  } catch (...) {
    return nullptr;
  }
}

void * operator new[](std::size_t size, const std::nothrow_t &) noexcept
{
  try {
    return allocate_storage(size);
  } catch (...) {
    return nullptr;
  }
}

void operator delete(void * storage) noexcept {deallocate_storage(storage);}

void operator delete[](void * storage) noexcept {deallocate_storage(storage);}

void operator delete(void * storage, std::size_t) noexcept {deallocate_storage(storage);}

void operator delete[](void * storage, std::size_t) noexcept {deallocate_storage(storage);}

void operator delete(void * storage, const std::nothrow_t &) noexcept
{
  deallocate_storage(storage);
}

void operator delete[](void * storage, const std::nothrow_t &) noexcept
{
  deallocate_storage(storage);
}

namespace restocker_task_executor
{
namespace
{

using namespace std::chrono_literals;
using Action = restocker_interfaces::action::RestockProduct;
using restocker_world_state::FaultState;
using restocker_world_state::GraspState;
using restocker_world_state::LaneId;
using restocker_world_state::ObjectId;
using restocker_world_state::ObjectOrientation;
using restocker_world_state::ProductClass;
using restocker_world_state::ReservationStage;
using restocker_world_state::TaskPhase;
using restocker_world_state::TrackingState;

class ThrowingActionResultPublisher final : public ActionResultPublisher
{
public:
  ThrowingActionResultPublisher(
    std::atomic<std::size_t> & calls, bool throw_standard_exception) noexcept
  : calls_(calls), throw_standard_exception_(throw_standard_exception)
  {
  }

  void publish(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<Action>> &,
    ActionTerminalKind,
    const std::shared_ptr<Action::Result> &) override
  {
    ++calls_;
    if (throw_standard_exception_) {
      throw std::runtime_error("injected action result publisher failure");
    }
    throw 7;
  }

private:
  std::atomic<std::size_t> & calls_;
  bool throw_standard_exception_{false};
};

void send_grasp_center_transform(
  tf2_ros::StaticTransformBroadcaster & broadcaster,
  const rclcpp::Time & stamp)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = stamp;
  transform.header.frame_id = "tool0";
  transform.child_frame_id = "grasp_center";
  transform.transform.translation.z = 0.14;
  transform.transform.rotation.w = 1.0;
  broadcaster.sendTransform(transform);
}

[[nodiscard]] rclcpp::NodeOptions ready_coordinator_options(
  const std::string & action_name, const std::string & world_state_prefix = "/test_world")
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {rclcpp::Parameter("product_catalog_path", RESTOCKER_TEST_PRODUCT_CATALOG),
      rclcpp::Parameter("workcell_geometry_path", RESTOCKER_TEST_WORKCELL_GEOMETRY),
      rclcpp::Parameter("gripper_geometry_path", RESTOCKER_TEST_GRIPPER_GEOMETRY),
      rclcpp::Parameter("world_state.get_snapshot_service", world_state_prefix + "/get_snapshot"),
      rclcpp::Parameter("world_state.reserve_task_service", world_state_prefix + "/reserve_task"),
      rclcpp::Parameter(
        "world_state.validate_reservation_service",
        world_state_prefix + "/validate_reservation"),
      rclcpp::Parameter(
        "world_state.release_reservation_service",
        world_state_prefix + "/release_reservation"),
      rclcpp::Parameter(
        "world_state.validate_execution_authority_service",
        world_state_prefix + "/validate_execution_authority"),
      rclcpp::Parameter("action_name", action_name), rclcpp::Parameter("pump_period_ms", 5),
      rclcpp::Parameter("selection.maximum_object_age_ms", 5000),
      rclcpp::Parameter("selection.lane_evidence_validity_ms", 5000)});
  return options;
}

class AdmissionReadyObserver final
{
public:
  AdmissionReadyObserver(const rclcpp::Node::SharedPtr & node, const std::string & status_topic)
  {
    subscription_ = node->create_subscription<restocker_interfaces::msg::RestockCoordinatorStatus>(
      status_topic, rclcpp::QoS(1).reliable().transient_local(),
      [this](const restocker_interfaces::msg::RestockCoordinatorStatus::ConstSharedPtr message) {
        {
          std::lock_guard lock(mutex_);
          latest_ = *message;
        }
        condition_.notify_all();
      });
  }

  [[nodiscard]] bool wait_until_ready(std::chrono::milliseconds timeout)
  {
    std::unique_lock lock(mutex_);
    return condition_.wait_for(
      lock, timeout,
      [this]() {return latest_ && latest_->admission_ready;});
  }

private:
  std::mutex mutex_;
  std::condition_variable condition_;
  std::optional<restocker_interfaces::msg::RestockCoordinatorStatus> latest_;
  rclcpp::Subscription<restocker_interfaces::msg::RestockCoordinatorStatus>::SharedPtr
    subscription_;
};

class ExactDeleteInterceptionGuard final
{
public:
  ExactDeleteInterceptionGuard(
    ExactDeleteInterceptor & first,
    ExactDeleteInterceptor * second = nullptr)
  : first_(first), second_(second)
  {
    exact_delete_interceptors[0].store(&first_, std::memory_order_release);
    exact_delete_interceptors[1].store(second_, std::memory_order_release);
  }

  ~ExactDeleteInterceptionGuard()
  {
    first_.release();
    if (second_) {
      second_->release();
    }
    exact_delete_interceptors[0].store(nullptr, std::memory_order_release);
    exact_delete_interceptors[1].store(nullptr, std::memory_order_release);
  }

  ExactDeleteInterceptionGuard(const ExactDeleteInterceptionGuard &) = delete;
  ExactDeleteInterceptionGuard & operator=(const ExactDeleteInterceptionGuard &) = delete;

private:
  ExactDeleteInterceptor & first_;
  ExactDeleteInterceptor * second_;
};

class WorldStateFixture
{
public:
  explicit WorldStateFixture(
    const rclcpp::Node::SharedPtr & node, std::string lane_id = "lane_01",
    std::string service_prefix = "/test_world")
  : node_(node), lane_id_(std::move(lane_id))
  {
    snapshot_service_ = node_->create_service<restocker_interfaces::srv::GetWorldState>(
      service_prefix + "/get_snapshot",
      [this](restocker_interfaces::srv::GetWorldState::Request::SharedPtr,
      restocker_interfaces::srv::GetWorldState::Response::SharedPtr response) {
        {
          std::unique_lock gate_lock(snapshot_gate_mutex_);
          ++snapshot_count_;
          snapshot_gate_.notify_all();
          snapshot_gate_.wait(gate_lock, [this]() {return !snapshot_blocked_;});
        }
        response->snapshot = snapshot_message();
      });
    reserve_service_ = node_->create_service<restocker_interfaces::srv::ReserveTask>(
      service_prefix + "/reserve_task",
      [this](restocker_interfaces::srv::ReserveTask::Request::SharedPtr request,
      restocker_interfaces::srv::ReserveTask::Response::SharedPtr response) {
        reserve(*request, *response);
      });
    validate_service_ = node_->create_service<restocker_interfaces::srv::ValidateTaskReservation>(
      service_prefix + "/validate_reservation",
      [this](restocker_interfaces::srv::ValidateTaskReservation::Request::SharedPtr,
      restocker_interfaces::srv::ValidateTaskReservation::Response::SharedPtr response) {
        validate(*response);
      });
    release_service_ = node_->create_service<restocker_interfaces::srv::ReleaseTaskReservation>(
      service_prefix + "/release_reservation",
      [this](restocker_interfaces::srv::ReleaseTaskReservation::Request::SharedPtr request,
      restocker_interfaces::srv::ReleaseTaskReservation::Response::SharedPtr response) {
        release(*request, *response);
      });
    execution_authority_service_ =
      node_->create_service<restocker_interfaces::srv::ValidateExecutionWorldAuthority>(
      service_prefix + "/validate_execution_authority",
      [this](restocker_interfaces::srv::ValidateExecutionWorldAuthority::Request::SharedPtr
      request,
      restocker_interfaces::srv::ValidateExecutionWorldAuthority::Response::SharedPtr
      response) {validate_execution_authority(*request, *response);});
  }

  [[nodiscard]] std::size_t release_count() const
  {
    std::lock_guard lock(release_gate_mutex_);
    return release_count_;
  }

  [[nodiscard]] std::size_t snapshot_count() const noexcept {return snapshot_count_.load();}
  [[nodiscard]] std::size_t reserve_count() const noexcept {return reserve_count_.load();}
  [[nodiscard]] std::size_t validate_count() const noexcept {return validate_count_.load();}

  void block_snapshot()
  {
    std::lock_guard lock(snapshot_gate_mutex_);
    snapshot_blocked_ = true;
  }

  void unblock_snapshot()
  {
    {
      std::lock_guard lock(snapshot_gate_mutex_);
      snapshot_blocked_ = false;
    }
    snapshot_gate_.notify_all();
  }

  void seed_reserved_orphan()
  {
    std::lock_guard lock(mutex_);
    ++revision_;
    restocker_world_state::TaskReservation reservation;
    reservation.reservation_id = 81U;
    reservation.request_id = "orphaned-request-81";
    reservation.object_id = ObjectId{17U};
    reservation.object_source_id = "sim:can_17";
    reservation.product_class = ProductClass::Can;
    reservation.sku = "SIM-CAN-STD";
    reservation.destination_lane = LaneId{"lane_01"};
    reservation.destination_expected_product_class = ProductClass::Can;
    reservation.destination_expected_sku = "SIM-CAN-STD";
    reservation.stage = ReservationStage::Reserved;
    reservation.created_at = node_->now();
    reservation.created_revision = revision_;
    reservation.admitted_robot_telemetry_revision = revision_ - 1U;
    reservation.revision = revision_;
    reservation_ = reservation;
    task_phase_ = TaskPhase::Executing;
    fault_state_ = FaultState::None;
  }

  [[nodiscard]] bool reserved() const
  {
    std::lock_guard lock(mutex_);
    return reservation_.has_value();
  }

  void withdraw_release_service() {release_service_.reset();}

  void block_release()
  {
    std::lock_guard lock(release_gate_mutex_);
    release_blocked_ = true;
  }

  void unblock_release()
  {
    {
      std::lock_guard lock(release_gate_mutex_);
      release_blocked_ = false;
    }
    release_gate_.notify_all();
  }

private:
  restocker_world_state::WorldStateSnapshot snapshot() const
  {
    restocker_world_state::WorldStateSnapshot value;
    value.revision = revision_;
    value.robot.revision = revision_ - 1U;
    value.robot.telemetry_revision = revision_ - 1U;
    value.robot.telemetry_time = node_->now();
    value.robot.telemetry_source_id = "test/action-coordinator";
    value.robot.task_phase = task_phase_;
    value.robot.fault_state = fault_state_;
    value.active_reservation = reservation_;

    restocker_world_state::TrackedObject object;
    object.id = ObjectId{17U};
    object.source_object_id = "sim:can_17";
    object.product_class = ProductClass::Can;
    object.sku = "SIM-CAN-STD";
    object.pose_in_world.translation() = Eigen::Vector3d(-0.25, -0.80, 0.65);
    object.pose_covariance.setIdentity();
    object.orientation = ObjectOrientation::Upright;
    object.tracking_state = TrackingState::Tracked;
    object.grasp_state = GraspState::Free;
    object.observation_time = node_->now();
    object.transition_time = object.observation_time;
    object.revision = 17U;
    value.objects.emplace(object.id, object);

    restocker_world_state::ShelfLane lane;
    lane.id = LaneId{lane_id_};
    lane.expected_product_class = ProductClass::Can;
    lane.expected_sku = "SIM-CAN-STD";
    lane.depth_m = 0.85;
    lane.available_depth_m = 0.85;
    lane.last_verified = node_->now();
    lane.evidence_revision = 18U;
    lane.revision = 18U;
    value.lanes.emplace(lane.id, lane);
    return value;
  }

  restocker_interfaces::msg::WorldStateSnapshot snapshot_message() const
  {
    std::lock_guard lock(mutex_);
    return restocker_world_state::snapshot_to_message(
      snapshot(),
      restocker_world_state::SnapshotMessageOptions{"world", node_->now(), false, false});
  }

  void reserve(
    const restocker_interfaces::srv::ReserveTask::Request & request,
    restocker_interfaces::srv::ReserveTask::Response & response)
  {
    ++reserve_count_;
    std::lock_guard lock(mutex_);
    ++revision_;
    restocker_world_state::TaskReservation reservation;
    reservation.reservation_id = 71U;
    reservation.request_id = request.request_id;
    reservation.object_id = ObjectId{request.object_id};
    reservation.object_source_id = "sim:can_17";
    reservation.product_class = ProductClass::Can;
    reservation.sku = "SIM-CAN-STD";
    reservation.destination_lane = LaneId{request.destination_lane_id};
    // Captured from the destination lane's policy at grant, as the store does.
    reservation.destination_expected_product_class = ProductClass::Can;
    reservation.destination_expected_sku = "SIM-CAN-STD";
    reservation.stage = ReservationStage::Reserved;
    reservation.created_revision = revision_;
    reservation.admitted_robot_telemetry_revision = revision_ - 1;
    reservation.revision = revision_;
    reservation_ = reservation;
    task_phase_ = TaskPhase::Executing;
    fault_state_ = FaultState::None;
    response.status = restocker_world_state::operation_status_ok();
    response.world_revision = revision_;
    response.token = token_;
    response.reservation = restocker_world_state::task_reservation_to_message(reservation);
  }

  void validate(restocker_interfaces::srv::ValidateTaskReservation::Response & response) const
  {
    ++validate_count_;
    std::lock_guard lock(mutex_);
    response.status = restocker_world_state::operation_status_ok();
    response.world_revision = revision_;
    response.has_reservation = reservation_.has_value();
    if (reservation_) {
      response.reservation = restocker_world_state::task_reservation_to_message(*reservation_);
    }
  }

  void release(
    const restocker_interfaces::srv::ReleaseTaskReservation::Request & request,
    restocker_interfaces::srv::ReleaseTaskReservation::Response & response)
  {
    {
      std::unique_lock gate_lock(release_gate_mutex_);
      ++release_count_;
      release_gate_.notify_all();
      release_gate_.wait(gate_lock, [this]() {return !release_blocked_;});
    }
    std::lock_guard lock(mutex_);
    ++revision_;
    reservation_.reset();
    task_phase_ = static_cast<TaskPhase>(request.terminal_task_phase);
    fault_state_ = static_cast<FaultState>(request.terminal_fault_state);
    response.status = restocker_world_state::operation_status_ok();
    response.world_revision = revision_;
    ++revision_;
  }

  void validate_execution_authority(
    const restocker_interfaces::srv::ValidateExecutionWorldAuthority::Request & request,
    restocker_interfaces::srv::ValidateExecutionWorldAuthority::Response & response) const
  {
    std::lock_guard lock(mutex_);
    response.world_revision = revision_;
    const auto current = snapshot();
    const auto object = current.objects.find(ObjectId{request.expected_object_id});
    const auto lane = current.lanes.find(LaneId{request.expected_destination_lane_id});
    const bool matches =
      request.token == token_ && reservation_ &&
      reservation_->reservation_id == request.expected_reservation_id &&
      reservation_->revision == request.expected_reservation_revision &&
      reservation_->object_id.value == request.expected_object_id &&
      reservation_->destination_lane.value == request.expected_destination_lane_id &&
      object != current.objects.end() && lane != current.lanes.end();
    if (!matches) {
      response.status = restocker_world_state::operation_status_from_error(
        {restocker_world_state::WorldStateErrorCode::PredicateFailed,
          "test execution authority does not match retained reservation"});
      return;
    }
    response.status = restocker_world_state::operation_status_ok();
    response.planning_frame = "world";
    response.has_proof = true;
    response.reservation = restocker_world_state::task_reservation_to_message(*reservation_);
    response.object = restocker_world_state::tracked_object_to_message(object->second);
    response.destination_lane = restocker_world_state::shelf_lane_to_message(lane->second);
    response.robot = restocker_world_state::robot_execution_state_to_message(current.robot);
  }

  rclcpp::Node::SharedPtr node_;
  std::string lane_id_;
  mutable std::mutex mutex_;
  mutable std::mutex release_gate_mutex_;
  mutable std::mutex snapshot_gate_mutex_;
  std::condition_variable release_gate_;
  std::condition_variable snapshot_gate_;
  bool release_blocked_{false};
  bool snapshot_blocked_{false};
  std::uint64_t revision_{20U};
  std::optional<restocker_world_state::TaskReservation> reservation_;
  std::size_t release_count_{0U};
  std::atomic<std::size_t> snapshot_count_{0U};
  std::atomic<std::size_t> reserve_count_{0U};
  mutable std::atomic<std::size_t> validate_count_{0U};
  TaskPhase task_phase_{TaskPhase::Idle};
  FaultState fault_state_{FaultState::None};
  const std::string token_{"test-private-capability"};
  rclcpp::Service<restocker_interfaces::srv::GetWorldState>::SharedPtr snapshot_service_;
  rclcpp::Service<restocker_interfaces::srv::ReserveTask>::SharedPtr reserve_service_;
  rclcpp::Service<restocker_interfaces::srv::ValidateTaskReservation>::SharedPtr validate_service_;
  rclcpp::Service<restocker_interfaces::srv::ReleaseTaskReservation>::SharedPtr release_service_;
  rclcpp::Service<restocker_interfaces::srv::ValidateExecutionWorldAuthority>::SharedPtr
    execution_authority_service_;
};

class SnapshotBlockGuard final
{
public:
  explicit SnapshotBlockGuard(WorldStateFixture & fixture)
  : fixture_(fixture)
  {
    fixture_.block_snapshot();
  }

  ~SnapshotBlockGuard() {unblock();}

  SnapshotBlockGuard(const SnapshotBlockGuard &) = delete;
  SnapshotBlockGuard & operator=(const SnapshotBlockGuard &) = delete;

  void unblock()
  {
    if (blocked_) {
      fixture_.unblock_snapshot();
      blocked_ = false;
    }
  }

private:
  WorldStateFixture & fixture_;
  bool blocked_{true};
};

class ReleaseBlockGuard final
{
public:
  explicit ReleaseBlockGuard(WorldStateFixture & fixture)
  : fixture_(fixture)
  {
    fixture_.block_release();
  }

  ~ReleaseBlockGuard() {unblock();}

  ReleaseBlockGuard(const ReleaseBlockGuard &) = delete;
  ReleaseBlockGuard & operator=(const ReleaseBlockGuard &) = delete;

  void unblock()
  {
    if (blocked_) {
      fixture_.unblock_release();
      blocked_ = false;
    }
  }

private:
  WorldStateFixture & fixture_;
  bool blocked_{true};
};

class OneShotCallBlocker final
{
public:
  ~OneShotCallBlocker() {release();}

  OneShotCallBlocker(const OneShotCallBlocker &) = delete;
  OneShotCallBlocker & operator=(const OneShotCallBlocker &) = delete;

  OneShotCallBlocker() = default;

  void arm()
  {
    std::lock_guard lock(mutex_);
    armed_ = true;
    blocked_ = false;
    released_ = false;
  }

  void intercept()
  {
    std::unique_lock lock(mutex_);
    if (!armed_) {
      return;
    }
    armed_ = false;
    blocked_ = true;
    condition_.notify_all();
    (void)condition_.wait_for(lock, std::chrono::seconds(5), [this]() {return released_;});
  }

  [[nodiscard]] bool wait_until_blocked(std::chrono::milliseconds timeout)
  {
    std::unique_lock lock(mutex_);
    return condition_.wait_for(lock, timeout, [this]() {return blocked_;});
  }

  void release()
  {
    {
      std::lock_guard lock(mutex_);
      released_ = true;
    }
    condition_.notify_all();
  }

private:
  std::mutex mutex_;
  std::condition_variable condition_;
  bool armed_{false};
  bool blocked_{false};
  bool released_{false};
};

class RestockActionCoordinatorNodeTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
};

class ExecutorSpinGuard
{
public:
  explicit ExecutorSpinGuard(rclcpp::Executor & executor)
  : executor_(executor), thread_([this]() {executor_.spin();})
  {
  }

  ~ExecutorSpinGuard() {executor_.cancel();}

private:
  rclcpp::Executor & executor_;
  std::jthread thread_;
};

[[nodiscard]] std::shared_ptr<rclcpp_action::ClientGoalHandle<Action>> send_goal_until_accepted(
  const rclcpp_action::Client<Action>::SharedPtr & client, const Action::Goal & goal)
{
  for (std::size_t attempt = 0U; attempt < 40U; ++attempt) {
    auto sent = client->async_send_goal(goal);
    if (sent.wait_for(500ms) == std::future_status::ready) {
      auto handle = sent.get();
      if (handle) {
        return handle;
      }
    }
    std::this_thread::sleep_for(25ms);
  }
  return {};
}

[[nodiscard]] std::optional<CoordinatorGenerationAuthoritySnapshot>
wait_for_fresh_returned_pump_lease(
  const std::shared_ptr<RestockActionCoordinatorNode> & coordinator,
  std::chrono::milliseconds timeout)
{
  const auto initial = coordinator->generation_authority_snapshot();
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  do {
    const auto observed = coordinator->generation_authority_snapshot();
    const bool fresh_attempt =
      initial.pump_lease.outstanding || observed.pump_lease.attempt > initial.pump_lease.attempt;
    if (fresh_attempt && !observed.pump_lease.outstanding) {
      return observed;
    }
    std::this_thread::yield();
  } while (std::chrono::steady_clock::now() < deadline);
  return std::nullopt;
}

[[nodiscard]] std::optional<CoordinatorGenerationAuthoritySnapshot>
wait_for_active_retirement_after_completed_pumps(
  const std::shared_ptr<RestockActionCoordinatorNode> & coordinator,
  std::size_t completed_pump_budget, std::chrono::milliseconds timeout)
{
  if (completed_pump_budget == 0U) {
    return std::nullopt;
  }
  auto observed = coordinator->generation_authority_snapshot();
  if (!observed.active_binding) {
    return observed;
  }
  if (observed.pump_lease.outstanding && observed.pump_lease.attempt == 0U) {
    return std::nullopt;
  }
  auto counted_through_attempt = observed.pump_lease.attempt -
    static_cast<PumpLeaseAttempt>(observed.pump_lease.outstanding);
  std::size_t completed_pumps = 0U;
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  do {
    observed = coordinator->generation_authority_snapshot();
    if (!observed.active_binding) {
      return observed;
    }
    if (!observed.pump_lease.outstanding &&
      observed.pump_lease.attempt > counted_through_attempt)
    {
      const auto newly_completed = observed.pump_lease.attempt - counted_through_attempt;
      completed_pumps += static_cast<std::size_t>(std::min<PumpLeaseAttempt>(
          newly_completed, completed_pump_budget - completed_pumps));
      counted_through_attempt = observed.pump_lease.attempt;
      if (completed_pumps >= completed_pump_budget) {
        return std::nullopt;
      }
    }
    std::this_thread::yield();
  } while (std::chrono::steady_clock::now() < deadline);
  return std::nullopt;
}

// Milestone 10 §6 (Card 060): world state admits exactly the float excess past a joint bound
// that the motion port clamps as a start-state correction, so both consumers of one joint state
// agree on what is admissible. Changing one default without the other fails here.
TEST(JointLimitToleranceAgreement, WorldStateAdmitsExactlyTheMotionPortCorrection)
{
  EXPECT_EQ(
    restocker_world_state::WorldStateConfig{}.joint_limit_tolerance,
    MoveItMotionPortConfig{}.maximum_start_state_bounds_correction);
}

TEST_F(RestockActionCoordinatorNodeTest, RejectsInvalidSelectionConfiguration)
{
  rclcpp::NodeOptions missing_gripper;
  missing_gripper.parameter_overrides(
    {rclcpp::Parameter("product_catalog_path", RESTOCKER_TEST_PRODUCT_CATALOG),
      rclcpp::Parameter("workcell_geometry_path", RESTOCKER_TEST_WORKCELL_GEOMETRY)});
  EXPECT_THROW(
    (void)std::make_shared<RestockActionCoordinatorNode>(missing_gripper),
    std::invalid_argument);

  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {rclcpp::Parameter("product_catalog_path", RESTOCKER_TEST_PRODUCT_CATALOG),
      rclcpp::Parameter("workcell_geometry_path", RESTOCKER_TEST_WORKCELL_GEOMETRY),
      rclcpp::Parameter("gripper_geometry_path", RESTOCKER_TEST_GRIPPER_GEOMETRY),
      rclcpp::Parameter("selection.maximum_upright_tilt_rad", 1.5707963267948966)});
  EXPECT_THROW(
    (void)std::make_shared<RestockActionCoordinatorNode>(options),
    std::invalid_argument);

  rclcpp::NodeOptions short_shutdown;
  short_shutdown.parameter_overrides(
    {rclcpp::Parameter("product_catalog_path", RESTOCKER_TEST_PRODUCT_CATALOG),
      rclcpp::Parameter("workcell_geometry_path", RESTOCKER_TEST_WORKCELL_GEOMETRY),
      rclcpp::Parameter("gripper_geometry_path", RESTOCKER_TEST_GRIPPER_GEOMETRY),
      rclcpp::Parameter("shutdown_timeout_ms", 16079)});
  EXPECT_THROW(
    (void)std::make_shared<RestockActionCoordinatorNode>(short_shutdown),
    std::invalid_argument);

  rclcpp::NodeOptions invalid_startup;
  invalid_startup.parameter_overrides(
    {rclcpp::Parameter("product_catalog_path", RESTOCKER_TEST_PRODUCT_CATALOG),
      rclcpp::Parameter("workcell_geometry_path", RESTOCKER_TEST_WORKCELL_GEOMETRY),
      rclcpp::Parameter("gripper_geometry_path", RESTOCKER_TEST_GRIPPER_GEOMETRY),
      rclcpp::Parameter("startup.snapshot_timeout_ms", 0)});
  EXPECT_THROW(
    (void)std::make_shared<RestockActionCoordinatorNode>(invalid_startup),
    std::invalid_argument);
}

TEST_F(RestockActionCoordinatorNodeTest, ValidatesImmutableGenerationDepositCapacity)
{
  {
    auto options = ready_coordinator_options("/default_deposit_capacity_restock_product");
    auto coordinator = std::make_shared<RestockActionCoordinatorNode>(options);
    EXPECT_EQ(coordinator->get_parameter("generation_deposit_capacity").as_int(), 64);
    const auto changed =
      coordinator->set_parameter(rclcpp::Parameter("generation_deposit_capacity", 65));
    EXPECT_FALSE(changed.successful);
  }
  {
    auto options = ready_coordinator_options("/equal_deposit_capacity_restock_product");
    options.append_parameter_override("executor_threads", 3);
    options.append_parameter_override("generation_deposit_capacity", 3);
    EXPECT_NO_THROW((void)std::make_shared<RestockActionCoordinatorNode>(options));
  }
  {
    auto options = ready_coordinator_options("/maximum_deposit_capacity_restock_product");
    options.append_parameter_override("generation_deposit_capacity", 4096);
    EXPECT_NO_THROW((void)std::make_shared<RestockActionCoordinatorNode>(options));
  }
  for (const auto capacity : {0, 2, 4097}) {
    auto options = ready_coordinator_options(
      "/invalid_deposit_capacity_" +
      std::to_string(capacity) + "_restock_product");
    options.append_parameter_override("generation_deposit_capacity", capacity);
    EXPECT_THROW(
      (void)std::make_shared<RestockActionCoordinatorNode>(options),
      std::invalid_argument);
  }
  {
    auto options = ready_coordinator_options("/negative_deposit_capacity_restock_product");
    options.append_parameter_override("generation_deposit_capacity", -1);
    EXPECT_THROW(
      (void)std::make_shared<RestockActionCoordinatorNode>(options),
      std::invalid_argument);
  }
  {
    auto options = ready_coordinator_options("/excessive_executor_threads_restock_product");
    options.append_parameter_override("executor_threads", 4097);
    EXPECT_THROW(
      (void)std::make_shared<RestockActionCoordinatorNode>(options),
      std::invalid_argument);
  }
  {
    auto options = ready_coordinator_options("/independent_deposit_capacity_restock_product");
    options.append_parameter_override("inbox_capacity", 1);
    options.append_parameter_override("generation_deposit_capacity", 64);
    EXPECT_NO_THROW((void)std::make_shared<RestockActionCoordinatorNode>(options));
  }
  {
    auto options = ready_coordinator_options("/reverse_independent_capacity_restock_product");
    options.append_parameter_override("executor_threads", 3);
    options.append_parameter_override("inbox_capacity", 64);
    options.append_parameter_override("generation_deposit_capacity", 3);
    EXPECT_NO_THROW((void)std::make_shared<RestockActionCoordinatorNode>(options));
  }
}

TEST_F(RestockActionCoordinatorNodeTest, RejectsNullActionResultPublisher)
{
  auto dependencies = RestockActionCoordinatorNodeDependencies{};
  dependencies.action_result_publisher.reset();
  EXPECT_THROW(
    (void)std::make_shared<RestockActionCoordinatorNode>(
      ready_coordinator_options("/null_action_result_publisher_restock_product"),
      std::move(dependencies)),
    std::invalid_argument);
}

TEST_F(RestockActionCoordinatorNodeTest, OldPumpLeaseBlocksSealedAcceptedEnvelopeUntilExactReturn)
{
  CoordinatorGoalId goal_id{};
  goal_id.front() = 9U;
  constexpr GoalGeneration kGeneration = 41U;
  const PendingHandoffBindingKey binding{goal_id, kGeneration, 3U};
  CoordinatorPumpLeaseGate pump_gate;
  PendingAcceptedHandoffMachine handoff(binding);
  CoordinatorInbox inbox(4U);
  std::mutex authority_mutex;
  bool pending_binding_published = false;
  std::optional<CoordinatorPumpLease> old_lease;
  std::optional<CoordinatorPumpLeaseAcquireStatus> acquire_status;
  std::optional<CoordinatorPumpLeaseReturnStatus> return_status;
  OneShotCallBlocker old_pump_blocker;
  old_pump_blocker.arm();

  std::jthread old_pump([&]() {
      {
        std::lock_guard lock(authority_mutex);
        auto acquired = pump_gate.acquire(pending_binding_published);
        acquire_status = acquired.status();
        old_lease = acquired.take_lease();
      }
      old_pump_blocker.intercept();
      std::lock_guard lock(authority_mutex);
      if (old_lease) {
        return_status = pump_gate.return_lease(*old_lease);
      }
    });

  const bool old_pump_blocked = old_pump_blocker.wait_until_blocked(2s);
  if (!old_pump_blocked) {
    old_pump_blocker.release();
    old_pump.join();
  }
  ASSERT_TRUE(old_pump_blocked);

  bool old_lease_was_live = false;
  {
    std::lock_guard lock(authority_mutex);
    old_lease_was_live = old_lease && old_lease->live();
    pending_binding_published = true;
    EXPECT_EQ(handoff.install_epoch(binding), PendingHandoffPhaseEventStatus::kApplied);
    EXPECT_EQ(handoff.adopt_accepted_handle(binding), PendingHandoffPhaseEventStatus::kApplied);
    auto waiting = handoff.advance({pump_gate.snapshot(), false});
    EXPECT_EQ(waiting.status(), PendingHandoffAdvanceStatus::kWait);
    EXPECT_FALSE(waiting.take_work());
    EXPECT_EQ(handoff.snapshot().phase, PendingAcceptedHandoffPhase::kWaitingForPriorPumpLease);
  }
  const auto blocked_inbox = inbox.snapshot();
  EXPECT_EQ(blocked_inbox.size, 0U);
  EXPECT_EQ(blocked_inbox.accepted_handoff_emergency_size, 0U);

  old_pump_blocker.release();
  old_pump.join();
  ASSERT_TRUE(acquire_status);
  EXPECT_EQ(*acquire_status, CoordinatorPumpLeaseAcquireStatus::kAcquired);
  ASSERT_TRUE(old_lease);
  EXPECT_TRUE(old_lease_was_live);
  ASSERT_TRUE(return_status);
  EXPECT_EQ(*return_status, CoordinatorPumpLeaseReturnStatus::kReturned);
  EXPECT_FALSE(old_lease->live());

  auto prepare = handoff.advance({pump_gate.snapshot(), false});
  ASSERT_EQ(prepare.status(), PendingHandoffAdvanceStatus::kPrepareAcceptedEnvelope);
  auto preparation = prepare.take_work();
  ASSERT_TRUE(preparation);
  auto prepared = handoff.complete_preparation(*preparation);
  ASSERT_EQ(prepared.status(), PendingPreparationCompletionStatus::kPublishAcceptedEnvelope);
  auto publication = prepared.take_handoff_work();
  ASSERT_TRUE(publication);

  CoordinatorAcceptedGoal accepted;
  accepted.goal_id = goal_id;
  accepted.goal_generation = kGeneration;
  accepted.sealed_pending_control_handoff = true;
  const auto deposited =
    inbox.push_accepted_goal(std::move(accepted), std::nullopt, SteadyTime{} + 7ms);
  EXPECT_EQ(
    classify_accepted_envelope_deposit(deposited),
    AcceptedEnvelopeDepositDecision::kCommitted);
  EXPECT_EQ(
    handoff.complete_accepted_publication(
      *publication,
      classify_accepted_envelope_deposit(deposited)),
    PendingAcceptedPublicationStatus::kCommitted);
  auto ready = handoff.advance({pump_gate.snapshot(), false});
  EXPECT_EQ(ready.status(), PendingHandoffAdvanceStatus::kReadyToActivate);
  EXPECT_FALSE(ready.take_work());

  auto delivery = inbox.try_pop();
  ASSERT_TRUE(delivery);
  const auto * delivered_accepted = std::get_if<CoordinatorAcceptedGoal>(&*delivery);
  ASSERT_NE(delivered_accepted, nullptr);
  EXPECT_EQ(delivered_accepted->goal_id, goal_id);
  EXPECT_EQ(delivered_accepted->goal_generation, kGeneration);
  EXPECT_TRUE(delivered_accepted->sealed_pending_control_handoff);
  EXPECT_FALSE(inbox.try_pop());
}

TEST_F(RestockActionCoordinatorNodeTest, RollsBackEveryPreacceptAuthorityAllocationFailure)
{
  auto support = std::make_shared<rclcpp::Node>("preaccept_allocation_test_support");
  const std::string world_state_prefix{"/preaccept_allocation_world"};
  WorldStateFixture world_state(support, "lane_01", world_state_prefix);
  tf2_ros::StaticTransformBroadcaster broadcaster(support);
  geometry_msgs::msg::TransformStamped shelf;
  shelf.header.stamp = support->now();
  shelf.header.frame_id = "world";
  shelf.child_frame_id = "shelf";
  shelf.transform.translation.y = 0.55;
  shelf.transform.translation.z = 0.75;
  shelf.transform.rotation.w = 1.0;
  broadcaster.sendTransform(shelf);
  send_grasp_center_transform(broadcaster, support->now());

  std::atomic<std::size_t> arm_after_clock{kAllocationDisarmed};
  std::atomic<std::size_t> post_install_hook_calls{0U};
  std::atomic<bool> block_control_hook{false};
  OneShotCallBlocker control_hook_blocker;
  const std::string status_topic{"/preaccept_allocation_coordinator/status"};
  AdmissionReadyObserver readiness(support, status_topic);
  auto options =
    ready_coordinator_options("/preaccept_allocation_restock_product", world_state_prefix);
  options.append_parameter_override("status_topic", status_topic);
  options.append_parameter_override("executor_threads", 3);
  options.append_parameter_override("pump_period_ms", 2000);
  options.append_parameter_override("shutdown_timeout_ms", 25000);
  auto coordinator = std::make_shared<RestockActionCoordinatorNode>(
    options, RestockActionCoordinatorNodeDependencies{
        [&arm_after_clock]() {
          arm_allocation_injection_from_clock(arm_after_clock);
          return std::chrono::steady_clock::now();
        },
        [&post_install_hook_calls, &block_control_hook, &control_hook_blocker, &arm_after_clock]() {
          ++post_install_hook_calls;
          // Authority region complete: disarm this thread and the shared arm source, or a later
          // allocation (or a pump thread armed from the same source) fails uncaught.
          arm_after_clock.store(kAllocationDisarmed, std::memory_order_release);
          reset_allocation_injection();
          if (block_control_hook.exchange(false, std::memory_order_acq_rel)) {
            control_hook_blocker.intercept();
          }
        },
        {}});
  auto client =
    rclcpp_action::create_client<Action>(support, "/preaccept_allocation_restock_product");
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions{}, 3U);
  executor.add_node(support);
  executor.add_node(coordinator);
  ExecutorSpinGuard spin(executor);
  ASSERT_TRUE(client->wait_for_action_server(2s));
  ASSERT_TRUE(readiness.wait_until_ready(7s));
  ASSERT_TRUE(wait_for_fresh_returned_pump_lease(coordinator, 5s).has_value());
  const auto startup_snapshot_count = world_state.snapshot_count();

  Action::Goal goal;
  goal.has_object_id = true;
  goal.object_id = 17U;
  goal.has_lane_id = true;
  goal.lane_id = std::string(kAllocationTriggerLaneLength, 'l');

  // The distinctive lane copy anchors the script to this goal callback; one admission-record
  // allocation precedes the authority allocations.
  //
  // Card 078: every window below is opened right after a completed pump tick. The pump reads the
  // injected clock on every pump_period_ms tick, and a thread it arms first fires the injection
  // on the goal's own lane copy during deserialization - before handle_goal has read the clock -
  // so the failure lands outside handle_goal's catch(...) instead of in the authority region.
  // The next tick is a full 2000 ms away while the window is milliseconds long; the attempt
  // counter asserted after each window fails closed if a tick ever lands inside one anyway.
  const auto pump_attempt_at_window_open = [&]() {
    const auto drained = wait_for_fresh_returned_pump_lease(coordinator, 5s);
    if (!drained) {
      ADD_FAILURE() << "the pump did not complete a tick to drain before the window";
      return std::size_t{0U};
    }
    return drained->pump_lease.attempt;
  };
  const auto expect_window_pump_idle = [&](std::size_t attempt_at_open, const char * what) {
    EXPECT_EQ(
      coordinator->generation_authority_snapshot().pump_lease.attempt, attempt_at_open) << what;
  };

  constexpr std::size_t kPreacceptAllocations = 3U;
  for (std::size_t allocation_countdown = 1U; allocation_countdown <= kPreacceptAllocations;
    ++allocation_countdown)
  {
    const auto pump_attempt_before = pump_attempt_at_window_open();
    arm_after_clock.store(allocation_countdown, std::memory_order_release);
    auto sent = client->async_send_goal(goal);
    const auto send_status = sent.wait_for(2s);
    arm_after_clock.store(kAllocationDisarmed, std::memory_order_release);
    expect_window_pump_idle(
      pump_attempt_before, "a pump tick landed inside the injection window");
    ASSERT_EQ(send_status, std::future_status::ready);
    EXPECT_FALSE(sent.get()) << "allocation countdown " << allocation_countdown;
    const auto rolled_back = coordinator->generation_authority_snapshot();
    EXPECT_FALSE(rolled_back.pending_binding);
    EXPECT_FALSE(rolled_back.active_binding);
    EXPECT_FALSE(rolled_back.pending_epoch.has_value());
    EXPECT_FALSE(rolled_back.pending_quiescence.has_value());
    EXPECT_EQ(post_install_hook_calls.load(), 0U);
    EXPECT_EQ(world_state.snapshot_count(), startup_snapshot_count);
    EXPECT_EQ(world_state.reserve_count(), 0U);
  }

  // One past the last failing countdown is the non-failing boundary.
  control_hook_blocker.arm();
  block_control_hook.store(true, std::memory_order_release);
  const auto control_pump_attempt_before = pump_attempt_at_window_open();
  arm_after_clock.store(kPreacceptAllocations + 1U, std::memory_order_release);
  auto control_sent = client->async_send_goal(goal);
  const bool control_hook_blocked = control_hook_blocker.wait_until_blocked(2s);
  arm_after_clock.store(kAllocationDisarmed, std::memory_order_release);
  expect_window_pump_idle(
    control_pump_attempt_before, "a pump tick landed inside the control injection window");
  if (control_hook_blocked) {
    const auto pending_authority = coordinator->generation_authority_snapshot();
    EXPECT_TRUE(pending_authority.pending_epoch);
    EXPECT_TRUE(pending_authority.pending_quiescence);
    if (pending_authority.pending_epoch && pending_authority.pending_quiescence) {
      EXPECT_EQ(
        pending_authority.pending_epoch->goal_generation, kPreacceptAllocations + 1U);
      EXPECT_EQ(
        pending_authority.pending_quiescence->generation, kPreacceptAllocations + 1U);
    }
  }
  control_hook_blocker.release();
  ASSERT_TRUE(control_hook_blocked);
  ASSERT_EQ(control_sent.wait_for(2s), std::future_status::ready);
  const auto accepted = control_sent.get();
  ASSERT_TRUE(accepted);
  EXPECT_EQ(post_install_hook_calls.load(), 1U);
  auto result = client->async_get_result(accepted);
  // The 2 s pump period preserves the handoff boundary above; selection failure needs two later
  // pump ticks, so leave more than one tick of slack.
  ASSERT_EQ(result.wait_for(10s), std::future_status::ready);
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::ABORTED);

  coordinator->request_shutdown();
}

// Milestone 10 §6 result finality (Card 070): rclcpp_action answers a goal before it registers
// the goal, and answers a result request for an unregistered goal with STATUS_UNKNOWN at once. A
// result request that overtakes admission must wait for it and then receive the coordinator's
// own terminal result; a client must never be handed UNKNOWN for a goal the coordinator accepted.
TEST_F(RestockActionCoordinatorNodeTest, ResultRequestWaitsForGoalAdmission)
{
  auto support = std::make_shared<rclcpp::Node>("result_finality_test_support");
  const std::string world_state_prefix{"/result_finality_world"};
  WorldStateFixture world_state(support, "lane_01", world_state_prefix);
  tf2_ros::StaticTransformBroadcaster broadcaster(support);
  geometry_msgs::msg::TransformStamped shelf;
  shelf.header.stamp = support->now();
  shelf.header.frame_id = "world";
  shelf.child_frame_id = "shelf";
  shelf.transform.translation.y = 0.55;
  shelf.transform.translation.z = 0.75;
  shelf.transform.rotation.w = 1.0;
  broadcaster.sendTransform(shelf);
  send_grasp_center_transform(broadcaster, support->now());

  OneShotCallBlocker admission_blocker;
  const std::string action_name{"/result_finality_restock_product"};
  const std::string status_topic{"/result_finality_coordinator/status"};
  AdmissionReadyObserver readiness(support, status_topic);
  auto options = ready_coordinator_options(action_name, world_state_prefix);
  options.append_parameter_override("status_topic", status_topic);
  options.append_parameter_override("executor_threads", 3);
  auto coordinator = std::make_shared<RestockActionCoordinatorNode>(
    options, RestockActionCoordinatorNodeDependencies{
        []() {return std::chrono::steady_clock::now();},
        [&admission_blocker]() {admission_blocker.intercept();}, {}});
  // Raw service clients, so the result request can be sent for the goal ID while admission is
  // held inside the goal callback, before the goal response is sent. A real client sends it
  // only after reading the response, before rcl_action_accept_new_goal; both reach the same
  // unregistered-goal window, and serialising the whole goal execute closes both.
  auto send_goal = support->create_client<Action::Impl::SendGoalService>(
    action_name + "/_action/send_goal");
  auto get_result = support->create_client<Action::Impl::GetResultService>(
    action_name + "/_action/get_result");
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions{}, 3U);
  executor.add_node(support);
  executor.add_node(coordinator);
  ExecutorSpinGuard spin(executor);
  ASSERT_TRUE(send_goal->wait_for_service(2s));
  ASSERT_TRUE(get_result->wait_for_service(2s));
  ASSERT_TRUE(readiness.wait_until_ready(7s));

  auto goal_request = std::make_shared<Action::Impl::SendGoalService::Request>();
  for (std::size_t index = 0U; index < goal_request->goal_id.uuid.size(); ++index) {
    goal_request->goal_id.uuid[index] = static_cast<std::uint8_t>(0x70U + index);
  }
  goal_request->goal.has_object_id = true;
  goal_request->goal.object_id = 17U;
  goal_request->goal.has_lane_id = true;
  // No such lane: the goal is admitted and then ends ABORTED at selection.
  goal_request->goal.lane_id = "result_finality_missing_lane";

  admission_blocker.arm();
  auto goal_response = send_goal->async_send_request(goal_request);
  ASSERT_TRUE(admission_blocker.wait_until_blocked(2s));

  auto result_request = std::make_shared<Action::Impl::GetResultService::Request>();
  result_request->goal_id = goal_request->goal_id;
  auto result_response = get_result->async_send_request(result_request).future.share();
  // Admission is held inside the goal callback: nothing about this goal is final yet, so the
  // result request must not be answered, least of all with STATUS_UNKNOWN.
  const bool answered_during_admission =
    result_response.wait_for(300ms) == std::future_status::ready;
  EXPECT_FALSE(answered_during_admission);
  if (answered_during_admission) {
    EXPECT_NE(
      result_response.get()->status, action_msgs::msg::GoalStatus::STATUS_UNKNOWN);
  }

  admission_blocker.release();
  ASSERT_EQ(goal_response.wait_for(2s), std::future_status::ready);
  ASSERT_TRUE(goal_response.get()->accepted);
  ASSERT_EQ(result_response.wait_for(10s), std::future_status::ready);
  const auto result = result_response.get();
  EXPECT_EQ(result->status, action_msgs::msg::GoalStatus::STATUS_ABORTED);
  EXPECT_NE(result->result.status, Action::Result::STATUS_SUCCEEDED);

  coordinator->request_shutdown();
}

TEST_F(RestockActionCoordinatorNodeTest, InstallsGenerationAuthorityBeforeAcceptingGoal)
{
  auto support = std::make_shared<rclcpp::Node>("preaccept_install_test_support");
  WorldStateFixture world_state(support);
  tf2_ros::StaticTransformBroadcaster broadcaster(support);
  geometry_msgs::msg::TransformStamped shelf;
  shelf.header.stamp = support->now();
  shelf.header.frame_id = "world";
  shelf.child_frame_id = "shelf";
  shelf.transform.translation.y = 0.55;
  shelf.transform.translation.z = 0.75;
  shelf.transform.rotation.w = 1.0;
  broadcaster.sendTransform(shelf);
  send_grasp_center_transform(broadcaster, support->now());

  OneShotCallBlocker post_install_hook;
  post_install_hook.arm();
  std::optional<CoordinatorGenerationAuthoritySnapshot> observed_authority;
  std::weak_ptr<RestockActionCoordinatorNode> weak_coordinator;
  const std::string status_topic{"/preaccept_install_coordinator/status"};
  AdmissionReadyObserver readiness(support, status_topic);
  auto options = ready_coordinator_options("/preaccept_install_restock_product");
  options.append_parameter_override("status_topic", status_topic);
  options.append_parameter_override("executor_threads", 3);
  options.append_parameter_override("generation_deposit_capacity", 3);
  auto coordinator = std::make_shared<RestockActionCoordinatorNode>(
    options, RestockActionCoordinatorNodeDependencies{
        []() {return std::chrono::steady_clock::now();},
        [&post_install_hook, &observed_authority, &weak_coordinator]() {
          if (const auto locked = weak_coordinator.lock()) {
            observed_authority = locked->generation_authority_snapshot();
          }
          post_install_hook.intercept();
        },
        {}});
  weak_coordinator = coordinator;
  auto client = rclcpp_action::create_client<Action>(support, "/preaccept_install_restock_product");
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions{}, 3U);
  executor.add_node(support);
  executor.add_node(coordinator);
  ExecutorSpinGuard spin(executor);
  ASSERT_TRUE(client->wait_for_action_server(2s));
  ASSERT_TRUE(readiness.wait_until_ready(4s));

  Action::Goal goal;
  goal.has_object_id = true;
  goal.object_id = 17U;
  goal.has_lane_id = true;
  goal.lane_id = "lane_01";
  auto sent = client->async_send_goal(goal);
  const bool hook_blocked = post_install_hook.wait_until_blocked(2s);
  EXPECT_TRUE(hook_blocked);
  EXPECT_EQ(sent.wait_for(100ms), std::future_status::timeout);
  EXPECT_TRUE(observed_authority.has_value());
  if (observed_authority) {
    EXPECT_EQ(observed_authority->configured_deposit_capacity, 3U);
    EXPECT_TRUE(observed_authority->pending_binding);
    EXPECT_FALSE(observed_authority->active_binding);
    EXPECT_FALSE(observed_authority->accepted_handle_fail_stopped);
    EXPECT_EQ(observed_authority->orphaned_accepted_handle_count, 0U);
    EXPECT_FALSE(observed_authority->pending_accepted_handle_identity);
    EXPECT_FALSE(observed_authority->active_accepted_handle_identity);
    EXPECT_TRUE(observed_authority->pending_handoff_phase);
    if (observed_authority->pending_handoff_phase) {
      EXPECT_EQ(
        *observed_authority->pending_handoff_phase,
        PendingAcceptedHandoffPhase::kAwaitingAcceptedHandle);
    }
    EXPECT_FALSE(observed_authority->pending_route_token_live);
    EXPECT_FALSE(observed_authority->pending_handoff_work_token_live);
    EXPECT_FALSE(observed_authority->pending_retained_control_occupied);
    EXPECT_FALSE(observed_authority->pending_deferred_control_occupied);
    EXPECT_TRUE(observed_authority->pending_epoch.has_value());
    EXPECT_TRUE(observed_authority->pending_quiescence.has_value());
    if (observed_authority->pending_epoch && observed_authority->pending_quiescence) {
      EXPECT_EQ(
        observed_authority->pending_epoch->goal_id,
        observed_authority->pending_quiescence->goal_id);
      EXPECT_EQ(
        observed_authority->pending_epoch->goal_generation,
        observed_authority->pending_quiescence->generation);
      EXPECT_NE(observed_authority->pending_epoch->goal_generation, 0U);
      EXPECT_FALSE(observed_authority->pending_quiescence->sealed);
      EXPECT_EQ(observed_authority->pending_quiescence->active_deposit_count, 0U);
    }
  }
  post_install_hook.release();
  ASSERT_TRUE(hook_blocked);
  ASSERT_EQ(sent.wait_for(2s), std::future_status::ready);
  const auto accepted = sent.get();
  ASSERT_TRUE(accepted);

  const auto active_deadline = std::chrono::steady_clock::now() + 2s;
  CoordinatorGenerationAuthoritySnapshot active_authority;
  do {
    active_authority = coordinator->generation_authority_snapshot();
    if (active_authority.active_binding) {
      break;
    }
    std::this_thread::yield();
  } while (std::chrono::steady_clock::now() < active_deadline);
  ASSERT_TRUE(active_authority.active_binding);
  EXPECT_FALSE(active_authority.pending_binding);
  EXPECT_FALSE(active_authority.pending_handoff_phase);
  EXPECT_FALSE(active_authority.pending_route_token_live);
  EXPECT_FALSE(active_authority.pending_handoff_work_token_live);
  EXPECT_FALSE(active_authority.pending_retained_control_occupied);
  EXPECT_FALSE(active_authority.pending_deferred_control_occupied);
  EXPECT_FALSE(active_authority.accepted_handle_fail_stopped);
  EXPECT_EQ(active_authority.orphaned_accepted_handle_count, 0U);
  EXPECT_FALSE(active_authority.pending_accepted_handle_identity);
  EXPECT_TRUE(active_authority.active_accepted_handle_identity);
  ASSERT_TRUE(active_authority.active_epoch);
  ASSERT_TRUE(active_authority.active_quiescence);
  EXPECT_EQ(active_authority.active_epoch->goal_id, active_authority.active_quiescence->goal_id);
  EXPECT_EQ(
    active_authority.active_epoch->goal_generation,
    active_authority.active_quiescence->generation);

  auto canceled = client->async_cancel_goal(accepted);
  ASSERT_EQ(canceled.wait_for(2s), std::future_status::ready);
  ASSERT_FALSE(canceled.get()->goals_canceling.empty());
  const auto cancel_authority = coordinator->generation_authority_snapshot();
  ASSERT_TRUE(cancel_authority.active_epoch);
  EXPECT_EQ(cancel_authority.active_epoch->route_phase, ActiveFaultRoutePhase::kSecured);
  EXPECT_FALSE(cancel_authority.active_epoch->route_claim_outstanding);
  auto result = client->async_get_result(accepted);
  ASSERT_EQ(result.wait_for(5s), std::future_status::ready);
  coordinator->request_shutdown();
}

TEST_F(RestockActionCoordinatorNodeTest, PreservesPendingDrainDuringAuthorityConstruction)
{
  auto support = std::make_shared<rclcpp::Node>("preaccept_drain_race_test_support");
  const std::string world_state_prefix{"/preaccept_drain_race_world"};
  WorldStateFixture world_state(support, "lane_01", world_state_prefix);
  tf2_ros::StaticTransformBroadcaster broadcaster(support);
  geometry_msgs::msg::TransformStamped shelf;
  shelf.header.stamp = support->now();
  shelf.header.frame_id = "world";
  shelf.child_frame_id = "shelf";
  shelf.transform.translation.y = 0.55;
  shelf.transform.translation.z = 0.75;
  shelf.transform.rotation.w = 1.0;
  broadcaster.sendTransform(shelf);
  send_grasp_center_transform(broadcaster, support->now());

  std::atomic<std::size_t> arm_after_clock{kAllocationDisarmed};
  AllocationCallBlocker construction_blocker;
  const std::string status_topic{"/preaccept_drain_race_coordinator/status"};
  AdmissionReadyObserver readiness(support, status_topic);
  auto options =
    ready_coordinator_options("/preaccept_drain_race_restock_product", world_state_prefix);
  options.append_parameter_override("status_topic", status_topic);
  options.append_parameter_override("executor_threads", 3);
  options.append_parameter_override("pump_period_ms", 2000);
  options.append_parameter_override("shutdown_timeout_ms", 25000);
  auto coordinator = std::make_shared<RestockActionCoordinatorNode>(
    options, RestockActionCoordinatorNodeDependencies{
        [&arm_after_clock, &construction_blocker]() {
          arm_allocation_injection_from_clock(
            arm_after_clock, kAllocationTriggerLaneLength + 1U,
            &construction_blocker);
          return std::chrono::steady_clock::now();
        },
        {},
        {}});
  auto client =
    rclcpp_action::create_client<Action>(support, "/preaccept_drain_race_restock_product");
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions{}, 3U);
  executor.add_node(support);
  executor.add_node(coordinator);
  ExecutorSpinGuard spin(executor);
  ASSERT_TRUE(client->wait_for_action_server(2s));
  ASSERT_TRUE(readiness.wait_until_ready(7s));
  ASSERT_TRUE(wait_for_fresh_returned_pump_lease(coordinator, 5s).has_value());

  Action::Goal goal;
  goal.has_object_id = true;
  goal.object_id = 17U;
  goal.has_lane_id = true;
  goal.lane_id = std::string(kAllocationTriggerLaneLength, 'l');
  // Every executor thread sampling the clock while armed gets the thread-local script; the
  // distinctive lane copy disarms the shared trigger and anchors the block to this goal callback.
  arm_after_clock.store(1U, std::memory_order_release);
  auto sent = client->async_send_goal(goal);
  const bool construction_blocked = construction_blocker.wait_until_blocked(4s);
  arm_after_clock.store(kAllocationDisarmed, std::memory_order_release);
  if (construction_blocked) {
    const auto pending = coordinator->generation_authority_snapshot();
    EXPECT_TRUE(pending.pending_binding);
    EXPECT_TRUE(pending.pending_handoff_phase);
    if (pending.pending_handoff_phase) {
      EXPECT_EQ(*pending.pending_handoff_phase, PendingAcceptedHandoffPhase::kAwaitingEpoch);
    }
    EXPECT_FALSE(pending.pending_route_token_live);
    EXPECT_FALSE(pending.pending_handoff_work_token_live);
    EXPECT_FALSE(pending.pending_epoch.has_value());
    EXPECT_FALSE(pending.pending_quiescence.has_value());
    coordinator->request_shutdown();
  }
  construction_blocker.release();
  ASSERT_TRUE(construction_blocked);
  EXPECT_FALSE(construction_blocker.timed_out());
  ASSERT_EQ(sent.wait_for(2s), std::future_status::ready);
  const auto accepted = sent.get();
  ASSERT_TRUE(accepted);

  auto result = client->async_get_result(accepted);
  ASSERT_EQ(result.wait_for(5s), std::future_status::ready);
  EXPECT_NE(result.get().code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_EQ(world_state.reserve_count(), 0U);
}

TEST_F(RestockActionCoordinatorNodeTest, ShutdownReclassifiesAtomicallyAtActivationBoundary)
{
  auto support = std::make_shared<rclcpp::Node>("shutdown_activation_race_test_support");
  constexpr std::string_view kWorldStatePrefix = "/shutdown_activation_race_world";
  WorldStateFixture world_state(support, "lane_01", std::string{kWorldStatePrefix});
  tf2_ros::StaticTransformBroadcaster broadcaster(support);
  geometry_msgs::msg::TransformStamped shelf;
  shelf.header.stamp = support->now();
  shelf.header.frame_id = "world";
  shelf.child_frame_id = "shelf";
  shelf.transform.translation.y = 0.55;
  shelf.transform.translation.z = 0.75;
  shelf.transform.rotation.w = 1.0;
  broadcaster.sendTransform(shelf);
  send_grasp_center_transform(broadcaster, support->now());

  OneShotCallBlocker reservation_hook;
  reservation_hook.arm();
  auto options = ready_coordinator_options(
    "/shutdown_activation_race_restock_product",
    std::string{kWorldStatePrefix});
  options.append_parameter_override("executor_threads", 4);
  options.append_parameter_override("pump_period_ms", 2000);
  options.append_parameter_override("shutdown_timeout_ms", 25000);
  auto coordinator = std::make_shared<RestockActionCoordinatorNode>(
    options, RestockActionCoordinatorNodeDependencies{
        []() {return std::chrono::steady_clock::now();},
        [&reservation_hook]() {reservation_hook.intercept();},
        {}});
  auto client =
    rclcpp_action::create_client<Action>(support, "/shutdown_activation_race_restock_product");
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions{}, 4U);
  executor.add_node(support);
  executor.add_node(coordinator);
  ExecutorSpinGuard spin(executor);
  ASSERT_TRUE(client->wait_for_action_server(2s));

  const auto startup_deadline = std::chrono::steady_clock::now() + 7s;
  while (coordinator->startup_snapshot().status != CoordinatorStartupStatus::kReady &&
    std::chrono::steady_clock::now() < startup_deadline)
  {
    std::this_thread::yield();
  }
  ASSERT_EQ(coordinator->startup_snapshot().status, CoordinatorStartupStatus::kReady);
  ASSERT_TRUE(wait_for_fresh_returned_pump_lease(coordinator, 5s));

  Action::Goal goal;
  goal.has_object_id = true;
  goal.object_id = 17U;
  goal.has_lane_id = true;
  goal.lane_id = "lane_01";
  auto sent = client->async_send_goal(goal);
  const bool reservation_blocked = reservation_hook.wait_until_blocked(2s);
  if (!reservation_blocked) {
    reservation_hook.release();
  }
  ASSERT_TRUE(reservation_blocked);
  const auto pending = coordinator->generation_authority_snapshot();
  ASSERT_TRUE(pending.pending_binding);
  ASSERT_FALSE(pending.active_binding);

  constexpr std::string_view kShutdownDetail = "coordinator process shutdown requested";
  AllocationCallBlocker classification_blocker;
  std::jthread shutdown_request([&classification_blocker, coordinator, kShutdownDetail]() {
      allocation_trigger_size = kShutdownDetail.size() + 1U;
      allocations_after_trigger = 0U;
      allocation_trigger_blocker = &classification_blocker;
      coordinator->request_shutdown();
      allocation_trigger_size.reset();
      allocations_before_failure.reset();
      allocation_call_blocker = nullptr;
      allocation_trigger_blocker = nullptr;
    });
  const bool classification_blocked = classification_blocker.wait_until_blocked(2s);
  if (!classification_blocked) {
    reservation_hook.release();
    classification_blocker.release();
    shutdown_request.join();
  }
  ASSERT_TRUE(classification_blocked);

  // Both route strings are prepared before the binding decision, so accepted handoff may cross
  // pending-to-active while the second allocation is held without invalidating a copied key.
  reservation_hook.release();
  ASSERT_EQ(sent.wait_for(2s), std::future_status::ready);
  const auto accepted = sent.get();
  ASSERT_TRUE(accepted);
  CoordinatorGenerationAuthoritySnapshot activated;
  const auto activation_deadline = std::chrono::steady_clock::now() + 2s;
  do {
    activated = coordinator->generation_authority_snapshot();
    if (activated.active_binding && !activated.pending_binding) {
      break;
    }
    std::this_thread::yield();
  } while (std::chrono::steady_clock::now() < activation_deadline);
  if (!activated.active_binding || activated.pending_binding) {
    classification_blocker.release();
    shutdown_request.join();
  }
  ASSERT_TRUE(activated.active_binding);
  ASSERT_FALSE(activated.pending_binding);

  classification_blocker.release();
  shutdown_request.join();
  EXPECT_FALSE(classification_blocker.timed_out());
  auto result = client->async_get_result(accepted);
  ASSERT_EQ(result.wait_for(6s), std::future_status::ready);
  EXPECT_NE(result.get().code, rclcpp_action::ResultCode::SUCCEEDED);
}

TEST_F(
  RestockActionCoordinatorNodeTest,
  ActivatesCompleteBindingBeforeNextTimerConsumesAcceptedEnvelope)
{
  auto support = std::make_shared<rclcpp::Node>("activation_timer_boundary_test_support");
  const std::string world_state_prefix{"/activation_timer_boundary_world"};
  WorldStateFixture world_state(support, "lane_01", world_state_prefix);
  tf2_ros::StaticTransformBroadcaster broadcaster(support);
  geometry_msgs::msg::TransformStamped shelf;
  shelf.header.stamp = support->now();
  shelf.header.frame_id = "world";
  shelf.child_frame_id = "shelf";
  shelf.transform.translation.y = 0.55;
  shelf.transform.translation.z = 0.75;
  shelf.transform.rotation.w = 1.0;
  broadcaster.sendTransform(shelf);
  send_grasp_center_transform(broadcaster, support->now());

  auto options =
    ready_coordinator_options("/activation_timer_boundary_restock_product", world_state_prefix);
  options.append_parameter_override("pump_period_ms", 2000);
  options.append_parameter_override("shutdown_timeout_ms", 25000);
  auto coordinator = std::make_shared<RestockActionCoordinatorNode>(options);
  auto client =
    rclcpp_action::create_client<Action>(support, "/activation_timer_boundary_restock_product");
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions{}, 3U);
  executor.add_node(support);
  executor.add_node(coordinator);
  ExecutorSpinGuard spin(executor);
  ASSERT_TRUE(client->wait_for_action_server(2s));

  const auto startup_deadline = std::chrono::steady_clock::now() + 7s;
  while (coordinator->startup_snapshot().status != CoordinatorStartupStatus::kReady &&
    std::chrono::steady_clock::now() < startup_deadline)
  {
    std::this_thread::yield();
  }
  ASSERT_EQ(coordinator->startup_snapshot().status, CoordinatorStartupStatus::kReady);
  const auto before_goal = wait_for_fresh_returned_pump_lease(coordinator, 5s);
  ASSERT_TRUE(before_goal.has_value());
  const auto snapshots_before_goal = world_state.snapshot_count();

  Action::Goal goal;
  goal.has_object_id = true;
  goal.object_id = 17U;
  goal.has_lane_id = true;
  goal.lane_id = "lane_01";
  auto sent = client->async_send_goal(goal);
  ASSERT_EQ(sent.wait_for(1s), std::future_status::ready);
  const auto goal_handle = sent.get();
  ASSERT_TRUE(goal_handle);

  CoordinatorGenerationAuthoritySnapshot activated;
  const auto activation_deadline = std::chrono::steady_clock::now() + 1s;
  do {
    activated = coordinator->generation_authority_snapshot();
    if (activated.active_binding && !activated.pending_binding) {
      break;
    }
    std::this_thread::yield();
  } while (std::chrono::steady_clock::now() < activation_deadline);
  ASSERT_TRUE(activated.active_binding);
  EXPECT_FALSE(activated.pending_binding);
  EXPECT_FALSE(activated.pending_handoff_phase);
  EXPECT_TRUE(activated.active_accepted_handle_identity);
  EXPECT_TRUE(activated.active_epoch);
  EXPECT_TRUE(activated.active_quiescence);
  EXPECT_EQ(activated.pump_lease.attempt, before_goal->pump_lease.attempt);
  EXPECT_FALSE(activated.pump_lease.outstanding);
  EXPECT_EQ(world_state.snapshot_count(), snapshots_before_goal);
  EXPECT_EQ(world_state.reserve_count(), 0U);

  const auto ingress_deadline = std::chrono::steady_clock::now() + 3s;
  while (world_state.snapshot_count() == snapshots_before_goal &&
    std::chrono::steady_clock::now() < ingress_deadline)
  {
    std::this_thread::yield();
  }
  EXPECT_EQ(world_state.snapshot_count(), snapshots_before_goal + 1U);

  coordinator->request_shutdown();
  auto result = client->async_get_result(goal_handle);
  ASSERT_EQ(result.wait_for(5s), std::future_status::ready);
  EXPECT_NE(result.get().code, rclcpp_action::ResultCode::SUCCEEDED);
}

TEST_F(RestockActionCoordinatorNodeTest, ActiveDrainFollowsAcceptedEnvelopeBeforeForwardWork)
{
  auto support = std::make_shared<rclcpp::Node>("accepted_then_drain_test_support");
  const std::string world_state_prefix{"/accepted_then_drain_world"};
  WorldStateFixture world_state(support, "lane_01", world_state_prefix);
  tf2_ros::StaticTransformBroadcaster broadcaster(support);
  geometry_msgs::msg::TransformStamped shelf;
  shelf.header.stamp = support->now();
  shelf.header.frame_id = "world";
  shelf.child_frame_id = "shelf";
  shelf.transform.translation.y = 0.55;
  shelf.transform.translation.z = 0.75;
  shelf.transform.rotation.w = 1.0;
  broadcaster.sendTransform(shelf);
  send_grasp_center_transform(broadcaster, support->now());

  const std::string status_topic{"/accepted_then_drain_coordinator/status"};
  AdmissionReadyObserver readiness(support, status_topic);
  auto options =
    ready_coordinator_options("/accepted_then_drain_restock_product", world_state_prefix);
  options.append_parameter_override("status_topic", status_topic);
  options.append_parameter_override("executor_threads", 4);
  options.append_parameter_override("shutdown_timeout_ms", 25000);
  auto coordinator = std::make_shared<RestockActionCoordinatorNode>(options);
  auto client =
    rclcpp_action::create_client<Action>(support, "/accepted_then_drain_restock_product");

  {
    rclcpp::executors::MultiThreadedExecutor startup_executor(rclcpp::ExecutorOptions{}, 4U);
    startup_executor.add_node(support);
    startup_executor.add_node(coordinator);
    ExecutorSpinGuard startup_spin(startup_executor);
    ASSERT_TRUE(client->wait_for_action_server(2s));
    ASSERT_TRUE(readiness.wait_until_ready(7s));
    ASSERT_TRUE(wait_for_fresh_returned_pump_lease(coordinator, 5s));
  }

  const auto snapshots_before_goal = world_state.snapshot_count();
  const auto reserves_before_goal = world_state.reserve_count();
  const auto validations_before_goal = world_state.validate_count();
  std::shared_future<typename rclcpp_action::ClientGoalHandle<Action>::WrappedResult> result_future;
  {
    // Only the action, ingress and completion groups run; the orchestration group owns the pump
    // timer, so accepted and drain ingress are ordered before any consumption.
    rclcpp::executors::MultiThreadedExecutor ingress_executor(rclcpp::ExecutorOptions{}, 4U);
    ingress_executor.add_node(support);
    const auto node_base = coordinator->get_node_base_interface();
    const auto default_group = node_base->get_default_callback_group();
    std::size_t ingress_groups = 0U;
    node_base->for_each_callback_group(
      [&ingress_executor, &node_base, &default_group, &ingress_groups](
        const rclcpp::CallbackGroup::SharedPtr & group) {
        const bool owns_timer =
        group->find_timer_ptrs_if([](const rclcpp::TimerBase::SharedPtr &) {return true;}) !=
        nullptr;
        if (group != default_group && !owns_timer) {
          ingress_executor.add_callback_group(group, node_base);
          ++ingress_groups;
        }
      });
    ASSERT_EQ(ingress_groups, 3U);
    ExecutorSpinGuard ingress_spin(ingress_executor);

    Action::Goal goal;
    goal.has_object_id = true;
    goal.object_id = 17U;
    goal.has_lane_id = true;
    goal.lane_id = "lane_01";
    const auto goal_handle = send_goal_until_accepted(client, goal);
    ASSERT_TRUE(goal_handle);

    CoordinatorGenerationAuthoritySnapshot activated;
    const auto activation_deadline = std::chrono::steady_clock::now() + 2s;
    do {
      activated = coordinator->generation_authority_snapshot();
      if (activated.active_binding && !activated.pending_binding) {
        break;
      }
      std::this_thread::yield();
    } while (std::chrono::steady_clock::now() < activation_deadline);
    ASSERT_TRUE(activated.active_binding);
    ASSERT_FALSE(activated.pending_binding);
    ASSERT_TRUE(activated.active_accepted_handle_identity);

    result_future = client->async_get_result(goal_handle);
    coordinator->request_shutdown();
    EXPECT_EQ(coordinator->shutdown_status(), CoordinatorShutdownStatus::kDraining);
    const auto drain_authority = coordinator->generation_authority_snapshot();
    ASSERT_TRUE(drain_authority.active_epoch);
    EXPECT_EQ(drain_authority.active_epoch->route_phase, ActiveFaultRoutePhase::kSecured);
    EXPECT_FALSE(drain_authority.active_epoch->route_claim_outstanding);
    EXPECT_EQ(result_future.wait_for(100ms), std::future_status::timeout);
    EXPECT_EQ(world_state.snapshot_count(), snapshots_before_goal);
    EXPECT_EQ(world_state.reserve_count(), reserves_before_goal);
    EXPECT_EQ(world_state.validate_count(), validations_before_goal);
  }

  {
    rclcpp::executors::MultiThreadedExecutor completion_executor(rclcpp::ExecutorOptions{}, 4U);
    completion_executor.add_node(support);
    completion_executor.add_node(coordinator);
    ExecutorSpinGuard completion_spin(completion_executor);

    ASSERT_EQ(result_future.wait_for(5s), std::future_status::ready);
    const auto result = result_future.get();
    EXPECT_EQ(result.code, rclcpp_action::ResultCode::ABORTED);
    ASSERT_TRUE(result.result);
    EXPECT_EQ(result.result->status, Action::Result::STATUS_SHUTDOWN);
    EXPECT_EQ(world_state.snapshot_count(), snapshots_before_goal);
    EXPECT_EQ(world_state.reserve_count(), reserves_before_goal);
    EXPECT_EQ(world_state.validate_count(), validations_before_goal);

    const auto clean_deadline = std::chrono::steady_clock::now() + 2s;
    while (coordinator->shutdown_status() == CoordinatorShutdownStatus::kDraining &&
      std::chrono::steady_clock::now() < clean_deadline)
    {
      std::this_thread::yield();
    }
    EXPECT_EQ(coordinator->shutdown_status(), CoordinatorShutdownStatus::kClean);
  }
}

TEST_F(RestockActionCoordinatorNodeTest, DisposesPreacceptRollbackOwnershipOutsideBindingLock)
{
  auto support = std::make_shared<rclcpp::Node>("preaccept_disposal_test_support");
  const std::string world_state_prefix{"/preaccept_disposal_world"};
  WorldStateFixture world_state(support, "lane_01", world_state_prefix);
  tf2_ros::StaticTransformBroadcaster broadcaster(support);
  geometry_msgs::msg::TransformStamped shelf;
  shelf.header.stamp = support->now();
  shelf.header.frame_id = "world";
  shelf.child_frame_id = "shelf";
  shelf.transform.translation.y = 0.55;
  shelf.transform.translation.z = 0.75;
  shelf.transform.rotation.w = 1.0;
  broadcaster.sendTransform(shelf);
  send_grasp_center_transform(broadcaster, support->now());

  constexpr std::size_t kTriggerLaneLength = 4096U;
  std::atomic<bool> script_armed{false};
  ExactDeleteInterceptor pending_disposal;
  ExactDeleteInterceptor epoch_disposal;
  ExactDeleteInterceptionGuard delete_guard(pending_disposal, &epoch_disposal);
  const std::string status_topic{"/preaccept_disposal_coordinator/status"};
  AdmissionReadyObserver readiness(support, status_topic);
  auto options =
    ready_coordinator_options("/preaccept_disposal_restock_product", world_state_prefix);
  options.append_parameter_override("status_topic", status_topic);
  options.append_parameter_override("pump_period_ms", 2000);
  options.append_parameter_override("shutdown_timeout_ms", 25000);
  auto coordinator = std::make_shared<RestockActionCoordinatorNode>(
    options, RestockActionCoordinatorNodeDependencies{
        [&script_armed, &pending_disposal, &epoch_disposal, kTriggerLaneLength]() {
          if (script_armed.load(std::memory_order_acquire)) {
            allocation_capture_script = AllocationCaptureScript{};
            allocation_capture_script.trigger_size = kTriggerLaneLength + 1U;
            allocation_capture_script.fail_ordinal = 4U;
            allocation_capture_script.capture_ordinals = {0U, 2U};
            allocation_capture_script.captures = {&pending_disposal, &epoch_disposal};
            allocation_capture_script.arm_source = &script_armed;
          } else if (!allocation_capture_script.active) {
            allocation_capture_script = AllocationCaptureScript{};
          }
          return std::chrono::steady_clock::now();
        },
        {},
        {}});
  auto client =
    rclcpp_action::create_client<Action>(support, "/preaccept_disposal_restock_product");
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions{}, 3U);
  executor.add_node(support);
  executor.add_node(coordinator);
  ExecutorSpinGuard spin(executor);
  ASSERT_TRUE(client->wait_for_action_server(2s));
  ASSERT_TRUE(readiness.wait_until_ready(7s));
  ASSERT_TRUE(wait_for_fresh_returned_pump_lease(coordinator, 5s).has_value());

  Action::Goal goal;
  goal.has_object_id = true;
  goal.object_id = 17U;
  goal.has_lane_id = true;
  goal.lane_id = std::string(kTriggerLaneLength, 'r');
  script_armed.store(true, std::memory_order_release);
  auto sent = client->async_send_goal(goal);

  const bool pending_blocked = pending_disposal.wait_until_blocked(2s);
  if (!pending_blocked) {
    script_armed.store(false, std::memory_order_release);
    pending_disposal.release();
    epoch_disposal.release();
  }
  ASSERT_TRUE(pending_blocked);
  auto pending_snapshot = std::async(
    std::launch::async, [coordinator]() {return coordinator->generation_authority_snapshot();});
  const auto pending_snapshot_status = pending_snapshot.wait_for(250ms);
  pending_disposal.release();
  const auto after_pending_disposal = pending_snapshot.get();
  EXPECT_EQ(pending_snapshot_status, std::future_status::ready);
  EXPECT_FALSE(after_pending_disposal.pending_binding);
  EXPECT_FALSE(after_pending_disposal.active_binding);

  const bool epoch_blocked = epoch_disposal.wait_until_blocked(2s);
  if (!epoch_blocked) {
    script_armed.store(false, std::memory_order_release);
    epoch_disposal.release();
  }
  ASSERT_TRUE(epoch_blocked);
  auto epoch_snapshot = std::async(
    std::launch::async, [coordinator]() {return coordinator->generation_authority_snapshot();});
  const auto epoch_snapshot_status = epoch_snapshot.wait_for(250ms);
  epoch_disposal.release();
  const auto after_epoch_disposal = epoch_snapshot.get();
  script_armed.store(false, std::memory_order_release);
  EXPECT_EQ(epoch_snapshot_status, std::future_status::ready);
  EXPECT_FALSE(after_epoch_disposal.pending_binding);
  EXPECT_FALSE(after_epoch_disposal.active_binding);
  EXPECT_EQ(pending_disposal.captured_size(), kTriggerLaneLength + 1U);
  EXPECT_EQ(epoch_disposal.captured_size(), sizeof(CoordinatorActiveFaultEpoch));
  EXPECT_FALSE(pending_disposal.timed_out());
  EXPECT_FALSE(epoch_disposal.timed_out());

  ASSERT_EQ(sent.wait_for(2s), std::future_status::ready);
  EXPECT_FALSE(sent.get());
  coordinator->request_shutdown();
}

TEST_F(RestockActionCoordinatorNodeTest, DisposesRetirementBundleOwnersAfterLeaseReturnAndUnlock)
{
  constexpr std::size_t kTriggerLaneLength = 4096U;
  const std::string lane_id(kTriggerLaneLength, 'a');
  auto support = std::make_shared<rclcpp::Node>("active_epoch_disposal_test_support");
  const std::string world_state_prefix{"/active_epoch_disposal_world"};
  WorldStateFixture world_state(support, lane_id, world_state_prefix);
  tf2_ros::StaticTransformBroadcaster broadcaster(support);
  geometry_msgs::msg::TransformStamped shelf;
  shelf.header.stamp = support->now();
  shelf.header.frame_id = "world";
  shelf.child_frame_id = "shelf";
  shelf.transform.translation.y = 0.55;
  shelf.transform.translation.z = 0.75;
  shelf.transform.rotation.w = 1.0;
  broadcaster.sendTransform(shelf);
  send_grasp_center_transform(broadcaster, support->now());

  std::atomic<bool> script_armed{false};
  ExactDeleteInterceptor epoch_disposal;
  ExactDeleteInterceptor quiescence_disposal;
  ExactDeleteInterceptionGuard delete_guard(epoch_disposal, &quiescence_disposal);
  const std::string status_topic{"/active_epoch_disposal_coordinator/status"};
  AdmissionReadyObserver readiness(support, status_topic);
  auto options =
    ready_coordinator_options("/active_epoch_disposal_restock_product", world_state_prefix);
  options.append_parameter_override("status_topic", status_topic);
  options.append_parameter_override("pump_period_ms", 2000);
  options.append_parameter_override("shutdown_timeout_ms", 25000);
  auto coordinator = std::make_shared<RestockActionCoordinatorNode>(
    options, RestockActionCoordinatorNodeDependencies{
        [&script_armed, &epoch_disposal, &quiescence_disposal, kTriggerLaneLength]() {
          if (script_armed.load(std::memory_order_acquire)) {
            allocation_capture_script = AllocationCaptureScript{};
            allocation_capture_script.trigger_size = kTriggerLaneLength + 1U;
            allocation_capture_script.capture_ordinals = {
              2U, 4U};
            allocation_capture_script.captures = {&epoch_disposal, &quiescence_disposal};
            allocation_capture_script.stop_after_ordinal = 4U;
            allocation_capture_script.arm_source = &script_armed;
          } else if (!allocation_capture_script.active) {
            allocation_capture_script = AllocationCaptureScript{};
          }
          return std::chrono::steady_clock::now();
        },
        {},
        {}});
  auto client =
    rclcpp_action::create_client<Action>(support, "/active_epoch_disposal_restock_product");
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions{}, 3U);
  executor.add_node(support);
  executor.add_node(coordinator);
  ExecutorSpinGuard spin(executor);
  ASSERT_TRUE(client->wait_for_action_server(2s));
  ASSERT_TRUE(readiness.wait_until_ready(7s));
  ASSERT_TRUE(wait_for_fresh_returned_pump_lease(coordinator, 5s).has_value());

  SnapshotBlockGuard snapshot_guard(world_state);
  Action::Goal goal;
  goal.has_object_id = true;
  goal.object_id = 17U;
  goal.has_lane_id = true;
  goal.lane_id = lane_id;
  script_armed.store(true, std::memory_order_release);
  auto sent = client->async_send_goal(goal);
  ASSERT_EQ(sent.wait_for(2s), std::future_status::ready);
  script_armed.store(false, std::memory_order_release);
  const auto accepted = sent.get();
  ASSERT_TRUE(accepted);
  EXPECT_EQ(epoch_disposal.captured_size(), sizeof(CoordinatorActiveFaultEpoch));
  const auto active_deadline = std::chrono::steady_clock::now() + 3s;
  CoordinatorGenerationAuthoritySnapshot active_authority;
  do {
    active_authority = coordinator->generation_authority_snapshot();
    if (active_authority.active_epoch_identity) {
      break;
    }
    std::this_thread::yield();
  } while (std::chrono::steady_clock::now() < active_deadline);
  if (!active_authority.active_epoch_identity) {
    epoch_disposal.release();
    quiescence_disposal.release();
    snapshot_guard.unblock();
  }
  ASSERT_TRUE(active_authority.active_epoch_identity);
  ASSERT_EQ(*active_authority.active_epoch_identity, epoch_disposal.captured_identity());

  auto result = client->async_get_result(accepted);
  auto canceled = client->async_cancel_goal(accepted);
  ASSERT_EQ(canceled.wait_for(2s), std::future_status::ready);
  ASSERT_FALSE(canceled.get()->goals_canceling.empty());
  snapshot_guard.unblock();
  const bool quiescence_blocked = quiescence_disposal.wait_until_blocked(5s);
  if (!quiescence_blocked) {
    quiescence_disposal.release();
    epoch_disposal.release();
  }
  ASSERT_TRUE(quiescence_blocked);
  auto quiescence_authority_snapshot = std::async(
    std::launch::async, [coordinator]() {return coordinator->generation_authority_snapshot();});
  const auto quiescence_snapshot_status = quiescence_authority_snapshot.wait_for(250ms);
  const auto retired_before_quiescence_disposal = quiescence_authority_snapshot.get();
  EXPECT_EQ(quiescence_snapshot_status, std::future_status::ready);
  EXPECT_FALSE(retired_before_quiescence_disposal.pending_binding);
  EXPECT_FALSE(retired_before_quiescence_disposal.active_binding);
  EXPECT_FALSE(retired_before_quiescence_disposal.pump_lease.outstanding);
  quiescence_disposal.release();

  const bool epoch_blocked = epoch_disposal.wait_until_blocked(5s);
  if (!epoch_blocked) {
    epoch_disposal.release();
  }
  ASSERT_TRUE(epoch_blocked);
  auto authority_snapshot = std::async(
    std::launch::async, [coordinator]() {return coordinator->generation_authority_snapshot();});
  const auto authority_snapshot_status = authority_snapshot.wait_for(250ms);
  epoch_disposal.release();
  const auto retired = authority_snapshot.get();
  EXPECT_EQ(authority_snapshot_status, std::future_status::ready);
  EXPECT_FALSE(retired.pending_binding);
  EXPECT_FALSE(retired.active_binding);
  EXPECT_FALSE(retired.pump_lease.outstanding);
  EXPECT_GE(quiescence_disposal.captured_size(), sizeof(CoordinatorGenerationQuiescence));
  EXPECT_FALSE(quiescence_disposal.timed_out());
  EXPECT_FALSE(epoch_disposal.timed_out());
  ASSERT_EQ(result.wait_for(5s), std::future_status::ready);
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::CANCELED);
  coordinator->request_shutdown();
}

TEST_F(RestockActionCoordinatorNodeTest, RejectsEmptySteadyClockDependency)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {rclcpp::Parameter("product_catalog_path", RESTOCKER_TEST_PRODUCT_CATALOG),
      rclcpp::Parameter("workcell_geometry_path", RESTOCKER_TEST_WORKCELL_GEOMETRY),
      rclcpp::Parameter("gripper_geometry_path", RESTOCKER_TEST_GRIPPER_GEOMETRY),
      rclcpp::Parameter("action_name", "/empty_clock_restock_product")});

  EXPECT_THROW(
    (void)std::make_shared<RestockActionCoordinatorNode>(
      options, RestockActionCoordinatorNodeDependencies{}),
    std::invalid_argument);
}

TEST_F(RestockActionCoordinatorNodeTest, ShutdownUsesOneExactInjectedClockSample)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {rclcpp::Parameter("product_catalog_path", RESTOCKER_TEST_PRODUCT_CATALOG),
      rclcpp::Parameter("workcell_geometry_path", RESTOCKER_TEST_WORKCELL_GEOMETRY),
      rclcpp::Parameter("gripper_geometry_path", RESTOCKER_TEST_GRIPPER_GEOMETRY),
      rclcpp::Parameter("action_name", "/injected_clock_shutdown_restock_product"),
      rclcpp::Parameter("shutdown_timeout_ms", 21000)});
  const auto injected_now = SteadyTime{} + 31ms;
  std::atomic<std::size_t> provider_calls{0U};
  auto coordinator = std::make_shared<RestockActionCoordinatorNode>(
    options, RestockActionCoordinatorNodeDependencies{[injected_now, &provider_calls]() {
        ++provider_calls;
        return injected_now;
      },
      {},
      {}});

  coordinator->request_shutdown();

  const auto shutdown = coordinator->shutdown_snapshot();
  ASSERT_TRUE(shutdown.deadline);
  EXPECT_EQ(*shutdown.deadline, injected_now + 21000ms);
  EXPECT_EQ(shutdown.status, CoordinatorShutdownStatus::kDraining);
  EXPECT_EQ(provider_calls.load(), 1U);
}

TEST_F(RestockActionCoordinatorNodeTest, SteadyClockFailurePermanentlyInhibitsAdmission)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {rclcpp::Parameter("product_catalog_path", RESTOCKER_TEST_PRODUCT_CATALOG),
      rclcpp::Parameter("workcell_geometry_path", RESTOCKER_TEST_WORKCELL_GEOMETRY),
      rclcpp::Parameter("gripper_geometry_path", RESTOCKER_TEST_GRIPPER_GEOMETRY),
      rclcpp::Parameter("action_name", "/failed_clock_restock_product"),
      rclcpp::Parameter("status_topic", "/failed_clock_coordinator/status"),
      rclcpp::Parameter("pump_period_ms", 1)});
  std::atomic<std::size_t> provider_calls{0U};
  auto coordinator = std::make_shared<RestockActionCoordinatorNode>(
    options, RestockActionCoordinatorNodeDependencies{[&provider_calls]() -> SteadyTime {
        ++provider_calls;
        throw std::runtime_error(
                "injected node clock failure");
      },
      {},
      {}});
  auto support = std::make_shared<rclcpp::Node>("failed_clock_coordinator_test_support");
  std::mutex status_mutex;
  std::optional<restocker_interfaces::msg::RestockCoordinatorStatus> status;
  auto subscription =
    support->create_subscription<restocker_interfaces::msg::RestockCoordinatorStatus>(
    "/failed_clock_coordinator/status", rclcpp::QoS(1).reliable().transient_local(),
    [&status_mutex, &status](
      const restocker_interfaces::msg::RestockCoordinatorStatus::ConstSharedPtr message) {
      std::lock_guard lock(status_mutex);
      status = *message;
    });
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions{}, 3U);
  executor.add_node(coordinator);
  executor.add_node(support);
  // The client must outlive the spin guard. Releasing the last reference to an action client
  // while an executor thread is in CallbackGroup::collect_all_ptrs() deadlocks on the group
  // mutex (the deleter calls remove_waitable()). Declaring the client first joins the spin thread
  // before the client is released.
  auto client = rclcpp_action::create_client<Action>(support, "/failed_clock_restock_product");
  ExecutorSpinGuard spin(executor);
  ASSERT_TRUE(client->wait_for_action_server(2s));

  bool inhibited = false;
  const auto deadline = std::chrono::steady_clock::now() + 2s;
  while (!inhibited && std::chrono::steady_clock::now() < deadline) {
    {
      std::lock_guard lock(status_mutex);
      inhibited = status && status->inhibited && !status->admission_ready &&
        status->detail.find("steady-clock provider failed") != std::string::npos;
    }
    std::this_thread::yield();
  }
  EXPECT_TRUE(inhibited);
  EXPECT_EQ(provider_calls.load(), 1U);

  Action::Goal goal;
  goal.has_object_id = true;
  goal.object_id = 17U;
  goal.has_lane_id = true;
  goal.lane_id = "lane_01";
  auto sent = client->async_send_goal(goal);
  ASSERT_EQ(sent.wait_for(1s), std::future_status::ready);
  EXPECT_FALSE(sent.get());
  EXPECT_EQ(provider_calls.load(), 1U);

  coordinator->request_shutdown();
  const auto clean_deadline = std::chrono::steady_clock::now() + 1s;
  while (coordinator->shutdown_status() == CoordinatorShutdownStatus::kDraining &&
    std::chrono::steady_clock::now() < clean_deadline)
  {
    std::this_thread::yield();
  }
  EXPECT_EQ(coordinator->shutdown_status(), CoordinatorShutdownStatus::kClean);
  EXPECT_EQ(provider_calls.load(), 1U);
  (void)subscription;
}

TEST_F(RestockActionCoordinatorNodeTest, ClockProviderCanReenterScalarDiagnostics)
{
  auto support = std::make_shared<rclcpp::Node>("reentrant_clock_diagnostics_test_support");
  WorldStateFixture world_state(support);
  std::weak_ptr<RestockActionCoordinatorNode> weak_coordinator;
  std::atomic<std::size_t> reentrant_snapshots{0U};
  auto coordinator = std::make_shared<RestockActionCoordinatorNode>(
    ready_coordinator_options("/reentrant_clock_diagnostics_restock_product"),
    RestockActionCoordinatorNodeDependencies{[&weak_coordinator, &reentrant_snapshots]() {
        if (const auto locked = weak_coordinator.lock()) {
          (void)locked->generation_authority_snapshot();
          ++reentrant_snapshots;
        }
        return std::chrono::steady_clock::now();
      }});
  weak_coordinator = coordinator;

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions{}, 3U);
  executor.add_node(support);
  executor.add_node(coordinator);
  ExecutorSpinGuard spin(executor);
  const auto startup_deadline = std::chrono::steady_clock::now() + 3s;
  while (coordinator->startup_snapshot().status != CoordinatorStartupStatus::kReady &&
    std::chrono::steady_clock::now() < startup_deadline)
  {
    std::this_thread::yield();
  }
  EXPECT_EQ(coordinator->startup_snapshot().status, CoordinatorStartupStatus::kReady);
  EXPECT_GT(reentrant_snapshots.load(), 0U);

  coordinator->request_shutdown();
  EXPECT_NE(coordinator->shutdown_status(), CoordinatorShutdownStatus::kRunning);
}

TEST_F(RestockActionCoordinatorNodeTest, PumpLeaseInvalidatesDiagnosticsBeforeExternalWork)
{
  auto support = std::make_shared<rclcpp::Node>("pump_cache_fence_test_support");
  WorldStateFixture world_state(support, "lane_01", "/pump_cache_fence_world");
  OneShotCallBlocker clock_blocker;
  std::weak_ptr<RestockActionCoordinatorNode> weak_coordinator;
  auto options =
    ready_coordinator_options("/pump_cache_fence_restock_product", "/pump_cache_fence_world");
  auto coordinator = std::make_shared<RestockActionCoordinatorNode>(
    options, RestockActionCoordinatorNodeDependencies{[&clock_blocker, &weak_coordinator]() {
        if (const auto locked = weak_coordinator.lock();
        locked && locked->generation_authority_snapshot().pump_lease.outstanding)
        {
          clock_blocker.intercept();
        }
        return std::chrono::steady_clock::now();
      }});
  weak_coordinator = coordinator;

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions{}, 3U);
  executor.add_node(support);
  executor.add_node(coordinator);
  ExecutorSpinGuard spin(executor);
  const auto startup_deadline = std::chrono::steady_clock::now() + 5s;
  while (coordinator->startup_snapshot().status != CoordinatorStartupStatus::kReady &&
    std::chrono::steady_clock::now() < startup_deadline)
  {
    std::this_thread::yield();
  }
  ASSERT_EQ(coordinator->startup_snapshot().status, CoordinatorStartupStatus::kReady);
  ASSERT_TRUE(wait_for_fresh_returned_pump_lease(coordinator, 5s).has_value());

  clock_blocker.arm();
  const bool pump_blocked = clock_blocker.wait_until_blocked(2s);
  if (!pump_blocked) {
    clock_blocker.release();
  }
  ASSERT_TRUE(pump_blocked);
  const auto during_pump = coordinator->generation_authority_snapshot();
  EXPECT_TRUE(during_pump.pump_lease.outstanding);
  EXPECT_FALSE(during_pump.diagnostic_caches_fresh);
  clock_blocker.release();
  ASSERT_TRUE(wait_for_fresh_returned_pump_lease(coordinator, 2s).has_value());
  coordinator->request_shutdown();
  EXPECT_NE(coordinator->shutdown_status(), CoordinatorShutdownStatus::kRunning);
}

TEST_F(RestockActionCoordinatorNodeTest, ActiveSteadyClockFailureAbortsBeforeReservation)
{
  auto support = std::make_shared<rclcpp::Node>("active_clock_failure_test_support");
  WorldStateFixture world_state(support);
  tf2_ros::StaticTransformBroadcaster broadcaster(support);
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = support->now();
  transform.header.frame_id = "world";
  transform.child_frame_id = "shelf";
  transform.transform.translation.y = 0.55;
  transform.transform.translation.z = 0.75;
  transform.transform.rotation.w = 1.0;
  broadcaster.sendTransform(transform);
  send_grasp_center_transform(broadcaster, support->now());

  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {rclcpp::Parameter("product_catalog_path", RESTOCKER_TEST_PRODUCT_CATALOG),
      rclcpp::Parameter("workcell_geometry_path", RESTOCKER_TEST_WORKCELL_GEOMETRY),
      rclcpp::Parameter("gripper_geometry_path", RESTOCKER_TEST_GRIPPER_GEOMETRY),
      rclcpp::Parameter("world_state.get_snapshot_service", "/test_world/get_snapshot"),
      rclcpp::Parameter("world_state.reserve_task_service", "/test_world/reserve_task"),
      rclcpp::Parameter(
        "world_state.validate_reservation_service",
        "/test_world/validate_reservation"),
      rclcpp::Parameter(
        "world_state.release_reservation_service",
        "/test_world/release_reservation"),
      rclcpp::Parameter(
        "world_state.validate_execution_authority_service",
        "/test_world/validate_execution_authority"),
      rclcpp::Parameter("action_name", "/active_clock_failure_restock_product"),
      rclcpp::Parameter("pump_period_ms", 5),
      rclcpp::Parameter("selection.maximum_object_age_ms", 5000),
      rclcpp::Parameter("selection.lane_evidence_validity_ms", 5000)});
  std::atomic<bool> fail_clock{false};
  std::atomic<std::size_t> failure_calls{0U};
  OneShotCallBlocker clock_call_blocker;
  // One pump iteration publishes the secured terminal and retires the active binding. Holding the
  // terminal delivery keeps retirement out of that iteration so the secured epoch is still live.
  OneShotCallBlocker terminal_delivery_blocker;
  auto coordinator = std::make_shared<RestockActionCoordinatorNode>(
    options, RestockActionCoordinatorNodeDependencies{
        [&clock_call_blocker, &fail_clock, &failure_calls]() -> SteadyTime {
          clock_call_blocker.intercept();
          if (fail_clock.load(std::memory_order_acquire)) {
            ++failure_calls;
            throw std::runtime_error("injected active clock failure");
          }
          return std::chrono::steady_clock::now();
        },
        {},
        [&terminal_delivery_blocker]() {terminal_delivery_blocker.intercept();}});
  auto client =
    rclcpp_action::create_client<Action>(support, "/active_clock_failure_restock_product");
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions{}, 4U);
  executor.add_node(support);
  executor.add_node(coordinator);
  ExecutorSpinGuard spin(executor);
  ASSERT_TRUE(client->wait_for_action_server(2s));

  const auto startup_deadline = std::chrono::steady_clock::now() + 2s;
  while (coordinator->startup_snapshot().status != CoordinatorStartupStatus::kReady &&
    std::chrono::steady_clock::now() < startup_deadline)
  {
    std::this_thread::yield();
  }
  ASSERT_EQ(coordinator->startup_snapshot().status, CoordinatorStartupStatus::kReady);
  const auto startup_snapshots = world_state.snapshot_count();
  SnapshotBlockGuard snapshot_block(world_state);
  Action::Goal goal;
  goal.has_object_id = true;
  goal.object_id = 17U;
  goal.has_lane_id = true;
  goal.lane_id = "lane_01";
  std::shared_ptr<rclcpp_action::ClientGoalHandle<Action>> goal_handle;
  // Twenty attempts at 500 ms is a retry count, not a time budget: a refused goal is retried,
  // a slow one is fatal on the first attempt. The phase budget equals the wall time the retries
  // allow; what is left is spent waiting on the send in flight. Re-sending could abandon a goal
  // the server already accepted.
  constexpr std::size_t kAcceptAttempts = 20U;
  constexpr auto kRefusalPause = 25ms;
  constexpr auto kSendResponseBudget = 500ms;
  constexpr auto kAcceptPhaseBudget =
    static_cast<int>(kAcceptAttempts) * (kSendResponseBudget + kRefusalPause);
  const auto accept_deadline = std::chrono::steady_clock::now() + kAcceptPhaseBudget;
  for (std::size_t attempt = 0U; attempt < kAcceptAttempts && !goal_handle; ++attempt) {
    auto sent = client->async_send_goal(goal);
    const auto remaining = accept_deadline - std::chrono::steady_clock::now();
    ASSERT_EQ(
      sent.wait_for(std::max<std::chrono::steady_clock::duration>(remaining, kSendResponseBudget)),
      std::future_status::ready);
    goal_handle = sent.get();
    if (!goal_handle) {
      std::this_thread::sleep_for(kRefusalPause);
    }
  }
  ASSERT_TRUE(goal_handle);
  const auto request_deadline = std::chrono::steady_clock::now() + 2s;
  while (world_state.snapshot_count() == startup_snapshots &&
    std::chrono::steady_clock::now() < request_deadline)
  {
    std::this_thread::yield();
  }
  ASSERT_GT(world_state.snapshot_count(), startup_snapshots);
  auto result_future = client->async_get_result(goal_handle);

  // Hold the timer's provider call while the snapshot callback stays blocked. Provider calls are
  // serialized, so releasing this call publishes one sticky failure first.
  clock_call_blocker.arm();
  const bool timer_clock_call_blocked = clock_call_blocker.wait_until_blocked(2s);
  if (!timer_clock_call_blocked) {
    clock_call_blocker.release();
  }
  ASSERT_TRUE(timer_clock_call_blocked);
  terminal_delivery_blocker.arm();
  fail_clock.store(true, std::memory_order_release);
  clock_call_blocker.release();
  const auto async_failure_deadline = std::chrono::steady_clock::now() + 2s;
  auto failure_snapshot = coordinator->steady_clock_failure_snapshot();
  while (!failure_snapshot.handled && std::chrono::steady_clock::now() < async_failure_deadline) {
    std::this_thread::yield();
    failure_snapshot = coordinator->steady_clock_failure_snapshot();
  }
  ASSERT_GE(failure_calls.load(), 1U);
  EXPECT_TRUE(failure_snapshot.provider_failed);
  EXPECT_TRUE(failure_snapshot.handled);
  EXPECT_TRUE(failure_snapshot.authority_secured);
  EXPECT_FALSE(failure_snapshot.first_observed_asynchronously);
  const auto clock_authority = coordinator->generation_authority_snapshot();
  ASSERT_TRUE(clock_authority.active_epoch);
  EXPECT_EQ(clock_authority.active_epoch->route_phase, ActiveFaultRoutePhase::kSecured);
  EXPECT_FALSE(clock_authority.active_epoch->route_claim_outstanding);
  terminal_delivery_blocker.release();

  // Release the retained driver callback only now; its later latch record must not replace the
  // synchronous classification already made under the binding mutex.
  snapshot_block.unblock();
  ASSERT_EQ(result_future.wait_for(5s), std::future_status::ready);
  const auto result = result_future.get();
  EXPECT_NE(result.code, rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_TRUE(result.result);
  EXPECT_NE(result.result->status, Action::Result::STATUS_SUCCEEDED);
  EXPECT_GE(failure_calls.load(), 1U);
  EXPECT_EQ(world_state.reserve_count(), 0U);
  EXPECT_EQ(world_state.release_count(), 0U);
  EXPECT_FALSE(world_state.reserved());
  EXPECT_FALSE(
    coordinator->steady_clock_failure_snapshot().first_observed_asynchronously);

  const auto calls_after_retirement = failure_calls.load();
  coordinator->request_shutdown();
  const auto shutdown_deadline = std::chrono::steady_clock::now() + 1s;
  while (coordinator->shutdown_status() == CoordinatorShutdownStatus::kDraining &&
    std::chrono::steady_clock::now() < shutdown_deadline)
  {
    std::this_thread::yield();
  }
  EXPECT_EQ(coordinator->shutdown_status(), CoordinatorShutdownStatus::kClean);
  EXPECT_EQ(failure_calls.load(), calls_after_retirement);
}

TEST_F(RestockActionCoordinatorNodeTest, PendingSteadyClockFailureIsHandedOffBeforeForwardWork)
{
  auto support = std::make_shared<rclcpp::Node>("pending_clock_failure_test_support");
  const std::string status_topic{"/pending_clock_failure_coordinator/status"};
  AdmissionReadyObserver readiness(support, status_topic);
  WorldStateFixture world_state(support);
  tf2_ros::StaticTransformBroadcaster broadcaster(support);
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = support->now();
  transform.header.frame_id = "world";
  transform.child_frame_id = "shelf";
  transform.transform.translation.y = 0.55;
  transform.transform.translation.z = 0.75;
  transform.transform.rotation.w = 1.0;
  broadcaster.sendTransform(transform);
  send_grasp_center_transform(broadcaster, support->now());

  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {rclcpp::Parameter("product_catalog_path", RESTOCKER_TEST_PRODUCT_CATALOG),
      rclcpp::Parameter("workcell_geometry_path", RESTOCKER_TEST_WORKCELL_GEOMETRY),
      rclcpp::Parameter("gripper_geometry_path", RESTOCKER_TEST_GRIPPER_GEOMETRY),
      rclcpp::Parameter("world_state.get_snapshot_service", "/test_world/get_snapshot"),
      rclcpp::Parameter("world_state.reserve_task_service", "/test_world/reserve_task"),
      rclcpp::Parameter(
        "world_state.validate_reservation_service",
        "/test_world/validate_reservation"),
      rclcpp::Parameter(
        "world_state.release_reservation_service",
        "/test_world/release_reservation"),
      rclcpp::Parameter(
        "world_state.validate_execution_authority_service",
        "/test_world/validate_execution_authority"),
      rclcpp::Parameter("action_name", "/pending_clock_failure_restock_product"),
      rclcpp::Parameter("status_topic", status_topic),
      rclcpp::Parameter("pump_period_ms", 5),
      rclcpp::Parameter("selection.maximum_object_age_ms", 5000),
      rclcpp::Parameter("selection.lane_evidence_validity_ms", 5000)});
  std::atomic<bool> fail_clock{false};
  auto coordinator = std::make_shared<RestockActionCoordinatorNode>(
    options, RestockActionCoordinatorNodeDependencies{
        [&fail_clock]() -> SteadyTime {
          if (fail_clock.load(std::memory_order_acquire)) {
            throw std::runtime_error("injected pending-goal clock failure");
          }
          return std::chrono::steady_clock::now();
        },
        [&fail_clock]() {fail_clock.store(true, std::memory_order_release);},
        {}});
  auto client =
    rclcpp_action::create_client<Action>(support, "/pending_clock_failure_restock_product");
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions{}, 4U);
  executor.add_node(support);
  executor.add_node(coordinator);
  ExecutorSpinGuard spin(executor);
  ASSERT_TRUE(client->wait_for_action_server(2s));

  const auto startup_deadline = std::chrono::steady_clock::now() + 2s;
  while (coordinator->startup_snapshot().status != CoordinatorStartupStatus::kReady &&
    std::chrono::steady_clock::now() < startup_deadline)
  {
    std::this_thread::yield();
  }
  ASSERT_EQ(coordinator->startup_snapshot().status, CoordinatorStartupStatus::kReady);
  // Startup readiness is not admission readiness: refresh_readiness() runs after drive_startup()
  // and also gates on the world-state services, both transforms and the composed backends. A goal
  // sent before then is refused, so wait on the published readiness; the loop is a backstop.
  ASSERT_TRUE(readiness.wait_until_ready(7s));

  Action::Goal goal;
  goal.has_object_id = true;
  goal.object_id = 17U;
  goal.has_lane_id = true;
  goal.lane_id = "lane_01";
  std::shared_ptr<rclcpp_action::ClientGoalHandle<Action>> goal_handle;
  // Same loop shape as ActiveSteadyClockFailureAbortsBeforeReservation. The goal may already be
  // accepted when a wait times out, so wait out the remaining budget instead of re-sending.
  constexpr std::size_t kAcceptAttempts = 20U;
  constexpr auto kRefusalPause = 25ms;
  constexpr auto kSendResponseBudget = 500ms;
  constexpr auto kAcceptPhaseBudget =
    static_cast<int>(kAcceptAttempts) * (kSendResponseBudget + kRefusalPause);
  const auto accept_deadline = std::chrono::steady_clock::now() + kAcceptPhaseBudget;
  for (std::size_t attempt = 0U; attempt < kAcceptAttempts && !goal_handle; ++attempt) {
    auto sent = client->async_send_goal(goal);
    const auto remaining = accept_deadline - std::chrono::steady_clock::now();
    ASSERT_EQ(
      sent.wait_for(std::max<std::chrono::steady_clock::duration>(remaining, kSendResponseBudget)),
      std::future_status::ready);
    goal_handle = sent.get();
    if (!goal_handle) {
      std::this_thread::sleep_for(kRefusalPause);
    }
  }
  ASSERT_TRUE(goal_handle);
  auto result_future = client->async_get_result(goal_handle);

  ASSERT_EQ(result_future.wait_for(5s), std::future_status::ready);
  const auto result = result_future.get();
  EXPECT_NE(result.code, rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_TRUE(result.result);
  EXPECT_NE(result.result->status, Action::Result::STATUS_SUCCEEDED);
  EXPECT_EQ(world_state.reserve_count(), 0U);
  EXPECT_EQ(world_state.release_count(), 0U);
  EXPECT_FALSE(world_state.reserved());
  const auto failure = coordinator->steady_clock_failure_snapshot();
  EXPECT_TRUE(failure.provider_failed);
  EXPECT_TRUE(failure.handled);
  EXPECT_TRUE(failure.authority_secured);

  coordinator->request_shutdown();
  const auto shutdown_deadline = std::chrono::steady_clock::now() + 1s;
  while (coordinator->shutdown_status() == CoordinatorShutdownStatus::kDraining &&
    std::chrono::steady_clock::now() < shutdown_deadline)
  {
    std::this_thread::yield();
  }
  EXPECT_EQ(coordinator->shutdown_status(), CoordinatorShutdownStatus::kClean);
}

TEST_F(
  RestockActionCoordinatorNodeTest,
  FirstSteadyClockFailureDuringPreparationAbortsWithoutForwardWork)
{
  auto support = std::make_shared<rclcpp::Node>("preparation_clock_failure_test_support");
  constexpr std::string_view kWorldStatePrefix = "/preparation_clock_failure_world";
  const std::string status_topic{"/preparation_clock_failure_coordinator/status"};
  AdmissionReadyObserver readiness(support, status_topic);
  WorldStateFixture world_state(support, "lane_01", std::string{kWorldStatePrefix});
  tf2_ros::StaticTransformBroadcaster broadcaster(support);
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = support->now();
  transform.header.frame_id = "world";
  transform.child_frame_id = "shelf";
  transform.transform.translation.y = 0.55;
  transform.transform.translation.z = 0.75;
  transform.transform.rotation.w = 1.0;
  broadcaster.sendTransform(transform);
  send_grasp_center_transform(broadcaster, support->now());

  auto options = ready_coordinator_options(
    "/preparation_clock_failure_restock_product",
    std::string{kWorldStatePrefix});
  // Everything waited on happens inside a pump tick, so budgets are in pump periods.
  constexpr auto kPumpPeriod = 2000ms;
  options.append_parameter_override("pump_period_ms", static_cast<int>(kPumpPeriod.count()));
  options.append_parameter_override("status_topic", status_topic);
  options.append_parameter_override("shutdown_timeout_ms", 25000);
  std::atomic<int> post_hook_successes_remaining{-1};
  std::atomic<std::size_t> hook_calls{0U};
  std::atomic<std::size_t> post_hook_successes{0U};
  std::atomic<std::size_t> preparation_failures{0U};
  auto coordinator = std::make_shared<RestockActionCoordinatorNode>(
    options,
    RestockActionCoordinatorNodeDependencies{
        [&post_hook_successes_remaining, &post_hook_successes,
        &preparation_failures]() -> SteadyTime {
          int expected = 1;
          if (post_hook_successes_remaining.compare_exchange_strong(
            expected, 0, std::memory_order_acq_rel, std::memory_order_acquire))
          {
            ++post_hook_successes;
            return std::chrono::steady_clock::now();
          }
          if (expected == 0) {
            ++preparation_failures;
            throw std::runtime_error("injected accepted-handoff preparation clock failure");
          }
          return std::chrono::steady_clock::now();
        },
        [&post_hook_successes_remaining, &hook_calls]() {
          ++hook_calls;
          post_hook_successes_remaining.store(1, std::memory_order_release);
        }});
  auto client =
    rclcpp_action::create_client<Action>(support, "/preparation_clock_failure_restock_product");
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions{}, 4U);
  executor.add_node(support);
  executor.add_node(coordinator);
  ExecutorSpinGuard spin(executor);
  ASSERT_TRUE(client->wait_for_action_server(2s));

  // Startup is quantised in pump periods: waiting_for_authority, probe_pending, ready (three
  // when the probe is retried under load). Budget is five periods.
  const auto startup_deadline = std::chrono::steady_clock::now() + 5 * kPumpPeriod;
  while (coordinator->startup_snapshot().status != CoordinatorStartupStatus::kReady &&
    std::chrono::steady_clock::now() < startup_deadline)
  {
    std::this_thread::yield();
  }
  ASSERT_EQ(coordinator->startup_snapshot().status, CoordinatorStartupStatus::kReady);
  // Startup readiness is not admission readiness (see above). With one send and no retry, wait on
  // the published readiness.
  ASSERT_TRUE(readiness.wait_until_ready(4 * kPumpPeriod));

  Action::Goal goal;
  goal.has_object_id = true;
  goal.object_id = 17U;
  goal.has_lane_id = true;
  goal.lane_id = "lane_01";
  auto sent = client->async_send_goal(goal);
  ASSERT_EQ(sent.wait_for(2s), std::future_status::ready);
  const auto goal_handle = sent.get();
  ASSERT_TRUE(goal_handle);
  auto result_future = client->async_get_result(goal_handle);
  // Secured retained publication is prepared only once route_phase reaches kSecured, so delivery
  // is quantised in pump periods (one, occasionally three); budget is in periods.
  constexpr auto kSecuredTerminalBudget = 4 * kPumpPeriod;
  ASSERT_EQ(result_future.wait_for(kSecuredTerminalBudget), std::future_status::ready);
  const auto result = result_future.get();
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::ABORTED);
  ASSERT_TRUE(result.result);
  EXPECT_EQ(result.result->status, Action::Result::STATUS_EXTERNAL_INCONSISTENCY);
  EXPECT_FALSE(result.result->detail.empty());
  EXPECT_NE(result.result->detail.find("operator"), std::string::npos);
  EXPECT_EQ(hook_calls.load(), 1U);
  EXPECT_EQ(post_hook_successes.load(), 1U);
  EXPECT_EQ(preparation_failures.load(), 1U);
  EXPECT_EQ(world_state.reserve_count(), 0U);
  EXPECT_EQ(world_state.release_count(), 0U);
  EXPECT_FALSE(world_state.reserved());
  const auto failure = coordinator->steady_clock_failure_snapshot();
  EXPECT_TRUE(failure.provider_failed);
  EXPECT_TRUE(failure.handled);
  EXPECT_TRUE(failure.authority_secured);

  coordinator->request_shutdown();
  const auto shutdown_deadline = std::chrono::steady_clock::now() + 3s;
  while (coordinator->shutdown_status() == CoordinatorShutdownStatus::kDraining &&
    std::chrono::steady_clock::now() < shutdown_deadline)
  {
    std::this_thread::yield();
  }
  EXPECT_EQ(coordinator->shutdown_status(), CoordinatorShutdownStatus::kClean);
}

TEST_F(RestockActionCoordinatorNodeTest, ActiveSteadyClockFailureReleasesOwnedReservation)
{
  auto support = std::make_shared<rclcpp::Node>("reserved_clock_failure_test_support");
  WorldStateFixture world_state(support);
  tf2_ros::StaticTransformBroadcaster broadcaster(support);
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = support->now();
  transform.header.frame_id = "world";
  transform.child_frame_id = "shelf";
  transform.transform.translation.y = 0.55;
  transform.transform.translation.z = 0.75;
  transform.transform.rotation.w = 1.0;
  broadcaster.sendTransform(transform);
  send_grasp_center_transform(broadcaster, support->now());

  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {rclcpp::Parameter("product_catalog_path", RESTOCKER_TEST_PRODUCT_CATALOG),
      rclcpp::Parameter("workcell_geometry_path", RESTOCKER_TEST_WORKCELL_GEOMETRY),
      rclcpp::Parameter("gripper_geometry_path", RESTOCKER_TEST_GRIPPER_GEOMETRY),
      rclcpp::Parameter("world_state.get_snapshot_service", "/test_world/get_snapshot"),
      rclcpp::Parameter("world_state.reserve_task_service", "/test_world/reserve_task"),
      rclcpp::Parameter(
        "world_state.validate_reservation_service",
        "/test_world/validate_reservation"),
      rclcpp::Parameter(
        "world_state.release_reservation_service",
        "/test_world/release_reservation"),
      rclcpp::Parameter(
        "world_state.validate_execution_authority_service",
        "/test_world/validate_execution_authority"),
      rclcpp::Parameter("action_name", "/reserved_clock_failure_restock_product"),
      rclcpp::Parameter("pump_period_ms", 5),
      rclcpp::Parameter("selection.maximum_object_age_ms", 5000),
      rclcpp::Parameter("selection.lane_evidence_validity_ms", 5000)});
  std::atomic<bool> fail_clock{false};
  auto coordinator = std::make_shared<RestockActionCoordinatorNode>(
    options, RestockActionCoordinatorNodeDependencies{
        [&fail_clock]() -> SteadyTime {
          if (fail_clock.load(std::memory_order_acquire)) {
            throw std::runtime_error("injected reserved-goal clock failure");
          }
          return std::chrono::steady_clock::now();
        },
        {},
        {}});
  auto client =
    rclcpp_action::create_client<Action>(support, "/reserved_clock_failure_restock_product");
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions{}, 4U);
  executor.add_node(support);
  executor.add_node(coordinator);
  ExecutorSpinGuard spin(executor);
  ASSERT_TRUE(client->wait_for_action_server(2s));

  const auto startup_deadline = std::chrono::steady_clock::now() + 2s;
  while (coordinator->startup_snapshot().status != CoordinatorStartupStatus::kReady &&
    std::chrono::steady_clock::now() < startup_deadline)
  {
    std::this_thread::yield();
  }
  ASSERT_EQ(coordinator->startup_snapshot().status, CoordinatorStartupStatus::kReady);

  Action::Goal goal;
  goal.has_object_id = true;
  goal.object_id = 17U;
  goal.has_lane_id = true;
  goal.lane_id = "lane_01";
  const auto goal_handle = send_goal_until_accepted(client, goal);
  ASSERT_TRUE(goal_handle);
  auto result_future = client->async_get_result(goal_handle);

  const auto reservation_deadline = std::chrono::steady_clock::now() + 2s;
  while (world_state.validate_count() == 0U &&
    std::chrono::steady_clock::now() < reservation_deadline)
  {
    std::this_thread::yield();
  }
  ASSERT_GE(world_state.reserve_count(), 1U);
  ASSERT_GE(world_state.validate_count(), 1U);
  ASSERT_TRUE(world_state.reserved());

  fail_clock.store(true, std::memory_order_release);

  ASSERT_EQ(result_future.wait_for(5s), std::future_status::ready);
  const auto result = result_future.get();
  EXPECT_NE(result.code, rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_TRUE(result.result);
  EXPECT_NE(result.result->status, Action::Result::STATUS_SUCCEEDED);
  EXPECT_EQ(world_state.release_count(), 1U);
  EXPECT_FALSE(world_state.reserved());
  const auto failure = coordinator->steady_clock_failure_snapshot();
  EXPECT_TRUE(failure.provider_failed);
  EXPECT_TRUE(failure.handled);
  EXPECT_TRUE(failure.authority_secured);

  coordinator->request_shutdown();
  const auto shutdown_deadline = std::chrono::steady_clock::now() + 1s;
  while (coordinator->shutdown_status() == CoordinatorShutdownStatus::kDraining &&
    std::chrono::steady_clock::now() < shutdown_deadline)
  {
    std::this_thread::yield();
  }
  EXPECT_EQ(coordinator->shutdown_status(), CoordinatorShutdownStatus::kClean);
}

TEST_F(RestockActionCoordinatorNodeTest, ExistingFaultRouteOwnsConcurrentClockFailure)
{
  auto support = std::make_shared<rclcpp::Node>("fault_route_clock_race_test_support");
  WorldStateFixture world_state(support);
  tf2_ros::StaticTransformBroadcaster broadcaster(support);
  geometry_msgs::msg::TransformStamped shelf;
  shelf.header.stamp = support->now();
  shelf.header.frame_id = "world";
  shelf.child_frame_id = "shelf";
  shelf.transform.translation.y = 0.55;
  shelf.transform.translation.z = 0.75;
  shelf.transform.rotation.w = 1.0;
  broadcaster.sendTransform(shelf);
  send_grasp_center_transform(broadcaster, support->now());

  std::atomic<bool> fail_clock{false};
  std::atomic<std::size_t> route_calls{0U};
  OneShotCallBlocker route_blocker;
  auto coordinator = std::make_shared<RestockActionCoordinatorNode>(
    ready_coordinator_options("/fault_route_clock_race_restock_product"),
    RestockActionCoordinatorNodeDependencies{[&fail_clock]() -> SteadyTime {
        if (fail_clock.load(std::memory_order_acquire)) {
          throw std::runtime_error(
                  "injected concurrent clock failure");
        }
        return std::chrono::steady_clock::now();
      },
      {},
      {},
      [&route_calls, &route_blocker]() {
        ++route_calls;
        route_blocker.intercept();
      }});
  auto client =
    rclcpp_action::create_client<Action>(support, "/fault_route_clock_race_restock_product");
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions{}, 4U);
  executor.add_node(support);
  executor.add_node(coordinator);
  ExecutorSpinGuard spin(executor);
  ASSERT_TRUE(client->wait_for_action_server(2s));

  const auto startup_deadline = std::chrono::steady_clock::now() + 2s;
  while (coordinator->startup_snapshot().status != CoordinatorStartupStatus::kReady &&
    std::chrono::steady_clock::now() < startup_deadline)
  {
    std::this_thread::yield();
  }
  ASSERT_EQ(coordinator->startup_snapshot().status, CoordinatorStartupStatus::kReady);

  const auto startup_snapshots = world_state.snapshot_count();
  SnapshotBlockGuard snapshot_block(world_state);
  Action::Goal goal;
  goal.has_object_id = true;
  goal.object_id = 17U;
  goal.has_lane_id = true;
  goal.lane_id = "lane_01";
  const auto goal_handle = send_goal_until_accepted(client, goal);
  ASSERT_TRUE(goal_handle);
  auto result_future = client->async_get_result(goal_handle);

  const auto request_deadline = std::chrono::steady_clock::now() + 2s;
  while (world_state.snapshot_count() == startup_snapshots &&
    std::chrono::steady_clock::now() < request_deadline)
  {
    std::this_thread::yield();
  }
  ASSERT_GT(world_state.snapshot_count(), startup_snapshots);

  route_blocker.arm();
  shelf.header.stamp = support->now();
  shelf.transform.translation.y += 0.01;
  broadcaster.sendTransform(shelf);
  const bool route_blocked = route_blocker.wait_until_blocked(2s);
  if (!route_blocked) {
    route_blocker.release();
  }
  ASSERT_TRUE(route_blocked);

  auto authority_snapshot = std::async(
    std::launch::async, [coordinator]() {return coordinator->generation_authority_snapshot();});
  const auto authority_snapshot_status = authority_snapshot.wait_for(250ms);
  if (authority_snapshot_status != std::future_status::ready) {
    route_blocker.release();
  }
  const auto authority_while_route_blocked = authority_snapshot.get();
  EXPECT_EQ(authority_snapshot_status, std::future_status::ready);
  EXPECT_TRUE(authority_while_route_blocked.active_binding);
  EXPECT_FALSE(authority_while_route_blocked.pending_binding);
  EXPECT_FALSE(authority_while_route_blocked.pending_route_token_live);
  EXPECT_FALSE(authority_while_route_blocked.diagnostic_caches_fresh);

  fail_clock.store(true, std::memory_order_release);
  snapshot_block.unblock();
  const auto asynchronous_failure_deadline = std::chrono::steady_clock::now() + 2s;
  auto failure = coordinator->steady_clock_failure_snapshot();
  while ((!failure.provider_failed || !failure.first_observed_asynchronously) &&
    std::chrono::steady_clock::now() < asynchronous_failure_deadline)
  {
    std::this_thread::yield();
    failure = coordinator->steady_clock_failure_snapshot();
  }
  EXPECT_TRUE(failure.provider_failed);
  EXPECT_TRUE(failure.first_observed_asynchronously);
  EXPECT_FALSE(failure.handled);
  EXPECT_FALSE(failure.authority_secured);

  route_blocker.release();
  ASSERT_EQ(result_future.wait_for(5s), std::future_status::ready);
  const auto result = result_future.get();
  EXPECT_NE(result.code, rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_TRUE(result.result);
  EXPECT_NE(result.result->status, Action::Result::STATUS_SUCCEEDED);
  EXPECT_EQ(route_calls.load(), 1U);
  EXPECT_EQ(world_state.reserve_count(), 0U);
  EXPECT_EQ(world_state.release_count(), 0U);
  EXPECT_FALSE(world_state.reserved());

  const auto handled_deadline = std::chrono::steady_clock::now() + 1s;
  failure = coordinator->steady_clock_failure_snapshot();
  while (!failure.handled && std::chrono::steady_clock::now() < handled_deadline) {
    std::this_thread::yield();
    failure = coordinator->steady_clock_failure_snapshot();
  }
  EXPECT_TRUE(failure.handled);
  EXPECT_TRUE(failure.authority_secured);
  EXPECT_TRUE(failure.first_observed_asynchronously);

  coordinator->request_shutdown();
  const auto shutdown_deadline = std::chrono::steady_clock::now() + 1s;
  while (coordinator->shutdown_status() == CoordinatorShutdownStatus::kDraining &&
    std::chrono::steady_clock::now() < shutdown_deadline)
  {
    std::this_thread::yield();
  }
  EXPECT_EQ(coordinator->shutdown_status(), CoordinatorShutdownStatus::kClean);
}

TEST_F(RestockActionCoordinatorNodeTest, SimulationTimeRegressionCleansUpBeforeInhibition)
{
  rclcpp::NodeOptions support_options;
  support_options.append_parameter_override("use_sim_time", true);
  auto support =
    std::make_shared<rclcpp::Node>("simulation_time_regression_test_support", support_options);
  WorldStateFixture world_state(support);
  tf2_ros::StaticTransformBroadcaster broadcaster(support);
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "world";
  transform.child_frame_id = "shelf";
  transform.transform.translation.y = 0.55;
  transform.transform.translation.z = 0.75;
  transform.transform.rotation.w = 1.0;
  broadcaster.sendTransform(transform);
  send_grasp_center_transform(broadcaster, support->now());

  auto options = ready_coordinator_options("/simulation_time_regression_restock_product");
  options.append_parameter_override("use_sim_time", true);
  options.append_parameter_override(
    "status_topic",
    "/simulation_time_regression_coordinator/status");
  auto coordinator = std::make_shared<RestockActionCoordinatorNode>(options);
  auto clock_publisher =
    support->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::ClockQoS());
  auto client =
    rclcpp_action::create_client<Action>(support, "/simulation_time_regression_restock_product");
  auto status_observer =
    std::make_shared<rclcpp::Node>("simulation_time_regression_status_observer");
  std::mutex status_mutex;
  std::optional<restocker_interfaces::msg::RestockCoordinatorStatus> status;
  auto status_subscription =
    status_observer->create_subscription<restocker_interfaces::msg::RestockCoordinatorStatus>(
    "/simulation_time_regression_coordinator/status",
    rclcpp::QoS(1).reliable().transient_local(),
    [&status_mutex, &status](
      const restocker_interfaces::msg::RestockCoordinatorStatus::ConstSharedPtr message) {
      std::lock_guard lock(status_mutex);
      status = *message;
    });
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions{}, 4U);
  executor.add_node(support);
  executor.add_node(status_observer);
  ExecutorSpinGuard spin(executor);

  // The world-state fixture stamps robot telemetry with the support node's simulated clock, and
  // a zero stamp makes the snapshot invalid, which faults startup permanently. The coordinator
  // joins the executor only once that clock has run, so /clock reaching the coordinator first
  // cannot get its startup probe answered with a zero stamp.
  rosgraph_msgs::msg::Clock initial_clock;
  initial_clock.clock.sec = 100;
  const auto initial_clock_deadline = std::chrono::steady_clock::now() + 2s;
  while (support->now() < rclcpp::Time(100, 0, RCL_ROS_TIME) &&
    std::chrono::steady_clock::now() < initial_clock_deadline)
  {
    clock_publisher->publish(initial_clock);
    std::this_thread::sleep_for(5ms);
  }
  ASSERT_EQ(support->now(), rclcpp::Time(100, 0, RCL_ROS_TIME));
  executor.add_node(coordinator);
  ASSERT_TRUE(client->wait_for_action_server(2s));

  const auto startup_deadline = std::chrono::steady_clock::now() + 2s;
  while (coordinator->startup_snapshot().status != CoordinatorStartupStatus::kReady &&
    std::chrono::steady_clock::now() < startup_deadline)
  {
    clock_publisher->publish(initial_clock);
    std::this_thread::sleep_for(5ms);
  }
  ASSERT_EQ(coordinator->startup_snapshot().status, CoordinatorStartupStatus::kReady);

  Action::Goal goal;
  goal.has_object_id = true;
  goal.object_id = 17U;
  goal.has_lane_id = true;
  goal.lane_id = "lane_01";
  const auto goal_handle = send_goal_until_accepted(client, goal);
  ASSERT_TRUE(goal_handle);
  auto result_future = client->async_get_result(goal_handle);

  const auto reservation_deadline = std::chrono::steady_clock::now() + 2s;
  while (world_state.validate_count() == 0U &&
    std::chrono::steady_clock::now() < reservation_deadline)
  {
    clock_publisher->publish(initial_clock);
    std::this_thread::yield();
  }
  ASSERT_GE(world_state.validate_count(), 1U);
  ASSERT_TRUE(world_state.reserved());

  ReleaseBlockGuard release_block(world_state);
  rosgraph_msgs::msg::Clock regressed_clock;
  regressed_clock.clock.sec = 99;
  const auto cleanup_deadline = std::chrono::steady_clock::now() + 2s;
  bool cleanup_pending = false;
  while ((!cleanup_pending || world_state.release_count() == 0U) &&
    std::chrono::steady_clock::now() < cleanup_deadline)
  {
    clock_publisher->publish(regressed_clock);
    {
      std::lock_guard lock(status_mutex);
      cleanup_pending = status && !status->inhibited && !status->admission_ready &&
        status->detail.find("cleanup pending") != std::string::npos;
    }
    std::this_thread::sleep_for(5ms);
  }
  {
    std::lock_guard lock(status_mutex);
    ASSERT_TRUE(status);
    EXPECT_FALSE(status->inhibited) << status->detail;
    EXPECT_FALSE(status->admission_ready) << status->detail;
    EXPECT_NE(status->detail.find("cleanup pending"), std::string::npos) << status->detail;
  }
  EXPECT_TRUE(cleanup_pending);
  EXPECT_EQ(world_state.release_count(), 1U);
  EXPECT_TRUE(world_state.reserved());
  EXPECT_EQ(result_future.wait_for(100ms), std::future_status::timeout);
  const auto time_authority = coordinator->generation_authority_snapshot();
  ASSERT_TRUE(time_authority.active_epoch);
  EXPECT_EQ(time_authority.active_epoch->route_phase, ActiveFaultRoutePhase::kSecured);
  EXPECT_FALSE(time_authority.active_epoch->route_claim_outstanding);

  release_block.unblock();
  ASSERT_EQ(result_future.wait_for(5s), std::future_status::ready);
  const auto result = result_future.get();
  EXPECT_NE(result.code, rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_TRUE(result.result);
  EXPECT_NE(result.result->status, Action::Result::STATUS_SUCCEEDED);
  EXPECT_EQ(world_state.release_count(), 1U);
  EXPECT_FALSE(world_state.reserved());

  coordinator->request_shutdown();
  const auto shutdown_deadline = std::chrono::steady_clock::now() + 1s;
  while (coordinator->shutdown_status() == CoordinatorShutdownStatus::kDraining &&
    std::chrono::steady_clock::now() < shutdown_deadline)
  {
    std::this_thread::yield();
  }
  EXPECT_EQ(coordinator->shutdown_status(), CoordinatorShutdownStatus::kClean);
  (void)status_subscription;
}

TEST_F(RestockActionCoordinatorNodeTest, TerminalReservationWinsConcurrentClockFailure)
{
  auto support = std::make_shared<rclcpp::Node>("terminal_reservation_clock_test_support");
  WorldStateFixture world_state(support);
  tf2_ros::StaticTransformBroadcaster broadcaster(support);
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = support->now();
  transform.header.frame_id = "world";
  transform.child_frame_id = "shelf";
  transform.transform.translation.y = 0.55;
  transform.transform.translation.z = 0.75;
  transform.transform.rotation.w = 1.0;
  broadcaster.sendTransform(transform);
  send_grasp_center_transform(broadcaster, support->now());

  std::atomic<bool> fail_clock{false};
  std::atomic<std::size_t> hook_calls{0U};
  std::mutex reserved_epoch_mutex;
  std::optional<CoordinatorActiveFaultEpochSnapshot> reserved_epoch;
  std::weak_ptr<RestockActionCoordinatorNode> weak_coordinator;
  auto coordinator = std::make_shared<RestockActionCoordinatorNode>(
    ready_coordinator_options("/terminal_reservation_clock_restock_product"),
    RestockActionCoordinatorNodeDependencies{[&fail_clock]() -> SteadyTime {
        if (fail_clock.load(std::memory_order_acquire)) {
          throw std::runtime_error(
                  "injected post-reservation clock failure");
        }
        return std::chrono::steady_clock::now();
      },
      {},
      [&fail_clock, &hook_calls, &reserved_epoch_mutex, &reserved_epoch, &weak_coordinator]() {
        ++hook_calls;
        if (const auto locked = weak_coordinator.lock()) {
          const auto authority = locked->generation_authority_snapshot();
          std::lock_guard snapshot_lock(reserved_epoch_mutex);
          reserved_epoch = authority.active_epoch;
        }
        fail_clock.store(true, std::memory_order_release);
        if (const auto locked = weak_coordinator.lock()) {
          locked->request_shutdown();
        }
      }});
  weak_coordinator = coordinator;
  auto client =
    rclcpp_action::create_client<Action>(support, "/terminal_reservation_clock_restock_product");
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions{}, 4U);
  executor.add_node(support);
  executor.add_node(coordinator);
  ExecutorSpinGuard spin(executor);
  ASSERT_TRUE(client->wait_for_action_server(2s));

  const auto startup_deadline = std::chrono::steady_clock::now() + 2s;
  while (coordinator->startup_snapshot().status != CoordinatorStartupStatus::kReady &&
    std::chrono::steady_clock::now() < startup_deadline)
  {
    std::this_thread::yield();
  }
  ASSERT_EQ(coordinator->startup_snapshot().status, CoordinatorStartupStatus::kReady);

  Action::Goal goal;
  goal.has_object_id = true;
  goal.object_id = 17U;
  goal.has_lane_id = true;
  goal.lane_id = "lane_01";
  std::shared_ptr<rclcpp_action::ClientGoalHandle<Action>> goal_handle;
  for (std::size_t attempt = 0U; attempt < 20U && !goal_handle; ++attempt) {
    auto sent = client->async_send_goal(goal);
    ASSERT_EQ(sent.wait_for(500ms), std::future_status::ready);
    goal_handle = sent.get();
    if (!goal_handle) {
      std::this_thread::sleep_for(25ms);
    }
  }
  ASSERT_TRUE(goal_handle);
  const auto reservation_deadline = std::chrono::steady_clock::now() + 2s;
  while (world_state.validate_count() == 0U &&
    std::chrono::steady_clock::now() < reservation_deadline)
  {
    std::this_thread::yield();
  }
  ASSERT_GE(world_state.validate_count(), 1U);
  ASSERT_TRUE(world_state.reserved());

  auto canceled = client->async_cancel_goal(goal_handle);
  ASSERT_EQ(canceled.wait_for(2s), std::future_status::ready);
  ASSERT_FALSE(canceled.get()->goals_canceling.empty());
  auto result_future = client->async_get_result(goal_handle);
  ASSERT_EQ(result_future.wait_for(5s), std::future_status::ready);
  const auto result = result_future.get();
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::CANCELED);
  ASSERT_TRUE(result.result);
  EXPECT_EQ(result.result->status, Action::Result::STATUS_CANCELED);
  EXPECT_EQ(hook_calls.load(), 1U);
  EXPECT_EQ(world_state.release_count(), 1U);
  EXPECT_FALSE(world_state.reserved());
  {
    std::lock_guard snapshot_lock(reserved_epoch_mutex);
    ASSERT_TRUE(reserved_epoch);
    EXPECT_EQ(reserved_epoch->route_phase, ActiveFaultRoutePhase::kSecured);
    EXPECT_EQ(
      reserved_epoch->terminal_phase, ActiveTerminalPhase::kPublicationReserved);
    EXPECT_FALSE(reserved_epoch->retained_terminal);
    EXPECT_FALSE(reserved_epoch->middleware_publication_returned);
    EXPECT_FALSE(reserved_epoch->retirement_eligible);
  }
  const auto failure = coordinator->steady_clock_failure_snapshot();
  EXPECT_TRUE(failure.provider_failed);
  EXPECT_TRUE(failure.handled);
  EXPECT_TRUE(failure.authority_secured);

  const auto shutdown_deadline = std::chrono::steady_clock::now() + 1s;
  while (coordinator->shutdown_status() == CoordinatorShutdownStatus::kDraining &&
    std::chrono::steady_clock::now() < shutdown_deadline)
  {
    std::this_thread::yield();
  }
  EXPECT_EQ(coordinator->shutdown_status(), CoordinatorShutdownStatus::kClean);
}

// A withheld terminal is retained only after the release reply, its readback and a pump tick;
// under load that outlasts any fixed wait, so the finality tests poll for the state they assert
// and keep checking that no result reached the client meanwhile (Card 070).
[[nodiscard]] CoordinatorGenerationAuthoritySnapshot wait_for_retained_unsecured_terminal(
  RestockActionCoordinatorNode & coordinator, std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  auto authority = coordinator.generation_authority_snapshot();
  while (!(authority.active_epoch && authority.active_epoch->retained_terminal &&
    authority.active_epoch->terminal_phase == ActiveTerminalPhase::kRetained &&
    authority.active_epoch->route_phase == ActiveFaultRoutePhase::kUnsecured) &&
    std::chrono::steady_clock::now() < deadline)
  {
    std::this_thread::sleep_for(5ms);
    authority = coordinator.generation_authority_snapshot();
  }
  return authority;
}

TEST_F(RestockActionCoordinatorNodeTest, ThrowingPostReservationHookWithholdsFinality)
{
  auto support = std::make_shared<rclcpp::Node>("throwing_terminal_hook_test_support");
  WorldStateFixture world_state(support);
  tf2_ros::StaticTransformBroadcaster broadcaster(support);
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = support->now();
  transform.header.frame_id = "world";
  transform.child_frame_id = "shelf";
  transform.transform.translation.y = 0.55;
  transform.transform.translation.z = 0.75;
  transform.transform.rotation.w = 1.0;
  broadcaster.sendTransform(transform);
  send_grasp_center_transform(broadcaster, support->now());

  std::atomic<std::size_t> hook_calls{0U};
  auto coordinator = std::make_shared<RestockActionCoordinatorNode>(
    ready_coordinator_options("/throwing_terminal_hook_restock_product"),
    RestockActionCoordinatorNodeDependencies{[]() {return std::chrono::steady_clock::now();},
      {},
      [&hook_calls]() {
        ++hook_calls;
        throw std::runtime_error(
                "injected post-reservation hook failure");
      }});
  auto client =
    rclcpp_action::create_client<Action>(support, "/throwing_terminal_hook_restock_product");
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions{}, 4U);
  executor.add_node(support);
  executor.add_node(coordinator);
  ExecutorSpinGuard spin(executor);
  ASSERT_TRUE(client->wait_for_action_server(2s));

  const auto startup_deadline = std::chrono::steady_clock::now() + 2s;
  while (coordinator->startup_snapshot().status != CoordinatorStartupStatus::kReady &&
    std::chrono::steady_clock::now() < startup_deadline)
  {
    std::this_thread::yield();
  }
  ASSERT_EQ(coordinator->startup_snapshot().status, CoordinatorStartupStatus::kReady);

  Action::Goal goal;
  goal.has_object_id = true;
  goal.object_id = 17U;
  goal.has_lane_id = true;
  goal.lane_id = "lane_01";
  std::shared_ptr<rclcpp_action::ClientGoalHandle<Action>> goal_handle;
  for (std::size_t attempt = 0U; attempt < 20U && !goal_handle; ++attempt) {
    auto sent = client->async_send_goal(goal);
    ASSERT_EQ(sent.wait_for(500ms), std::future_status::ready);
    goal_handle = sent.get();
    if (!goal_handle) {
      std::this_thread::sleep_for(25ms);
    }
  }
  ASSERT_TRUE(goal_handle);
  const auto reservation_deadline = std::chrono::steady_clock::now() + 2s;
  while (world_state.validate_count() == 0U &&
    std::chrono::steady_clock::now() < reservation_deadline)
  {
    std::this_thread::yield();
  }
  ASSERT_GE(world_state.validate_count(), 1U);
  ASSERT_TRUE(world_state.reserved());

  auto canceled = client->async_cancel_goal(goal_handle);
  ASSERT_EQ(canceled.wait_for(2s), std::future_status::ready);
  ASSERT_FALSE(canceled.get()->goals_canceling.empty());
  auto result_future = client->async_get_result(goal_handle);
  const auto authority = wait_for_retained_unsecured_terminal(*coordinator, 5s);
  EXPECT_EQ(result_future.wait_for(250ms), std::future_status::timeout);
  EXPECT_EQ(hook_calls.load(), 1U);
  EXPECT_EQ(world_state.release_count(), 1U);
  EXPECT_FALSE(world_state.reserved());
  ASSERT_TRUE(authority.active_epoch);
  EXPECT_EQ(authority.active_epoch->route_phase, ActiveFaultRoutePhase::kUnsecured);
  EXPECT_EQ(authority.active_epoch->terminal_phase, ActiveTerminalPhase::kRetained);
  EXPECT_TRUE(authority.active_epoch->retained_terminal);
  EXPECT_FALSE(authority.active_epoch->middleware_publication_returned);
  EXPECT_FALSE(authority.active_epoch->retirement_eligible);

  coordinator->request_shutdown();
  std::this_thread::sleep_for(50ms);
  EXPECT_EQ(coordinator->shutdown_status(), CoordinatorShutdownStatus::kDraining);
}

void verify_throwing_action_result_publisher(
  const std::string & test_name, bool throw_standard_exception)
{
  auto support = std::make_shared<rclcpp::Node>(test_name + "_test_support");
  WorldStateFixture world_state(support);
  tf2_ros::StaticTransformBroadcaster broadcaster(support);
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = support->now();
  transform.header.frame_id = "world";
  transform.child_frame_id = "shelf";
  transform.transform.translation.y = 0.55;
  transform.transform.translation.z = 0.75;
  transform.transform.rotation.w = 1.0;
  broadcaster.sendTransform(transform);
  send_grasp_center_transform(broadcaster, support->now());

  std::atomic<std::size_t> barrier_calls{0U};
  std::atomic<std::size_t> publisher_calls{0U};
  auto throwing_publisher = std::make_shared<ThrowingActionResultPublisher>(
    publisher_calls, throw_standard_exception);
  std::weak_ptr<ActionResultPublisher> publisher_lifetime = throwing_publisher;
  auto coordinator = std::make_shared<RestockActionCoordinatorNode>(
    ready_coordinator_options("/" + test_name + "_restock_product"),
    RestockActionCoordinatorNodeDependencies{[]() {return std::chrono::steady_clock::now();},
      {},
      [&barrier_calls]() {++barrier_calls;},
      {},
      throwing_publisher});
  throwing_publisher.reset();
  EXPECT_FALSE(publisher_lifetime.expired());
  auto client = rclcpp_action::create_client<Action>(
    support, "/" + test_name + "_restock_product");
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions{}, 4U);
  executor.add_node(support);
  executor.add_node(coordinator);
  ExecutorSpinGuard spin(executor);
  ASSERT_TRUE(client->wait_for_action_server(2s));

  const auto startup_deadline = std::chrono::steady_clock::now() + 2s;
  while (coordinator->startup_snapshot().status != CoordinatorStartupStatus::kReady &&
    std::chrono::steady_clock::now() < startup_deadline)
  {
    std::this_thread::yield();
  }
  ASSERT_EQ(coordinator->startup_snapshot().status, CoordinatorStartupStatus::kReady);

  Action::Goal goal;
  goal.has_object_id = true;
  goal.object_id = 17U;
  goal.has_lane_id = true;
  goal.lane_id = "lane_01";
  const auto goal_handle = send_goal_until_accepted(client, goal);
  ASSERT_TRUE(goal_handle);
  const auto reservation_deadline = std::chrono::steady_clock::now() + 2s;
  while (world_state.validate_count() == 0U &&
    std::chrono::steady_clock::now() < reservation_deadline)
  {
    std::this_thread::yield();
  }
  ASSERT_GE(world_state.validate_count(), 1U);
  ASSERT_TRUE(world_state.reserved());

  auto canceled = client->async_cancel_goal(goal_handle);
  ASSERT_EQ(canceled.wait_for(2s), std::future_status::ready);
  ASSERT_FALSE(canceled.get()->goals_canceling.empty());
  auto result_future = client->async_get_result(goal_handle);
  const auto authority = wait_for_retained_unsecured_terminal(*coordinator, 5s);
  EXPECT_EQ(result_future.wait_for(250ms), std::future_status::timeout);
  EXPECT_EQ(barrier_calls.load(), 1U);
  EXPECT_EQ(publisher_calls.load(), 1U);
  EXPECT_EQ(world_state.release_count(), 1U);
  EXPECT_FALSE(world_state.reserved());
  ASSERT_TRUE(authority.active_epoch);
  EXPECT_EQ(authority.active_epoch->route_phase, ActiveFaultRoutePhase::kUnsecured);
  EXPECT_EQ(authority.active_epoch->terminal_phase, ActiveTerminalPhase::kRetained);
  EXPECT_TRUE(authority.active_epoch->retained_terminal);
  EXPECT_FALSE(authority.active_epoch->middleware_publication_returned);
  EXPECT_FALSE(authority.active_epoch->retirement_eligible);

  coordinator->request_shutdown();
  std::this_thread::sleep_for(50ms);
  EXPECT_EQ(coordinator->shutdown_status(), CoordinatorShutdownStatus::kDraining);
}

TEST_F(RestockActionCoordinatorNodeTest, ThrowingActionResultPublisherWithholdsFinality)
{
  verify_throwing_action_result_publisher("throwing_action_result_publisher", true);
}

TEST_F(RestockActionCoordinatorNodeTest, NonstandardThrowingPublisherWithholdsFinality)
{
  verify_throwing_action_result_publisher("nonstandard_throwing_publisher", false);
}

TEST_F(RestockActionCoordinatorNodeTest, FailedActiveRouteWithholdsFinalityAndForwardWork)
{
  auto support = std::make_shared<rclcpp::Node>("failed_active_route_test_support");
  WorldStateFixture world_state(support);
  tf2_ros::StaticTransformBroadcaster broadcaster(support);
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = support->now();
  transform.header.frame_id = "world";
  transform.child_frame_id = "shelf";
  transform.transform.translation.y = 0.55;
  transform.transform.translation.z = 0.75;
  transform.transform.rotation.w = 1.0;
  broadcaster.sendTransform(transform);
  send_grasp_center_transform(broadcaster, support->now());

  std::atomic<std::size_t> route_attempts{0U};
  auto coordinator = std::make_shared<RestockActionCoordinatorNode>(
    ready_coordinator_options("/failed_active_route_restock_product"),
    RestockActionCoordinatorNodeDependencies{[]() {return std::chrono::steady_clock::now();},
      {},
      {},
      [&route_attempts]() {
        ++route_attempts;
        throw std::runtime_error(
                "injected active route failure");
      }});
  auto client =
    rclcpp_action::create_client<Action>(support, "/failed_active_route_restock_product");
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions{}, 4U);
  executor.add_node(support);
  executor.add_node(coordinator);
  ExecutorSpinGuard spin(executor);
  ASSERT_TRUE(client->wait_for_action_server(2s));

  const auto startup_deadline = std::chrono::steady_clock::now() + 2s;
  while (coordinator->startup_snapshot().status != CoordinatorStartupStatus::kReady &&
    std::chrono::steady_clock::now() < startup_deadline)
  {
    std::this_thread::yield();
  }
  ASSERT_EQ(coordinator->startup_snapshot().status, CoordinatorStartupStatus::kReady);

  Action::Goal goal;
  goal.has_object_id = true;
  goal.object_id = 17U;
  goal.has_lane_id = true;
  goal.lane_id = "lane_01";
  const auto goal_handle = send_goal_until_accepted(client, goal);
  ASSERT_TRUE(goal_handle);
  const auto reservation_deadline = std::chrono::steady_clock::now() + 2s;
  while (world_state.validate_count() == 0U &&
    std::chrono::steady_clock::now() < reservation_deadline)
  {
    std::this_thread::yield();
  }
  ASSERT_GE(world_state.validate_count(), 1U);
  ASSERT_TRUE(world_state.reserved());

  auto canceled = client->async_cancel_goal(goal_handle);
  ASSERT_EQ(canceled.wait_for(2s), std::future_status::ready);
  EXPECT_TRUE(canceled.get()->goals_canceling.empty());
  const auto failed_authority = coordinator->generation_authority_snapshot();
  ASSERT_TRUE(failed_authority.active_binding);
  ASSERT_TRUE(failed_authority.active_epoch);
  EXPECT_EQ(failed_authority.active_epoch->route_phase, ActiveFaultRoutePhase::kUnsecured);
  EXPECT_FALSE(failed_authority.active_epoch->route_claim_outstanding);
  auto result_future = client->async_get_result(goal_handle);
  EXPECT_EQ(result_future.wait_for(250ms), std::future_status::timeout);
  EXPECT_EQ(route_attempts.load(), 1U);
  EXPECT_EQ(world_state.release_count(), 0U);
  EXPECT_TRUE(world_state.reserved());

  auto second_sent = client->async_send_goal(goal);
  ASSERT_EQ(second_sent.wait_for(2s), std::future_status::ready);
  EXPECT_FALSE(second_sent.get());

  coordinator->request_shutdown();
  std::this_thread::sleep_for(50ms);
  EXPECT_EQ(coordinator->shutdown_status(), CoordinatorShutdownStatus::kDraining);
}

TEST_F(RestockActionCoordinatorNodeTest, RejectsStructurallyValidGoalUntilDependenciesAreReady)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {rclcpp::Parameter("product_catalog_path", RESTOCKER_TEST_PRODUCT_CATALOG),
      rclcpp::Parameter("workcell_geometry_path", RESTOCKER_TEST_WORKCELL_GEOMETRY),
      rclcpp::Parameter("gripper_geometry_path", RESTOCKER_TEST_GRIPPER_GEOMETRY),
      rclcpp::Parameter("action_name", "/unready_restock_product"),
      rclcpp::Parameter("world_state.get_snapshot_service", "/missing/get_snapshot"),
      rclcpp::Parameter("world_state.reserve_task_service", "/missing/reserve_task"),
      rclcpp::Parameter(
        "world_state.validate_reservation_service",
        "/missing/validate_reservation"),
      rclcpp::Parameter(
        "world_state.release_reservation_service",
        "/missing/release_reservation")});
  auto coordinator = std::make_shared<RestockActionCoordinatorNode>(options);
  auto client_node = std::make_shared<rclcpp::Node>("unready_restock_action_test_client");
  auto client = rclcpp_action::create_client<Action>(client_node, "/unready_restock_product");
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions{}, 3U);
  executor.add_node(coordinator);
  executor.add_node(client_node);
  ExecutorSpinGuard spin(executor);
  ASSERT_TRUE(client->wait_for_action_server(2s));

  Action::Goal goal;
  goal.has_object_id = true;
  goal.object_id = 17U;
  goal.has_lane_id = true;
  goal.lane_id = "lane_01";
  auto sent = client->async_send_goal(goal);
  ASSERT_EQ(sent.wait_for(1s), std::future_status::ready);
  EXPECT_FALSE(sent.get());
  coordinator->request_shutdown();
  const auto deadline = std::chrono::steady_clock::now() + 1s;
  while (coordinator->shutdown_status() == CoordinatorShutdownStatus::kDraining &&
    std::chrono::steady_clock::now() < deadline)
  {
    std::this_thread::sleep_for(5ms);
  }
  EXPECT_EQ(coordinator->shutdown_status(), CoordinatorShutdownStatus::kClean);
}

TEST_F(RestockActionCoordinatorNodeTest, MissingInitialTransformIsVisibleAndDoesNotInhibit)
{
  auto support = std::make_shared<rclcpp::Node>("restock_missing_transform_test_support");
  WorldStateFixture world_state(support);
  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {rclcpp::Parameter("product_catalog_path", RESTOCKER_TEST_PRODUCT_CATALOG),
      rclcpp::Parameter("workcell_geometry_path", RESTOCKER_TEST_WORKCELL_GEOMETRY),
      rclcpp::Parameter("gripper_geometry_path", RESTOCKER_TEST_GRIPPER_GEOMETRY),
      rclcpp::Parameter("world_state.get_snapshot_service", "/test_world/get_snapshot"),
      rclcpp::Parameter("world_state.reserve_task_service", "/test_world/reserve_task"),
      rclcpp::Parameter(
        "world_state.validate_reservation_service",
        "/test_world/validate_reservation"),
      rclcpp::Parameter(
        "world_state.release_reservation_service",
        "/test_world/release_reservation"),
      rclcpp::Parameter(
        "world_state.validate_execution_authority_service",
        "/test_world/validate_execution_authority"),
      rclcpp::Parameter("action_name", "/missing_transform_restock_product"),
      rclcpp::Parameter("status_topic", "/missing_transform_coordinator/status"),
      rclcpp::Parameter("pump_period_ms", 5)});
  auto coordinator = std::make_shared<RestockActionCoordinatorNode>(options);
  std::mutex status_mutex;
  std::optional<restocker_interfaces::msg::RestockCoordinatorStatus> status;
  auto subscription =
    support->create_subscription<restocker_interfaces::msg::RestockCoordinatorStatus>(
    "/missing_transform_coordinator/status",
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
    [&status_mutex, &status](
      const restocker_interfaces::msg::RestockCoordinatorStatus::ConstSharedPtr message) {
      std::lock_guard lock(status_mutex);
      status = *message;
    });
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions{}, 4U);
  executor.add_node(support);
  executor.add_node(coordinator);
  ExecutorSpinGuard spin(executor);

  bool observed = false;
  const auto deadline = std::chrono::steady_clock::now() + 2s;
  while (!observed && std::chrono::steady_clock::now() < deadline) {
    {
      std::lock_guard lock(status_mutex);
      observed = status && !status->inhibited && !status->admission_ready &&
        status->detail.find("missing") != std::string::npos &&
        status->detail.find("non-rigid") != std::string::npos;
    }
    std::this_thread::sleep_for(5ms);
  }
  EXPECT_TRUE(observed);
  coordinator->request_shutdown();
  (void)subscription;
}

TEST_F(RestockActionCoordinatorNodeTest, InhibitsWhenRuntimeToolDatumContradictsYaml)
{
  auto support = std::make_shared<rclcpp::Node>("restock_tool_datum_test_support");
  WorldStateFixture world_state(support);
  tf2_ros::StaticTransformBroadcaster broadcaster(support);
  geometry_msgs::msg::TransformStamped shelf;
  shelf.header.stamp = support->now();
  shelf.header.frame_id = "world";
  shelf.child_frame_id = "shelf";
  shelf.transform.rotation.w = 1.0;
  broadcaster.sendTransform(shelf);
  geometry_msgs::msg::TransformStamped grasp;
  grasp.header.stamp = support->now();
  grasp.header.frame_id = "tool0";
  grasp.child_frame_id = "grasp_center";
  grasp.transform.translation.z = 0.13;
  grasp.transform.rotation.w = 1.0;
  broadcaster.sendTransform(grasp);

  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {rclcpp::Parameter("product_catalog_path", RESTOCKER_TEST_PRODUCT_CATALOG),
      rclcpp::Parameter("workcell_geometry_path", RESTOCKER_TEST_WORKCELL_GEOMETRY),
      rclcpp::Parameter("gripper_geometry_path", RESTOCKER_TEST_GRIPPER_GEOMETRY),
      rclcpp::Parameter("world_state.get_snapshot_service", "/test_world/get_snapshot"),
      rclcpp::Parameter("world_state.reserve_task_service", "/test_world/reserve_task"),
      rclcpp::Parameter(
        "world_state.validate_reservation_service",
        "/test_world/validate_reservation"),
      rclcpp::Parameter(
        "world_state.release_reservation_service",
        "/test_world/release_reservation"),
      rclcpp::Parameter(
        "world_state.validate_execution_authority_service",
        "/test_world/validate_execution_authority"),
      rclcpp::Parameter("action_name", "/tool_datum_restock_product"),
      rclcpp::Parameter("status_topic", "/tool_datum_coordinator/status"),
      rclcpp::Parameter("pump_period_ms", 5)});
  auto coordinator = std::make_shared<RestockActionCoordinatorNode>(options);
  std::mutex status_mutex;
  std::optional<restocker_interfaces::msg::RestockCoordinatorStatus> status;
  auto subscription =
    support->create_subscription<restocker_interfaces::msg::RestockCoordinatorStatus>(
    "/tool_datum_coordinator/status",
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
    [&status_mutex, &status](
      const restocker_interfaces::msg::RestockCoordinatorStatus::ConstSharedPtr message) {
      std::lock_guard lock(status_mutex);
      status = *message;
    });
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions{}, 4U);
  executor.add_node(support);
  executor.add_node(coordinator);
  ExecutorSpinGuard spin(executor);

  const auto deadline = std::chrono::steady_clock::now() + 2s;
  bool inhibited = false;
  while (!inhibited && std::chrono::steady_clock::now() < deadline) {
    {
      std::lock_guard lock(status_mutex);
      inhibited = status && status->inhibited;
    }
    std::this_thread::sleep_for(5ms);
  }
  EXPECT_TRUE(inhibited);
  {
    std::lock_guard lock(status_mutex);
    ASSERT_TRUE(status);
    EXPECT_FALSE(status->admission_ready);
    EXPECT_NE(status->detail.find("disagrees"), std::string::npos);
  }
  coordinator->request_shutdown();
  (void)subscription;
}

TEST_F(RestockActionCoordinatorNodeTest, OrphanedReservationInhibitsWithoutMutation)
{
  auto support = std::make_shared<rclcpp::Node>("restock_action_orphan_support");
  WorldStateFixture world_state(support);
  world_state.seed_reserved_orphan();
  tf2_ros::StaticTransformBroadcaster broadcaster(support);
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = support->now();
  transform.header.frame_id = "world";
  transform.child_frame_id = "shelf";
  transform.transform.rotation.w = 1.0;
  broadcaster.sendTransform(transform);
  send_grasp_center_transform(broadcaster, support->now());

  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {rclcpp::Parameter("product_catalog_path", RESTOCKER_TEST_PRODUCT_CATALOG),
      rclcpp::Parameter("workcell_geometry_path", RESTOCKER_TEST_WORKCELL_GEOMETRY),
      rclcpp::Parameter("gripper_geometry_path", RESTOCKER_TEST_GRIPPER_GEOMETRY),
      rclcpp::Parameter("world_state.get_snapshot_service", "/test_world/get_snapshot"),
      rclcpp::Parameter("world_state.reserve_task_service", "/test_world/reserve_task"),
      rclcpp::Parameter(
        "world_state.validate_reservation_service",
        "/test_world/validate_reservation"),
      rclcpp::Parameter(
        "world_state.release_reservation_service",
        "/test_world/release_reservation"),
      rclcpp::Parameter(
        "world_state.validate_execution_authority_service",
        "/test_world/validate_execution_authority"),
      rclcpp::Parameter("action_name", "/test_orphan_restock_product"),
      rclcpp::Parameter("status_topic", "/test_orphan_coordinator/status"),
      rclcpp::Parameter("pump_period_ms", 5)});
  auto coordinator = std::make_shared<RestockActionCoordinatorNode>(options);

  std::mutex status_mutex;
  std::optional<restocker_interfaces::msg::RestockCoordinatorStatus> observed_status;
  auto status_subscription =
    support->create_subscription<restocker_interfaces::msg::RestockCoordinatorStatus>(
    "/test_orphan_coordinator/status",
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
    [&status_mutex, &observed_status](
      const restocker_interfaces::msg::RestockCoordinatorStatus::ConstSharedPtr message) {
      std::lock_guard lock(status_mutex);
      observed_status = *message;
    });
  auto client = rclcpp_action::create_client<Action>(support, "/test_orphan_restock_product");
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions{}, 4U);
  executor.add_node(support);
  executor.add_node(coordinator);
  ExecutorSpinGuard spin(executor);
  ASSERT_TRUE(client->wait_for_action_server(2s));

  const auto authority_deadline = std::chrono::steady_clock::now() + 2s;
  while (coordinator->startup_snapshot().status != CoordinatorStartupStatus::kOrphanedReservation &&
    std::chrono::steady_clock::now() < authority_deadline)
  {
    std::this_thread::sleep_for(5ms);
  }
  const auto startup = coordinator->startup_snapshot();
  ASSERT_EQ(startup.status, CoordinatorStartupStatus::kOrphanedReservation);
  ASSERT_TRUE(startup.orphan);
  EXPECT_EQ(startup.orphan->reservation_id, 81U);
  EXPECT_EQ(startup.orphan->request_id, "orphaned-request-81");
  EXPECT_EQ(startup.orphan->stage, ReservationStage::Reserved);

  const auto status_deadline = std::chrono::steady_clock::now() + 1s;
  for (;; ) {
    {
      std::lock_guard lock(status_mutex);
      if (observed_status &&
        observed_status->startup_state == observed_status->STARTUP_ORPHANED_RESERVATION)
      {
        EXPECT_TRUE(observed_status->inhibited);
        EXPECT_FALSE(observed_status->admission_ready);
        EXPECT_TRUE(observed_status->has_orphaned_reservation);
        EXPECT_EQ(observed_status->orphaned_reservation.reservation_id, 81U);
        break;
      }
    }
    ASSERT_LT(std::chrono::steady_clock::now(), status_deadline);
    std::this_thread::sleep_for(5ms);
  }

  Action::Goal goal;
  goal.has_object_id = true;
  goal.object_id = 17U;
  goal.has_lane_id = true;
  goal.lane_id = "lane_01";
  auto sent = client->async_send_goal(goal);
  ASSERT_EQ(sent.wait_for(1s), std::future_status::ready);
  EXPECT_FALSE(sent.get());
  EXPECT_GE(world_state.snapshot_count(), 1U);
  EXPECT_EQ(world_state.reserve_count(), 0U);
  EXPECT_EQ(world_state.validate_count(), 0U);
  EXPECT_EQ(world_state.release_count(), 0U);

  coordinator->request_shutdown();
  const auto shutdown_deadline = std::chrono::steady_clock::now() + 1s;
  while (coordinator->shutdown_status() == CoordinatorShutdownStatus::kDraining &&
    std::chrono::steady_clock::now() < shutdown_deadline)
  {
    std::this_thread::sleep_for(5ms);
  }
  EXPECT_EQ(coordinator->shutdown_status(), CoordinatorShutdownStatus::kClean);
  (void)status_subscription;
}

TEST_F(RestockActionCoordinatorNodeTest, ReservesReachesStagingAndCancelsWithReleaseProof)
{
  constexpr std::size_t retirement_pump_budget = 10U;
  constexpr auto retirement_observation_timeout = 2s;
  constexpr auto pump_period = 5ms;
  auto support = std::make_shared<rclcpp::Node>("restock_action_coordinator_test_support");
  WorldStateFixture world_state(support);
  tf2_ros::StaticTransformBroadcaster broadcaster(support);
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = support->now();
  transform.header.frame_id = "world";
  transform.child_frame_id = "shelf";
  transform.transform.translation.y = 0.55;
  transform.transform.translation.z = 0.75;
  transform.transform.rotation.w = 1.0;
  broadcaster.sendTransform(transform);
  send_grasp_center_transform(broadcaster, support->now());

  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {rclcpp::Parameter("product_catalog_path", RESTOCKER_TEST_PRODUCT_CATALOG),
      rclcpp::Parameter("workcell_geometry_path", RESTOCKER_TEST_WORKCELL_GEOMETRY),
      rclcpp::Parameter("gripper_geometry_path", RESTOCKER_TEST_GRIPPER_GEOMETRY),
      rclcpp::Parameter("world_state.get_snapshot_service", "/test_world/get_snapshot"),
      rclcpp::Parameter("world_state.reserve_task_service", "/test_world/reserve_task"),
      rclcpp::Parameter(
        "world_state.validate_reservation_service",
        "/test_world/validate_reservation"),
      rclcpp::Parameter(
        "world_state.release_reservation_service",
        "/test_world/release_reservation"),
      rclcpp::Parameter(
        "world_state.validate_execution_authority_service",
        "/test_world/validate_execution_authority"),
      rclcpp::Parameter("action_name", "/test_restock_product"),
      rclcpp::Parameter("pump_period_ms", pump_period.count()),
      rclcpp::Parameter("selection.maximum_object_age_ms", 5000),
      rclcpp::Parameter("selection.lane_evidence_validity_ms", 5000)});
  auto coordinator = std::make_shared<RestockActionCoordinatorNode>(options);
  const auto parameter_result = coordinator->set_parameter(rclcpp::Parameter("pump_period_ms", 6));
  EXPECT_FALSE(parameter_result.successful);
  const auto grasp_parameter_result =
    coordinator->set_parameter(
    rclcpp::Parameter("grasp.minimum_center_height_above_base_m", 0.01));
  EXPECT_FALSE(grasp_parameter_result.successful);
  const auto gripper_path_result = coordinator->set_parameter(
    rclcpp::Parameter("gripper_geometry_path", "/tmp/untrusted-gripper.yaml"));
  EXPECT_FALSE(gripper_path_result.successful);
  auto client = rclcpp_action::create_client<Action>(support, "/test_restock_product");

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions{}, 4U);
  executor.add_node(support);
  executor.add_node(coordinator);
  ExecutorSpinGuard spin(executor);
  ASSERT_TRUE(client->wait_for_action_server(2s));

  Action::Goal malformed;
  malformed.object_id = 17U;
  auto malformed_sent = client->async_send_goal(malformed);
  ASSERT_EQ(malformed_sent.wait_for(1s), std::future_status::ready);
  EXPECT_FALSE(malformed_sent.get());

  std::promise<void> staging_promise;
  auto staging = staging_promise.get_future();
  std::once_flag staging_once;
  std::mutex feedback_mutex;
  std::vector<std::string> feedback_trace;
  std::vector<std::int64_t> feedback_elapsed_nanoseconds;
  rclcpp_action::Client<Action>::SendGoalOptions send_options;
  send_options.feedback_callback = [&staging_promise, &staging_once, &feedback_mutex,
      &feedback_trace, &feedback_elapsed_nanoseconds](
    const auto, const auto feedback) {
    {
      std::lock_guard lock(feedback_mutex);
      feedback_trace.push_back(std::to_string(feedback->state) + ": " + feedback->detail);
      feedback_elapsed_nanoseconds.push_back(
        rclcpp::Duration(feedback->elapsed_sim_time).nanoseconds());
    }
    if (feedback->state == Action::Feedback::STATE_PLAN_PRE_GRASP) {
      std::call_once(staging_once, [&staging_promise]() {staging_promise.set_value();});
    }
  };

  Action::Goal goal;
  goal.has_object_id = true;
  goal.object_id = 17U;
  goal.has_lane_id = true;
  goal.lane_id = "lane_01";
  std::shared_ptr<rclcpp_action::ClientGoalHandle<Action>> goal_handle;
  for (std::size_t attempt = 0U; attempt < 20U && !goal_handle; ++attempt) {
    auto sent = client->async_send_goal(goal, send_options);
    ASSERT_EQ(sent.wait_for(500ms), std::future_status::ready);
    goal_handle = sent.get();
    if (!goal_handle) {
      std::this_thread::sleep_for(25ms);
    }
  }
  ASSERT_TRUE(goal_handle);
  const auto staging_status = staging.wait_for(5s);
  {
    std::lock_guard lock(feedback_mutex);
    std::string trace;
    for (const auto & item : feedback_trace) {
      trace += item + "\n";
    }
    ASSERT_EQ(staging_status, std::future_status::ready) << trace;
    ASSERT_FALSE(feedback_elapsed_nanoseconds.empty());
    EXPECT_GE(feedback_elapsed_nanoseconds.front(), 0);
    EXPECT_TRUE(
      std::is_sorted(feedback_elapsed_nanoseconds.begin(), feedback_elapsed_nanoseconds.end()));
  }

  {
    const auto quiesce_deadline = std::chrono::steady_clock::now() + 2s;
    std::optional<CoordinatorActiveFaultEpochSnapshot> epoch;
    do {
      const auto authority = coordinator->generation_authority_snapshot();
      if (authority.active_epoch && !authority.active_epoch->feedback_outstanding) {
        epoch = authority.active_epoch;
        break;
      }
      std::this_thread::sleep_for(1ms);
    } while (std::chrono::steady_clock::now() < quiesce_deadline);
    ASSERT_TRUE(epoch);
    EXPECT_FALSE(epoch->feedback_outstanding);
    EXPECT_GE(epoch->last_feedback_attempt, 1U);
    EXPECT_EQ(epoch->route_phase, ActiveFaultRoutePhase::kClear);
    EXPECT_EQ(epoch->terminal_phase, ActiveTerminalPhase::kEmpty);
  }

  auto overlapping = client->async_send_goal(goal);
  ASSERT_EQ(overlapping.wait_for(1s), std::future_status::ready);
  EXPECT_FALSE(overlapping.get());

  auto canceled = client->async_cancel_goal(goal_handle);
  ASSERT_EQ(canceled.wait_for(2s), std::future_status::ready);
  ASSERT_FALSE(canceled.get()->goals_canceling.empty());
  auto result_future = client->async_get_result(goal_handle);
  ASSERT_EQ(result_future.wait_for(5s), std::future_status::ready);
  const auto result = result_future.get();
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::CANCELED);
  ASSERT_TRUE(result.result);
  EXPECT_EQ(result.result->status, Action::Result::STATUS_CANCELED);
  EXPECT_EQ(world_state.release_count(), 1U);
  EXPECT_FALSE(world_state.reserved());
  auto first_retirement = wait_for_active_retirement_after_completed_pumps(
    coordinator, retirement_pump_budget, retirement_observation_timeout);
  // The result future is ready at ROS publication; acknowledgement, quiescence proof, receipt
  // consumption and lease-return retirement follow. Count completed leased pump transactions, not
  // elapsed timer periods.
  ASSERT_TRUE(first_retirement);
  EXPECT_FALSE(first_retirement->pending_binding);
  EXPECT_FALSE(first_retirement->active_binding);
  EXPECT_FALSE(first_retirement->active_epoch);

  std::promise<void> second_staging_promise;
  auto second_staging = second_staging_promise.get_future();
  std::once_flag second_staging_once;
  rclcpp_action::Client<Action>::SendGoalOptions second_options;
  second_options.feedback_callback = [&second_staging_promise, &second_staging_once](
    const auto, const auto feedback) {
    if (feedback->state == Action::Feedback::STATE_PLAN_PRE_GRASP) {
      std::call_once(
        second_staging_once,
        [&second_staging_promise]() {second_staging_promise.set_value();});
    }
  };
  auto second_sent = client->async_send_goal(goal, second_options);
  ASSERT_EQ(second_sent.wait_for(1s), std::future_status::ready);
  auto second_handle = second_sent.get();
  ASSERT_TRUE(second_handle);
  ASSERT_EQ(second_staging.wait_for(5s), std::future_status::ready);
  auto second_canceled = client->async_cancel_goal(second_handle);
  ASSERT_EQ(second_canceled.wait_for(2s), std::future_status::ready);
  ASSERT_FALSE(second_canceled.get()->goals_canceling.empty());
  auto second_result_future = client->async_get_result(second_handle);
  ASSERT_EQ(second_result_future.wait_for(5s), std::future_status::ready);
  const auto second_result = second_result_future.get();
  EXPECT_EQ(second_result.code, rclcpp_action::ResultCode::CANCELED);
  ASSERT_TRUE(second_result.result);
  EXPECT_EQ(second_result.result->status, Action::Result::STATUS_CANCELED);
  EXPECT_EQ(world_state.release_count(), 2U);
  EXPECT_FALSE(world_state.reserved());
  // The same retirement boundary as after the first cancel: until the second goal's binding
  // retires, the coordinator correctly refuses the next goal.
  auto second_retirement = wait_for_active_retirement_after_completed_pumps(
    coordinator, retirement_pump_budget, retirement_observation_timeout);
  ASSERT_TRUE(second_retirement);
  EXPECT_FALSE(second_retirement->pending_binding);
  EXPECT_FALSE(second_retirement->active_binding);
  EXPECT_FALSE(second_retirement->active_epoch);

  std::promise<void> shutdown_staging_promise;
  auto shutdown_staging = shutdown_staging_promise.get_future();
  std::once_flag shutdown_staging_once;
  rclcpp_action::Client<Action>::SendGoalOptions shutdown_options;
  shutdown_options.feedback_callback = [&shutdown_staging_promise, &shutdown_staging_once](
    const auto, const auto feedback) {
    if (feedback->state == Action::Feedback::STATE_PLAN_PRE_GRASP) {
      std::call_once(
        shutdown_staging_once,
        [&shutdown_staging_promise]() {shutdown_staging_promise.set_value();});
    }
  };
  auto shutdown_sent = client->async_send_goal(goal, shutdown_options);
  ASSERT_EQ(shutdown_sent.wait_for(1s), std::future_status::ready);
  auto shutdown_handle = shutdown_sent.get();
  ASSERT_TRUE(shutdown_handle);
  ASSERT_EQ(shutdown_staging.wait_for(5s), std::future_status::ready);
  coordinator->request_shutdown();
  coordinator->request_shutdown();
  EXPECT_EQ(coordinator->shutdown_status(), CoordinatorShutdownStatus::kDraining);
  auto shutdown_result_future = client->async_get_result(shutdown_handle);
  ASSERT_EQ(shutdown_result_future.wait_for(5s), std::future_status::ready);
  const auto shutdown_result = shutdown_result_future.get();
  EXPECT_EQ(shutdown_result.code, rclcpp_action::ResultCode::ABORTED);
  ASSERT_TRUE(shutdown_result.result);
  EXPECT_EQ(shutdown_result.result->status, Action::Result::STATUS_SHUTDOWN);
  EXPECT_EQ(world_state.release_count(), 3U);
  EXPECT_FALSE(world_state.reserved());

  const auto clean_deadline = std::chrono::steady_clock::now() + 2s;
  while (coordinator->shutdown_status() == CoordinatorShutdownStatus::kDraining &&
    std::chrono::steady_clock::now() < clean_deadline)
  {
    std::this_thread::sleep_for(5ms);
  }
  EXPECT_EQ(coordinator->shutdown_status(), CoordinatorShutdownStatus::kClean);
  const auto clean_snapshot = coordinator->shutdown_snapshot();
  EXPECT_EQ(clean_snapshot.status, CoordinatorShutdownStatus::kClean);
  EXPECT_TRUE(clean_snapshot.deadline);
  EXPECT_EQ(clean_snapshot.goal_generation, 0U);
  EXPECT_FALSE(clean_snapshot.reservation_capability_may_remain);
  const auto clean_authority = coordinator->generation_authority_snapshot();
  EXPECT_TRUE(clean_authority.diagnostic_caches_initialized);
  EXPECT_TRUE(clean_authority.diagnostic_caches_fresh);
  EXPECT_FALSE(clean_authority.pump_lease.outstanding);
  EXPECT_FALSE(clean_authority.pump_lease.fail_stopped);
  EXPECT_EQ(clean_authority.cached_inbox.size, 0U);
  EXPECT_EQ(clean_authority.cached_inbox.accepted_handoff_emergency_size, 0U);
  EXPECT_EQ(clean_authority.cached_inbox.cleanup_emergency_size, 0U);
  EXPECT_FALSE(clean_authority.cached_inbox.overflow_latched);
  EXPECT_FALSE(clean_authority.cached_inbox.overflow_notification_pending);
  EXPECT_FALSE(clean_authority.cached_inbox.cleanup_evidence_lost);
  EXPECT_FALSE(clean_authority.cached_inbox.generation_accounting_conflict);
  EXPECT_EQ(clean_authority.cached_admission.phase, GoalSlotPhase::kIdle);
  EXPECT_FALSE(clean_authority.cached_admission.inhibited);
  EXPECT_FALSE(clean_authority.cached_admission.mutation_submission_occupied);
  EXPECT_FALSE(clean_authority.cached_driver.active);
  EXPECT_FALSE(clean_authority.cached_driver.inhibited);
  EXPECT_FALSE(clean_authority.cached_driver.pending_operation_occupied);
  EXPECT_EQ(clean_authority.cached_driver.pending_transport_requests, 0U);
  EXPECT_FALSE(clean_authority.cached_driver.reservation_capability_may_remain);
  auto rejected_after_shutdown = client->async_send_goal(goal);
  ASSERT_EQ(rejected_after_shutdown.wait_for(1s), std::future_status::ready);
  EXPECT_FALSE(rejected_after_shutdown.get());
}

TEST_F(RestockActionCoordinatorNodeTest, TransformFaultReleasesAuthorityBeforeInhibition)
{
  auto support = std::make_shared<rclcpp::Node>("restock_transform_fault_test_support");
  WorldStateFixture world_state(support);
  tf2_ros::StaticTransformBroadcaster broadcaster(support);
  geometry_msgs::msg::TransformStamped shelf;
  shelf.header.stamp = support->now();
  shelf.header.frame_id = "world";
  shelf.child_frame_id = "shelf";
  shelf.transform.translation.y = 0.55;
  shelf.transform.translation.z = 0.75;
  shelf.transform.rotation.w = 1.0;
  broadcaster.sendTransform(shelf);
  send_grasp_center_transform(broadcaster, support->now());

  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {rclcpp::Parameter("product_catalog_path", RESTOCKER_TEST_PRODUCT_CATALOG),
      rclcpp::Parameter("workcell_geometry_path", RESTOCKER_TEST_WORKCELL_GEOMETRY),
      rclcpp::Parameter("gripper_geometry_path", RESTOCKER_TEST_GRIPPER_GEOMETRY),
      rclcpp::Parameter("world_state.get_snapshot_service", "/test_world/get_snapshot"),
      rclcpp::Parameter("world_state.reserve_task_service", "/test_world/reserve_task"),
      rclcpp::Parameter(
        "world_state.validate_reservation_service",
        "/test_world/validate_reservation"),
      rclcpp::Parameter(
        "world_state.release_reservation_service",
        "/test_world/release_reservation"),
      rclcpp::Parameter(
        "world_state.validate_execution_authority_service",
        "/test_world/validate_execution_authority"),
      rclcpp::Parameter("action_name", "/transform_fault_restock_product"),
      rclcpp::Parameter("status_topic", "/transform_fault_coordinator/status"),
      rclcpp::Parameter("pump_period_ms", 5),
      rclcpp::Parameter("selection.maximum_object_age_ms", 5000),
      rclcpp::Parameter("selection.lane_evidence_validity_ms", 5000)});
  auto coordinator = std::make_shared<RestockActionCoordinatorNode>(options);
  auto client = rclcpp_action::create_client<Action>(support, "/transform_fault_restock_product");
  std::mutex status_mutex;
  std::optional<restocker_interfaces::msg::RestockCoordinatorStatus> status;
  auto status_group = support->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  rclcpp::SubscriptionOptions status_options;
  status_options.callback_group = status_group;
  auto status_subscription =
    support->create_subscription<restocker_interfaces::msg::RestockCoordinatorStatus>(
    "/transform_fault_coordinator/status",
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
    [&status_mutex, &status](
      const restocker_interfaces::msg::RestockCoordinatorStatus::ConstSharedPtr message) {
      std::lock_guard lock(status_mutex);
      status = *message;
    },
    status_options);

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions{}, 4U);
  executor.add_node(support);
  executor.add_node(coordinator);
  ExecutorSpinGuard spin(executor);
  ASSERT_TRUE(client->wait_for_action_server(2s));

  std::promise<void> staging_promise;
  auto staging = staging_promise.get_future();
  std::once_flag staging_once;
  rclcpp_action::Client<Action>::SendGoalOptions send_options;
  send_options.feedback_callback = [&staging_promise, &staging_once](const auto,
    const auto feedback) {
    if (feedback->state == Action::Feedback::STATE_PLAN_PRE_GRASP) {
      std::call_once(staging_once, [&staging_promise]() {staging_promise.set_value();});
    }
  };
  Action::Goal goal;
  goal.has_object_id = true;
  goal.object_id = 17U;
  goal.has_lane_id = true;
  goal.lane_id = "lane_01";
  std::shared_ptr<rclcpp_action::ClientGoalHandle<Action>> goal_handle;
  for (std::size_t attempt = 0U; attempt < 20U && !goal_handle; ++attempt) {
    auto sent = client->async_send_goal(goal, send_options);
    ASSERT_EQ(sent.wait_for(500ms), std::future_status::ready);
    goal_handle = sent.get();
    if (!goal_handle) {
      std::this_thread::sleep_for(25ms);
    }
  }
  ASSERT_TRUE(goal_handle);
  ASSERT_EQ(staging.wait_for(5s), std::future_status::ready);
  ASSERT_TRUE(world_state.reserved());

  world_state.block_release();
  shelf.header.stamp = support->now();
  shelf.transform.translation.y += 0.01;
  broadcaster.sendTransform(shelf);

  const auto release_started_deadline = std::chrono::steady_clock::now() + 3s;
  while (world_state.release_count() == 0U &&
    std::chrono::steady_clock::now() < release_started_deadline)
  {
    std::this_thread::sleep_for(5ms);
  }
  const bool release_started = world_state.release_count() == 1U;
  bool cleanup_visible_without_inhibition = false;
  const auto status_deadline = std::chrono::steady_clock::now() + 1s;
  while (!cleanup_visible_without_inhibition &&
    std::chrono::steady_clock::now() < status_deadline)
  {
    {
      std::lock_guard lock(status_mutex);
      cleanup_visible_without_inhibition =
        status && !status->inhibited && !status->admission_ready &&
        status->detail.find("cleanup pending") != std::string::npos;
    }
    std::this_thread::sleep_for(5ms);
  }
  EXPECT_TRUE(release_started);
  EXPECT_TRUE(cleanup_visible_without_inhibition);
  const auto transform_authority = coordinator->generation_authority_snapshot();
  ASSERT_TRUE(transform_authority.active_epoch);
  EXPECT_EQ(transform_authority.active_epoch->route_phase, ActiveFaultRoutePhase::kSecured);
  EXPECT_FALSE(transform_authority.active_epoch->route_claim_outstanding);
  world_state.unblock_release();

  auto result_future = client->async_get_result(goal_handle);
  ASSERT_EQ(result_future.wait_for(5s), std::future_status::ready);
  const auto result = result_future.get();
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::ABORTED);
  ASSERT_TRUE(result.result);
  EXPECT_EQ(result.result->status, Action::Result::STATUS_EXTERNAL_INCONSISTENCY);
  EXPECT_EQ(world_state.release_count(), 1U);
  EXPECT_FALSE(world_state.reserved());

  bool inhibited = false;
  const auto inhibition_deadline = std::chrono::steady_clock::now() + 1s;
  while (!inhibited && std::chrono::steady_clock::now() < inhibition_deadline) {
    {
      std::lock_guard lock(status_mutex);
      inhibited = status && status->inhibited &&
        status->detail.find("shelf transform changed") != std::string::npos;
    }
    std::this_thread::sleep_for(5ms);
  }
  EXPECT_TRUE(inhibited);
  coordinator->request_shutdown();
  (void)status_subscription;
}

TEST_F(RestockActionCoordinatorNodeTest, TimesOutWithoutDiscardingReservationAuthority)
{
  auto support = std::make_shared<rclcpp::Node>("restock_action_shutdown_timeout_support");
  WorldStateFixture world_state(support);
  tf2_ros::StaticTransformBroadcaster broadcaster(support);
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = support->now();
  transform.header.frame_id = "world";
  transform.child_frame_id = "shelf";
  transform.transform.translation.y = 0.55;
  transform.transform.translation.z = 0.75;
  transform.transform.rotation.w = 1.0;
  broadcaster.sendTransform(transform);
  send_grasp_center_transform(broadcaster, support->now());

  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {rclcpp::Parameter("product_catalog_path", RESTOCKER_TEST_PRODUCT_CATALOG),
      rclcpp::Parameter("workcell_geometry_path", RESTOCKER_TEST_WORKCELL_GEOMETRY),
      rclcpp::Parameter("gripper_geometry_path", RESTOCKER_TEST_GRIPPER_GEOMETRY),
      rclcpp::Parameter("world_state.get_snapshot_service", "/test_world/get_snapshot"),
      rclcpp::Parameter("world_state.reserve_task_service", "/test_world/reserve_task"),
      rclcpp::Parameter(
        "world_state.validate_reservation_service",
        "/test_world/validate_reservation"),
      rclcpp::Parameter(
        "world_state.release_reservation_service",
        "/test_world/release_reservation"),
      rclcpp::Parameter(
        "world_state.validate_execution_authority_service",
        "/test_world/validate_execution_authority"),
      rclcpp::Parameter("action_name", "/test_restock_shutdown_timeout"),
      rclcpp::Parameter("pump_period_ms", 2), rclcpp::Parameter("task.validation_timeout_ms", 10),
      rclcpp::Parameter("reconciliation.window_ms", 20),
      rclcpp::Parameter("reconciliation.attempt_timeout_ms", 5),
      rclcpp::Parameter("reconciliation.max_attempts", 2),
      rclcpp::Parameter("shutdown_timeout_ms", 75),
      rclcpp::Parameter("selection.maximum_object_age_ms", 5000),
      rclcpp::Parameter("selection.lane_evidence_validity_ms", 5000)});
  auto coordinator = std::make_shared<RestockActionCoordinatorNode>(options);
  auto client = rclcpp_action::create_client<Action>(support, "/test_restock_shutdown_timeout");

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions{}, 4U);
  executor.add_node(support);
  executor.add_node(coordinator);
  ExecutorSpinGuard spin(executor);
  ASSERT_TRUE(client->wait_for_action_server(2s));

  std::promise<void> staging_promise;
  auto staging = staging_promise.get_future();
  std::once_flag staging_once;
  rclcpp_action::Client<Action>::SendGoalOptions send_options;
  send_options.feedback_callback = [&staging_promise, &staging_once](const auto,
    const auto feedback) {
    if (feedback->state == Action::Feedback::STATE_PLAN_PRE_GRASP) {
      std::call_once(staging_once, [&staging_promise]() {staging_promise.set_value();});
    }
  };

  Action::Goal goal;
  goal.has_object_id = true;
  goal.object_id = 17U;
  goal.has_lane_id = true;
  goal.lane_id = "lane_01";
  std::shared_ptr<rclcpp_action::ClientGoalHandle<Action>> goal_handle;
  for (std::size_t attempt = 0U; attempt < 20U && !goal_handle; ++attempt) {
    auto sent = client->async_send_goal(goal, send_options);
    ASSERT_EQ(sent.wait_for(500ms), std::future_status::ready);
    goal_handle = sent.get();
    if (!goal_handle) {
      std::this_thread::sleep_for(25ms);
    }
  }
  ASSERT_TRUE(goal_handle);
  ASSERT_EQ(staging.wait_for(5s), std::future_status::ready);
  ASSERT_TRUE(world_state.reserved());

  world_state.withdraw_release_service();
  coordinator->request_shutdown();
  const auto timeout_deadline = std::chrono::steady_clock::now() + 2s;
  while (coordinator->shutdown_status() == CoordinatorShutdownStatus::kDraining &&
    std::chrono::steady_clock::now() < timeout_deadline)
  {
    std::this_thread::sleep_for(2ms);
  }

  EXPECT_EQ(coordinator->shutdown_status(), CoordinatorShutdownStatus::kTimedOut);
  const auto timeout_snapshot = coordinator->shutdown_snapshot();
  EXPECT_EQ(timeout_snapshot.status, CoordinatorShutdownStatus::kTimedOut);
  EXPECT_TRUE(timeout_snapshot.deadline);
  EXPECT_GT(timeout_snapshot.goal_generation, 0U);
  EXPECT_TRUE(timeout_snapshot.reservation_capability_may_remain);
  EXPECT_TRUE(world_state.reserved());
  EXPECT_EQ(world_state.release_count(), 0U);
  const auto timeout_authority = coordinator->generation_authority_snapshot();
  ASSERT_TRUE(timeout_authority.active_binding);
  ASSERT_TRUE(timeout_authority.active_epoch);
  EXPECT_EQ(timeout_authority.active_epoch->route_phase, ActiveFaultRoutePhase::kUnsecured);
  EXPECT_FALSE(timeout_authority.active_epoch->route_claim_outstanding);
}

namespace
{
enum class CallbackFailureSeam {kAcceptedAdoption, kPendingHandoff};

void run_allocation_failure_receipt_case(
  CallbackFailureSeam seam, bool reporting_fails, bool second_callback_failure = false)
{
  auto support = std::make_shared<rclcpp::Node>("callback_failure_receipt_support");
  const std::string world_state_prefix{"/callback_failure_receipt_world"};
  WorldStateFixture world_state(support, "lane_01", world_state_prefix);
  tf2_ros::StaticTransformBroadcaster broadcaster(support);
  geometry_msgs::msg::TransformStamped shelf;
  shelf.header.stamp = support->now();
  shelf.header.frame_id = "world";
  shelf.child_frame_id = "shelf";
  shelf.transform.translation.y = 0.55;
  shelf.transform.translation.z = 0.75;
  shelf.transform.rotation.w = 1.0;
  broadcaster.sendTransform(shelf);
  send_grasp_center_transform(broadcaster, support->now());

  std::atomic<std::size_t> arm{kAllocationDisarmed};
  std::atomic<std::size_t> seam_entries{0U};
  std::atomic<bool> injected{false};
  std::atomic<bool> receipt_completed{false};
  std::atomic<std::size_t> failed_allocations{0U};
  std::atomic<bool> latched_during_outage{false};
  std::atomic<bool> observe_pump_samples{false};
  std::atomic<std::size_t> pump_samples{0U};
  std::atomic<std::size_t> receipts{0U};
  std::atomic<bool> second_injected{false};
  AllocationCallBlocker reporting_blocker;
  const auto inject = [&]() {
    ++seam_entries;
    if (!injected.exchange(true)) {
      engage_allocation_failure(
        arm, reporting_fails ? std::numeric_limits<std::size_t>::max() : 1U);
      if (reporting_fails) {
        allocation_call_blocker = &reporting_blocker;
      }
      // An actual operator-new failure at the callback seam; no direct throw surrogate.
      // The returned storage is freed if injection is broken, so the assertions can diagnose it.
      auto * storage = ::operator new(64U);
      ::operator delete(storage);
    }
  };
  const std::string status_topic{"/callback_failure_receipt/status"};
  AdmissionReadyObserver readiness(support, status_topic);
  auto options =
    ready_coordinator_options("/callback_failure_receipt_restock_product", world_state_prefix);
  options.append_parameter_override("status_topic", status_topic);
  options.append_parameter_override("executor_threads", 3);
  std::shared_ptr<RestockActionCoordinatorNode> coordinator;
  RestockActionCoordinatorNodeDependencies dependencies;
  dependencies.steady_now = [&]() {
    if (observe_pump_samples.load() && allocation_failure_thrown == 0U) {
      ++pump_samples;
    }
    return std::chrono::steady_clock::now();
  };
  if (seam == CallbackFailureSeam::kAcceptedAdoption) {
    dependencies.before_accepted_goal_adoption = inject;
  } else {
    dependencies.before_pending_handoff = inject;
  }
  if (second_callback_failure) {
    dependencies.before_pending_handoff = [&]() {
      if (!second_injected.exchange(true)) {
        failed_allocations.fetch_add(allocation_failure_thrown);
        engage_allocation_failure(arm, 1U);
        auto * storage = ::operator new(64U);
        ::operator delete(storage);
      }
    };
  }
  dependencies.after_callback_failure = [&]() {
    const auto receipt = coordinator->generation_authority_snapshot();
    latched_during_outage.store(receipt.callback_failure_latched);
    failed_allocations.fetch_add(allocation_failure_thrown);
    arm.store(kAllocationDisarmed, std::memory_order_release);
    reset_allocation_injection();
    if (++receipts == (second_callback_failure ? 2U : 1U)) {
      receipt_completed.store(true, std::memory_order_release);
    }
  };
  coordinator = std::make_shared<RestockActionCoordinatorNode>(options, std::move(dependencies));
  auto client =
    rclcpp_action::create_client<Action>(support, "/callback_failure_receipt_restock_product");
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions{}, 3U);
  executor.add_node(support);
  executor.add_node(coordinator);
  ExecutorSpinGuard spin(executor);
  ASSERT_TRUE(client->wait_for_action_server(2s));
  ASSERT_TRUE(readiness.wait_until_ready(7s));
  ASSERT_TRUE(wait_for_fresh_returned_pump_lease(coordinator, 5s).has_value());
  const auto initial_snapshots = world_state.snapshot_count();

  Action::Goal goal;
  goal.has_object_id = true;
  goal.object_id = 17U;
  goal.has_lane_id = true;
  goal.lane_id = "lane_01";
  auto sent = client->async_send_goal(goal);
  ASSERT_EQ(sent.wait_for(4s), std::future_status::ready);
  const auto handle = sent.get();
  ASSERT_TRUE(handle) << "injection must occur after admission, at the accepted callback seam";
  auto result = client->async_get_result(handle);
  if (reporting_fails) {
    ASSERT_TRUE(reporting_blocker.wait_until_blocked(2s));
    const auto before_report = coordinator->generation_authority_snapshot();
    EXPECT_TRUE(before_report.callback_failure_latched);
    EXPECT_TRUE(before_report.pending_accepted_handle_identity);
    EXPECT_FALSE(before_report.active_binding);
    EXPECT_EQ(seam_entries.load(), 1U);
    // Observe actual timer clock samples while Half B is parked, before its allocating route
    // sets other flags. This proves the latch against serviced pumps, not a scheduling delay.
    observe_pump_samples.store(true);
    const auto pump_deadline = std::chrono::steady_clock::now() + 2s;
    while (pump_samples.load() < 3U && std::chrono::steady_clock::now() < pump_deadline) {
      std::this_thread::sleep_for(2ms);
    }
    EXPECT_GE(pump_samples.load(), 3U);
    observe_pump_samples.store(false);
    const auto still_parked = coordinator->generation_authority_snapshot();
    EXPECT_TRUE(still_parked.pending_binding);
    EXPECT_EQ(
      still_parked.pending_accepted_handle_identity,
      before_report.pending_accepted_handle_identity);
    EXPECT_FALSE(still_parked.active_binding);
    EXPECT_EQ(seam_entries.load(), 1U);
    EXPECT_EQ(world_state.snapshot_count(), initial_snapshots);
    EXPECT_EQ(world_state.reserve_count(), 0U);
    reporting_blocker.release();
  }
  const auto receipt_deadline = std::chrono::steady_clock::now() + 3s;
  while (!receipt_completed.load(std::memory_order_acquire) &&
    std::chrono::steady_clock::now() < receipt_deadline)
  {
    std::this_thread::sleep_for(2ms);
  }
  ASSERT_TRUE(receipt_completed.load(std::memory_order_acquire));
  EXPECT_FALSE(reporting_blocker.timed_out());
  EXPECT_TRUE(latched_during_outage.load());
  EXPECT_EQ(injected.load(), true);
  const auto observed = coordinator->generation_authority_snapshot();
  EXPECT_TRUE(observed.callback_failure_latched);
  ASSERT_NE(observed.callback_failure_detail, nullptr);
  EXPECT_STREQ(
    observed.callback_failure_detail,
    seam == CallbackFailureSeam::kAcceptedAdoption ?
    "accepted-goal adoption callback failed" : "pending accepted-goal handoff failed");
  if (reporting_fails || second_callback_failure) {
    EXPECT_GE(failed_allocations.load(), 2U);
    EXPECT_EQ(second_injected.load(), second_callback_failure);
    EXPECT_EQ(result.wait_for(100ms), std::future_status::timeout);
    EXPECT_TRUE(observed.pending_accepted_handle_identity);
    EXPECT_FALSE(observed.active_binding);
  } else {
    EXPECT_EQ(failed_allocations.load(), 1U);
    ASSERT_EQ(result.wait_for(4s), std::future_status::ready);
    EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::ABORTED);
    EXPECT_EQ(result.get().result->status, Action::Result::STATUS_EXTERNAL_INCONSISTENCY);
  }
  auto refused = client->async_send_goal(goal);
  ASSERT_EQ(refused.wait_for(4s), std::future_status::ready);
  EXPECT_FALSE(refused.get()) << "a latched callback failure must refuse later admission";
  std::this_thread::sleep_for(100ms);
  EXPECT_EQ(world_state.snapshot_count(), initial_snapshots);
  EXPECT_EQ(world_state.reserve_count(), 0U);
  EXPECT_EQ(world_state.validate_count(), 0U);
  const auto persistent = coordinator->generation_authority_snapshot();
  EXPECT_TRUE(persistent.callback_failure_latched);
  EXPECT_EQ(persistent.callback_failure_detail, observed.callback_failure_detail);
  std::fprintf(
    stderr, "callback receipt: seam=%s failures=%zu latch=1 terminal=%s\n",
    seam == CallbackFailureSeam::kAcceptedAdoption ? "adoption" : "handoff",
    failed_allocations.load(),
    (reporting_fails || second_callback_failure) ? "withheld" : "aborted");
  coordinator->request_shutdown();
  EXPECT_NE(coordinator->shutdown_status(), CoordinatorShutdownStatus::kClean);
}
}  // namespace

TEST_F(RestockActionCoordinatorNodeTest, AllocationFailureInAcceptedAdoptionFailsClosed)
{
  run_allocation_failure_receipt_case(CallbackFailureSeam::kAcceptedAdoption, false);
}

TEST_F(RestockActionCoordinatorNodeTest, AllocationFailureInPendingHandoffFailsClosed)
{
  run_allocation_failure_receipt_case(CallbackFailureSeam::kPendingHandoff, false);
}

TEST_F(RestockActionCoordinatorNodeTest, AcceptedAdoptionReceiptSurvivesAllocationFailure)
{
  run_allocation_failure_receipt_case(CallbackFailureSeam::kAcceptedAdoption, true);
}

TEST_F(RestockActionCoordinatorNodeTest, PendingHandoffReceiptSurvivesAllocationFailure)
{
  run_allocation_failure_receipt_case(CallbackFailureSeam::kPendingHandoff, true);
}

TEST_F(RestockActionCoordinatorNodeTest, SecondCallbackFailureRevokesFaultCleanup)
{
  run_allocation_failure_receipt_case(CallbackFailureSeam::kAcceptedAdoption, false, true);
}

}  // namespace
}  // namespace restocker_task_executor
