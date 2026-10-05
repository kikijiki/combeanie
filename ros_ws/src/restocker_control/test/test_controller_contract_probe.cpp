// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "restocker_control/controller_contract_probe.hpp"

namespace restocker_control
{
namespace
{

using namespace std::chrono_literals;

std::vector<ControllerObservation> healthy_controllers()
{
  return {
    {"joint_state_broadcaster", "joint_state_broadcaster/JointStateBroadcaster", "active", {}},
    {"arm_controller",
      "joint_trajectory_controller/JointTrajectoryController",
      "active",
      {"shoulder_pan_joint/velocity", "shoulder_lift_joint/velocity", "elbow_joint/velocity",
        "wrist_1_joint/velocity",
        "wrist_2_joint/velocity", "wrist_3_joint/velocity"}},
    {"rail_controller",
      "joint_trajectory_controller/JointTrajectoryController",
      "active",
      {"rail_joint/velocity"}},
    {"gripper_controller",
      "joint_trajectory_controller/JointTrajectoryController",
      "active",
      {"left_finger_joint/position", "right_finger_joint/position"}},
  };
}

struct ScriptedResponse
{
  ControllerResponseWaitStatus status{ControllerResponseWaitStatus::kTimeout};
  std::vector<ControllerObservation> observations;
  bool throw_on_wait{false};
};

class FakeTransport final : public ControllerProbeTransport
{
public:
  explicit FakeTransport(ControllerProbeTime & now)
  : now_(now) {}

  ControllerServiceWaitStatus wait_for_service(ControllerProbeDuration timeout) override
  {
    if (throw_on_service_wait) {
      throw std::runtime_error("injected service wait failure");
    }
    service_waits.push_back(timeout);
    if (service_status != ControllerServiceWaitStatus::kAvailable) {
      now_ += timeout;
    }
    return service_status;
  }

  ControllerRequestId send_request() override
  {
    if (throw_on_send) {
      throw std::runtime_error("injected request send failure");
    }
    events.push_back("send:" + std::to_string(next_request_id));
    now_ += send_advance;
    return next_request_id++;
  }

  ControllerResponseWaitResult wait_for_response(
    ControllerRequestId request_id, ControllerProbeDuration timeout) override
  {
    events.push_back("wait:" + std::to_string(request_id));
    response_waits.push_back(timeout);
    if (responses.empty()) {
      now_ += timeout;
      return {};
    }
    auto response = std::move(responses.front());
    responses.pop_front();
    if (response.throw_on_wait) {
      throw std::runtime_error("injected response wait failure");
    }
    if (response.status == ControllerResponseWaitStatus::kTimeout) {
      now_ += timeout;
    }
    return {response.status, std::move(response.observations)};
  }

  ControllerRequestRemovalStatus remove_pending_request(
    ControllerRequestId request_id) override
  {
    events.push_back("remove:" + std::to_string(request_id));
    removed.push_back(request_id);
    if (throw_on_remove) {
      throw std::runtime_error("injected removal failure");
    }
    return removal_status;
  }

  ControllerBackoffStatus wait_for_retry(ControllerProbeDuration timeout) override
  {
    if (throw_on_backoff) {
      throw std::runtime_error("injected backoff failure");
    }
    backoffs.push_back(timeout);
    if (backoff_status == ControllerBackoffStatus::kElapsed) {
      now_ += timeout;
    }
    return backoff_status;
  }

  ControllerProbeTime & now_;
  ControllerServiceWaitStatus service_status{ControllerServiceWaitStatus::kAvailable};
  ControllerRequestRemovalStatus removal_status{ControllerRequestRemovalStatus::kRemoved};
  ControllerBackoffStatus backoff_status{ControllerBackoffStatus::kElapsed};
  bool throw_on_remove{false};
  bool throw_on_service_wait{false};
  bool throw_on_send{false};
  bool throw_on_backoff{false};
  ControllerProbeDuration send_advance{};
  ControllerRequestId next_request_id{1};
  std::deque<ScriptedResponse> responses;
  std::vector<ControllerProbeDuration> service_waits;
  std::vector<ControllerProbeDuration> response_waits;
  std::vector<ControllerProbeDuration> backoffs;
  std::vector<ControllerRequestId> removed;
  std::vector<std::string> events;
};

ControllerProbeConfig config(
  ControllerProbeDuration startup = 500ms,
  ControllerProbeDuration attempt = 100ms)
{
  return {startup, attempt, 100ms};
}

TEST(ControllerContractProbeConfig, ValidatesEveryDurationBoundary)
{
  EXPECT_TRUE(make_controller_probe_config(1.0, 1.0).valid());
  EXPECT_EQ(
    make_controller_probe_config(0.0, 0.5).status,
    ControllerProbeConfigStatus::kInvalidStartupTimeout);
  EXPECT_EQ(
    make_controller_probe_config(-1.0, 0.5).status,
    ControllerProbeConfigStatus::kInvalidStartupTimeout);
  EXPECT_EQ(
    make_controller_probe_config(std::numeric_limits<double>::quiet_NaN(), 0.5).status,
    ControllerProbeConfigStatus::kInvalidStartupTimeout);
  EXPECT_EQ(
    make_controller_probe_config(std::numeric_limits<double>::infinity(), 0.5).status,
    ControllerProbeConfigStatus::kInvalidStartupTimeout);
  EXPECT_EQ(
    make_controller_probe_config(std::numeric_limits<double>::max(), 0.5).status,
    ControllerProbeConfigStatus::kInvalidStartupTimeout);
  EXPECT_EQ(
    make_controller_probe_config(1.0, 0.0).status,
    ControllerProbeConfigStatus::kInvalidResponseAttemptTimeout);
  EXPECT_EQ(
    make_controller_probe_config(1.0, -1.0).status,
    ControllerProbeConfigStatus::kInvalidResponseAttemptTimeout);
  EXPECT_EQ(
    make_controller_probe_config(1.0, std::numeric_limits<double>::quiet_NaN()).status,
    ControllerProbeConfigStatus::kInvalidResponseAttemptTimeout);
  EXPECT_EQ(
    make_controller_probe_config(1.0, std::numeric_limits<double>::infinity()).status,
    ControllerProbeConfigStatus::kInvalidResponseAttemptTimeout);
  EXPECT_EQ(
    make_controller_probe_config(1.0, 2.0).status,
    ControllerProbeConfigStatus::kAttemptExceedsStartupTimeout);

  const double rounded_maximum =
    std::chrono::duration<double>(ControllerProbeDuration::max()).count();
  EXPECT_EQ(
    make_controller_probe_config(rounded_maximum, 1.0).status,
    ControllerProbeConfigStatus::kInvalidStartupTimeout);
  EXPECT_TRUE(
    make_controller_probe_config(std::nextafter(rounded_maximum, 0.0), 1.0).valid());
  EXPECT_EQ(
    make_controller_probe_config(std::nextafter(rounded_maximum, INFINITY), 1.0).status,
    ControllerProbeConfigStatus::kInvalidStartupTimeout);
}

TEST(ControllerContractProbe, AcceptsImmediateHealthyResponse)
{
  ControllerProbeTime now{};
  FakeTransport transport(now);
  transport.responses.push_back(
    {ControllerResponseWaitStatus::kResponse, healthy_controllers(), false});
  const ControllerContractProbe probe(config(), [&now]() {return now;});
  const auto result = probe.run(transport);
  EXPECT_EQ(result.status, ControllerProbeStatus::kHealthy);
  EXPECT_EQ(result.request_attempts, 1U);
  EXPECT_TRUE(result.contract_errors.empty());
}

TEST(ControllerContractProbe, RetiresTimedOutIdentityBeforeSuccessfulRetry)
{
  ControllerProbeTime now{};
  FakeTransport transport(now);
  transport.responses.push_back({ControllerResponseWaitStatus::kTimeout, {}, false});
  transport.responses.push_back(
    {ControllerResponseWaitStatus::kResponse, healthy_controllers(), false});
  const ControllerContractProbe probe(config(), [&now]() {return now;});
  const auto result = probe.run(transport);
  EXPECT_EQ(result.status, ControllerProbeStatus::kHealthy);
  EXPECT_EQ(result.request_attempts, 2U);
  EXPECT_EQ(transport.removed, std::vector<ControllerRequestId>{1});
  EXPECT_EQ(
    transport.events,
    (std::vector<std::string>{"send:1", "wait:1", "remove:1", "send:2", "wait:2"}));
}

TEST(ControllerContractProbe, ExhaustsResponseDeadlineWithClampedFinalAttempt)
{
  ControllerProbeTime now{};
  FakeTransport transport(now);
  const ControllerContractProbe probe(config(250ms, 100ms), [&now]() {return now;});
  const auto result = probe.run(transport);
  EXPECT_EQ(result.status, ControllerProbeStatus::kResponseTimeout);
  EXPECT_EQ(result.request_attempts, 3U);
  ASSERT_EQ(transport.response_waits.size(), 3U);
  EXPECT_EQ(transport.response_waits.back(), 50ms);
  EXPECT_EQ(result.timed_out_requests, 3U);
}

TEST(ControllerContractProbe, RetiresRequestWhenSendingExhaustsDeadline)
{
  ControllerProbeTime now{};
  FakeTransport transport(now);
  transport.send_advance = 100ms;
  const ControllerContractProbe probe(config(100ms, 100ms), [&now]() {return now;});
  const auto result = probe.run(transport);
  EXPECT_EQ(result.status, ControllerProbeStatus::kResponseTimeout);
  EXPECT_EQ(result.request_attempts, 1U);
  EXPECT_EQ(transport.removed, std::vector<ControllerRequestId>{1});
  EXPECT_TRUE(transport.response_waits.empty());
}

TEST(ControllerContractProbe, RetainsLatestContractErrorsAcrossLaterTimeouts)
{
  ControllerProbeTime now{};
  FakeTransport transport(now);
  transport.responses.push_back(
    {ControllerResponseWaitStatus::kResponse, {}, false});
  const ControllerContractProbe probe(config(250ms, 100ms), [&now]() {return now;});
  const auto result = probe.run(transport);
  EXPECT_EQ(result.status, ControllerProbeStatus::kContractFailure);
  EXPECT_FALSE(result.contract_errors.empty());
  EXPECT_GT(result.timed_out_requests, 0U);
}

TEST(ControllerContractProbe, ClampsContractBackoffToDeadline)
{
  ControllerProbeTime now{};
  FakeTransport transport(now);
  transport.responses.push_back(
    {ControllerResponseWaitStatus::kResponse, {}, false});
  const ControllerContractProbe probe(config(50ms, 50ms), [&now]() {return now;});
  const auto result = probe.run(transport);
  EXPECT_EQ(result.status, ControllerProbeStatus::kContractFailure);
  ASSERT_EQ(transport.backoffs.size(), 1U);
  EXPECT_EQ(transport.backoffs.front(), 50ms);
  EXPECT_EQ(result.request_attempts, 1U);
}

TEST(ControllerContractProbe, DistinguishesDiscoveryAndResponseInterruption)
{
  ControllerProbeTime now{};
  FakeTransport discovery_transport(now);
  discovery_transport.service_status = ControllerServiceWaitStatus::kInterrupted;
  const ControllerContractProbe probe(config(), [&now]() {return now;});
  EXPECT_EQ(probe.run(discovery_transport).status, ControllerProbeStatus::kInterrupted);

  now = ControllerProbeTime{};
  FakeTransport response_transport(now);
  response_transport.responses.push_back(
    {ControllerResponseWaitStatus::kInterrupted, {}, false});
  const auto response_result = probe.run(response_transport);
  EXPECT_EQ(response_result.status, ControllerProbeStatus::kInterrupted);
  EXPECT_EQ(response_transport.removed, std::vector<ControllerRequestId>{1});
  EXPECT_EQ(response_result.request_attempts, 1U);
}

TEST(ControllerContractProbe, BackoffInterruptionDoesNotSendAnotherRequest)
{
  ControllerProbeTime now{};
  FakeTransport transport(now);
  transport.responses.push_back(
    {ControllerResponseWaitStatus::kResponse, {}, false});
  transport.backoff_status = ControllerBackoffStatus::kInterrupted;
  const ControllerContractProbe probe(config(), [&now]() {return now;});
  const auto result = probe.run(transport);
  EXPECT_EQ(result.status, ControllerProbeStatus::kInterrupted);
  EXPECT_EQ(result.request_attempts, 1U);
}

TEST(ControllerContractProbe, WaitOrResponseReadExceptionAttemptsExactCleanup)
{
  ControllerProbeTime now{};
  FakeTransport transport(now);
  transport.responses.push_back({ControllerResponseWaitStatus::kTimeout, {}, true});
  const ControllerContractProbe probe(config(), [&now]() {return now;});
  const auto result = probe.run(transport);
  EXPECT_EQ(result.status, ControllerProbeStatus::kTransportFailure);
  EXPECT_EQ(transport.removed, std::vector<ControllerRequestId>{1});

  now = ControllerProbeTime{};
  FakeTransport cleanup_failure(now);
  cleanup_failure.responses.push_back({ControllerResponseWaitStatus::kTimeout, {}, true});
  cleanup_failure.throw_on_remove = true;
  EXPECT_EQ(probe.run(cleanup_failure).status, ControllerProbeStatus::kTransportFailure);
}

TEST(ControllerContractProbe, MapsEveryOtherTransportException)
{
  ControllerProbeTime now{};
  const ControllerContractProbe probe(config(), [&now]() {return now;});

  FakeTransport discovery_failure(now);
  discovery_failure.throw_on_service_wait = true;
  EXPECT_EQ(
    probe.run(discovery_failure).status, ControllerProbeStatus::kTransportFailure);

  FakeTransport send_failure(now);
  send_failure.throw_on_send = true;
  EXPECT_EQ(probe.run(send_failure).status, ControllerProbeStatus::kTransportFailure);

  FakeTransport backoff_failure(now);
  backoff_failure.responses.push_back(
    {ControllerResponseWaitStatus::kResponse, {}, false});
  backoff_failure.throw_on_backoff = true;
  EXPECT_EQ(probe.run(backoff_failure).status, ControllerProbeStatus::kTransportFailure);
}

TEST(ControllerContractProbe, ReportsCleanupContentionWithoutAliasingAttempts)
{
  ControllerProbeTime now{};
  FakeTransport transport(now);
  transport.removal_status = ControllerRequestRemovalStatus::kAlreadyAbsent;
  transport.responses.push_back({ControllerResponseWaitStatus::kTimeout, {}, false});
  transport.responses.push_back(
    {ControllerResponseWaitStatus::kResponse, healthy_controllers(), false});
  const ControllerContractProbe probe(config(), [&now]() {return now;});
  const auto result = probe.run(transport);
  EXPECT_EQ(result.status, ControllerProbeStatus::kHealthy);
  EXPECT_EQ(result.cleanup_contentions, 1U);
  EXPECT_EQ(result.request_attempts, 2U);
}

TEST(ControllerContractProbe, RejectsInvalidRuntimeConfiguration)
{
  ControllerProbeTime now{};
  FakeTransport transport(now);
  const ControllerContractProbe probe({}, [&now]() {return now;});
  EXPECT_EQ(probe.run(transport).status, ControllerProbeStatus::kInvalidConfiguration);
  EXPECT_TRUE(transport.service_waits.empty());
}

}  // namespace
}  // namespace restocker_control
