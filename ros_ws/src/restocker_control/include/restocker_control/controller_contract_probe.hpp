// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "restocker_control/control_contract.hpp"

namespace restocker_control
{

using ControllerProbeDuration = std::chrono::nanoseconds;
using ControllerProbeTime = std::chrono::steady_clock::time_point;
using ControllerProbeNow = std::function<ControllerProbeTime ()>;
using ControllerRequestId = std::int64_t;

struct ControllerProbeConfig
{
  ControllerProbeDuration startup_timeout{};
  ControllerProbeDuration response_attempt_timeout{};
  ControllerProbeDuration contract_retry_backoff{std::chrono::milliseconds(100)};
};

enum class ControllerProbeConfigStatus : std::uint8_t
{
  kValid,
  kInvalidStartupTimeout,
  kInvalidResponseAttemptTimeout,
  kAttemptExceedsStartupTimeout,
};

struct ControllerProbeConfigResult
{
  ControllerProbeConfigStatus status{ControllerProbeConfigStatus::kInvalidStartupTimeout};
  ControllerProbeConfig config;
  std::string detail;

  [[nodiscard]] bool valid() const noexcept
  {
    return status == ControllerProbeConfigStatus::kValid;
  }
};

[[nodiscard]] ControllerProbeConfigResult make_controller_probe_config(
  double startup_timeout_sec, double response_attempt_timeout_sec);

enum class ControllerServiceWaitStatus : std::uint8_t
{
  kAvailable,
  kTimeout,
  kInterrupted,
};

enum class ControllerResponseWaitStatus : std::uint8_t
{
  kResponse,
  kTimeout,
  kInterrupted,
};

struct ControllerResponseWaitResult
{
  ControllerResponseWaitStatus status{ControllerResponseWaitStatus::kTimeout};
  std::vector<ControllerObservation> observations;
};

enum class ControllerRequestRemovalStatus : std::uint8_t
{
  kRemoved,
  kAlreadyAbsent,
};

enum class ControllerBackoffStatus : std::uint8_t
{
  kElapsed,
  kInterrupted,
};

class ControllerProbeTransport
{
public:
  virtual ~ControllerProbeTransport() = default;

  [[nodiscard]] virtual ControllerServiceWaitStatus wait_for_service(
    ControllerProbeDuration timeout) = 0;
  [[nodiscard]] virtual ControllerRequestId send_request() = 0;
  [[nodiscard]] virtual ControllerResponseWaitResult wait_for_response(
    ControllerRequestId request_id, ControllerProbeDuration timeout) = 0;
  [[nodiscard]] virtual ControllerRequestRemovalStatus remove_pending_request(
    ControllerRequestId request_id) = 0;
  [[nodiscard]] virtual ControllerBackoffStatus wait_for_retry(
    ControllerProbeDuration timeout) = 0;
};

enum class ControllerProbeStatus : std::uint8_t
{
  kHealthy,
  kServiceTimeout,
  kResponseTimeout,
  kContractFailure,
  kInterrupted,
  kTransportFailure,
  kInvalidConfiguration,
};

struct ControllerProbeResult
{
  ControllerProbeStatus status{ControllerProbeStatus::kInvalidConfiguration};
  std::vector<std::string> contract_errors;
  std::size_t request_attempts{0U};
  std::size_t timed_out_requests{0U};
  std::size_t cleanup_contentions{0U};
};

class ControllerContractProbe final
{
public:
  ControllerContractProbe(ControllerProbeConfig config, ControllerProbeNow now);

  [[nodiscard]] ControllerProbeResult run(ControllerProbeTransport & transport) const;

private:
  ControllerProbeConfig config_;
  ControllerProbeNow now_;
};

}  // namespace restocker_control
