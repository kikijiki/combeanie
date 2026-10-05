// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_control/controller_contract_probe.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <utility>

namespace restocker_control
{
namespace
{

[[nodiscard]] std::optional<ControllerProbeDuration> duration_from_seconds(double seconds)
{
  if (!std::isfinite(seconds) || seconds <= 0.0) {
    return std::nullopt;
  }
  constexpr long double kNanosecondsPerSecond = 1'000'000'000.0L;
  const long double ticks = static_cast<long double>(seconds) * kNanosecondsPerSecond;
  const long double maximum_ticks =
    static_cast<long double>(
    ControllerProbeDuration::rep{std::numeric_limits<std::int64_t>::max()});
  if (ticks > maximum_ticks) {
    return std::nullopt;
  }
  const auto duration = ControllerProbeDuration{static_cast<ControllerProbeDuration::rep>(ticks)};
  if (duration <= ControllerProbeDuration::zero()) {
    return std::nullopt;
  }
  return duration;
}

[[nodiscard]] bool remove_after_incomplete_wait(
  ControllerProbeTransport & transport, ControllerRequestId request_id,
  ControllerProbeResult & result) noexcept
{
  try {
    if (transport.remove_pending_request(request_id) ==
      ControllerRequestRemovalStatus::kAlreadyAbsent)
    {
      ++result.cleanup_contentions;
    }
    return true;
  } catch (...) {
    result.status = ControllerProbeStatus::kTransportFailure;
    return false;
  }
}

}  // namespace

ControllerProbeConfigResult make_controller_probe_config(
  double startup_timeout_sec, double response_attempt_timeout_sec)
{
  ControllerProbeConfigResult result;
  const auto startup_timeout = duration_from_seconds(startup_timeout_sec);
  if (!startup_timeout) {
    result.status = ControllerProbeConfigStatus::kInvalidStartupTimeout;
    result.detail = "startup_timeout_sec must be finite, positive, and chrono-representable";
    return result;
  }
  const auto response_timeout = duration_from_seconds(response_attempt_timeout_sec);
  if (!response_timeout) {
    result.status = ControllerProbeConfigStatus::kInvalidResponseAttemptTimeout;
    result.detail =
      "controller_response_attempt_timeout_sec must be finite, positive, and chrono-representable";
    return result;
  }
  if (*response_timeout > *startup_timeout) {
    result.status = ControllerProbeConfigStatus::kAttemptExceedsStartupTimeout;
    result.detail = "controller response attempt timeout must not exceed startup timeout";
    return result;
  }
  result.status = ControllerProbeConfigStatus::kValid;
  result.config.startup_timeout = *startup_timeout;
  result.config.response_attempt_timeout = *response_timeout;
  result.detail = "controller probe timeout configuration is valid";
  return result;
}

ControllerContractProbe::ControllerContractProbe(
  ControllerProbeConfig config, ControllerProbeNow now)
: config_(config), now_(std::move(now))
{
}

ControllerProbeResult ControllerContractProbe::run(ControllerProbeTransport & transport) const
{
  ControllerProbeResult result;
  if (!now_ || config_.startup_timeout <= ControllerProbeDuration::zero() ||
    config_.response_attempt_timeout <= ControllerProbeDuration::zero() ||
    config_.response_attempt_timeout > config_.startup_timeout ||
    config_.contract_retry_backoff < ControllerProbeDuration::zero())
  {
    return result;
  }

  ControllerProbeTime deadline;
  try {
    const auto started = now_();
    const auto clock_timeout =
      std::chrono::duration_cast<ControllerProbeTime::duration>(config_.startup_timeout);
    if (clock_timeout <= ControllerProbeTime::duration::zero() ||
      clock_timeout > ControllerProbeTime::max() - started)
    {
      return result;
    }
    deadline = started + clock_timeout;
    const auto discovery = transport.wait_for_service(config_.startup_timeout);
    if (discovery == ControllerServiceWaitStatus::kInterrupted) {
      result.status = ControllerProbeStatus::kInterrupted;
      return result;
    }
    if (discovery == ControllerServiceWaitStatus::kTimeout) {
      result.status = ControllerProbeStatus::kServiceTimeout;
      return result;
    }
  } catch (...) {
    result.status = ControllerProbeStatus::kTransportFailure;
    return result;
  }

  bool received_response = false;
  while (true) {
    try {
      const auto current = now_();
      if (current >= deadline) {
        result.status = received_response ? ControllerProbeStatus::kContractFailure :
          ControllerProbeStatus::kResponseTimeout;
        return result;
      }
    } catch (...) {
      result.status = ControllerProbeStatus::kTransportFailure;
      return result;
    }

    ControllerRequestId request_id{};
    try {
      request_id = transport.send_request();
      ++result.request_attempts;
    } catch (...) {
      result.status = ControllerProbeStatus::kTransportFailure;
      return result;
    }

    ControllerProbeDuration remaining;
    try {
      const auto current = now_();
      if (current >= deadline) {
        if (!remove_after_incomplete_wait(transport, request_id, result)) {
          return result;
        }
        result.status = received_response ? ControllerProbeStatus::kContractFailure :
          ControllerProbeStatus::kResponseTimeout;
        return result;
      }
      remaining = std::chrono::duration_cast<ControllerProbeDuration>(deadline - current);
    } catch (...) {
      (void)remove_after_incomplete_wait(transport, request_id, result);
      result.status = ControllerProbeStatus::kTransportFailure;
      return result;
    }

    ControllerResponseWaitResult response;
    try {
      response = transport.wait_for_response(
        request_id, std::min(config_.response_attempt_timeout, remaining));
    } catch (...) {
      (void)remove_after_incomplete_wait(transport, request_id, result);
      result.status = ControllerProbeStatus::kTransportFailure;
      return result;
    }

    if (response.status != ControllerResponseWaitStatus::kResponse) {
      if (!remove_after_incomplete_wait(transport, request_id, result)) {
        return result;
      }
      if (response.status == ControllerResponseWaitStatus::kInterrupted) {
        result.status = ControllerProbeStatus::kInterrupted;
        return result;
      }
      ++result.timed_out_requests;
      continue;
    }

    received_response = true;
    try {
      result.contract_errors = validate_controller_contract(response.observations);
    } catch (...) {
      result.status = ControllerProbeStatus::kTransportFailure;
      return result;
    }
    if (result.contract_errors.empty()) {
      result.status = ControllerProbeStatus::kHealthy;
      return result;
    }

    ControllerProbeDuration backoff;
    try {
      const auto current = now_();
      if (current >= deadline) {
        result.status = ControllerProbeStatus::kContractFailure;
        return result;
      }
      backoff = std::min(
        config_.contract_retry_backoff,
        std::chrono::duration_cast<ControllerProbeDuration>(deadline - current));
      if (backoff == ControllerProbeDuration::zero()) {
        result.status = ControllerProbeStatus::kContractFailure;
        return result;
      }
      if (transport.wait_for_retry(backoff) == ControllerBackoffStatus::kInterrupted) {
        result.status = ControllerProbeStatus::kInterrupted;
        return result;
      }
    } catch (...) {
      result.status = ControllerProbeStatus::kTransportFailure;
      return result;
    }
  }
}

}  // namespace restocker_control
