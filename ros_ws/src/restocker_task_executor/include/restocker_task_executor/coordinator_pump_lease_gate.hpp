// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <type_traits>

#include "restocker_task_executor/coordinator_one_shot.hpp"

namespace restocker_task_executor
{

using PumpLeaseAttempt = std::uint64_t;

[[nodiscard]] constexpr std::optional<PumpLeaseAttempt> checked_next_pump_lease_attempt(
  PumpLeaseAttempt current) noexcept
{
  if (current == std::numeric_limits<PumpLeaseAttempt>::max()) {
    return std::nullopt;
  }
  return current + 1U;
}

enum class CoordinatorPumpLeaseAcquireStatus : std::uint8_t
{
  kAcquired,
  kPendingBinding,
  kAlreadyLeased,
  kAttemptExhausted,
};

enum class CoordinatorPumpLeaseReturnStatus : std::uint8_t
{
  kReturned,
  kNotLive,
  kForeignIssuer,
  kStaleLease,
};

struct CoordinatorPumpLeaseGateSnapshot
{
  PumpLeaseAttempt attempt{0U};
  bool outstanding{false};
  bool fail_stopped{false};
  bool attempt_exhausted{false};
};

namespace detail
{
struct CoordinatorPumpLeaseIssuerCore;
}

class CoordinatorPumpLeaseGate;

class CoordinatorPumpLease final
{
public:
  CoordinatorPumpLease() = delete;
  CoordinatorPumpLease(const CoordinatorPumpLease &) = delete;
  CoordinatorPumpLease(CoordinatorPumpLease &&) noexcept = default;
  CoordinatorPumpLease & operator=(const CoordinatorPumpLease &) = delete;
  CoordinatorPumpLease & operator=(CoordinatorPumpLease &&) noexcept = default;
  ~CoordinatorPumpLease() = default;

  [[nodiscard]] bool live() const noexcept {return live_.live();}
  [[nodiscard]] PumpLeaseAttempt attempt() const noexcept {return attempt_;}

private:
  friend class CoordinatorPumpLeaseGate;
  CoordinatorPumpLease(
    std::shared_ptr<const detail::CoordinatorPumpLeaseIssuerCore> issuer,
    PumpLeaseAttempt attempt) noexcept;

  std::shared_ptr<const detail::CoordinatorPumpLeaseIssuerCore> issuer_;
  PumpLeaseAttempt attempt_{0U};
  CoordinatorOneShot live_;
};

class CoordinatorPumpLeaseDecision final
{
public:
  CoordinatorPumpLeaseDecision() = delete;
  CoordinatorPumpLeaseDecision(const CoordinatorPumpLeaseDecision &) = delete;
  CoordinatorPumpLeaseDecision(CoordinatorPumpLeaseDecision && other) noexcept;
  CoordinatorPumpLeaseDecision & operator=(const CoordinatorPumpLeaseDecision &) = delete;
  CoordinatorPumpLeaseDecision & operator=(CoordinatorPumpLeaseDecision && other) noexcept;
  ~CoordinatorPumpLeaseDecision() = default;

  [[nodiscard]] CoordinatorPumpLeaseAcquireStatus status() const noexcept {return status_;}
  [[nodiscard]] bool has_lease() const noexcept {return lease_.has_value();}
  [[nodiscard]] std::optional<CoordinatorPumpLease> take_lease() noexcept;

private:
  friend class CoordinatorPumpLeaseGate;
  explicit CoordinatorPumpLeaseDecision(CoordinatorPumpLeaseAcquireStatus status) noexcept;
  explicit CoordinatorPumpLeaseDecision(CoordinatorPumpLease && lease) noexcept;

  CoordinatorPumpLeaseAcquireStatus status_;
  std::optional<CoordinatorPumpLease> lease_;
};

// All methods require external synchronization by the owner.
class CoordinatorPumpLeaseGate final
{
public:
  CoordinatorPumpLeaseGate();
  CoordinatorPumpLeaseGate(const CoordinatorPumpLeaseGate &) = delete;
  CoordinatorPumpLeaseGate(CoordinatorPumpLeaseGate &&) = delete;
  CoordinatorPumpLeaseGate & operator=(const CoordinatorPumpLeaseGate &) = delete;
  CoordinatorPumpLeaseGate & operator=(CoordinatorPumpLeaseGate &&) = delete;
  ~CoordinatorPumpLeaseGate() = default;

  [[nodiscard]] CoordinatorPumpLeaseDecision acquire(bool pending_binding_exists) noexcept;
  [[nodiscard]] CoordinatorPumpLeaseReturnStatus return_lease(
    CoordinatorPumpLease & lease) noexcept;
  [[nodiscard]] CoordinatorPumpLeaseGateSnapshot snapshot() const noexcept;

private:
  std::shared_ptr<const detail::CoordinatorPumpLeaseIssuerCore> issuer_;
  PumpLeaseAttempt attempt_{0U};
  bool outstanding_{false};
  bool fail_stopped_{false};
  bool attempt_exhausted_{false};
};

}  // namespace restocker_task_executor
