// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_task_executor/coordinator_pump_lease_gate.hpp"

#include <utility>

namespace restocker_task_executor
{
namespace detail
{

struct CoordinatorPumpLeaseIssuerCore
{
};

}  // namespace detail
namespace
{

template<typename Capability>
[[nodiscard]] std::optional<Capability> take_optional(
  std::optional<Capability> & source) noexcept
{
  std::optional<Capability> result;
  if (source) {
    result.emplace(std::move(*source));
    source.reset();
  }
  return result;
}

}  // namespace

CoordinatorPumpLease::CoordinatorPumpLease(
  std::shared_ptr<const detail::CoordinatorPumpLeaseIssuerCore> issuer,
  PumpLeaseAttempt attempt) noexcept
: issuer_(std::move(issuer)), attempt_(attempt), live_(true)
{
}

CoordinatorPumpLeaseDecision::CoordinatorPumpLeaseDecision(
  CoordinatorPumpLeaseAcquireStatus status) noexcept
: status_(status)
{
}

CoordinatorPumpLeaseDecision::CoordinatorPumpLeaseDecision(
  CoordinatorPumpLease && lease) noexcept
: status_(CoordinatorPumpLeaseAcquireStatus::kAcquired), lease_(std::move(lease))
{
}

CoordinatorPumpLeaseDecision::CoordinatorPumpLeaseDecision(
  CoordinatorPumpLeaseDecision && other) noexcept
: status_(other.status_), lease_(take_optional(other.lease_))
{
}

CoordinatorPumpLeaseDecision & CoordinatorPumpLeaseDecision::operator=(
  CoordinatorPumpLeaseDecision && other) noexcept
{
  if (this != &other) {
    status_ = other.status_;
    lease_ = take_optional(other.lease_);
  }
  return *this;
}

std::optional<CoordinatorPumpLease> CoordinatorPumpLeaseDecision::take_lease() noexcept
{
  return take_optional(lease_);
}

CoordinatorPumpLeaseGate::CoordinatorPumpLeaseGate()
: issuer_(std::make_shared<const detail::CoordinatorPumpLeaseIssuerCore>())
{
}

CoordinatorPumpLeaseDecision CoordinatorPumpLeaseGate::acquire(
  bool pending_binding_exists) noexcept
{
  if (pending_binding_exists) {
    return CoordinatorPumpLeaseDecision{CoordinatorPumpLeaseAcquireStatus::kPendingBinding};
  }
  if (outstanding_) {
    return CoordinatorPumpLeaseDecision{CoordinatorPumpLeaseAcquireStatus::kAlreadyLeased};
  }
  if (fail_stopped_ || attempt_exhausted_) {
    return CoordinatorPumpLeaseDecision{CoordinatorPumpLeaseAcquireStatus::kAttemptExhausted};
  }
  const auto next_attempt = checked_next_pump_lease_attempt(attempt_);
  if (!next_attempt || *next_attempt == 0U) {
    fail_stopped_ = true;
    attempt_exhausted_ = true;
    return CoordinatorPumpLeaseDecision{CoordinatorPumpLeaseAcquireStatus::kAttemptExhausted};
  }
  attempt_ = *next_attempt;
  outstanding_ = true;
  return CoordinatorPumpLeaseDecision{CoordinatorPumpLease{issuer_, attempt_}};
}

CoordinatorPumpLeaseReturnStatus CoordinatorPumpLeaseGate::return_lease(
  CoordinatorPumpLease & lease) noexcept
{
  if (!lease.live_.live()) {
    return CoordinatorPumpLeaseReturnStatus::kNotLive;
  }
  if (lease.issuer_ != issuer_) {
    return CoordinatorPumpLeaseReturnStatus::kForeignIssuer;
  }
  if (!outstanding_ || lease.attempt_ != attempt_) {
    return CoordinatorPumpLeaseReturnStatus::kStaleLease;
  }
  lease.live_.consume();
  outstanding_ = false;
  return CoordinatorPumpLeaseReturnStatus::kReturned;
}

CoordinatorPumpLeaseGateSnapshot CoordinatorPumpLeaseGate::snapshot() const noexcept
{
  return {attempt_, outstanding_, fail_stopped_, attempt_exhausted_};
}

}  // namespace restocker_task_executor
