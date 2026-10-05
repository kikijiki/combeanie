// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#ifndef DETAIL__PENDING_ROUTE_EXECUTOR_HPP_
#define DETAIL__PENDING_ROUTE_EXECUTOR_HPP_

#include <cstdint>
#include <exception>
#include <functional>
#include <optional>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>

#include "restocker_task_executor/coordinator_handoff_reducer.hpp"
#include "restocker_task_executor/coordinator_termination_router.hpp"

namespace restocker_task_executor::detail
{

/// Immutable inputs and the capability authorizing one pending route attempt.
///
/// Construction consumes the reducer-issued token and the owned diagnostic. Keeping them together
/// prevents an out-of-lock router invocation from being completed against a different pending
/// binding. The value holds no retry counter, logger, mutex, or reducer reference.
class PendingRouteExecutionWork final
{
public:
  PendingRouteExecutionWork() = delete;
  PendingRouteExecutionWork(
    PendingRouteWorkToken && token, SteadyTime arrived_at,
    std::string && detail) noexcept
  : token_(std::move(token)), arrived_at_(arrived_at), detail_(std::move(detail))
  {
  }

  PendingRouteExecutionWork(const PendingRouteExecutionWork &) = delete;
  PendingRouteExecutionWork(PendingRouteExecutionWork &&) noexcept = default;
  PendingRouteExecutionWork & operator=(const PendingRouteExecutionWork &) = delete;
  PendingRouteExecutionWork & operator=(PendingRouteExecutionWork &&) = delete;
  ~PendingRouteExecutionWork() = default;

  [[nodiscard]] const PendingRouteWorkToken & token() const noexcept {return token_;}
  [[nodiscard]] const PendingHandoffBindingKey & binding() const noexcept
  {
    return token_.binding();
  }
  [[nodiscard]] PendingRouteKind kind() const noexcept {return token_.kind();}
  [[nodiscard]] PendingRouteDeliveryClass delivery_class() const noexcept
  {
    return token_.delivery_class();
  }
  [[nodiscard]] PendingRouteTokenId route_token() const noexcept {return token_.token();}
  [[nodiscard]] SteadyTime arrived_at() const noexcept {return arrived_at_;}
  [[nodiscard]] const std::string & detail() const noexcept {return detail_;}

private:
  friend class PendingRouteExecutionCompletion;
  friend class PendingRouteExecutor;

  [[nodiscard]] bool valid() const noexcept
  {
    return token_.live() && valid_pending_handoff_binding_key(token_.binding()) &&
           valid_pending_route_kind(token_.kind()) &&
           valid_pending_route_delivery_class(token_.delivery_class()) && token_.token() != 0U;
  }

  PendingRouteWorkToken token_;
  SteadyTime arrived_at_{};
  std::string detail_;
};

enum class PendingRouteExecutionStatus : std::uint8_t
{
  kCompleted,
  kInvalidWork,
  kStandardException,
  kNonStandardException,
};

/// Owns the work capability on every route outcome.
///
/// The route decision exists only for kCompleted. The caller may transfer that decision to its
/// node-local merge step, but the reducer token remains in this completion until explicitly passed
/// by reference to PendingAcceptedHandoffMachine::complete_route().
class PendingRouteExecutionCompletion final
{
public:
  PendingRouteExecutionCompletion() = delete;
  PendingRouteExecutionCompletion(const PendingRouteExecutionCompletion &) = delete;
  PendingRouteExecutionCompletion(PendingRouteExecutionCompletion &&) noexcept = default;
  PendingRouteExecutionCompletion & operator=(const PendingRouteExecutionCompletion &) = delete;
  PendingRouteExecutionCompletion & operator=(PendingRouteExecutionCompletion &&) = delete;
  ~PendingRouteExecutionCompletion() = default;

  [[nodiscard]] PendingRouteExecutionStatus status() const noexcept {return status_;}
  [[nodiscard]] const PendingRouteExecutionWork & work() const noexcept {return work_;}
  [[nodiscard]] PendingRouteWorkToken & token() noexcept {return work_.token_;}
  [[nodiscard]] const PendingRouteWorkToken & token() const noexcept {return work_.token_;}
  [[nodiscard]] bool has_decision() const noexcept {return decision_.has_value();}
  [[nodiscard]] const std::optional<CoordinatorTerminationRouteDecision> & decision() const noexcept
  {
    return decision_;
  }

  [[nodiscard]] std::optional<CoordinatorTerminationRouteDecision> take_decision() noexcept
  {
    std::optional<CoordinatorTerminationRouteDecision> result{std::move(decision_)};
    decision_.reset();
    return result;
  }

private:
  friend class PendingRouteExecutor;

  PendingRouteExecutionCompletion(
    PendingRouteExecutionStatus status, PendingRouteExecutionWork && work,
    std::optional<CoordinatorTerminationRouteDecision> && decision) noexcept
  : status_(status), work_(std::move(work)), decision_(std::move(decision))
  {
  }

  PendingRouteExecutionStatus status_{PendingRouteExecutionStatus::kInvalidWork};
  PendingRouteExecutionWork work_;
  std::optional<CoordinatorTerminationRouteDecision> decision_;
};

/// Runs one external route attempt and converts all exits into owned completion evidence.
///
/// This component neither synchronizes nor mutates the pending reducer. Its caller must claim work
/// under the binding lock, unlock, execute here, and later authenticate the retained token under
/// the binding lock. A blocking router therefore cannot block scalar binding diagnostics.
class PendingRouteExecutor final
{
public:
  PendingRouteExecutor() = delete;

  template<typename Router>
  [[nodiscard]] static PendingRouteExecutionCompletion execute(
    PendingRouteExecutionWork && work, Router && router) noexcept
  {
    using Result = std::invoke_result_t<Router, const PendingRouteExecutionWork &>;
    static_assert(
      std::is_same_v<Result, CoordinatorTerminationRouteDecision>,
      "a pending route callable must return an owned CoordinatorTerminationRouteDecision");

    if (!work.valid()) {
      return PendingRouteExecutionCompletion{
        PendingRouteExecutionStatus::kInvalidWork, std::move(work), std::nullopt};
    }

    try {
      std::optional<CoordinatorTerminationRouteDecision> decision;
      decision.emplace(
        std::invoke(std::forward<Router>(router), std::as_const(work)));
      return PendingRouteExecutionCompletion{
        PendingRouteExecutionStatus::kCompleted, std::move(work), std::move(decision)};
    } catch (const std::exception &) {
      return PendingRouteExecutionCompletion{
        PendingRouteExecutionStatus::kStandardException, std::move(work), std::nullopt};
    } catch (...) {
      return PendingRouteExecutionCompletion{
        PendingRouteExecutionStatus::kNonStandardException, std::move(work), std::nullopt};
    }
  }
};

/// Production callable translating the closed pending-route vocabulary into termination routes.
class PendingRouteRouterAdapter final
{
public:
  explicit PendingRouteRouterAdapter(const CoordinatorTerminationRouter & router) noexcept
  : router_(router)
  {
  }
  PendingRouteRouterAdapter(CoordinatorTerminationRouter &&) = delete;
  PendingRouteRouterAdapter(const CoordinatorTerminationRouter &&) = delete;

  [[nodiscard]] CoordinatorTerminationRouteDecision operator()(
    const PendingRouteExecutionWork & work) const
  {
    constexpr OperationGeneration operation_generation = 0U;
    const auto & binding = work.binding();
    switch (work.kind()) {
      case PendingRouteKind::kDrain:
        return router_.route_drain(
          binding.goal_id, binding.generation, work.arrived_at(), operation_generation,
          work.detail());
      case PendingRouteKind::kTransformAuthorityLoss:
        return router_.route_transform_authority_loss(
          binding.goal_id, binding.generation, work.arrived_at(), operation_generation,
          work.detail());
      case PendingRouteKind::kSteadyClockFailure:
      case PendingRouteKind::kCallbackOrAdapterFailure:
        return router_.route_protocol_failure(
          binding.goal_id, binding.generation, work.arrived_at(), operation_generation,
          work.detail());
    }
    // execute() rejects an unknown kind before invoking this adapter. Retain a closed fallback for
    // defensive direct use without inventing a fifth semantic route.
    throw std::invalid_argument("invalid pending route kind");
  }

private:
  const CoordinatorTerminationRouter & router_;
};

static_assert(std::is_trivially_copyable_v<PendingRouteExecutionStatus>);

}  // namespace restocker_task_executor::detail

#endif  // DETAIL__PENDING_ROUTE_EXECUTOR_HPP_
