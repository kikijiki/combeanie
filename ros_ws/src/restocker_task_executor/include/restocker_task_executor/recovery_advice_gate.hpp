// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <optional>
#include <string>
#include <vector>

#include <restocker_reasoner/recovery_advice.hpp>

namespace restocker_task_executor
{

// Which recovery primitives the deterministic authorisation permits, at one instant.
//
// This is not the reasoner's to fill in. It is computed from the driver's own evidence at the
// moment an action would be taken, and it is computed again there even though the same question
// was answered when the advisory request was sent: a recommendation is stale from the moment it
// is generated, and this project's most expensive recurring defect is a value valid at one
// instant being read at another.
struct RecoveryAdvicePermissions
{
  bool retry_segment{false};
  bool resume_at_recovery_state{false};
  bool abandon_task{false};

  [[nodiscard]] bool permits(restocker_reasoner::RecoveryPrimitive primitive) const noexcept;
  [[nodiscard]] std::vector<restocker_reasoner::RecoveryPrimitive> permitted() const;
};

struct RecoveryAdviceDecision
{
  // What the driver will actually do. Always a primitive the permitted set allows.
  restocker_reasoner::RecoveryPrimitive primitive{
    restocker_reasoner::RecoveryPrimitive::kAbandonTask};
  // True when a recommendation was present, permitted, no less cautious than the deterministic
  // choice, and therefore taken. It may still name the same primitive the deterministic policy
  // had chosen, which is agreement rather than influence.
  bool followed_advice{false};
  // True only when the recommendation changed what the driver does.
  bool changed_the_action{false};
  // Empty exactly when a recommendation was followed; otherwise why it was not.
  std::string rejection;
  // One sentence for the log and the audit record.
  std::string detail;
};

// Decide what to do, given what the deterministic policy had already chosen, what is permitted at
// this instant, and whatever recommendation happens to have arrived.
//
// Two independent guards keep the reasoner an advisor. First, a recommendation must name a
// primitive the permitted set allows right now. Second, it must either agree with the
// deterministic choice, which changes nothing, or name `kAbandonTask`, which stops. There is no
// third possibility: a recommendation cannot talk the system into acting where it had decided not
// to, and it cannot swap one action for another. A recommendation that would widen what is
// permitted is by construction rejected, so no reasoner output can reach an action the
// deterministic authorisation had refused.
//
// With no recommendation at all, absent backend, expired deadline, malformed answer, an answer
// to a superseded question, the deterministic choice is returned unchanged. That is the same
// value the driver would have used had this function never been called.
[[nodiscard]] RecoveryAdviceDecision resolve_recovery_advice(
  restocker_reasoner::RecoveryPrimitive deterministic_choice,
  const RecoveryAdvicePermissions & permitted_now,
  const std::optional<restocker_reasoner::RecoveryAdvice> & advice);

}  // namespace restocker_task_executor
