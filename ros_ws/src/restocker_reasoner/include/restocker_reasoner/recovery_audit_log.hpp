// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <cstddef>
#include <cstdint>
#include <fstream>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "restocker_reasoner/recovery_advice.hpp"

namespace restocker_reasoner
{

enum class RecoveryAuditEvent : std::uint8_t
{
  // A question was asked of a backend.
  kAsked,
  // A decision was taken at the moment the recommendation would have been acted on.
  kDecided,
  // What the deterministic system then did.
  kOutcome,
};

[[nodiscard]] const char * recovery_audit_event_name(RecoveryAuditEvent event) noexcept;

// One line of the decision record.
//
// The log must show afterwards whether the reasoner helped, so it records the decision and not
// only the result: what was asked, what came back verbatim, whether it validated (and why not),
// what the deterministic policy had already chosen, whether the recommendation was used, and what
// followed. An outcome-only log could not distinguish a reasoner that agreed from one that was
// ignored.
struct RecoveryAuditRecord
{
  RecoveryAuditEvent event{RecoveryAuditEvent::kAsked};
  std::string request_id;
  std::uint64_t goal_generation{0U};
  // The canonical rendering of the question, from render_recovery_query_json.
  std::string query_json;
  // Which backend was asked, and how long it took. -1 when no call was made.
  std::string backend;
  std::int64_t latency_ms{-1};
  // Exactly what came back, bounded, before anything looked at it. Empty when nothing came back.
  std::string response_verbatim;
  // Why nothing came back: a deadline, a transport failure, an absent backend.
  std::string transport_detail;
  // The permitted set recomputed at the instant of the decision, which is the one that governs.
  std::vector<RecoveryPrimitive> permitted_at_decision;
  bool validated{false};
  std::string rejection;
  std::optional<RecoveryPrimitive> recommended_primitive;
  std::optional<RecoveryFailureClass> failure_class;
  double confidence{0.0};
  std::string explanation;
  bool used{false};
  std::optional<RecoveryPrimitive> deterministic_primitive;
  std::optional<RecoveryPrimitive> applied_primitive;
  std::string outcome;
};

// One JSON object on one line. Every string is escaped, so a backend cannot inject structure into
// the log by returning brackets or newlines.
[[nodiscard]] std::string render_audit_line(const RecoveryAuditRecord & record);

// Append-only JSON Lines, opened once and flushed per record.
//
// Failing to write an audit record never fails a task and never throws: the deterministic system
// must behave identically whether or not this file can be written. Write failures are counted in
// failed_appends(). An empty path is not a failure: no audit file was requested, and records are
// discarded without being counted as lost.
class RecoveryAuditLog
{
public:
  explicit RecoveryAuditLog(const std::string & path);

  // False when no path was configured, and false when a configured path could not be opened.
  [[nodiscard]] bool open() const;

  // True when a path was configured, whether or not it opened.
  [[nodiscard]] bool configured() const;

  void append(const RecoveryAuditRecord & record);

  [[nodiscard]] std::size_t appended() const;
  [[nodiscard]] std::size_t failed_appends() const;

private:
  mutable std::mutex mutex_;
  std::ofstream stream_;
  bool open_{false};
  bool configured_{false};
  std::size_t appended_{0U};
  std::size_t failed_{0U};
};

}  // namespace restocker_reasoner
