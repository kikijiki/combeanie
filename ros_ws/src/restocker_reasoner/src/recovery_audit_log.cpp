// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_reasoner/recovery_audit_log.hpp"

#include <ios>
#include <string>

#include "restocker_reasoner/strict_json.hpp"

namespace restocker_reasoner
{

namespace
{

[[nodiscard]] std::string boolean(bool value)
{
  return value ? "true" : "false";
}

}  // namespace

const char * recovery_audit_event_name(RecoveryAuditEvent event) noexcept
{
  switch (event) {
    case RecoveryAuditEvent::kAsked: return "asked";
    case RecoveryAuditEvent::kDecided: return "decided";
    case RecoveryAuditEvent::kOutcome: return "outcome";
  }
  return "outcome";
}

std::string render_audit_line(const RecoveryAuditRecord & record)
{
  std::string permitted;
  for (const auto primitive : record.permitted_at_decision) {
    if (!permitted.empty()) {
      permitted += ",";
    }
    permitted += json_quote(recovery_primitive_name(primitive));
  }

  const auto optional_primitive = [](const std::optional<RecoveryPrimitive> & primitive) {
    return primitive ? json_quote(recovery_primitive_name(*primitive)) : std::string("null");
  };

  std::string out = "{";
  out += "\"event\":" + json_quote(recovery_audit_event_name(record.event));
  out += ",\"request_id\":" + json_quote(record.request_id);
  out += ",\"goal_generation\":" + std::to_string(record.goal_generation);
  // The query is already canonical JSON, so it is embedded as an object rather than a quoted
  // string; an empty one becomes null rather than invalid JSON.
  out += ",\"query\":" + (record.query_json.empty() ? std::string("null") : record.query_json);
  out += ",\"backend\":" + json_quote(record.backend);
  out += ",\"latency_ms\":" + std::to_string(record.latency_ms);
  out += ",\"response_verbatim\":" + json_quote(record.response_verbatim);
  out += ",\"transport_detail\":" + json_quote(record.transport_detail);
  out += ",\"permitted_at_decision\":[" + permitted + "]";
  out += ",\"validated\":" + boolean(record.validated);
  out += ",\"rejection\":" + json_quote(record.rejection);
  out += ",\"recommended_primitive\":" + optional_primitive(record.recommended_primitive);
  out += ",\"failure_class\":" +
    (record.failure_class ?
    json_quote(recovery_failure_class_name(*record.failure_class)) : std::string("null"));
  out += ",\"confidence\":" + json_number(record.confidence, 4);
  out += ",\"explanation\":" + json_quote(record.explanation);
  out += ",\"used\":" + boolean(record.used);
  out += ",\"deterministic_primitive\":" + optional_primitive(record.deterministic_primitive);
  out += ",\"applied_primitive\":" + optional_primitive(record.applied_primitive);
  out += ",\"outcome\":" + json_quote(record.outcome);
  out += "}";
  return out;
}

RecoveryAuditLog::RecoveryAuditLog(const std::string & path)
{
  if (path.empty()) {
    return;
  }
  configured_ = true;
  stream_.open(path, std::ios::out | std::ios::app);
  open_ = stream_.is_open();
}

bool RecoveryAuditLog::open() const
{
  const std::lock_guard<std::mutex> guard(mutex_);
  return open_;
}

bool RecoveryAuditLog::configured() const
{
  const std::lock_guard<std::mutex> guard(mutex_);
  return configured_;
}

void RecoveryAuditLog::append(const RecoveryAuditRecord & record)
{
  const std::lock_guard<std::mutex> guard(mutex_);
  if (!open_) {
    // A log nobody asked for loses nothing.
    failed_ += configured_ ? 1U : 0U;
    return;
  }
  stream_ << render_audit_line(record) << '\n';
  // Flushed per record so records written just before a crash survive.
  stream_.flush();
  if (!stream_) {
    ++failed_;
    return;
  }
  ++appended_;
}

std::size_t RecoveryAuditLog::appended() const
{
  const std::lock_guard<std::mutex> guard(mutex_);
  return appended_;
}

std::size_t RecoveryAuditLog::failed_appends() const
{
  const std::lock_guard<std::mutex> guard(mutex_);
  return failed_;
}

}  // namespace restocker_reasoner
