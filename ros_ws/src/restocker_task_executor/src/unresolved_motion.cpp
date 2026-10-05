// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_task_executor/unresolved_motion.hpp"

#include <cstdio>
#include <utility>

namespace restocker_task_executor
{

const char * to_string(MotionEndpoint endpoint) noexcept
{
  switch (endpoint) {
    case MotionEndpoint::kLane: return "lane";
    case MotionEndpoint::kTray: return "tray";
    case MotionEndpoint::kViewpoint: return "viewpoint";
    case MotionEndpoint::kRestock: return "restock";
  }
  return "unknown";
}

const char * to_string(UnresolvedCondition condition) noexcept
{
  switch (condition) {
    case UnresolvedCondition::kAdmissionUnresolved: return "admission_unresolved";
    case UnresolvedCondition::kResultUnresolved: return "result_unresolved";
    case UnresolvedCondition::kSettlementPendingCancel: return "settlement_pending_cancel";
  }
  return "unknown";
}

std::string goal_id_text(const AttemptGoalId & goal_id)
{
  std::string text;
  text.reserve(goal_id.size() * 2U);
  for (const std::uint8_t byte : goal_id) {
    char pair[3];
    std::snprintf(pair, sizeof(pair), "%02x", static_cast<unsigned>(byte));
    text += pair;
  }
  return text;
}

bool settles_attempt(const MotionEvidence & evidence) noexcept
{
  switch (evidence.kind) {
    case MotionEvidence::Kind::kAdmissionRejected:
      return true;
    case MotionEvidence::Kind::kDeliveredTerminal:
      // A parent that returned while a descendant may still run proves nothing about it.
      if (evidence.descendant_unsettled) {
        return false;
      }
      // Arrival is a verified stop for that attempt; the other two are the payload's own proof.
      // A failure carrying none of them leaves the arm's state unestablished.
      return evidence.arrived || evidence.motion_definitely_not_started ||
             evidence.execution_reached_terminal_stop;
    case MotionEvidence::Kind::kNonSettling:
      return false;
  }
  return false;
}

UnresolvedMotionRegistry::AttemptId UnresolvedMotionRegistry::open(
  MotionEndpoint endpoint, UnresolvedCondition condition, std::string label)
{
  const AttemptId id = next_id_++;
  Record record;
  record.id = id;
  record.endpoint = endpoint;
  record.condition = condition;
  record.label = std::move(label);
  records_.emplace(id, std::move(record));
  return id;
}

UnresolvedMotionRegistry::Result UnresolvedMotionRegistry::bind_goal(
  AttemptId id, const AttemptGoalId & goal_id)
{
  const auto found = records_.find(id);
  if (found == records_.end()) {
    return Result::kUnknownAttempt;
  }
  found->second.goal_id = goal_id;
  found->second.fragments.push_back("goal admitted late: " + goal_id_text(goal_id));
  return Result::kStillUnresolved;
}

UnresolvedMotionRegistry::Result UnresolvedMotionRegistry::set_condition(
  AttemptId id, UnresolvedCondition condition)
{
  const auto found = records_.find(id);
  if (found == records_.end()) {
    return Result::kUnknownAttempt;
  }
  found->second.condition = condition;
  return Result::kStillUnresolved;
}

UnresolvedMotionRegistry::Result UnresolvedMotionRegistry::apply(
  AttemptId id, const MotionEvidence & evidence)
{
  const auto found = records_.find(id);
  if (found == records_.end()) {
    return Result::kUnknownAttempt;
  }
  if (settles_attempt(evidence)) {
    retired_.insert(id);
    records_.erase(found);
    return Result::kSettled;
  }
  found->second.fragments.push_back(
    evidence.note.empty() ? std::string("non-settling evidence") : evidence.note);
  return Result::kStillUnresolved;
}

UnresolvedMotionRegistry::Result UnresolvedMotionRegistry::apply_for_goal(
  const AttemptGoalId & goal_id, const MotionEvidence & evidence)
{
  for (const auto & [id, record] : records_) {
    if (record.goal_id && *record.goal_id == goal_id) {
      return apply(id, evidence);
    }
  }
  return Result::kUnknownAttempt;
}

const UnresolvedMotionRegistry::Record * UnresolvedMotionRegistry::find(
  AttemptId id) const noexcept
{
  const auto found = records_.find(id);
  return found == records_.end() ? nullptr : &found->second;
}

std::vector<UnresolvedMotionRegistry::AttemptId> UnresolvedMotionRegistry::ids() const
{
  std::vector<AttemptId> result;
  result.reserve(records_.size());
  for (const auto & [id, record] : records_) {
    static_cast<void>(record);
    result.push_back(id);
  }
  return result;
}

std::vector<std::string> UnresolvedMotionRegistry::describe_each() const
{
  std::vector<std::string> entries;
  entries.reserve(records_.size());
  for (const auto & [id, record] : records_) {
    std::string text = std::string(to_string(record.endpoint)) + "#" + std::to_string(id) + " " +
      to_string(record.condition);
    if (record.goal_id) {
      text += " goal=" + goal_id_text(*record.goal_id);
    }
    if (!record.label.empty()) {
      text += " (" + record.label + ")";
    }
    entries.push_back(std::move(text));
  }
  return entries;
}

std::string UnresolvedMotionRegistry::describe() const
{
  if (records_.empty()) {
    return {};
  }
  std::string text = "motion_unresolved: " + std::to_string(records_.size()) + " attempt(s):";
  for (const auto & entry : describe_each()) {
    text += " [" + entry + "]";
  }
  return text;
}

}  // namespace restocker_task_executor
