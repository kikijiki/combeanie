// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT
// SPDX-License-Identifier: MIT

#pragma once

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <future>
#include <limits>
#include <memory>
#include <optional>

#include <restocker_interfaces/srv/commit_reserved_attachment.hpp>

namespace restocker_task_executor
{

using AttachmentCommitClock = std::chrono::steady_clock;
using AttachmentCommitService = restocker_interfaces::srv::CommitReservedAttachment;

// Fixed-size evidence for one immutable semantic operation, not a growing retry journal.
struct AttachmentCommitHistory
{
  std::uint64_t submitted{0};
  std::uint64_t matched{0};
  bool unresolved{false};
  bool inhibited{false};
  std::optional<std::uint16_t> last_status;
};

enum class AttachmentCommitDispatch : std::uint8_t
{
  kNotSent,
  kPossiblySubmitted,
  kMatchedReply,
};

struct AttachmentCommitCall
{
  AttachmentCommitDispatch dispatch{AttachmentCommitDispatch::kNotSent};
  bool unresolved_before{false};
  AttachmentCommitService::Response::SharedPtr response;
};

inline void count_attachment_commit(std::uint64_t & value) noexcept
{
  if (value != std::numeric_limits<std::uint64_t>::max()) {
    ++value;
  }
}

// Used directly with the production rclcpp client and with a deterministic client in tests.
// The client contract deliberately includes async_send_request itself: rclcpp can send before
// allocating its pending-request entry, so a throwing call is already possibly submitted.
// Removing local pending state is cleanup only; it never proves remote cancellation.
template<typename Client, typename Now>
[[nodiscard]] AttachmentCommitCall call_attachment_commit(
  Client & client, const AttachmentCommitService::Request::SharedPtr & request,
  AttachmentCommitClock::time_point deadline, AttachmentCommitClock::duration call_timeout,
  AttachmentCommitHistory & history, const Now & now)
{
  AttachmentCommitCall result;
  result.unresolved_before = history.unresolved;
  try {
    if (now() >= deadline || !client.service_is_ready() || now() >= deadline) {
      return result;
    }
    const auto call_deadline = std::min(deadline, now() + call_timeout);
    if (now() >= call_deadline) {
      return result;
    }
    result.dispatch = AttachmentCommitDispatch::kPossiblySubmitted;
    history.unresolved = true;
    count_attachment_commit(history.submitted);
    auto future = client.async_send_request(request);
    struct PendingCleanup
    {
      Client & client;
      decltype(future) & pending;
      ~PendingCleanup()
      {
        try {
          client.remove_pending_request(pending);
        } catch (...) {
          // A cleanup failure cannot alter submission evidence or escape the worker.
        }
      }
    } cleanup{client, future};
    if (future.wait_until(call_deadline) != std::future_status::ready) {
      return result;
    }
    result.response = future.get();
    if (result.response) {
      result.dispatch = AttachmentCommitDispatch::kMatchedReply;
      count_attachment_commit(history.matched);
    }
  } catch (...) {
    // Preserve the marker set BEFORE async_send_request, even if it returned no handle.
  }
  return result;
}

}  // namespace restocker_task_executor
