// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_perception/viewpoint_port.hpp"

namespace restocker_perception
{

const char * viewpoint_outcome_name(ViewpointOutcome outcome) noexcept
{
  switch (outcome) {
    case ViewpointOutcome::kResolved:
      return "resolved";
    case ViewpointOutcome::kMountUnavailable:
      return "mount unavailable";
    case ViewpointOutcome::kInvalidRequest:
      return "invalid request";
    case ViewpointOutcome::kCanceled:
      return "canceled";
    case ViewpointOutcome::kUnavailable:
      return "unavailable";
  }
  return "unknown";
}

}  // namespace restocker_perception
