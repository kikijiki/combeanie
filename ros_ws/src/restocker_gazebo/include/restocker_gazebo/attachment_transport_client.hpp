// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <chrono>
#include <memory>
#include <optional>
#include <string>

#include "restocker_gazebo/attachment_journal.hpp"

namespace restocker_gazebo
{

enum class AttachmentTransportOutcome
{
  kReply,
  kNoReply,
  kServiceRejected,
  kMalformedReply,
};

struct AttachmentTransportResult
{
  AttachmentTransportOutcome outcome{AttachmentTransportOutcome::kNoReply};
  AttachmentStatus status;
  std::optional<AttachmentJournalReply> reply;
};

// Blocking, token-free Gazebo Transport client. Call it only from a dedicated worker; a command
// timeout is outcome-unknown and must be reconciled by querying the same operation ID.
class AttachmentTransportClient
{
public:
  AttachmentTransportClient(
    std::string set_service, std::string query_service,
    std::chrono::milliseconds request_timeout);
  ~AttachmentTransportClient();

  AttachmentTransportClient(const AttachmentTransportClient &) = delete;
  AttachmentTransportClient & operator=(const AttachmentTransportClient &) = delete;
  AttachmentTransportClient(AttachmentTransportClient &&) noexcept;
  AttachmentTransportClient & operator=(AttachmentTransportClient &&) noexcept;

  [[nodiscard]] AttachmentTransportResult command(
    std::string expected_simulator_epoch, std::string operation_id,
    const AttachmentRequest & request);
  [[nodiscard]] AttachmentTransportResult query(
    std::string expected_simulator_epoch, std::string operation_id = {});

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace restocker_gazebo
