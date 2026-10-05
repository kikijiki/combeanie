// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <optional>
#include <string>

#include "msgs/attachment.pb.h"
#include "restocker_gazebo/attachment_journal.hpp"

namespace restocker_gazebo
{

struct DecodedAttachmentCommand
{
  std::string expected_simulator_epoch;
  std::string operation_id;
  AttachmentRequest request;
};

struct DecodedAttachmentQuery
{
  std::string expected_simulator_epoch;
  std::string operation_id;
};

[[nodiscard]] msgs::SetAttachmentRequest encode_command(
  std::string expected_simulator_epoch,
  std::string operation_id,
  const AttachmentRequest & request);

[[nodiscard]] msgs::QueryAttachmentRequest encode_query(
  std::string expected_simulator_epoch,
  std::string operation_id = {});

[[nodiscard]] std::optional<DecodedAttachmentCommand> decode_command(
  const msgs::SetAttachmentRequest & message,
  AttachmentStatus & error);

[[nodiscard]] std::optional<DecodedAttachmentQuery> decode_query(
  const msgs::QueryAttachmentRequest & message,
  AttachmentStatus & error);

[[nodiscard]] msgs::AttachmentReply encode_reply(const AttachmentJournalReply & reply);

[[nodiscard]] std::optional<AttachmentJournalReply> decode_reply(
  const msgs::AttachmentReply & message,
  AttachmentStatus & error);

}  // namespace restocker_gazebo
