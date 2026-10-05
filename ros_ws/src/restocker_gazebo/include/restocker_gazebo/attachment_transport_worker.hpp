// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <chrono>
#include <memory>
#include <optional>
#include <string>

#include "restocker_gazebo/attachment_transport_client.hpp"

namespace restocker_gazebo
{

enum class AttachmentTransportJobKind
{
  kCommand,
  kQuery,
};

struct AttachmentTransportCompletion
{
  AttachmentTransportJobKind kind{AttachmentTransportJobKind::kQuery};
  std::string operation_id;
  AttachmentTransportResult result;
};

// Single-slot worker that keeps blocking Gazebo Transport calls off ROS executor threads.
class AttachmentTransportWorker
{
public:
  AttachmentTransportWorker(
    std::string set_service, std::string query_service,
    std::chrono::milliseconds request_timeout);
  ~AttachmentTransportWorker();

  AttachmentTransportWorker(const AttachmentTransportWorker &) = delete;
  AttachmentTransportWorker & operator=(const AttachmentTransportWorker &) = delete;
  AttachmentTransportWorker(AttachmentTransportWorker &&) = delete;
  AttachmentTransportWorker & operator=(AttachmentTransportWorker &&) = delete;

  [[nodiscard]] bool submit_command(
    std::string expected_simulator_epoch, std::string operation_id,
    AttachmentRequest request);
  [[nodiscard]] bool submit_query(
    std::string expected_simulator_epoch, std::string operation_id = {});
  [[nodiscard]] std::optional<AttachmentTransportCompletion> take_completion();
  [[nodiscard]] bool busy() const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace restocker_gazebo
