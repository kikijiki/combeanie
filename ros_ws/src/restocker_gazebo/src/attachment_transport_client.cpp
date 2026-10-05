// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_gazebo/attachment_transport_client.hpp"

#include <cstdint>
#include <limits>
#include <stdexcept>
#include <utility>

#include <gz/transport/Node.hh>

#include "restocker_gazebo/attachment_protocol.hpp"

namespace restocker_gazebo
{
namespace
{

[[nodiscard]] AttachmentStatus status(AttachmentStatusCode code, std::string detail)
{
  return AttachmentStatus{code, std::move(detail)};
}

}  // namespace

class AttachmentTransportClient::Impl
{
public:
  Impl(
    std::string set_service, std::string query_service,
    std::chrono::milliseconds request_timeout)
  : set_service_(std::move(set_service)),
    query_service_(std::move(query_service)),
    request_timeout_ms_(checked_timeout(request_timeout))
  {
    if (set_service_.empty() || query_service_.empty()) {
      throw std::invalid_argument("attachment transport service names must be nonempty");
    }
  }

  [[nodiscard]] AttachmentTransportResult command(
    std::string expected_simulator_epoch, std::string operation_id,
    const AttachmentRequest & request)
  {
    return call(
      set_service_, encode_command(
        std::move(expected_simulator_epoch), std::move(operation_id), request));
  }

  [[nodiscard]] AttachmentTransportResult query(
    std::string expected_simulator_epoch, std::string operation_id)
  {
    return call(
      query_service_,
      encode_query(std::move(expected_simulator_epoch), std::move(operation_id)));
  }

private:
  [[nodiscard]] static unsigned int checked_timeout(std::chrono::milliseconds timeout)
  {
    if (timeout.count() <= 0 ||
      static_cast<std::uint64_t>(timeout.count()) > std::numeric_limits<unsigned int>::max())
    {
      throw std::invalid_argument("attachment transport timeout is outside its valid range");
    }
    return static_cast<unsigned int>(timeout.count());
  }

  template<typename Request>
  [[nodiscard]] AttachmentTransportResult call(
    const std::string & service, const Request & request)
  {
    msgs::AttachmentReply wire_reply;
    bool service_result = false;
    const bool received = node_.Request(
      service, request, request_timeout_ms_, wire_reply, service_result);
    if (!received) {
      return AttachmentTransportResult{
        AttachmentTransportOutcome::kNoReply,
        status(
          AttachmentStatusCode::kOutcomeUnknown,
          "Gazebo attachment service produced no reply before the steady-clock deadline"),
        std::nullopt};
    }
    if (!service_result) {
      return AttachmentTransportResult{
        AttachmentTransportOutcome::kServiceRejected,
        status(
          AttachmentStatusCode::kOutcomeUnknown,
          "Gazebo attachment service rejected the transport call without physical proof"),
        std::nullopt};
    }
    AttachmentStatus decode_error;
    auto decoded = decode_reply(wire_reply, decode_error);
    if (!decoded) {
      return AttachmentTransportResult{
        AttachmentTransportOutcome::kMalformedReply,
        status(
          AttachmentStatusCode::kExternalInconsistency,
          "Gazebo attachment reply violated the protocol: " + decode_error.detail),
        std::nullopt};
    }
    return AttachmentTransportResult{
      AttachmentTransportOutcome::kReply, decoded->status, std::move(decoded)};
  }

  std::string set_service_;
  std::string query_service_;
  unsigned int request_timeout_ms_;
  gz::transport::Node node_;
};

AttachmentTransportClient::AttachmentTransportClient(
  std::string set_service, std::string query_service,
  std::chrono::milliseconds request_timeout)
: impl_(std::make_unique<Impl>(
      std::move(set_service), std::move(query_service), request_timeout))
{}

AttachmentTransportClient::~AttachmentTransportClient() = default;
AttachmentTransportClient::AttachmentTransportClient(AttachmentTransportClient &&) noexcept =
  default;
AttachmentTransportClient & AttachmentTransportClient::operator=(
  AttachmentTransportClient &&) noexcept = default;

AttachmentTransportResult AttachmentTransportClient::command(
  std::string expected_simulator_epoch, std::string operation_id,
  const AttachmentRequest & request)
{
  return impl_->command(
    std::move(expected_simulator_epoch), std::move(operation_id), request);
}

AttachmentTransportResult AttachmentTransportClient::query(
  std::string expected_simulator_epoch, std::string operation_id)
{
  return impl_->query(std::move(expected_simulator_epoch), std::move(operation_id));
}

}  // namespace restocker_gazebo
