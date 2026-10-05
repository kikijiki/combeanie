// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_gazebo/attachment_transport_worker.hpp"

#include <stop_token>

#include <condition_variable>
#include <mutex>
#include <thread>
#include <utility>

namespace restocker_gazebo
{

class AttachmentTransportWorker::Impl
{
public:
  Impl(
    std::string set_service, std::string query_service,
    std::chrono::milliseconds request_timeout)
  : client_(std::move(set_service), std::move(query_service), request_timeout),
    thread_([this](std::stop_token stop) {run(stop);})
  {}

  ~Impl()
  {
    thread_.request_stop();
    condition_.notify_all();
  }

  [[nodiscard]] bool submit_command(
    std::string expected_simulator_epoch, std::string operation_id,
    AttachmentRequest request)
  {
    Job job;
    job.kind = AttachmentTransportJobKind::kCommand;
    job.expected_simulator_epoch = std::move(expected_simulator_epoch);
    job.operation_id = std::move(operation_id);
    job.request = std::move(request);
    return submit(std::move(job));
  }

  [[nodiscard]] bool submit_query(
    std::string expected_simulator_epoch, std::string operation_id)
  {
    Job job;
    job.kind = AttachmentTransportJobKind::kQuery;
    job.expected_simulator_epoch = std::move(expected_simulator_epoch);
    job.operation_id = std::move(operation_id);
    return submit(std::move(job));
  }

  [[nodiscard]] std::optional<AttachmentTransportCompletion> take_completion()
  {
    std::lock_guard lock(mutex_);
    auto result = std::move(completion_);
    completion_.reset();
    return result;
  }

  [[nodiscard]] bool busy() const
  {
    std::lock_guard lock(mutex_);
    return pending_.has_value() || running_ || completion_.has_value();
  }

private:
  struct Job
  {
    AttachmentTransportJobKind kind{AttachmentTransportJobKind::kQuery};
    std::string expected_simulator_epoch;
    std::string operation_id;
    std::optional<AttachmentRequest> request;
  };

  [[nodiscard]] bool submit(Job job)
  {
    std::lock_guard lock(mutex_);
    if (pending_ || running_ || completion_) {
      return false;
    }
    pending_ = std::move(job);
    condition_.notify_one();
    return true;
  }

  void run(std::stop_token stop)
  {
    while (!stop.stop_requested()) {
      Job job;
      {
        std::unique_lock lock(mutex_);
        if (!condition_.wait(lock, stop, [this] {return pending_.has_value();})) {
          return;
        }
        job = std::move(*pending_);
        pending_.reset();
        running_ = true;
      }

      AttachmentTransportResult result;
      if (job.kind == AttachmentTransportJobKind::kCommand) {
        result = client_.command(
          std::move(job.expected_simulator_epoch), job.operation_id, *job.request);
      } else {
        result = client_.query(
          std::move(job.expected_simulator_epoch), job.operation_id);
      }

      {
        std::lock_guard lock(mutex_);
        running_ = false;
        completion_ = AttachmentTransportCompletion{
          job.kind, std::move(job.operation_id), std::move(result)};
      }
    }
  }

  AttachmentTransportClient client_;
  mutable std::mutex mutex_;
  std::condition_variable_any condition_;
  std::optional<Job> pending_;
  bool running_{false};
  std::optional<AttachmentTransportCompletion> completion_;
  std::jthread thread_;
};

AttachmentTransportWorker::AttachmentTransportWorker(
  std::string set_service, std::string query_service,
  std::chrono::milliseconds request_timeout)
: impl_(std::make_unique<Impl>(
      std::move(set_service), std::move(query_service), request_timeout))
{}

AttachmentTransportWorker::~AttachmentTransportWorker() = default;

bool AttachmentTransportWorker::submit_command(
  std::string expected_simulator_epoch, std::string operation_id,
  AttachmentRequest request)
{
  return impl_->submit_command(
    std::move(expected_simulator_epoch), std::move(operation_id), std::move(request));
}

bool AttachmentTransportWorker::submit_query(
  std::string expected_simulator_epoch, std::string operation_id)
{
  return impl_->submit_query(
    std::move(expected_simulator_epoch), std::move(operation_id));
}

std::optional<AttachmentTransportCompletion> AttachmentTransportWorker::take_completion()
{
  return impl_->take_completion();
}

bool AttachmentTransportWorker::busy() const
{
  return impl_->busy();
}

}  // namespace restocker_gazebo
