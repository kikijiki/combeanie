// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <thread>

#include "restocker_reasoner/recovery_advisor_port.hpp"

namespace restocker_reasoner
{

struct HttpRecoveryAdvisorConfig
{
  std::string host{"127.0.0.1"};
  std::uint16_t port{8080U};
  std::string path{"/v1/chat/completions"};
  // Empty asks the backend for whatever model it has loaded.
  std::string model;
  // Hard bound on one call, on the steady clock from when the worker picks the question up.
  // Expiry produces the same completion as an absent backend.
  std::chrono::milliseconds deadline{std::chrono::milliseconds(4000)};
  std::size_t maximum_response_bytes{262144U};
  double temperature{0.0};
  int maximum_output_tokens{512};
  // Ask the backend to constrain decoding to the response schema. Validation is identical either
  // way.
  bool constrained_decoding{true};
  // Consecutive failed calls before the client stops calling, and the wait before trying again.
  std::size_t consecutive_failures_before_open{3U};
  std::chrono::milliseconds circuit_cooldown{std::chrono::milliseconds(30000)};

  [[nodiscard]] std::string describe() const;
};

// The pieces of an HTTP response this client understands, separate from the socket so the wire
// format can be tested without a server.
struct HttpResponseParts
{
  int status_code{0};
  std::string body;
  // Empty exactly when the response was understood.
  std::string error;
};

[[nodiscard]] std::string build_http_post_request(
  const HttpRecoveryAdvisorConfig & config, const std::string & body);

// Understands a status line, headers, and a body delimited by Content-Length or chunked transfer
// encoding. Anything else is an error.
[[nodiscard]] HttpResponseParts parse_http_response(
  std::string_view raw, std::size_t maximum_body_bytes);

[[nodiscard]] std::string build_chat_completion_body(
  const RecoveryQuery & query, const HttpRecoveryAdvisorConfig & config);

struct ChatCompletionContent
{
  std::string content;
  // Empty exactly when content was found.
  std::string error;
};

// Pulls choices[0].message.content out of an OpenAI-compatible envelope. Unused members are
// ignored; the ones read are checked strictly.
[[nodiscard]] ChatCompletionContent extract_chat_completion_content(std::string_view body);

// An optional advisory client for an OpenAI-compatible chat backend.
//
// All blocking network operations run on one worker thread, so the coordinator's executor never
// waits on a socket. One question may be outstanding at a time. Every call has a steady-clock
// deadline. A timeout, unreachable backend, refused connection, non-200 status or unreadable body
// all produce the same completion: no document and a sentence saying why.
class HttpRecoveryAdvisor final : public RecoveryAdvisorPort
{
public:
  explicit HttpRecoveryAdvisor(HttpRecoveryAdvisorConfig config = {});
  ~HttpRecoveryAdvisor() override;

  HttpRecoveryAdvisor(const HttpRecoveryAdvisor &) = delete;
  HttpRecoveryAdvisor & operator=(const HttpRecoveryAdvisor &) = delete;
  HttpRecoveryAdvisor(HttpRecoveryAdvisor &&) = delete;
  HttpRecoveryAdvisor & operator=(HttpRecoveryAdvisor &&) = delete;

  [[nodiscard]] bool ready() const override;

  [[nodiscard]] RecoveryAdviceSubmitResult submit(
    RecoveryQuery query, CompletionCallback callback) override;

  void cancel() noexcept override;

  // Stop the worker. Idempotent; the destructor calls it.
  void shutdown() noexcept;

  [[nodiscard]] std::size_t consecutive_failures() const;

private:
  struct PendingQuery
  {
    RecoveryQuery query;
    CompletionCallback callback;
  };

  void run_worker() noexcept;
  // Runs on the worker thread. Never throws; every failure becomes a not-responded completion.
  [[nodiscard]] RecoveryAdviceCompletion perform_call(const RecoveryQuery & query) noexcept;
  // Returns the response body, or leaves error non-empty. Bounded by the deadline throughout.
  [[nodiscard]] HttpResponseParts exchange(
    const std::string & request, std::chrono::steady_clock::time_point deadline) const;
  void record_call_result(bool succeeded);

  HttpRecoveryAdvisorConfig config_;

  mutable std::mutex mutex_;
  std::condition_variable work_available_;
  std::optional<PendingQuery> pending_;
  bool busy_{false};
  std::size_t consecutive_failures_{0U};
  std::optional<std::chrono::steady_clock::time_point> circuit_open_until_;

  std::atomic<bool> stopping_{false};
  std::atomic<bool> cancel_requested_{false};
  std::thread worker_;
};

}  // namespace restocker_reasoner
