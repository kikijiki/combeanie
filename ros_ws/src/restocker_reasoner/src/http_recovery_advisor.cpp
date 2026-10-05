// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_reasoner/http_recovery_advisor.hpp"

#include <netdb.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cerrno>
#include <charconv>
#include <cstring>
#include <exception>
#include <optional>
#include <string>
#include <utility>

#include "restocker_reasoner/strict_json.hpp"

namespace restocker_reasoner
{

namespace
{

// The largest single poll wait, so cancel() and shutdown() are noticed promptly even when the
// call's own deadline is seconds away.
constexpr std::chrono::milliseconds kPollSlice{100};

// The chat envelope may carry long usage and timing members this client ignores, so it is read
// with looser bounds than the answer itself.
constexpr JsonParseLimits kEnvelopeLimits{262144U, 32U};

[[nodiscard]] std::string lowercase(std::string_view text)
{
  std::string out;
  out.reserve(text.size());
  for (const char character : text) {
    out.push_back(
      static_cast<char>(std::tolower(static_cast<unsigned char>(character))));
  }
  return out;
}

[[nodiscard]] std::string trimmed(std::string_view text)
{
  // The same character set at both ends. Two different sets let `last` be npos while `first` is
  // not, and the length below then underflows to SIZE_MAX.
  constexpr std::string_view kSpace = " \t\r";
  const auto first = text.find_first_not_of(kSpace);
  if (first == std::string_view::npos) {
    return {};
  }
  const auto last = text.find_last_not_of(kSpace);
  return std::string(text.substr(first, last - first + 1U));
}

// A socket that closes itself, so no error path can leak a descriptor.
class OwnedSocket
{
public:
  OwnedSocket() = default;
  explicit OwnedSocket(int descriptor)
  : descriptor_(descriptor)
  {
  }
  ~OwnedSocket() {reset();}

  OwnedSocket(const OwnedSocket &) = delete;
  OwnedSocket & operator=(const OwnedSocket &) = delete;
  OwnedSocket(OwnedSocket && other) noexcept
  : descriptor_(other.descriptor_)
  {
    other.descriptor_ = -1;
  }
  OwnedSocket & operator=(OwnedSocket && other) noexcept
  {
    if (this != &other) {
      reset();
      descriptor_ = other.descriptor_;
      other.descriptor_ = -1;
    }
    return *this;
  }

  [[nodiscard]] int get() const noexcept {return descriptor_;}
  [[nodiscard]] bool valid() const noexcept {return descriptor_ >= 0;}

  void reset() noexcept
  {
    if (descriptor_ >= 0) {
      ::close(descriptor_);
      descriptor_ = -1;
    }
  }

private:
  int descriptor_{-1};
};

[[nodiscard]] std::string system_reason(std::string_view what, int code)
{
  // std::strerror rather than strerror_r, which has two incompatible signatures depending on
  // feature-test macros. glibc's buffer is per-thread, and the string only reaches audit records.
  const char * message = std::strerror(code);
  return std::string(what) + ": " +
         (message == nullptr ? "errno " + std::to_string(code) : std::string(message));
}

[[nodiscard]] int remaining_poll_ms(std::chrono::steady_clock::time_point deadline)
{
  const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
    deadline - std::chrono::steady_clock::now());
  if (remaining.count() <= 0) {
    return 0;
  }
  return static_cast<int>(std::min(remaining, kPollSlice).count());
}

[[nodiscard]] bool past(std::chrono::steady_clock::time_point deadline)
{
  return std::chrono::steady_clock::now() >= deadline;
}

}  // namespace

std::string HttpRecoveryAdvisorConfig::describe() const
{
  return "http://" + host + ":" + std::to_string(port) + path +
         (model.empty() ? std::string() : " model=" + model);
}

std::string build_http_post_request(
  const HttpRecoveryAdvisorConfig & config, const std::string & body)
{
  std::string request;
  request += "POST " + config.path + " HTTP/1.1\r\n";
  request += "Host: " + config.host + ":" + std::to_string(config.port) + "\r\n";
  request += "User-Agent: restocker_reasoner/1\r\n";
  request += "Accept: application/json\r\n";
  request += "Content-Type: application/json\r\n";
  request += "Content-Length: " + std::to_string(body.size()) + "\r\n";
  // One request per connection: no pool, so no state a backend restart can leave stale.
  request += "Connection: close\r\n";
  request += "\r\n";
  request += body;
  return request;
}

HttpResponseParts parse_http_response(std::string_view raw, std::size_t maximum_body_bytes)
{
  HttpResponseParts parts;
  const auto header_end = raw.find("\r\n\r\n");
  if (header_end == std::string_view::npos) {
    parts.error = "the response carries no complete header block";
    return parts;
  }
  const auto head = raw.substr(0U, header_end);
  auto body = raw.substr(header_end + 4U);

  const auto status_end = head.find("\r\n");
  const auto status_line = head.substr(
    0U, status_end == std::string_view::npos ? head.size() :
    status_end);
  if (!status_line.starts_with("HTTP/1.")) {
    parts.error = "the response does not begin with an HTTP/1.x status line";
    return parts;
  }
  const auto code_start = status_line.find(' ');
  if (code_start == std::string_view::npos || status_line.size() < code_start + 4U) {
    parts.error = "the response status line carries no status code";
    return parts;
  }
  const auto code_text = status_line.substr(code_start + 1U, 3U);
  int status_code = 0;
  const auto converted = std::from_chars(
    code_text.data(), code_text.data() + code_text.size(), status_code);
  if (converted.ec != std::errc{} || converted.ptr != code_text.data() + code_text.size()) {
    parts.error = "the response status line carries no status code";
    return parts;
  }
  parts.status_code = status_code;

  bool chunked = false;
  std::optional<std::size_t> content_length;
  std::size_t cursor = (status_end == std::string_view::npos) ? head.size() : status_end + 2U;
  while (cursor < head.size()) {
    const auto line_end = head.find("\r\n", cursor);
    const auto line = head.substr(
      cursor, line_end == std::string_view::npos ? head.size() - cursor : line_end - cursor);
    cursor = (line_end == std::string_view::npos) ? head.size() : line_end + 2U;
    const auto colon = line.find(':');
    if (colon == std::string_view::npos) {
      continue;
    }
    const auto name = lowercase(line.substr(0U, colon));
    const auto value = trimmed(line.substr(colon + 1U));
    if (name == "transfer-encoding" && lowercase(value).find("chunked") != std::string::npos) {
      chunked = true;
    } else if (name == "content-length") {
      std::size_t length = 0U;
      const auto parsed = std::from_chars(
        value.data(), value.data() + value.size(), length);
      if (parsed.ec != std::errc{} || parsed.ptr != value.data() + value.size()) {
        parts.error = "the response carries a Content-Length that is not a number";
        return parts;
      }
      content_length = length;
    }
  }

  if (chunked) {
    std::string decoded;
    std::size_t position = 0U;
    while (true) {
      const auto line_end = body.find("\r\n", position);
      if (line_end == std::string_view::npos) {
        parts.error = "the chunked response ends inside a chunk header";
        return parts;
      }
      auto size_text = body.substr(position, line_end - position);
      // A chunk extension follows a semicolon and is not something this client uses.
      if (const auto semicolon = size_text.find(';'); semicolon != std::string_view::npos) {
        size_text = size_text.substr(0U, semicolon);
      }
      std::size_t chunk_size = 0U;
      const auto parsed = std::from_chars(
        size_text.data(), size_text.data() + size_text.size(), chunk_size, 16);
      if (parsed.ec != std::errc{} || parsed.ptr != size_text.data() + size_text.size()) {
        parts.error = "the chunked response carries a chunk size that is not hexadecimal";
        return parts;
      }
      position = line_end + 2U;
      if (chunk_size == 0U) {
        break;
      }
      // Bounds are written as subtractions, not additions: a backend-chosen chunk size near
      // SIZE_MAX would make `position + chunk_size` wrap, pass the bound and move `position`
      // backwards, hanging the worker and, through the destructor's join, coordinator shutdown.
      if (chunk_size > body.size() - position) {
        parts.error = "the chunked response ends inside a chunk body";
        return parts;
      }
      if (chunk_size > maximum_body_bytes - decoded.size()) {
        parts.error = "the response body exceeds the configured byte bound";
        return parts;
      }
      decoded.append(body.substr(position, chunk_size));
      position += chunk_size;
      if (body.substr(position, 2U) != "\r\n") {
        parts.error = "a chunk body is not terminated by CRLF";
        return parts;
      }
      position += 2U;
    }
    parts.body = std::move(decoded);
    return parts;
  }

  if (content_length) {
    if (*content_length > maximum_body_bytes) {
      parts.error = "the response body exceeds the configured byte bound";
      return parts;
    }
    if (body.size() < *content_length) {
      parts.error = "the response body is shorter than its Content-Length";
      return parts;
    }
    body = body.substr(0U, *content_length);
  }
  if (body.size() > maximum_body_bytes) {
    parts.error = "the response body exceeds the configured byte bound";
    return parts;
  }
  parts.body = std::string(body);
  return parts;
}

std::string build_chat_completion_body(
  const RecoveryQuery & query, const HttpRecoveryAdvisorConfig & config)
{
  std::string body = "{";
  body += "\"model\":" + json_quote(config.model);
  body += ",\"messages\":[";
  body += "{\"role\":\"system\",\"content\":" +
    json_quote(render_recovery_system_prompt(query)) + "},";
  body += "{\"role\":\"user\",\"content\":" +
    json_quote(render_recovery_user_prompt(query)) + "}";
  body += "]";
  body += ",\"temperature\":" + json_number(config.temperature, 2);
  body += ",\"max_tokens\":" + std::to_string(config.maximum_output_tokens);
  body += ",\"stream\":false";
  // A reasoning model would spend the whole deadline thinking; the classification is a lookup.
  body += ",\"chat_template_kwargs\":{\"enable_thinking\":false}";
  if (config.constrained_decoding) {
    body += ",\"response_format\":{\"type\":\"json_schema\",\"json_schema\":{\"name\":"
      "\"restocker_recovery_advice\",\"strict\":true,\"schema\":" +
      recovery_advice_response_schema() + "}}";
  }
  body += "}";
  return body;
}

ChatCompletionContent extract_chat_completion_content(std::string_view body)
{
  const auto parsed = parse_strict_json(body, kEnvelopeLimits);
  if (!parsed.value) {
    return {{}, "the backend envelope is not strict JSON: " + parsed.error};
  }
  const auto * choices = parsed.value->member("choices");
  if (choices == nullptr || !choices->is_array() || choices->as_array().empty()) {
    return {{}, "the backend envelope carries no choices"};
  }
  const auto * message = choices->as_array().front().member("message");
  if (message == nullptr || !message->is_object()) {
    return {{}, "the backend envelope carries no message"};
  }
  const auto * content = message->member("content");
  if (content == nullptr || !content->is_string()) {
    return {{}, "the backend envelope carries no message content"};
  }
  if (content->as_string().empty()) {
    return {{}, "the backend returned an empty message content"};
  }
  return {content->as_string(), {}};
}

HttpRecoveryAdvisor::HttpRecoveryAdvisor(HttpRecoveryAdvisorConfig config)
: config_(std::move(config))
{
  if (config_.deadline.count() <= 0) {
    config_.deadline = std::chrono::milliseconds(1);
  }
  worker_ = std::thread([this]() {run_worker();});
}

HttpRecoveryAdvisor::~HttpRecoveryAdvisor()
{
  shutdown();
}

bool HttpRecoveryAdvisor::ready() const
{
  if (stopping_.load()) {
    return false;
  }
  const std::lock_guard<std::mutex> guard(mutex_);
  return !circuit_open_until_ || std::chrono::steady_clock::now() >= *circuit_open_until_;
}

RecoveryAdviceSubmitResult HttpRecoveryAdvisor::submit(
  RecoveryQuery query, CompletionCallback callback)
{
  if (!callback) {
    return {RecoveryAdviceSubmitStatus::kInvalidRequest, "a completion callback is required"};
  }
  if (query.request_id.empty()) {
    return {RecoveryAdviceSubmitStatus::kInvalidRequest, "a request id is required"};
  }
  if (query.permitted_primitives.empty()) {
    return {
      RecoveryAdviceSubmitStatus::kInvalidRequest,
      "a question with no permitted primitive has no answer worth asking for"};
  }
  if (stopping_.load()) {
    return {RecoveryAdviceSubmitStatus::kUnavailable, "the advisory client is shutting down"};
  }

  {
    std::unique_lock<std::mutex> guard(mutex_);
    // Re-read under the lock: the unlocked check races a concurrent shutdown, and a submission
    // accepted after the worker returned would never complete, breaking the port's
    // one-completion-per-acceptance contract. stopping_ only goes false to true and the worker
    // returns only after it is true, so a locked read of false proves the worker is still there.
    if (stopping_.load()) {
      return {RecoveryAdviceSubmitStatus::kUnavailable, "the advisory client is shutting down"};
    }
    if (circuit_open_until_ && std::chrono::steady_clock::now() < *circuit_open_until_) {
      return {
        RecoveryAdviceSubmitStatus::kUnavailable,
        "the advisory client stopped calling after consecutive failures"};
    }
    if (pending_ || busy_) {
      return {RecoveryAdviceSubmitStatus::kBusy, "a question is already outstanding"};
    }
    cancel_requested_.store(false);
    pending_.emplace(PendingQuery{std::move(query), std::move(callback)});
  }
  work_available_.notify_one();
  return {RecoveryAdviceSubmitStatus::kAccepted, {}};
}

void HttpRecoveryAdvisor::cancel() noexcept
{
  cancel_requested_.store(true);
}

void HttpRecoveryAdvisor::shutdown() noexcept
{
  if (stopping_.exchange(true)) {
    return;
  }
  cancel_requested_.store(true);
  // Taking the mutex before notifying makes the stopping_ store visible to a worker that has
  // evaluated its wait predicate but not yet blocked. Without it that worker misses the
  // notification and the join below never returns.
  try {
    const std::lock_guard<std::mutex> guard(mutex_);
  } catch (...) {
    // Nothing may throw out of a noexcept shutdown; the notification below is the fallback.
  }
  work_available_.notify_all();
  if (worker_.joinable()) {
    worker_.join();
  }
}

std::size_t HttpRecoveryAdvisor::consecutive_failures() const
{
  const std::lock_guard<std::mutex> guard(mutex_);
  return consecutive_failures_;
}

void HttpRecoveryAdvisor::run_worker() noexcept
{
  while (true) {
    PendingQuery pending;
    {
      std::unique_lock<std::mutex> guard(mutex_);
      work_available_.wait(guard, [this]() {return pending_.has_value() || stopping_.load();});
      if (!pending_) {
        if (stopping_.load()) {
          return;
        }
        continue;
      }
      pending = std::move(*pending_);
      pending_.reset();
      busy_ = true;
    }

    auto completion = perform_call(pending.query);
    // The busy flag and failure count are published together so a submission arriving right as
    // this call ends is never judged against a stale circuit state.
    record_call_result(completion.responded);
    try {
      pending.callback(std::move(completion));
    } catch (...) {
      // A throwing sink must not kill the worker, or later questions would never complete.
    }
  }
}

void HttpRecoveryAdvisor::record_call_result(bool succeeded)
{
  const std::lock_guard<std::mutex> guard(mutex_);
  busy_ = false;
  if (succeeded) {
    consecutive_failures_ = 0U;
    circuit_open_until_.reset();
    return;
  }
  ++consecutive_failures_;
  if (consecutive_failures_ >= config_.consecutive_failures_before_open) {
    circuit_open_until_ = std::chrono::steady_clock::now() + config_.circuit_cooldown;
  }
}

RecoveryAdviceCompletion HttpRecoveryAdvisor::perform_call(const RecoveryQuery & query) noexcept
{
  RecoveryAdviceCompletion completion;
  completion.request_id = query.request_id;
  completion.goal_generation = query.goal_generation;
  completion.backend = config_.describe();

  const auto started = std::chrono::steady_clock::now();
  const auto deadline = started + config_.deadline;
  const auto finish = [&completion, started](std::string detail, std::string document) {
    completion.latency = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - started);
    completion.transport_detail = std::move(detail);
    completion.document = std::move(document);
    completion.responded = completion.transport_detail.empty();
    return completion;
  };

  try {
    const auto body = build_chat_completion_body(query, config_);
    const auto request = build_http_post_request(config_, body);
    const auto response = exchange(request, deadline);
    if (!response.error.empty()) {
      return finish(response.error, {});
    }
    if (response.status_code != 200) {
      return finish(
        "the backend answered with HTTP status " + std::to_string(response.status_code), {});
    }
    const auto content = extract_chat_completion_content(response.body);
    if (!content.error.empty()) {
      return finish(content.error, {});
    }
    return finish({}, content.content);
  } catch (const std::exception & error) {
    return finish(std::string("the advisory call raised: ") + error.what(), {});
  } catch (...) {
    return finish("the advisory call raised an unknown exception", {});
  }
}

HttpResponseParts HttpRecoveryAdvisor::exchange(
  const std::string & request, std::chrono::steady_clock::time_point deadline) const
{
  HttpResponseParts parts;

  ::addrinfo hints{};
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;
  hints.ai_flags = AI_NUMERICSERV;
  ::addrinfo * resolved = nullptr;
  const auto service = std::to_string(config_.port);
  const int resolve = ::getaddrinfo(config_.host.c_str(), service.c_str(), &hints, &resolved);
  if (resolve != 0) {
    parts.error = std::string("the advisory endpoint did not resolve: ") +
      ::gai_strerror(resolve);
    return parts;
  }
  if (resolved == nullptr) {
    parts.error = "the advisory endpoint resolved to no addresses";
    return parts;
  }

  OwnedSocket socket;
  std::string connect_error = "the advisory endpoint offered no usable address";
  for (const ::addrinfo * candidate = resolved; candidate != nullptr;
    candidate = candidate->ai_next)
  {
    OwnedSocket attempt(
      ::socket(
        candidate->ai_family, candidate->ai_socktype | SOCK_NONBLOCK | SOCK_CLOEXEC,
        candidate->ai_protocol));
    if (!attempt.valid()) {
      connect_error = system_reason("the advisory client could not open a socket", errno);
      continue;
    }
    if (::connect(attempt.get(), candidate->ai_addr, candidate->ai_addrlen) == 0) {
      socket = std::move(attempt);
      break;
    }
    if (errno != EINPROGRESS) {
      connect_error = system_reason("the advisory endpoint refused the connection", errno);
      continue;
    }
    bool connected = false;
    while (!past(deadline) && !cancel_requested_.load()) {
      ::pollfd waiting{attempt.get(), POLLOUT, 0};
      const int ready = ::poll(&waiting, 1U, remaining_poll_ms(deadline));
      if (ready < 0) {
        if (errno == EINTR) {
          continue;
        }
        connect_error = system_reason("the advisory connection could not be waited on", errno);
        break;
      }
      if (ready == 0) {
        continue;
      }
      int error_code = 0;
      ::socklen_t length = sizeof(error_code);
      if (::getsockopt(attempt.get(), SOL_SOCKET, SO_ERROR, &error_code, &length) != 0) {
        connect_error = system_reason("the advisory connection state could not be read", errno);
        break;
      }
      if (error_code != 0) {
        connect_error =
          system_reason("the advisory endpoint refused the connection", error_code);
        break;
      }
      connected = true;
      break;
    }
    if (connected) {
      socket = std::move(attempt);
      break;
    }
    if (past(deadline)) {
      connect_error = "the advisory call passed its deadline while connecting";
      break;
    }
    if (cancel_requested_.load()) {
      connect_error = "the advisory call was abandoned while connecting";
      break;
    }
  }
  ::freeaddrinfo(resolved);
  if (!socket.valid()) {
    parts.error = connect_error;
    return parts;
  }

  std::size_t sent = 0U;
  while (sent < request.size()) {
    if (past(deadline)) {
      parts.error = "the advisory call passed its deadline while sending";
      return parts;
    }
    if (cancel_requested_.load()) {
      parts.error = "the advisory call was abandoned while sending";
      return parts;
    }
    ::pollfd waiting{socket.get(), POLLOUT, 0};
    const int ready = ::poll(&waiting, 1U, remaining_poll_ms(deadline));
    if (ready < 0) {
      if (errno == EINTR) {
        continue;
      }
      parts.error = system_reason("the advisory request could not be sent", errno);
      return parts;
    }
    if (ready == 0) {
      continue;
    }
    // MSG_NOSIGNAL: a closed connection must fail this call, not signal the process.
    const auto written = ::send(
      socket.get(), request.data() + sent, request.size() - sent, MSG_NOSIGNAL);
    if (written < 0) {
      if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) {
        continue;
      }
      parts.error = system_reason("the advisory request could not be sent", errno);
      return parts;
    }
    sent += static_cast<std::size_t>(written);
  }

  // The answer is complete when the server closes (`Connection: close`). A server that holds the
  // connection open runs out this call's deadline and completes as not-responded.
  std::string raw;
  std::array<char, 8192U> buffer{};
  while (true) {
    if (past(deadline)) {
      parts.error = "the advisory call passed its deadline while waiting for an answer";
      return parts;
    }
    if (cancel_requested_.load()) {
      parts.error = "the advisory call was abandoned while waiting for an answer";
      return parts;
    }
    ::pollfd waiting{socket.get(), POLLIN, 0};
    const int ready = ::poll(&waiting, 1U, remaining_poll_ms(deadline));
    if (ready < 0) {
      if (errno == EINTR) {
        continue;
      }
      parts.error = system_reason("the advisory answer could not be read", errno);
      return parts;
    }
    if (ready == 0) {
      continue;
    }
    const auto read = ::recv(socket.get(), buffer.data(), buffer.size(), 0);
    if (read < 0) {
      if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) {
        continue;
      }
      parts.error = system_reason("the advisory answer could not be read", errno);
      return parts;
    }
    if (read == 0) {
      break;
    }
    if (raw.size() + static_cast<std::size_t>(read) >
      config_.maximum_response_bytes + 8192U)
    {
      parts.error = "the advisory answer exceeds the configured byte bound";
      return parts;
    }
    raw.append(buffer.data(), static_cast<std::size_t>(read));
  }

  return parse_http_response(raw, config_.maximum_response_bytes);
}

}  // namespace restocker_reasoner
