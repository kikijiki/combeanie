// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace restocker_reasoner
{

// A JSON reader for text this process did not write.
//
// Model output is untrusted, so this parser accepts exactly RFC 8259 and rejects every
// convenience a forgiving parser allows. Comments, trailing commas, single quotes, unquoted keys,
// NaN and Infinity, leading plus signs, leading zeros, raw control characters inside strings, lone
// surrogates, invalid UTF-8, duplicate object keys and trailing content are all errors. Depth and
// byte count are bounded so a hostile document cannot exhaust the stack or memory of the parsing
// thread.
//
// yaml-cpp is not used because YAML accepts a strict superset of JSON: `{primitive: abandon_task}`
// is valid YAML and invalid JSON, and a reject-by-default schema cannot sit on a reader that
// accepts more than the contract allows.
class JsonValue
{
public:
  enum class Kind : std::uint8_t
  {
    kNull,
    kBool,
    kNumber,
    kString,
    kArray,
    kObject,
  };

  JsonValue() = default;

  [[nodiscard]] static JsonValue null_value();
  [[nodiscard]] static JsonValue boolean(bool value);
  [[nodiscard]] static JsonValue number(double value);
  [[nodiscard]] static JsonValue string(std::string value);
  [[nodiscard]] static JsonValue array(std::vector<JsonValue> value);
  [[nodiscard]] static JsonValue object(std::map<std::string, JsonValue> value);

  [[nodiscard]] Kind kind() const noexcept {return kind_;}
  [[nodiscard]] bool is_null() const noexcept {return kind_ == Kind::kNull;}
  [[nodiscard]] bool is_bool() const noexcept {return kind_ == Kind::kBool;}
  [[nodiscard]] bool is_number() const noexcept {return kind_ == Kind::kNumber;}
  [[nodiscard]] bool is_string() const noexcept {return kind_ == Kind::kString;}
  [[nodiscard]] bool is_array() const noexcept {return kind_ == Kind::kArray;}
  [[nodiscard]] bool is_object() const noexcept {return kind_ == Kind::kObject;}

  // Each accessor returns the type's zero value when the kind does not match. Schema validation
  // checks the kind first in every case.
  [[nodiscard]] bool as_bool() const noexcept {return bool_;}
  [[nodiscard]] double as_number() const noexcept {return number_;}
  [[nodiscard]] const std::string & as_string() const noexcept {return string_;}
  [[nodiscard]] const std::vector<JsonValue> & as_array() const noexcept {return array_;}
  [[nodiscard]] const std::map<std::string, JsonValue> & as_object() const noexcept
  {
    return object_;
  }

  // Null when this value is not an object or the member is absent.
  [[nodiscard]] const JsonValue * member(std::string_view name) const noexcept;

private:
  Kind kind_{Kind::kNull};
  bool bool_{false};
  double number_{0.0};
  std::string string_;
  std::vector<JsonValue> array_;
  std::map<std::string, JsonValue> object_;
};

struct JsonParseLimits
{
  // A recommendation is a handful of short scalars; the chat backend's transport envelope is
  // larger, so the two call sites configure different bounds.
  std::size_t maximum_bytes{65536U};
  std::size_t maximum_depth{8U};
};

struct JsonParseResult
{
  std::optional<JsonValue> value;
  // Empty exactly when value holds a document.
  std::string error;
};

[[nodiscard]] JsonParseResult parse_strict_json(
  std::string_view text, const JsonParseLimits & limits = {});

// Quote and escape one string as a JSON string literal. Control characters, quotes and
// backslashes are escaped; any byte sequence that is not valid UTF-8 is replaced with U+FFFD so
// the audit log stays parseable no matter what a backend returned.
[[nodiscard]] std::string json_quote(std::string_view text);

// True when the bytes form well-formed UTF-8 with no overlong forms, surrogates or values above
// U+10FFFF.
[[nodiscard]] bool valid_utf8(std::string_view text) noexcept;

// One finite double as a JSON number with a fixed number of decimals.
//
// Locale-independent, unlike printf and iostreams: setlocale anywhere in the process can make
// "%.4f" produce "0,5000", which is not a JSON number. Every number this package writes to the
// wire or audit log goes through here. A non-finite value becomes "null".
[[nodiscard]] std::string json_number(double value, int decimals);

}  // namespace restocker_reasoner
