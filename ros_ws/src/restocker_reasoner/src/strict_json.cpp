// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_reasoner/strict_json.hpp"

#include <array>
#include <charconv>
#include <cmath>
#include <cstdio>
#include <string>
#include <system_error>
#include <utility>

namespace restocker_reasoner
{

namespace
{

constexpr char kReplacementCharacter[] = "\xef\xbf\xbd";

[[nodiscard]] bool json_whitespace(char character) noexcept
{
  return character == ' ' || character == '\t' || character == '\n' || character == '\r';
}

[[nodiscard]] bool json_digit(char character) noexcept
{
  return character >= '0' && character <= '9';
}

// Appends one Unicode scalar value as UTF-8. The caller has already rejected surrogates and
// values above U+10FFFF.
void append_utf8(std::string & out, std::uint32_t code_point)
{
  if (code_point < 0x80U) {
    out.push_back(static_cast<char>(code_point));
  } else if (code_point < 0x800U) {
    out.push_back(static_cast<char>(0xC0U | (code_point >> 6U)));
    out.push_back(static_cast<char>(0x80U | (code_point & 0x3FU)));
  } else if (code_point < 0x10000U) {
    out.push_back(static_cast<char>(0xE0U | (code_point >> 12U)));
    out.push_back(static_cast<char>(0x80U | ((code_point >> 6U) & 0x3FU)));
    out.push_back(static_cast<char>(0x80U | (code_point & 0x3FU)));
  } else {
    out.push_back(static_cast<char>(0xF0U | (code_point >> 18U)));
    out.push_back(static_cast<char>(0x80U | ((code_point >> 12U) & 0x3FU)));
    out.push_back(static_cast<char>(0x80U | ((code_point >> 6U) & 0x3FU)));
    out.push_back(static_cast<char>(0x80U | (code_point & 0x3FU)));
  }
}

// A recursive-descent reader over one document. Failures set error_ and unwind; nothing throws,
// since the caller is a coordinator thread.
class StrictJsonReader
{
public:
  StrictJsonReader(std::string_view text, const JsonParseLimits & limits)
  : text_(text), limits_(limits)
  {
  }

  [[nodiscard]] JsonParseResult read()
  {
    if (text_.size() > limits_.maximum_bytes) {
      return {std::nullopt, "the document exceeds the configured byte bound"};
    }
    skip_whitespace();
    if (at_end()) {
      return {std::nullopt, "the document is empty"};
    }
    JsonValue value = read_value(1U);
    if (!error_.empty()) {
      return {std::nullopt, error_};
    }
    skip_whitespace();
    if (!at_end()) {
      return {std::nullopt, "the document carries content after its top-level value"};
    }
    return {std::move(value), {}};
  }

private:
  [[nodiscard]] bool at_end() const noexcept {return position_ >= text_.size();}
  [[nodiscard]] char peek() const noexcept {return text_[position_];}

  void fail(std::string reason)
  {
    if (error_.empty()) {
      error_ = std::move(reason);
    }
  }

  void skip_whitespace()
  {
    while (!at_end() && json_whitespace(peek())) {
      ++position_;
    }
  }

  [[nodiscard]] bool expect(char character)
  {
    if (at_end() || peek() != character) {
      fail(std::string("expected '") + character + "' in the document");
      return false;
    }
    ++position_;
    return true;
  }

  [[nodiscard]] JsonValue read_value(std::size_t depth)
  {
    if (depth > limits_.maximum_depth) {
      fail("the document nests deeper than the configured bound");
      return {};
    }
    if (at_end()) {
      fail("the document ends where a value was expected");
      return {};
    }
    switch (peek()) {
      case '{': return read_object(depth);
      case '[': return read_array(depth);
      case '"': return JsonValue::string(read_string());
      case 't': return read_literal("true", JsonValue::boolean(true));
      case 'f': return read_literal("false", JsonValue::boolean(false));
      case 'n': return read_literal("null", JsonValue::null_value());
      default: return read_number();
    }
  }

  [[nodiscard]] JsonValue read_literal(std::string_view literal, JsonValue value)
  {
    if (text_.substr(position_, literal.size()) != literal) {
      fail("the document carries a token that is not a JSON literal");
      return {};
    }
    position_ += literal.size();
    return value;
  }

  [[nodiscard]] JsonValue read_object(std::size_t depth)
  {
    if (!expect('{')) {
      return {};
    }
    std::map<std::string, JsonValue> members;
    skip_whitespace();
    if (!at_end() && peek() == '}') {
      ++position_;
      return JsonValue::object(std::move(members));
    }
    while (true) {
      skip_whitespace();
      if (at_end() || peek() != '"') {
        fail("an object member name must be a quoted string");
        return {};
      }
      std::string name = read_string();
      if (!error_.empty()) {
        return {};
      }
      skip_whitespace();
      if (!expect(':')) {
        return {};
      }
      skip_whitespace();
      JsonValue value = read_value(depth + 1U);
      if (!error_.empty()) {
        return {};
      }
      // Duplicate keys make the document ambiguous between readers.
      if (!members.emplace(std::move(name), std::move(value)).second) {
        fail("the object repeats a member name");
        return {};
      }
      skip_whitespace();
      if (at_end()) {
        fail("the object is unterminated");
        return {};
      }
      if (peek() == ',') {
        ++position_;
        continue;
      }
      if (peek() == '}') {
        ++position_;
        return JsonValue::object(std::move(members));
      }
      fail("the object carries a token that is neither ',' nor '}'");
      return {};
    }
  }

  [[nodiscard]] JsonValue read_array(std::size_t depth)
  {
    if (!expect('[')) {
      return {};
    }
    std::vector<JsonValue> elements;
    skip_whitespace();
    if (!at_end() && peek() == ']') {
      ++position_;
      return JsonValue::array(std::move(elements));
    }
    while (true) {
      skip_whitespace();
      JsonValue value = read_value(depth + 1U);
      if (!error_.empty()) {
        return {};
      }
      elements.push_back(std::move(value));
      skip_whitespace();
      if (at_end()) {
        fail("the array is unterminated");
        return {};
      }
      if (peek() == ',') {
        ++position_;
        continue;
      }
      if (peek() == ']') {
        ++position_;
        return JsonValue::array(std::move(elements));
      }
      fail("the array carries a token that is neither ',' nor ']'");
      return {};
    }
  }

  [[nodiscard]] std::string read_string()
  {
    if (!expect('"')) {
      return {};
    }
    std::string value;
    while (true) {
      if (at_end()) {
        fail("the string is unterminated");
        return {};
      }
      const char character = text_[position_];
      if (character == '"') {
        ++position_;
        if (!valid_utf8(value)) {
          fail("the string is not valid UTF-8");
          return {};
        }
        return value;
      }
      if (static_cast<unsigned char>(character) < 0x20U) {
        fail("the string carries an unescaped control character");
        return {};
      }
      if (character != '\\') {
        value.push_back(character);
        ++position_;
        continue;
      }
      ++position_;
      if (at_end()) {
        fail("the string ends inside an escape sequence");
        return {};
      }
      const char escape = text_[position_++];
      switch (escape) {
        case '"': value.push_back('"'); break;
        case '\\': value.push_back('\\'); break;
        case '/': value.push_back('/'); break;
        case 'b': value.push_back('\b'); break;
        case 'f': value.push_back('\f'); break;
        case 'n': value.push_back('\n'); break;
        case 'r': value.push_back('\r'); break;
        case 't': value.push_back('\t'); break;
        case 'u': read_unicode_escape(value); break;
        default:
          fail("the string carries an escape sequence JSON does not define");
          return {};
      }
      if (!error_.empty()) {
        return {};
      }
    }
  }

  void read_unicode_escape(std::string & value)
  {
    const auto unit = read_hex_quad();
    if (!unit) {
      return;
    }
    std::uint32_t code_point = *unit;
    if (code_point >= 0xD800U && code_point <= 0xDBFFU) {
      // A high surrogate must be followed by a low surrogate.
      if (text_.substr(position_, 2U) != "\\u") {
        fail("the string carries an unpaired UTF-16 surrogate");
        return;
      }
      position_ += 2U;
      const auto low = read_hex_quad();
      if (!low) {
        return;
      }
      if (*low < 0xDC00U || *low > 0xDFFFU) {
        fail("the string carries an unpaired UTF-16 surrogate");
        return;
      }
      code_point = 0x10000U + ((code_point - 0xD800U) << 10U) + (*low - 0xDC00U);
    } else if (code_point >= 0xDC00U && code_point <= 0xDFFFU) {
      fail("the string carries an unpaired UTF-16 surrogate");
      return;
    }
    append_utf8(value, code_point);
  }

  [[nodiscard]] std::optional<std::uint32_t> read_hex_quad()
  {
    if (text_.size() - position_ < 4U) {
      fail("the string ends inside a \\u escape");
      return std::nullopt;
    }
    std::uint32_t code_point = 0U;
    for (std::size_t index = 0U; index < 4U; ++index) {
      const char digit = text_[position_ + index];
      std::uint32_t nibble = 0U;
      if (json_digit(digit)) {
        nibble = static_cast<std::uint32_t>(digit - '0');
      } else if (digit >= 'a' && digit <= 'f') {
        nibble = static_cast<std::uint32_t>(digit - 'a') + 10U;
      } else if (digit >= 'A' && digit <= 'F') {
        nibble = static_cast<std::uint32_t>(digit - 'A') + 10U;
      } else {
        fail("the string carries a \\u escape that is not four hexadecimal digits");
        return std::nullopt;
      }
      code_point = (code_point << 4U) | nibble;
    }
    position_ += 4U;
    return code_point;
  }

  [[nodiscard]] JsonValue read_number()
  {
    const std::size_t start = position_;
    if (!at_end() && peek() == '-') {
      ++position_;
    }
    if (at_end() || !json_digit(peek())) {
      fail("the document carries a token that is not a JSON value");
      return {};
    }
    if (peek() == '0') {
      ++position_;
      // JSON forbids leading zeros.
      if (!at_end() && json_digit(peek())) {
        fail("the number carries a leading zero");
        return {};
      }
    } else {
      while (!at_end() && json_digit(peek())) {
        ++position_;
      }
    }
    if (!at_end() && peek() == '.') {
      ++position_;
      if (at_end() || !json_digit(peek())) {
        fail("the number has no digits after its decimal point");
        return {};
      }
      while (!at_end() && json_digit(peek())) {
        ++position_;
      }
    }
    if (!at_end() && (peek() == 'e' || peek() == 'E')) {
      ++position_;
      if (!at_end() && (peek() == '+' || peek() == '-')) {
        ++position_;
      }
      if (at_end() || !json_digit(peek())) {
        fail("the number has no digits in its exponent");
        return {};
      }
      while (!at_end() && json_digit(peek())) {
        ++position_;
      }
    }
    const auto literal = text_.substr(start, position_ - start);
    double parsed = 0.0;
    // from_chars is locale-independent; stod would stop at '.' under a comma-decimal locale.
    const auto converted = std::from_chars(
      literal.data(), literal.data() + literal.size(), parsed);
    if (converted.ec == std::errc::result_out_of_range) {
      fail("the number is outside the range this reader can represent");
      return {};
    }
    if (converted.ec != std::errc{} || converted.ptr != literal.data() + literal.size()) {
      fail("the number could not be read exactly");
      return {};
    }
    if (!std::isfinite(parsed)) {
      fail("the number is not finite");
      return {};
    }
    return JsonValue::number(parsed);
  }

  std::string_view text_;
  JsonParseLimits limits_;
  std::size_t position_{0U};
  std::string error_;
};

}  // namespace

JsonValue JsonValue::null_value()
{
  return {};
}

JsonValue JsonValue::boolean(bool value)
{
  JsonValue result;
  result.kind_ = Kind::kBool;
  result.bool_ = value;
  return result;
}

JsonValue JsonValue::number(double value)
{
  JsonValue result;
  result.kind_ = Kind::kNumber;
  result.number_ = value;
  return result;
}

JsonValue JsonValue::string(std::string value)
{
  JsonValue result;
  result.kind_ = Kind::kString;
  result.string_ = std::move(value);
  return result;
}

JsonValue JsonValue::array(std::vector<JsonValue> value)
{
  JsonValue result;
  result.kind_ = Kind::kArray;
  result.array_ = std::move(value);
  return result;
}

JsonValue JsonValue::object(std::map<std::string, JsonValue> value)
{
  JsonValue result;
  result.kind_ = Kind::kObject;
  result.object_ = std::move(value);
  return result;
}

const JsonValue * JsonValue::member(std::string_view name) const noexcept
{
  if (kind_ != Kind::kObject) {
    return nullptr;
  }
  const auto found = object_.find(std::string(name));
  return found == object_.end() ? nullptr : &found->second;
}

JsonParseResult parse_strict_json(std::string_view text, const JsonParseLimits & limits)
{
  StrictJsonReader reader(text, limits);
  return reader.read();
}

bool valid_utf8(std::string_view text) noexcept
{
  std::size_t index = 0U;
  while (index < text.size()) {
    const auto lead = static_cast<unsigned char>(text[index]);
    std::size_t length = 0U;
    std::uint32_t code_point = 0U;
    if (lead < 0x80U) {
      ++index;
      continue;
    }
    if ((lead & 0xE0U) == 0xC0U) {
      length = 2U;
      code_point = lead & 0x1FU;
    } else if ((lead & 0xF0U) == 0xE0U) {
      length = 3U;
      code_point = lead & 0x0FU;
    } else if ((lead & 0xF8U) == 0xF0U) {
      length = 4U;
      code_point = lead & 0x07U;
    } else {
      return false;
    }
    if (index + length > text.size()) {
      return false;
    }
    for (std::size_t offset = 1U; offset < length; ++offset) {
      const auto continuation = static_cast<unsigned char>(text[index + offset]);
      if ((continuation & 0xC0U) != 0x80U) {
        return false;
      }
      code_point = (code_point << 6U) | (continuation & 0x3FU);
    }
    // Reject overlong encodings and surrogates, so two byte strings cannot mean the same text.
    if (length == 2U && code_point < 0x80U) {
      return false;
    }
    if (length == 3U && code_point < 0x800U) {
      return false;
    }
    if (length == 4U && code_point < 0x10000U) {
      return false;
    }
    if (code_point > 0x10FFFFU || (code_point >= 0xD800U && code_point <= 0xDFFFU)) {
      return false;
    }
    index += length;
  }
  return true;
}

std::string json_number(double value, int decimals)
{
  if (!std::isfinite(value)) {
    return "null";
  }
  std::array<char, 64U> buffer{};
  const auto converted = std::to_chars(
    buffer.data(), buffer.data() + buffer.size(), value, std::chars_format::fixed, decimals);
  if (converted.ec != std::errc{}) {
    return "null";
  }
  return std::string(buffer.data(), converted.ptr);
}

std::string json_quote(std::string_view text)
{
  std::string out;
  out.reserve(text.size() + 2U);
  out.push_back('"');
  std::size_t index = 0U;
  while (index < text.size()) {
    const auto byte = static_cast<unsigned char>(text[index]);
    if (byte < 0x20U) {
      switch (text[index]) {
        case '\b': out += "\\b"; break;
        case '\f': out += "\\f"; break;
        case '\n': out += "\\n"; break;
        case '\r': out += "\\r"; break;
        case '\t': out += "\\t"; break;
        default:
          {
            std::array<char, 8> escape{};
            std::snprintf(escape.data(), escape.size(), "\\u%04x", static_cast<unsigned>(byte));
            out += escape.data();
            break;
          }
      }
      ++index;
      continue;
    }
    if (byte == '"') {
      out += "\\\"";
      ++index;
      continue;
    }
    if (byte == '\\') {
      out += "\\\\";
      ++index;
      continue;
    }
    if (byte < 0x80U) {
      out.push_back(text[index]);
      ++index;
      continue;
    }
    // Copy well-formed UTF-8 sequences and substitute the rest, so the output is always valid JSON.
    std::size_t length = 1U;
    if ((byte & 0xE0U) == 0xC0U) {
      length = 2U;
    } else if ((byte & 0xF0U) == 0xE0U) {
      length = 3U;
    } else if ((byte & 0xF8U) == 0xF0U) {
      length = 4U;
    } else {
      out += kReplacementCharacter;
      ++index;
      continue;
    }
    const auto candidate = text.substr(index, length);
    if (candidate.size() != length || !valid_utf8(candidate)) {
      out += kReplacementCharacter;
      ++index;
      continue;
    }
    out.append(candidate);
    index += length;
  }
  out.push_back('"');
  return out;
}

}  // namespace restocker_reasoner
