// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <clocale>
#include <limits>
#include <string>

#include "restocker_reasoner/strict_json.hpp"

namespace restocker_reasoner
{

namespace
{

[[nodiscard]] bool accepts(const std::string & document)
{
  return parse_strict_json(document).value.has_value();
}

}  // namespace

TEST(StrictJson, ReadsTheScalarsTheContractUses)
{
  const auto parsed = parse_strict_json(
    R"({"a":"text","b":1.5,"c":true,"d":null,"e":-2e3,"f":0})");
  ASSERT_TRUE(parsed.value) << parsed.error;
  ASSERT_TRUE(parsed.value->is_object());
  EXPECT_EQ(parsed.value->member("a")->as_string(), "text");
  EXPECT_DOUBLE_EQ(parsed.value->member("b")->as_number(), 1.5);
  EXPECT_TRUE(parsed.value->member("c")->as_bool());
  EXPECT_TRUE(parsed.value->member("d")->is_null());
  EXPECT_DOUBLE_EQ(parsed.value->member("e")->as_number(), -2000.0);
  EXPECT_DOUBLE_EQ(parsed.value->member("f")->as_number(), 0.0);
}

TEST(StrictJson, ReadsNestedArraysAndObjects)
{
  const auto parsed = parse_strict_json(R"({"choices":[{"message":{"content":"x"}}]})");
  ASSERT_TRUE(parsed.value) << parsed.error;
  const auto * choices = parsed.value->member("choices");
  ASSERT_NE(choices, nullptr);
  ASSERT_TRUE(choices->is_array());
  ASSERT_EQ(choices->as_array().size(), 1U);
  EXPECT_EQ(
    choices->as_array().front().member("message")->member("content")->as_string(), "x");
}

TEST(StrictJson, DecodesEscapesAndSurrogatePairs)
{
  const auto parsed = parse_strict_json(R"({"a":"line\nbreak \u00e9 \ud83d\ude00"})");
  ASSERT_TRUE(parsed.value) << parsed.error;
  const auto & text = parsed.value->member("a")->as_string();
  EXPECT_NE(text.find('\n'), std::string::npos);
  EXPECT_NE(text.find("\xc3\xa9"), std::string::npos);
  EXPECT_NE(text.find("\xf0\x9f\x98\x80"), std::string::npos);
}

TEST(StrictJson, MemberLookupOnANonObjectIsNull)
{
  const auto parsed = parse_strict_json("[1,2]");
  ASSERT_TRUE(parsed.value) << parsed.error;
  EXPECT_EQ(parsed.value->member("a"), nullptr);
}

// Each of these is a document some forgiving parser would accept.
TEST(StrictJson, RejectsEverythingOutsideTheJsonGrammar)
{
  EXPECT_FALSE(accepts(""));
  EXPECT_FALSE(accepts("   "));
  EXPECT_FALSE(accepts("{primitive: \"abandon_task\"}")) << "unquoted keys are YAML, not JSON";
  EXPECT_FALSE(accepts("{'a':1}")) << "single quotes are not JSON strings";
  EXPECT_FALSE(accepts("{\"a\":1,}")) << "a trailing comma";
  EXPECT_FALSE(accepts("{\"a\":1} // comment")) << "a comment";
  EXPECT_FALSE(accepts("{\"a\":1}{\"b\":2}")) << "two documents";
  EXPECT_FALSE(accepts("{\"a\":1}trailing")) << "content after the document";
  EXPECT_FALSE(accepts("{\"a\":NaN}"));
  EXPECT_FALSE(accepts("{\"a\":Infinity}"));
  EXPECT_FALSE(accepts("{\"a\":+1}"));
  EXPECT_FALSE(accepts("{\"a\":01}")) << "a leading zero";
  EXPECT_FALSE(accepts("{\"a\":1.}"));
  EXPECT_FALSE(accepts("{\"a\":.5}"));
  EXPECT_FALSE(accepts("{\"a\":1e}"));
  EXPECT_FALSE(accepts("{\"a\":TRUE}"));
  EXPECT_FALSE(accepts("{\"a\":\"unterminated}"));
  EXPECT_FALSE(accepts("{\"a\":\"bad \\x escape\"}"));
  EXPECT_FALSE(accepts("{\"a\":\"\\u00\"}"));
  EXPECT_FALSE(accepts("{\"a\":\"\\ud83d\"}")) << "an unpaired high surrogate";
  EXPECT_FALSE(accepts("{\"a\":\"\\ude00\"}")) << "an unpaired low surrogate";
  EXPECT_FALSE(accepts("{\"a\":\"raw\ncontrol\"}"));
  EXPECT_FALSE(accepts("{\"a\":1,\"a\":2}")) << "a repeated member name";
  EXPECT_FALSE(accepts("{\"a\"}"));
  EXPECT_FALSE(accepts("{\"a\":}"));
  EXPECT_FALSE(accepts("[1,2"));
  EXPECT_FALSE(accepts("{\"a\":\"\xff\xfe\"}")) << "invalid UTF-8 in a string";
}

TEST(StrictJson, RejectsDocumentsPastTheirBounds)
{
  const JsonParseLimits tight{16U, 8U};
  EXPECT_FALSE(parse_strict_json(R"({"a":"0123456789abcdef"})", tight).value);
  EXPECT_TRUE(parse_strict_json(R"({"a":1})", tight).value);

  const JsonParseLimits shallow{1024U, 3U};
  EXPECT_TRUE(parse_strict_json(R"({"a":{"b":1}})", shallow).value);
  EXPECT_FALSE(parse_strict_json(R"({"a":{"b":{"c":1}}})", shallow).value);
}

TEST(StrictJson, RejectsNumbersOutsideTheRepresentableRange)
{
  EXPECT_FALSE(accepts("{\"a\":1e400}"));
}

TEST(StrictJson, QuotesTextSoTheAuditLogStaysParseable)
{
  EXPECT_EQ(json_quote("plain"), "\"plain\"");
  EXPECT_EQ(json_quote("a\"b"), "\"a\\\"b\"");
  EXPECT_EQ(json_quote("a\\b"), "\"a\\\\b\"");
  EXPECT_EQ(json_quote("a\nb"), "\"a\\nb\"");
  EXPECT_EQ(json_quote(std::string("a\x01" "b")), "\"a\\u0001b\"");
  EXPECT_EQ(json_quote("caf\xc3\xa9"), "\"caf\xc3\xa9\"");
}

TEST(StrictJson, ReplacesInvalidUtf8RatherThanEmittingIt)
{
  const auto quoted = json_quote(std::string("bad\xff""tail"));
  EXPECT_EQ(quoted.find('\xff'), std::string::npos);
  // Re-reading what the log wrote must succeed.
  EXPECT_TRUE(parse_strict_json("{\"a\":" + quoted + "}").value);
}

// printf and iostreams read the global locale, and any library can install one with a comma
// decimal separator. "0,5000" is not a JSON number, so requests and audit lines built that way
// would be malformed.
TEST(StrictJson, WritesNumbersTheSameWhateverLocaleTheProcessIsIn)
{
  EXPECT_EQ(json_number(0.5, 4), "0.5000");
  EXPECT_EQ(json_number(-1.0, 2), "-1.00");
  EXPECT_EQ(json_number(0.0, 1), "0.0");
  EXPECT_EQ(json_number(std::numeric_limits<double>::infinity(), 2), "null");
  EXPECT_EQ(json_number(std::numeric_limits<double>::quiet_NaN(), 2), "null");

  // Whatever it wrote must read back as one number.
  const auto parsed = parse_strict_json("{\"a\":" + json_number(0.86, 4) + "}");
  ASSERT_TRUE(parsed.value) << parsed.error;
  EXPECT_NEAR(parsed.value->member("a")->as_number(), 0.86, 1e-9);
}

TEST(StrictJson, ReadsNumbersTheSameWhateverLocaleTheProcessIsIn)
{
  // If a comma-decimal locale is available, install it and check the writer ignores it.
  const char * installed = std::setlocale(LC_NUMERIC, "de_DE.UTF-8");
  const auto parsed = parse_strict_json(R"({"confidence":0.86})");
  ASSERT_TRUE(parsed.value) << parsed.error;
  EXPECT_NEAR(parsed.value->member("confidence")->as_number(), 0.86, 1e-9);
  EXPECT_EQ(json_number(0.86, 2), "0.86");
  if (installed != nullptr) {
    std::setlocale(LC_NUMERIC, "C");
  }
}

TEST(StrictJson, ValidatesUtf8)
{
  EXPECT_TRUE(valid_utf8(""));
  EXPECT_TRUE(valid_utf8("ascii"));
  EXPECT_TRUE(valid_utf8("caf\xc3\xa9"));
  EXPECT_FALSE(valid_utf8("\xc3")) << "a truncated sequence";
  EXPECT_FALSE(valid_utf8("\xc0\xaf")) << "an overlong encoding of '/'";
  EXPECT_FALSE(valid_utf8("\xed\xa0\x80")) << "a surrogate";
  EXPECT_FALSE(valid_utf8("\xf5\x80\x80\x80")) << "above U+10FFFF";
  EXPECT_FALSE(valid_utf8("\x80")) << "a stray continuation byte";
}

}  // namespace restocker_reasoner
