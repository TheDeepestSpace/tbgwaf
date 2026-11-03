#pragma once

// Minimal JSON (enough for the flat, line-per-object formats the game
// writes: repro logs, builder scenes). Shared by ReproLog.cpp and
// BuilderScene.cpp. Writers emit compact text; floats use %.9g, which
// round-trips every finite float exactly through strtod.

#include <string>
#include <utility>
#include <vector>

namespace tactics {
namespace json {

struct JsonValue {
  enum class Type { Null, Bool, Number, String, Array, Object };
  Type type = Type::Null;
  bool boolean = false;
  double number = 0.0;
  std::string str;
  std::vector<JsonValue> array;
  std::vector<std::pair<std::string, JsonValue>> object;

  const JsonValue* Find(const char* key) const {
    for (const auto& entry : object) {
      if (entry.first == key) return &entry.second;
    }
    return nullptr;
  }
};

// Parses one complete JSON value; false (and sets *error) on malformed text.
bool ParseJson(const std::string& text, JsonValue* out, std::string* error);

// Appends `s` as a quoted, escaped JSON string.
void AppendJsonString(std::string* out, const std::string& s);
// Shortest-enough text that parses back to exactly `v` ("0" if not finite).
std::string FloatText(float v);

// Appends `v` as compact JSON. Numbers use the shortest of %.9g / %.17g that
// parses back to the same double, so text written with FloatText survives a
// parse + AppendJson unchanged.
void AppendJson(std::string* out, const JsonValue& v);

// Typed readers: false (leaving *out untouched) when `v` is null or the
// wrong type.
bool ReadInt(const JsonValue* v, int* out);
bool ReadFloat(const JsonValue* v, float* out);
bool ReadString(const JsonValue* v, std::string* out);
bool ReadBool(const JsonValue* v, bool* out);

}  // namespace json
}  // namespace tactics
