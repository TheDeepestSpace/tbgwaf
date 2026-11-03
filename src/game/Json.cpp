#include "game/Json.h"

#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <utility>

namespace tactics {
namespace json {

class JsonParser {
 public:
  explicit JsonParser(const std::string& text) : text_(text) {}

  bool Parse(JsonValue* out, std::string* error) {
    SkipSpace();
    if (!ParseValue(out, error)) return false;
    SkipSpace();
    if (pos_ != text_.size()) {
      *error = "trailing characters after JSON value";
      return false;
    }
    return true;
  }

 private:
  void SkipSpace() {
    while (pos_ < text_.size() && std::isspace(static_cast<unsigned char>(text_[pos_]))) ++pos_;
  }
  bool Consume(char c) {
    if (pos_ < text_.size() && text_[pos_] == c) {
      ++pos_;
      return true;
    }
    return false;
  }
  bool Fail(std::string* error, const char* what) {
    *error = std::string(what) + " at offset " + std::to_string(pos_);
    return false;
  }

  bool ParseValue(JsonValue* out, std::string* error) {
    if (pos_ >= text_.size()) return Fail(error, "unexpected end of input");
    const char c = text_[pos_];
    if (c == '{') return ParseObject(out, error);
    if (c == '[') return ParseArray(out, error);
    if (c == '"') {
      out->type = JsonValue::Type::String;
      return ParseString(&out->str, error);
    }
    if (text_.compare(pos_, 4, "true") == 0) {
      pos_ += 4;
      out->type = JsonValue::Type::Bool;
      out->boolean = true;
      return true;
    }
    if (text_.compare(pos_, 5, "false") == 0) {
      pos_ += 5;
      out->type = JsonValue::Type::Bool;
      out->boolean = false;
      return true;
    }
    if (text_.compare(pos_, 4, "null") == 0) {
      pos_ += 4;
      out->type = JsonValue::Type::Null;
      return true;
    }
    if (c == '-' || (c >= '0' && c <= '9')) {
      const char* begin = text_.c_str() + pos_;
      char* end = nullptr;
      out->number = std::strtod(begin, &end);
      if (end == begin) return Fail(error, "bad number");
      pos_ += static_cast<size_t>(end - begin);
      out->type = JsonValue::Type::Number;
      return true;
    }
    return Fail(error, "unexpected character");
  }

  bool ParseString(std::string* out, std::string* error) {
    if (!Consume('"')) return Fail(error, "expected string");
    out->clear();
    while (pos_ < text_.size()) {
      const char c = text_[pos_++];
      if (c == '"') return true;
      if (c != '\\') {
        out->push_back(c);
        continue;
      }
      if (pos_ >= text_.size()) break;
      const char esc = text_[pos_++];
      switch (esc) {
        case '"': out->push_back('"'); break;
        case '\\': out->push_back('\\'); break;
        case '/': out->push_back('/'); break;
        case 'b': out->push_back('\b'); break;
        case 'f': out->push_back('\f'); break;
        case 'n': out->push_back('\n'); break;
        case 'r': out->push_back('\r'); break;
        case 't': out->push_back('\t'); break;
        case 'u': {
          if (pos_ + 4 > text_.size()) return Fail(error, "bad \\u escape");
          const unsigned code =
              static_cast<unsigned>(std::strtoul(text_.substr(pos_, 4).c_str(), nullptr, 16));
          pos_ += 4;
          // The writer only ever emits \u00XX for control characters.
          if (code < 0x80) {
            out->push_back(static_cast<char>(code));
          } else {
            out->push_back('?');
          }
          break;
        }
        default: return Fail(error, "bad escape");
      }
    }
    return Fail(error, "unterminated string");
  }

  bool ParseArray(JsonValue* out, std::string* error) {
    out->type = JsonValue::Type::Array;
    Consume('[');
    SkipSpace();
    if (Consume(']')) return true;
    while (true) {
      SkipSpace();
      JsonValue item;
      if (!ParseValue(&item, error)) return false;
      out->array.push_back(std::move(item));
      SkipSpace();
      if (Consume(']')) return true;
      if (!Consume(',')) return Fail(error, "expected ',' or ']'");
    }
  }

  bool ParseObject(JsonValue* out, std::string* error) {
    out->type = JsonValue::Type::Object;
    Consume('{');
    SkipSpace();
    if (Consume('}')) return true;
    while (true) {
      SkipSpace();
      std::string key;
      if (!ParseString(&key, error)) return false;
      SkipSpace();
      if (!Consume(':')) return Fail(error, "expected ':'");
      SkipSpace();
      JsonValue value;
      if (!ParseValue(&value, error)) return false;
      out->object.emplace_back(std::move(key), std::move(value));
      SkipSpace();
      if (Consume('}')) return true;
      if (!Consume(',')) return Fail(error, "expected ',' or '}'");
    }
  }

  const std::string& text_;
  size_t pos_ = 0;
};

bool ParseJson(const std::string& text, JsonValue* out, std::string* error) {
  return JsonParser(text).Parse(out, error);
}

void AppendJsonString(std::string* out, const std::string& s) {
  out->push_back('"');
  for (const unsigned char c : s) {
    switch (c) {
      case '"': *out += "\\\""; break;
      case '\\': *out += "\\\\"; break;
      case '\n': *out += "\\n"; break;
      case '\r': *out += "\\r"; break;
      case '\t': *out += "\\t"; break;
      default:
        if (c < 0x20) {
          char buf[8];
          std::snprintf(buf, sizeof buf, "\\u%04x", c);
          *out += buf;
        } else {
          out->push_back(static_cast<char>(c));
        }
    }
  }
  out->push_back('"');
}

// %.9g round-trips every finite float exactly through strtod.
std::string FloatText(float v) {
  if (!std::isfinite(v)) return "0";
  char buf[32];
  std::snprintf(buf, sizeof buf, "%.9g", static_cast<double>(v));
  return buf;
}

void AppendJson(std::string* out, const JsonValue& v) {
  switch (v.type) {
    case JsonValue::Type::Null: *out += "null"; break;
    case JsonValue::Type::Bool: *out += v.boolean ? "true" : "false"; break;
    case JsonValue::Type::Number: {
      if (!std::isfinite(v.number)) {
        *out += '0';
        break;
      }
      char buf[40];
      std::snprintf(buf, sizeof buf, "%.9g", v.number);
      if (std::strtod(buf, nullptr) != v.number) std::snprintf(buf, sizeof buf, "%.17g", v.number);
      *out += buf;
      break;
    }
    case JsonValue::Type::String: AppendJsonString(out, v.str); break;
    case JsonValue::Type::Array:
      out->push_back('[');
      for (size_t i = 0; i < v.array.size(); ++i) {
        if (i) out->push_back(',');
        AppendJson(out, v.array[i]);
      }
      out->push_back(']');
      break;
    case JsonValue::Type::Object:
      out->push_back('{');
      for (size_t i = 0; i < v.object.size(); ++i) {
        if (i) out->push_back(',');
        AppendJsonString(out, v.object[i].first);
        out->push_back(':');
        AppendJson(out, v.object[i].second);
      }
      out->push_back('}');
      break;
  }
}

bool ReadInt(const JsonValue* v, int* out) {
  if (!v || v->type != JsonValue::Type::Number) return false;
  *out = static_cast<int>(v->number);
  return true;
}

bool ReadFloat(const JsonValue* v, float* out) {
  if (!v || v->type != JsonValue::Type::Number) return false;
  *out = static_cast<float>(v->number);
  return true;
}

bool ReadString(const JsonValue* v, std::string* out) {
  if (!v || v->type != JsonValue::Type::String) return false;
  *out = v->str;
  return true;
}

bool ReadBool(const JsonValue* v, bool* out) {
  if (!v || v->type != JsonValue::Type::Bool) return false;
  *out = v->boolean;
  return true;
}

}  // namespace json
}  // namespace tactics
