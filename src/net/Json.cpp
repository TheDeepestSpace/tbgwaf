#include "net/Json.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace tactics::net {

const Json& Json::operator[](const std::string& key) const {
  static const Json kNull;
  if (!IsObject()) return kNull;
  auto it = object_.find(key);
  return it == object_.end() ? kNull : it->second;
}

Json& Json::Set(const std::string& key, Json value) {
  if (type_ != Type::Object) {
    *this = Json(Object{});
  }
  object_[key] = std::move(value);
  return *this;
}

Json& Json::Push(Json value) {
  if (type_ != Type::Array) {
    *this = Json(Array{});
  }
  array_.push_back(std::move(value));
  return *this;
}

namespace {

void DumpString(const std::string& s, std::string* out) {
  out->push_back('"');
  for (unsigned char c : s) {
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

class Parser {
 public:
  explicit Parser(const std::string& text) : text_(text) {}

  bool ParseDocument(Json* out) {
    SkipSpace();
    if (!ParseValue(out, 0)) return false;
    SkipSpace();
    return pos_ == text_.size();
  }

 private:
  void SkipSpace() {
    while (pos_ < text_.size() &&
           (text_[pos_] == ' ' || text_[pos_] == '\t' || text_[pos_] == '\n' || text_[pos_] == '\r')) {
      ++pos_;
    }
  }

  bool Literal(const char* word) {
    size_t n = 0;
    while (word[n]) ++n;
    if (text_.compare(pos_, n, word) != 0) return false;
    pos_ += n;
    return true;
  }

  bool ParseValue(Json* out, int depth) {
    if (depth > Json::kMaxDepth || pos_ >= text_.size()) return false;
    const char c = text_[pos_];
    if (c == '{') return ParseObject(out, depth);
    if (c == '[') return ParseArray(out, depth);
    if (c == '"') {
      std::string s;
      if (!ParseString(&s)) return false;
      *out = Json(std::move(s));
      return true;
    }
    if (c == 't') { if (!Literal("true")) return false; *out = Json(true); return true; }
    if (c == 'f') { if (!Literal("false")) return false; *out = Json(false); return true; }
    if (c == 'n') { if (!Literal("null")) return false; *out = Json(); return true; }
    return ParseNumber(out);
  }

  bool ParseNumber(Json* out) {
    const char* start = text_.c_str() + pos_;
    char* end = nullptr;
    const double v = std::strtod(start, &end);
    if (end == start || !std::isfinite(v)) return false;
    pos_ += static_cast<size_t>(end - start);
    *out = Json(v);
    return true;
  }

  bool ParseString(std::string* out) {
    ++pos_;  // Opening quote.
    while (pos_ < text_.size()) {
      const char c = text_[pos_++];
      if (c == '"') return true;
      if (static_cast<unsigned char>(c) < 0x20) return false;
      if (c != '\\') { out->push_back(c); continue; }
      if (pos_ >= text_.size()) return false;
      const char e = text_[pos_++];
      switch (e) {
        case '"': out->push_back('"'); break;
        case '\\': out->push_back('\\'); break;
        case '/': out->push_back('/'); break;
        case 'n': out->push_back('\n'); break;
        case 'r': out->push_back('\r'); break;
        case 't': out->push_back('\t'); break;
        case 'b': out->push_back('\b'); break;
        case 'f': out->push_back('\f'); break;
        case 'u': {
          if (pos_ + 4 > text_.size()) return false;
          unsigned cp = 0;
          for (int i = 0; i < 4; ++i) {
            const char h = text_[pos_++];
            cp <<= 4;
            if (h >= '0' && h <= '9') cp |= h - '0';
            else if (h >= 'a' && h <= 'f') cp |= h - 'a' + 10;
            else if (h >= 'A' && h <= 'F') cp |= h - 'A' + 10;
            else return false;
          }
          // Surrogate pairs aren't needed by this protocol; emit UTF-8 for the BMP.
          if (cp < 0x80) {
            out->push_back(static_cast<char>(cp));
          } else if (cp < 0x800) {
            out->push_back(static_cast<char>(0xC0 | (cp >> 6)));
            out->push_back(static_cast<char>(0x80 | (cp & 0x3F)));
          } else {
            out->push_back(static_cast<char>(0xE0 | (cp >> 12)));
            out->push_back(static_cast<char>(0x80 | ((cp >> 6) & 0x3F)));
            out->push_back(static_cast<char>(0x80 | (cp & 0x3F)));
          }
          break;
        }
        default: return false;
      }
    }
    return false;
  }

  bool ParseArray(Json* out, int depth) {
    ++pos_;
    Json::Array items;
    SkipSpace();
    if (pos_ < text_.size() && text_[pos_] == ']') { ++pos_; *out = Json(std::move(items)); return true; }
    while (true) {
      SkipSpace();
      Json item;
      if (!ParseValue(&item, depth + 1)) return false;
      items.push_back(std::move(item));
      SkipSpace();
      if (pos_ >= text_.size()) return false;
      if (text_[pos_] == ',') { ++pos_; continue; }
      if (text_[pos_] == ']') { ++pos_; break; }
      return false;
    }
    *out = Json(std::move(items));
    return true;
  }

  bool ParseObject(Json* out, int depth) {
    ++pos_;
    Json::Object members;
    SkipSpace();
    if (pos_ < text_.size() && text_[pos_] == '}') { ++pos_; *out = Json(std::move(members)); return true; }
    while (true) {
      SkipSpace();
      if (pos_ >= text_.size() || text_[pos_] != '"') return false;
      std::string key;
      if (!ParseString(&key)) return false;
      SkipSpace();
      if (pos_ >= text_.size() || text_[pos_] != ':') return false;
      ++pos_;
      SkipSpace();
      Json value;
      if (!ParseValue(&value, depth + 1)) return false;
      members[std::move(key)] = std::move(value);
      SkipSpace();
      if (pos_ >= text_.size()) return false;
      if (text_[pos_] == ',') { ++pos_; continue; }
      if (text_[pos_] == '}') { ++pos_; break; }
      return false;
    }
    *out = Json(std::move(members));
    return true;
  }

  const std::string& text_;
  size_t pos_ = 0;
};

void DumpNumber(double v, std::string* out) {
  if (!std::isfinite(v)) { *out += "0"; return; }
  char buf[40];
  if (v == std::floor(v) && std::fabs(v) < 1e15) {
    std::snprintf(buf, sizeof buf, "%.0f", v);
  } else {
    std::snprintf(buf, sizeof buf, "%.7g", v);
  }
  *out += buf;
}

void DumpValue(const Json& j, std::string* out) {
  switch (j.type()) {
    case Json::Type::Null: *out += "null"; break;
    case Json::Type::Bool: *out += j.AsBool() ? "true" : "false"; break;
    case Json::Type::Number: DumpNumber(j.AsNumber(), out); break;
    case Json::Type::String: DumpString(j.AsString(), out); break;
    case Json::Type::Array: {
      out->push_back('[');
      bool first = true;
      for (const Json& item : j.AsArray()) {
        if (!first) out->push_back(',');
        first = false;
        DumpValue(item, out);
      }
      out->push_back(']');
      break;
    }
    case Json::Type::Object: {
      out->push_back('{');
      bool first = true;
      for (const auto& [key, value] : j.AsObject()) {
        if (!first) out->push_back(',');
        first = false;
        DumpString(key, out);
        out->push_back(':');
        DumpValue(value, out);
      }
      out->push_back('}');
      break;
    }
  }
}

}  // namespace

std::string Json::Dump() const {
  std::string out;
  DumpValue(*this, &out);
  return out;
}

bool Json::Parse(const std::string& text, Json* out) {
  Parser parser(text);
  Json value;
  if (!parser.ParseDocument(&value)) return false;
  *out = std::move(value);
  return true;
}

}  // namespace tactics::net
