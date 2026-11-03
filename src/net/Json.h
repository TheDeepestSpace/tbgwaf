#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace tactics::net {

// Minimal JSON value for the wire protocol (shared by the server and the
// WASM client). Numbers are doubles; objects keep keys sorted.
class Json {
 public:
  enum class Type { Null, Bool, Number, String, Array, Object };
  using Array = std::vector<Json>;
  using Object = std::map<std::string, Json>;

  Json() = default;
  Json(bool v) : type_(Type::Bool), bool_(v) {}
  Json(int v) : type_(Type::Number), number_(v) {}
  Json(double v) : type_(Type::Number), number_(v) {}
  Json(const char* v) : type_(Type::String), string_(v) {}
  Json(std::string v) : type_(Type::String), string_(std::move(v)) {}
  Json(Array v) : type_(Type::Array), array_(std::move(v)) {}
  Json(Object v) : type_(Type::Object), object_(std::move(v)) {}

  Type type() const { return type_; }
  bool IsNumber() const { return type_ == Type::Number; }
  bool IsString() const { return type_ == Type::String; }
  bool IsArray() const { return type_ == Type::Array; }
  bool IsObject() const { return type_ == Type::Object; }
  bool IsBool() const { return type_ == Type::Bool; }

  double AsNumber(double fallback = 0.0) const { return IsNumber() ? number_ : fallback; }
  bool AsBool(bool fallback = false) const { return IsBool() ? bool_ : fallback; }
  const std::string& AsString() const { return string_; }  // Empty unless IsString().
  const Array& AsArray() const { return array_; }          // Empty unless IsArray().
  const Object& AsObject() const { return object_; }       // Empty unless IsObject().

  // Object member or a shared null if absent / not an object.
  const Json& operator[](const std::string& key) const;
  bool Has(const std::string& key) const { return IsObject() && object_.count(key) > 0; }

  // Mutable builders: operator[] on a non-object turns it into one.
  Json& Set(const std::string& key, Json value);
  Json& Push(Json value);

  std::string Dump() const;
  // Returns false on malformed input or nesting deeper than kMaxDepth.
  static bool Parse(const std::string& text, Json* out);

  static constexpr int kMaxDepth = 32;

 private:
  Type type_ = Type::Null;
  bool bool_ = false;
  double number_ = 0.0;
  std::string string_;
  Array array_;
  Object object_;
};

}  // namespace tactics::net
