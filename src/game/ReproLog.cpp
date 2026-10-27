#include "game/ReproLog.h"

#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <memory>
#include <utility>

namespace tactics {

const char* ReproBuildId() {
#ifdef TBGWAF_BUILD_COMMIT
  return TBGWAF_BUILD_COMMIT;
#else
  return "unknown";
#endif
}

namespace {

// --- Minimal JSON (enough for the flat, line-per-object log format). ---

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

// --- Writer helpers. ---

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

void AppendVec3(std::string* out, const glm::vec3& v) {
  *out += '[' + FloatText(v.x) + ',' + FloatText(v.y) + ',' + FloatText(v.z) + ']';
}

const char* TeamName(Team team) { return team == Team::Blue ? "blue" : "red"; }

bool ParseTeamName(const JsonValue* v, Team* out) {
  if (!v || v->type != JsonValue::Type::String) return false;
  if (v->str == "blue") {
    *out = Team::Blue;
    return true;
  }
  if (v->str == "red") {
    *out = Team::Red;
    return true;
  }
  return false;
}

bool ReadVec3(const JsonValue* v, glm::vec3* out) {
  if (!v || v->type != JsonValue::Type::Array || v->array.size() != 3) return false;
  for (size_t i = 0; i < 3; ++i) {
    if (v->array[i].type != JsonValue::Type::Number) return false;
    (*out)[static_cast<glm::length_t>(i)] = static_cast<float>(v->array[i].number);
  }
  return true;
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

bool ReadHash(const JsonValue* v, uint64_t* out) {
  std::string hex;
  return ReadString(v, &hex) && HexToHash(hex, out);
}

struct OpName {
  InputEvent::Op op;
  const char* name;
};
constexpr OpName kOpNames[] = {
    {InputEvent::Op::Reset, "reset"},
    {InputEvent::Op::Update, "update"},
    {InputEvent::Op::ClickUnit, "click_unit"},
    {InputEvent::Op::ClickGround, "click_ground"},
    {InputEvent::Op::FinishMovePlan, "finish_move"},
    {InputEvent::Op::ChooseMove, "choose_move"},
    {InputEvent::Op::ChooseShoot, "choose_shoot"},
    {InputEvent::Op::ChoosePass, "choose_pass"},
    {InputEvent::Op::CancelAction, "cancel"},
    {InputEvent::Op::SetMoveFacing, "set_move_facing"},
    {InputEvent::Op::ClickAimRay, "click_aim_ray"},
    {InputEvent::Op::PlaceAimPoint, "place_aim"},
    {InputEvent::Op::ConfirmAim, "confirm_aim"},
    {InputEvent::Op::SetShotCount, "set_shots"},
    {InputEvent::Op::SetPlaybook, "set_playbook"},
    {InputEvent::Op::SetFriendlyFire, "set_friendly_fire"},
    {InputEvent::Op::CommitRound, "commit"},
    {InputEvent::Op::ClearAllPlans, "clear_plans"},
    {InputEvent::Op::ImportTeamPlans, "import_plans"},
    {InputEvent::Op::ImportState, "import_state"},
};

// The op-specific fields of an input entry, appended after the common ones.
void AppendInputFields(std::string* out, const InputEvent& e) {
  using Op = InputEvent::Op;
  auto team = [&] { *out += ",\"team\":\""; *out += TeamName(e.team); *out += '"'; };
  switch (e.op) {
    case Op::ClickUnit:
      *out += ",\"unit\":" + std::to_string(e.unit);
      team();
      break;
    case Op::ClickGround:
    case Op::PlaceAimPoint:
      *out += ",\"point\":";
      AppendVec3(out, e.point);
      team();
      break;
    case Op::SetMoveFacing:
      *out += ",\"unit\":" + std::to_string(e.unit) + ",\"yaw\":" + FloatText(e.value);
      team();
      break;
    case Op::ClickAimRay:
      *out += ",\"origin\":";
      AppendVec3(out, e.point);
      *out += ",\"dir\":";
      AppendVec3(out, e.direction);
      team();
      break;
    case Op::ConfirmAim:
      team();
      break;
    case Op::SetShotCount:
      *out += ",\"count\":" + std::to_string(e.count);
      team();
      break;
    case Op::SetPlaybook:
      team();
      *out += ",\"table\":[";
      for (int m = 0; m < 2; ++m) {
        for (int s = 0; s < 2; ++s) {
          if (m || s) *out += ',';
          *out += std::to_string(static_cast<int>(e.playbook.table[m][s]));
        }
      }
      *out += "],\"ignore_idle\":";
      *out += e.playbook.ignoreIdle ? "true" : "false";
      break;
    case Op::SetFriendlyFire:
      *out += ",\"enabled\":";
      *out += e.flag ? "true" : "false";
      break;
    case Op::ImportTeamPlans:
      team();
      *out += ",\"snapshot\":";
      AppendJsonString(out, e.text);
      break;
    case Op::ImportState:
      *out += ",\"snapshot\":";
      AppendJsonString(out, e.text);
      break;
    case Op::Update:
      *out += ",\"dt\":" + FloatText(e.value);
      break;
    case Op::Reset:
    case Op::FinishMovePlan:
    case Op::ChooseMove:
    case Op::ChooseShoot:
    case Op::ChoosePass:
    case Op::CancelAction:
    case Op::CommitRound:
    case Op::ClearAllPlans:
      break;
  }
}

bool ReadInputFields(const JsonValue& obj, InputEvent* e, std::string* error) {
  using Op = InputEvent::Op;
  auto need = [&](bool ok, const char* what) {
    if (!ok) *error = std::string("input '") + InputOpName(e->op) + "' needs " + what;
    return ok;
  };
  switch (e->op) {
    case Op::ClickUnit:
      return need(ReadInt(obj.Find("unit"), &e->unit), "unit") &&
             need(ParseTeamName(obj.Find("team"), &e->team), "team");
    case Op::ClickGround:
    case Op::PlaceAimPoint:
      return need(ReadVec3(obj.Find("point"), &e->point), "point") &&
             need(ParseTeamName(obj.Find("team"), &e->team), "team");
    case Op::SetMoveFacing:
      return need(ReadInt(obj.Find("unit"), &e->unit), "unit") &&
             need(ReadFloat(obj.Find("yaw"), &e->value), "yaw") &&
             need(ParseTeamName(obj.Find("team"), &e->team), "team");
    case Op::ClickAimRay:
      return need(ReadVec3(obj.Find("origin"), &e->point), "origin") &&
             need(ReadVec3(obj.Find("dir"), &e->direction), "dir") &&
             need(ParseTeamName(obj.Find("team"), &e->team), "team");
    case Op::ConfirmAim:
      return need(ParseTeamName(obj.Find("team"), &e->team), "team");
    case Op::SetShotCount:
      return need(ReadInt(obj.Find("count"), &e->count), "count") &&
             need(ParseTeamName(obj.Find("team"), &e->team), "team");
    case Op::SetPlaybook: {
      if (!need(ParseTeamName(obj.Find("team"), &e->team), "team")) return false;
      const JsonValue* table = obj.Find("table");
      if (!need(table && table->type == JsonValue::Type::Array && table->array.size() == 4,
                "table[4]")) {
        return false;
      }
      for (int i = 0; i < 4; ++i) {
        int action = 0;
        if (!need(ReadInt(&table->array[i], &action), "table entries")) return false;
        if (action < 0 || action > static_cast<int>(ReactionAction::ShootContinue)) {
          *error = "set_playbook table entry out of range";
          return false;
        }
        e->playbook.table[i / 2][i % 2] = static_cast<ReactionAction>(action);
      }
      bool ignoreIdle = false;
      if (!need(ReadBool(obj.Find("ignore_idle"), &ignoreIdle), "ignore_idle")) return false;
      e->playbook.ignoreIdle = ignoreIdle;
      return true;
    }
    case Op::SetFriendlyFire:
      return need(ReadBool(obj.Find("enabled"), &e->flag), "enabled");
    case Op::ImportTeamPlans:
      return need(ParseTeamName(obj.Find("team"), &e->team), "team") &&
             need(ReadString(obj.Find("snapshot"), &e->text), "snapshot");
    case Op::ImportState:
      return need(ReadString(obj.Find("snapshot"), &e->text), "snapshot");
    case Op::Update:
      return need(ReadFloat(obj.Find("dt"), &e->value), "dt");
    case Op::Reset:
    case Op::FinishMovePlan:
    case Op::ChooseMove:
    case Op::ChooseShoot:
    case Op::ChoosePass:
    case Op::CancelAction:
    case Op::CommitRound:
    case Op::ClearAllPlans:
      return true;
  }
  return true;
}

}  // namespace

const char* InputOpName(InputEvent::Op op) {
  for (const OpName& entry : kOpNames) {
    if (entry.op == op) return entry.name;
  }
  return "unknown";
}

bool ParseInputOp(const std::string& name, InputEvent::Op* out) {
  for (const OpName& entry : kOpNames) {
    if (name == entry.name) {
      *out = entry.op;
      return true;
    }
  }
  return false;
}

std::string HashToHex(uint64_t hash) {
  char buf[24];
  std::snprintf(buf, sizeof buf, "%016llx", static_cast<unsigned long long>(hash));
  return buf;
}

bool HexToHash(const std::string& hex, uint64_t* out) {
  if (hex.empty() || hex.size() > 16) return false;
  uint64_t value = 0;
  for (const char c : hex) {
    int digit;
    if (c >= '0' && c <= '9') digit = c - '0';
    else if (c >= 'a' && c <= 'f') digit = 10 + c - 'a';
    else if (c >= 'A' && c <= 'F') digit = 10 + c - 'A';
    else return false;
    value = (value << 4) | static_cast<uint64_t>(digit);
  }
  *out = value;
  return true;
}

std::string SerializeReproLog(const ReproLog& log) {
  std::string out;
  const ReproHeader& h = log.header;
  out += "{\"type\":\"header\",\"format\":\"tbgwaf-repro-log\",\"version\":" +
         std::to_string(h.version) + ",\"scene\":";
  AppendJsonString(&out, h.scene);
  out += ",\"seed\":" + std::to_string(h.seed) + ",\"build\":";
  AppendJsonString(&out, h.build);
  out += ",\"created\":";
  AppendJsonString(&out, h.created);
  out += ",\"initial_hash\":\"" + HashToHex(h.initialHash) + "\"}\n";

  for (const ReproEntry& e : log.entries) {
    const char* type = e.kind == ReproEntry::Kind::Input     ? "input"
                       : e.kind == ReproEntry::Kind::Updates ? "updates"
                                                             : "log";
    out += "{\"type\":\"";
    out += type;
    out += "\",\"seq\":" + std::to_string(e.seq) + ",\"round\":" + std::to_string(e.round) +
           ",\"t\":" + std::to_string(e.timeMs);
    switch (e.kind) {
      case ReproEntry::Kind::Input:
        out += ",\"op\":\"";
        out += InputOpName(e.input.op);
        out += '"';
        AppendInputFields(&out, e.input);
        out += ",\"hash\":\"" + HashToHex(e.hash) + "\"";
        break;
      case ReproEntry::Kind::Updates:
        out += ",\"dts\":[";
        for (size_t i = 0; i < e.dts.size(); ++i) {
          if (i) out += ',';
          out += FloatText(e.dts[i]);
        }
        out += "],\"hash\":\"" + HashToHex(e.hash) + "\"";
        break;
      case ReproEntry::Kind::Log:
        out += ",\"msg\":";
        AppendJsonString(&out, e.message);
        break;
    }
    out += "}\n";
  }
  return out;
}

bool ParseReproLog(const std::string& text, ReproLog* out, std::string* error) {
  *out = ReproLog{};
  size_t lineStart = 0;
  int lineNumber = 0;
  bool haveHeader = false;
  while (lineStart < text.size()) {
    size_t lineEnd = text.find('\n', lineStart);
    if (lineEnd == std::string::npos) lineEnd = text.size();
    const std::string line = text.substr(lineStart, lineEnd - lineStart);
    lineStart = lineEnd + 1;
    ++lineNumber;
    if (line.find_first_not_of(" \t\r") == std::string::npos) continue;

    JsonValue obj;
    std::string parseError;
    if (!JsonParser(line).Parse(&obj, &parseError) || obj.type != JsonValue::Type::Object) {
      *error = "line " + std::to_string(lineNumber) + ": " +
               (parseError.empty() ? "not a JSON object" : parseError);
      return false;
    }
    auto fail = [&](const std::string& what) {
      *error = "line " + std::to_string(lineNumber) + ": " + what;
      return false;
    };
    std::string type;
    if (!ReadString(obj.Find("type"), &type)) return fail("missing 'type'");

    if (type == "header") {
      if (haveHeader) return fail("duplicate header");
      haveHeader = true;
      ReproHeader& h = out->header;
      if (!ReadInt(obj.Find("version"), &h.version)) return fail("header needs 'version'");
      if (h.version != kReproLogVersion) {
        return fail("unsupported repro log version " + std::to_string(h.version) +
                    " (this build reads version " + std::to_string(kReproLogVersion) + ")");
      }
      if (!ReadString(obj.Find("scene"), &h.scene)) return fail("header needs 'scene'");
      int seed = 0;
      const JsonValue* seedValue = obj.Find("seed");
      if (!seedValue || seedValue->type != JsonValue::Type::Number) return fail("header needs 'seed'");
      h.seed = static_cast<uint32_t>(seedValue->number);
      (void)seed;
      ReadString(obj.Find("build"), &h.build);
      ReadString(obj.Find("created"), &h.created);
      if (!ReadHash(obj.Find("initial_hash"), &h.initialHash)) {
        return fail("header needs 'initial_hash'");
      }
      continue;
    }
    if (!haveHeader) return fail("first line must be the header");

    ReproEntry entry;
    if (!ReadInt(obj.Find("seq"), &entry.seq)) return fail("missing 'seq'");
    ReadInt(obj.Find("round"), &entry.round);
    if (const JsonValue* t = obj.Find("t"); t && t->type == JsonValue::Type::Number) {
      entry.timeMs = static_cast<uint64_t>(t->number);
    }
    if (type == "input") {
      entry.kind = ReproEntry::Kind::Input;
      std::string opName;
      if (!ReadString(obj.Find("op"), &opName)) return fail("input needs 'op'");
      if (!ParseInputOp(opName, &entry.input.op)) return fail("unknown op '" + opName + "'");
      std::string fieldError;
      if (!ReadInputFields(obj, &entry.input, &fieldError)) return fail(fieldError);
      if (!ReadHash(obj.Find("hash"), &entry.hash)) return fail("input needs 'hash'");
    } else if (type == "updates") {
      entry.kind = ReproEntry::Kind::Updates;
      const JsonValue* dts = obj.Find("dts");
      if (!dts || dts->type != JsonValue::Type::Array) return fail("updates needs 'dts'");
      for (const JsonValue& v : dts->array) {
        if (v.type != JsonValue::Type::Number) return fail("dts entries must be numbers");
        entry.dts.push_back(static_cast<float>(v.number));
      }
      if (!ReadHash(obj.Find("hash"), &entry.hash)) return fail("updates needs 'hash'");
    } else if (type == "log") {
      entry.kind = ReproEntry::Kind::Log;
      ReadString(obj.Find("msg"), &entry.message);
    } else {
      return fail("unknown entry type '" + type + "'");
    }
    out->entries.push_back(std::move(entry));
  }
  if (!haveHeader) {
    *error = "empty log (no header)";
    return false;
  }
  return true;
}

// --- Recorder. ---

ReproRecorder::ReproRecorder(std::function<uint64_t()> nowMs) : nowMs_(std::move(nowMs)) {}

void ReproRecorder::Start(const GameLogic& game, const std::string& sceneId, uint32_t seed,
                          const std::string& created) {
  log_ = ReproLog{};
  log_.header.scene = sceneId;
  log_.header.seed = seed;
  log_.header.build = ReproBuildId();
  log_.header.created = created;
  log_.header.initialHash = game.StateHash();
  nextSeq_ = 1;
  startMs_ = nowMs_ ? nowMs_() : 0;
  pendingDts_.clear();
  pendingValid_ = false;
  pendingMode_ = game.Mode();
  started_ = true;
}

void ReproRecorder::FlushUpdates() {
  if (!pendingValid_) return;
  ReproEntry entry;
  entry.kind = ReproEntry::Kind::Updates;
  entry.seq = nextSeq_++;
  entry.round = pendingRound_;
  entry.timeMs = pendingTimeMs_;
  entry.dts = std::move(pendingDts_);
  entry.hash = pendingHash_;
  log_.entries.push_back(std::move(entry));
  pendingDts_.clear();
  pendingValid_ = false;
}

void ReproRecorder::OnInput(const GameLogic& game, const InputEvent& event) {
  if (!started_) return;
  const uint64_t now = (nowMs_ ? nowMs_() : 0) - startMs_;
  if (event.op == InputEvent::Op::Update) {
    if (!pendingValid_) {
      pendingValid_ = true;
      pendingTimeMs_ = now;
    }
    pendingDts_.push_back(event.value);
    pendingHash_ = game.StateHash();
    pendingRound_ = game.RoundNumber();
    // Close the batch on the frame that changed the mode (round ended, game
    // over) so a checkpoint sits exactly on that boundary.
    if (game.Mode() != pendingMode_ || pendingDts_.size() >= kMaxBatchedUpdates) FlushUpdates();
    pendingMode_ = game.Mode();
    return;
  }
  FlushUpdates();
  ReproEntry entry;
  entry.kind = ReproEntry::Kind::Input;
  entry.seq = nextSeq_++;
  entry.round = game.RoundNumber();
  entry.timeMs = now;
  entry.input = event;
  entry.hash = game.StateHash();
  log_.entries.push_back(std::move(entry));
  pendingMode_ = game.Mode();
}

void ReproRecorder::OnLog(const GameLogic& game, const std::string& message) {
  if (!started_) return;
  // A sim line lands mid-input (inside an Update, typically): close the
  // batch of frames before it so the file reads in order and the state just
  // before the frame it happened in is checkpointed.
  FlushUpdates();
  ReproEntry entry;
  entry.kind = ReproEntry::Kind::Log;
  entry.seq = nextSeq_++;
  entry.round = game.RoundNumber();
  entry.timeMs = (nowMs_ ? nowMs_() : 0) - startMs_;
  entry.message = message;
  log_.entries.push_back(std::move(entry));
}

const ReproLog& ReproRecorder::Log() {
  FlushUpdates();
  return log_;
}

std::string ReproRecorder::Text() { return SerializeReproLog(Log()); }

// --- Replay. ---

ReplayReport ReplayReproLog(const ReproLog& log, const Scene& scene, GameLogic* game,
                            const std::function<void(const ReproEntry&, float dt)>& afterEntry) {
  ReplayReport report;
  if (game->StateHash() != log.header.initialHash) {
    report.mismatches.push_back({0, log.header.initialHash, game->StateHash()});
  }
  auto check = [&](const ReproEntry& entry) {
    const uint64_t actual = game->StateHash();
    if (actual != entry.hash) report.mismatches.push_back({entry.seq, entry.hash, actual});
  };
  using Op = InputEvent::Op;
  for (const ReproEntry& entry : log.entries) {
    ++report.entriesApplied;
    if (entry.kind == ReproEntry::Kind::Log) continue;
    if (entry.kind == ReproEntry::Kind::Updates) {
      for (const float dt : entry.dts) {
        game->Update(dt);
        ++report.updatesApplied;
        if (afterEntry) afterEntry(entry, dt);
      }
      check(entry);
      continue;
    }
    const InputEvent& e = entry.input;
    ++report.inputsApplied;
    switch (e.op) {
      case Op::Reset: game->Reset(scene); break;
      case Op::Update: game->Update(e.value); break;
      case Op::ClickUnit: game->ClickUnit(e.unit, e.team); break;
      case Op::ClickGround: game->ClickGround(e.point, e.team); break;
      case Op::FinishMovePlan: game->FinishMovePlan(); break;
      case Op::ChooseMove: game->ChooseMove(); break;
      case Op::ChooseShoot: game->ChooseShoot(); break;
      case Op::ChoosePass: game->ChoosePass(); break;
      case Op::CancelAction: game->CancelAction(); break;
      case Op::SetMoveFacing: game->SetPlannedMoveFacing(e.unit, e.value, e.team); break;
      case Op::ClickAimRay: game->ClickAimRay(e.point, e.direction, e.team); break;
      case Op::PlaceAimPoint: game->PlaceAimPoint(e.point, e.team); break;
      case Op::ConfirmAim: game->ConfirmAim(e.team); break;
      case Op::SetShotCount: game->SetPlannedShotCount(e.count, e.team); break;
      case Op::SetPlaybook: game->SetPlaybook(e.team, e.playbook); break;
      case Op::SetFriendlyFire: game->SetFriendlyFireEnabled(e.flag); break;
      case Op::CommitRound: game->CommitRound(); break;
      case Op::ClearAllPlans: game->ClearAllPlans(); break;
      case Op::ImportTeamPlans:
      case Op::ImportState: {
        GameSnapshot snap;
        if (!DeserializeSnapshot(e.text, &snap)) {
          report.error = "entry " + std::to_string(entry.seq) + ": malformed snapshot";
          return report;
        }
        if (e.op == Op::ImportTeamPlans) game->ImportTeamPlans(snap, e.team);
        else game->ImportState(snap);
        break;
      }
    }
    if (afterEntry) afterEntry(entry, 0.0f);
    check(entry);
  }
  report.finalHash = game->StateHash();
  report.ok = report.mismatches.empty();
  return report;
}

}  // namespace tactics
