#include "net/BuilderSceneSync.h"

#include <cstdio>

#include "net/Json.h"

namespace tactics::net {

namespace {
Json Typed(const char* type) {
  Json j;
  j.Set("t", type);
  return j;
}
}  // namespace

void BuilderSceneSync::OnOpen() {
  open_ = true;
  error_.clear();
  outgoing_.push_back(Typed("scenes_list").Dump());
}

void BuilderSceneSync::OnClose() {
  open_ = false;
  outgoing_.clear();
}

bool BuilderSceneSync::Save(const std::string& id, const BuilderScene& scene) {
  if (!open_) return false;
  Json put = Typed("scene_put");
  put.Set("id", id);
  put.Set("scene", SerializeBuilderScene(scene));
  outgoing_.push_back(put.Dump());
  return true;
}

bool BuilderSceneSync::Remove(const std::string& id) {
  if (!open_) return false;
  Json del = Typed("scene_delete");
  del.Set("id", id);
  outgoing_.push_back(del.Dump());
  return true;
}

bool BuilderSceneSync::OnMessage(const std::string& text) {
  Json message;
  if (!Json::Parse(text, &message) || !message["t"].IsString()) return false;
  const std::string& type = message["t"].AsString();
  if (type == "error") {
    error_ = message["error"].AsString();
    return false;
  }
  if (type != "scenes") return false;
  loaded_.clear();
  for (const Json& item : message["scenes"].AsArray()) {
    Stored stored;
    if (!item["id"].IsString() || !item["scene"].IsString()) continue;
    if (!ParseBuilderScene(item["scene"].AsString(), &stored.scene)) continue;
    stored.id = item["id"].AsString();
    loaded_.push_back(std::move(stored));
  }
  hasLoaded_ = true;
  return true;
}

bool BuilderSceneSync::TakeLoaded(std::vector<Stored>* out) {
  if (!hasLoaded_) return false;
  *out = std::move(loaded_);
  loaded_.clear();
  hasLoaded_ = false;
  return true;
}

std::vector<std::string> BuilderSceneSync::TakeOutgoing() {
  std::vector<std::string> out;
  out.swap(outgoing_);
  return out;
}

std::string BuilderSceneSync::NewId(unsigned long long entropy) {
  char buf[16];
  std::snprintf(buf, sizeof buf, "s%012llx", entropy & 0xFFFFFFFFFFFFULL);
  return buf;
}

}  // namespace tactics::net
