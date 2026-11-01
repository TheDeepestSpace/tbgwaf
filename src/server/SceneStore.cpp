#include "server/SceneStore.h"

#include <filesystem>
#include <fstream>
#include <sstream>
#include <system_error>

#include "game/BuilderScene.h"

namespace tactics::server {

namespace fs = std::filesystem;

SceneStore::SceneStore(std::string dir) : dir_(std::move(dir)) {
  if (dir_.empty()) return;
  std::error_code ec;
  fs::create_directories(dir_, ec);
  for (const auto& file : fs::directory_iterator(dir_, ec)) {
    if (scenes_.size() >= kMaxScenes) break;
    if (!file.is_regular_file() || file.path().extension() != ".json") continue;
    const std::string id = file.path().stem().string();
    if (!ValidId(id)) continue;
    std::ifstream in(file.path(), std::ios::binary);
    std::stringstream text;
    text << in.rdbuf();
    BuilderScene parsed;
    // Skip unreadable/invalid files rather than refuse to start.
    if (ParseBuilderScene(text.str(), &parsed)) scenes_[id] = SerializeBuilderScene(parsed);
  }
}

bool SceneStore::ValidId(const std::string& id) {
  if (id.empty() || id.size() > kMaxIdLength) return false;
  for (char c : id) {
    const bool ok = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') ||
                    c == '-' || c == '_';
    if (!ok) return false;
  }
  return true;
}

std::vector<SceneStore::Entry> SceneStore::List() const {
  std::vector<Entry> out;
  out.reserve(scenes_.size());
  for (const auto& [id, scene] : scenes_) out.push_back({id, scene});
  return out;
}

bool SceneStore::Put(const std::string& id, const std::string& scene, std::string* error) {
  if (!ValidId(id)) {
    *error = "bad scene id";
    return false;
  }
  BuilderScene parsed;
  std::string parseError;
  if (!ParseBuilderScene(scene, &parsed, &parseError)) {
    *error = "bad scene: " + parseError;
    return false;
  }
  if (!scenes_.count(id) && scenes_.size() >= kMaxScenes) {
    *error = "scene storage is full";
    return false;
  }
  const std::string canonical = SerializeBuilderScene(parsed);
  if (!dir_.empty()) {
    const std::string tmp = PathFor(id) + ".tmp";
    {
      std::ofstream out(tmp, std::ios::binary | std::ios::trunc);
      out << canonical;
      if (!out) {
        *error = "could not write scene";
        return false;
      }
    }
    std::error_code ec;
    fs::rename(tmp, PathFor(id), ec);
    if (ec) {
      fs::remove(tmp, ec);
      *error = "could not write scene";
      return false;
    }
  }
  scenes_[id] = canonical;
  return true;
}

bool SceneStore::Delete(const std::string& id, std::string* error) {
  auto it = scenes_.find(id);
  if (it == scenes_.end()) return false;
  if (!dir_.empty()) {
    std::error_code ec;
    fs::remove(PathFor(id), ec);
    if (ec) {
      *error = "could not delete scene";
      return false;
    }
  }
  scenes_.erase(it);
  return true;
}

}  // namespace tactics::server
