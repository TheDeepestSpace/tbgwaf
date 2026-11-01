#pragma once

#include <cstddef>
#include <map>
#include <string>
#include <vector>

namespace tactics::server {

// Server-side storage for Builder scenes (game/BuilderScene.h), keyed by a
// client-chosen id. The stored value is the scene's serialized JSON, validated
// on the way in. With a directory each scene lives in `<dir>/<id>.json`
// (written via temp file + rename, loaded at construction), so scenes survive
// restarts; with an empty directory the store is memory-only.
class SceneStore {
 public:
  static constexpr size_t kMaxScenes = 100;
  static constexpr size_t kMaxIdLength = 40;

  explicit SceneStore(std::string dir = "");

  // [A-Za-z0-9_-]{1,kMaxIdLength}: safe as a file name.
  static bool ValidId(const std::string& id);

  struct Entry {
    std::string id;
    std::string scene;  // SerializeBuilderScene text.
  };
  std::vector<Entry> List() const;  // Sorted by id.
  size_t size() const { return scenes_.size(); }

  // Validates `scene` (ParseBuilderScene) and stores it, replacing any scene
  // with that id. False with *error set on a bad id/scene, a full store or a
  // failed write (nothing changes then).
  bool Put(const std::string& id, const std::string& scene, std::string* error);
  // True if it existed. False with *error set only when a write failed.
  bool Delete(const std::string& id, std::string* error);

 private:
  std::string PathFor(const std::string& id) const { return dir_ + "/" + id + ".json"; }

  std::string dir_;
  std::map<std::string, std::string> scenes_;
};

}  // namespace tactics::server
