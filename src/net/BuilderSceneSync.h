#pragma once

#include <string>
#include <vector>

#include "game/BuilderScene.h"

namespace tactics::net {

// Client side of the server's Builder scene storage (see net/Protocol.h and
// server/SceneStore.h). Transport-agnostic: the app moves frames between
// TakeOutgoing()/OnMessage() and its WebSocket, so the logic is testable
// without one. The server's list is authoritative: a fresh list replaces the
// local scenes.
class BuilderSceneSync {
 public:
  struct Stored {
    std::string id;
    BuilderScene scene;
  };

  // Socket opened/closed. Opening asks the server for its scene list.
  void OnOpen();
  void OnClose();
  bool connected() const { return open_; }

  // Queue a save/delete; ignored (returns false) while not connected, so the
  // caller can tell the user it was not stored.
  bool Save(const std::string& id, const BuilderScene& scene);
  bool Remove(const std::string& id);

  // One inbound server frame. Returns true if it was the scene list (then
  // TakeLoaded() has it). Errors are kept in lastError().
  bool OnMessage(const std::string& text);
  bool TakeLoaded(std::vector<Stored>* out);

  // Frames to send, in order (cleared).
  std::vector<std::string> TakeOutgoing();
  const std::string& lastError() const { return error_; }

  // A fresh scene id ("s" + 12 hex digits from `entropy`) that SceneStore accepts.
  static std::string NewId(unsigned long long entropy);

 private:
  bool open_ = false;
  bool hasLoaded_ = false;
  std::vector<Stored> loaded_;
  std::vector<std::string> outgoing_;
  std::string error_;
};

}  // namespace tactics::net
