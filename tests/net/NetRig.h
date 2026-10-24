#pragma once

// Test rig shared by the networked scenario tests and the visual runner's
// --server mode: spawns the test-control tbgwaf server, and drives one
// headless client per team (GameLogic mirror + net::RemoteClient over a
// native WebSocket) through it.

#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>
#include <poll.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "WsClient.h"
#include "game/GameLogic.h"
#include "net/Json.h"
#include "net/Protocol.h"
#include "net/RemoteClient.h"
#include "scenario/Scenario.h"

namespace tactics::testing {

using net::Json;
using scenario::Scenario;

inline constexpr int kWaitMs = 10000;

inline int64_t NowMs() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

// ---- Server process ----

struct ServerProcess {
  pid_t pid = -1;
  uint16_t gamePort = 0;
  uint16_t controlPort = 0;

  bool Start(const std::string& binary, std::string* error) {
    int fds[2];
    if (pipe(fds) != 0) return false;
    pid = fork();
    if (pid == 0) {
      dup2(fds[1], STDOUT_FILENO);
      close(fds[0]);
      close(fds[1]);
      // Scenario scripts are bursty; lift the per-connection message cap.
      execl(binary.c_str(), binary.c_str(), "--port", "0", "--control-port", "0", "--rate-limit",
            "100000", static_cast<char*>(nullptr));
      _exit(127);
    }
    close(fds[1]);
    // Read the startup banner: "...listening on A:PORT" then "...control tap on A:PORT".
    std::string text;
    const int64_t deadline = NowMs() + kWaitMs;
    while ((gamePort == 0 || controlPort == 0) && NowMs() < deadline) {
      pollfd p{fds[0], POLLIN, 0};
      if (poll(&p, 1, 100) <= 0) continue;
      char buf[256];
      const ssize_t n = read(fds[0], buf, sizeof buf);
      if (n <= 0) break;
      text.append(buf, static_cast<size_t>(n));
      auto portAfter = [&](const char* marker) -> uint16_t {
        const size_t at = text.find(marker);
        if (at == std::string::npos) return 0;
        const size_t colon = text.find(':', at);
        const size_t eol = text.find('\n', at);
        if (colon == std::string::npos || eol == std::string::npos) return 0;
        return static_cast<uint16_t>(std::atoi(text.c_str() + colon + 1));
      };
      gamePort = portAfter("listening on");
      controlPort = portAfter("control tap on");
    }
    close(fds[0]);
    if (gamePort == 0 || controlPort == 0) {
      *error = "server did not report its ports (is it the test-control build?)";
      Stop();
      return false;
    }
    return true;
  }

  void Stop() {
    if (pid <= 0) return;
    kill(pid, SIGTERM);
    int status = 0;
    waitpid(pid, &status, 0);
    pid = -1;
  }
};

// Loads `scenario`'s scene into a fresh `room` over the control tap.
inline bool LoadRoom(WsClient& control, const Scenario& scenario, const std::string& room,
                     std::string* error) {
  Json load;
  load.Set("t", "load_scenario");
  load.Set("room", room);
  load.Set("scene", scenario.sceneSpec);
  load.Set("friendly_fire", Json(scenario.friendlyFire));
  if (!scenario.shotRolls.empty()) {
    Json rolls{Json::Array{}};
    for (float r : scenario.shotRolls) rolls.Push(Json(static_cast<double>(r)));
    load.Set("shot_rolls", std::move(rolls));
  }
  control.Send(load.Dump());
  std::vector<std::string> reply;
  const int64_t deadline = NowMs() + kWaitMs;
  while (reply.empty() && NowMs() < deadline) control.Poll(20, &reply);
  Json parsed;
  if (reply.empty() || !Json::Parse(reply.front(), &parsed) || !parsed["ok"].AsBool()) {
    *error = "load_scenario failed: " + (reply.empty() ? std::string("no reply") : reply.front());
    return false;
  }
  return true;
}

// ---- One headless team client ----

struct Player {
  Team team;
  GameLogic game;
  net::RemoteClient client;
  WsClient ws;
  int barrierSeq = 0;
  bool barrierAcked = false, barrierDone = false;

  Player(Team t, const Scene& scene, const std::string& room)
      : team(t), game(scene), client(&game, room) {}
};

class Rig {
 public:
  Rig(uint16_t gamePort, WsClient* control) : gamePort_(gamePort), control_(control) {}

  std::vector<std::string> failures;

  bool Join(const Scenario& scenario, const std::string& room) {
    for (Team team : {Team::Blue, Team::Red}) {
      players_[static_cast<int>(team)] = std::make_unique<Player>(team, scenario.scene, room);
      Player& p = *players_[static_cast<int>(team)];
      std::string error;
      if (!p.ws.Connect(gamePort_, &error)) return Fail("connect: " + error);
      p.client.OnOpen();
      SendOutgoing(p);
      const auto want = team == Team::Blue ? net::RemoteClient::Status::Waiting
                                           : net::RemoteClient::Status::Playing;
      if (!PumpUntil([&] { return p.client.status() == want; })) {
        return Fail(std::string(net::TeamName(team)) + " client never reached expected status: " +
                    p.client.StatusText());
      }
    }
    return PumpUntil([&] { return Blue().client.status() == net::RemoteClient::Status::Playing; }) &&
           Barrier(Blue()) && Barrier(Red());
  }

  Player& Blue() { return *players_[0]; }
  Player& Red() { return *players_[1]; }
  Player& For(Team t) { return *players_[static_cast<int>(t)]; }

  bool Fail(const std::string& why) {
    failures.push_back(why);
    return false;
  }

  // Receives everything pending on both clients (waits up to `ms` for any).
  void Pump(int ms) {
    for (auto& p : players_) {
      if (!p) continue;
      std::vector<std::string> messages;
      if (!p->ws.Poll(ms, &messages) && p->client.status() != net::RemoteClient::Status::Disconnected) {
        p->client.OnClose();
      }
      for (const std::string& text : messages) Dispatch(*p, text);
      ms = 0;  // Only block on the first socket.
    }
  }

  template <typename Pred>
  bool PumpUntil(Pred done) {
    const int64_t deadline = NowMs() + kWaitMs;
    while (!done()) {
      if (NowMs() > deadline) return false;
      Pump(5);
    }
    return true;
  }

  void SendOutgoing(Player& p) {
    for (const std::string& frame : p.client.TakeOutgoing()) p.ws.Send(frame);
  }

  // Ships any plan changes the client's mirror holds, then waits until the
  // server has processed them: a no-op `reaction` (the client's own current
  // playbook) is acked, and answered with the authoritative `plans`, strictly
  // after everything sent before it on the same connection.
  bool Sync(Player& p) {
    p.client.Update(0.0f);
    SendOutgoing(p);
    return Barrier(p);
  }

  bool Barrier(Player& p) {
    net::Action ping;
    ping.kind = net::ActionKind::Reaction;
    ping.playbook = p.game.Playbook(p.team);
    ping.seq = 1000000 + (++p.barrierSeq);
    p.barrierAcked = p.barrierDone = false;
    p.ws.Send(net::EncodeAction(ping).Dump());
    if (!PumpUntil([&] { return p.barrierDone; })) {
      return Fail(std::string(net::TeamName(p.team)) + " barrier timed out (server stopped answering?)");
    }
    return true;
  }

  bool Commit() {
    for (Player* p : {&Blue(), &Red()}) {
      if (!p->game.CanCommitRound()) {
        return Fail("cannot commit: not every living figure (on both teams) has a plan yet");
      }
    }
    Blue().client.RequestCommit();
    SendOutgoing(Blue());
    if (!Barrier(Blue())) return false;
    Red().client.RequestCommit();
    SendOutgoing(Red());
    // Red's commit runs the round; its barrier is acked after the round
    // messages, then blue's (a separate connection) after the same.
    return Barrier(Red()) && Barrier(Blue()) && PlayOut();
  }

  // Advances both mirrors' round playback to the end, as the frame loop would.
  // `playDt` is the step (seconds); `onFrame`, if set, fires after each step
  // (the visual runner renders a video frame there).
  float playDt = 0.1f;
  std::function<void()> onFrame;
  bool PlayOut() {
    for (int i = 0; i < 40000; ++i) {
      if (!Blue().client.playingBack() && !Red().client.playingBack()) return true;
      for (Player* p : {&Blue(), &Red()}) {
        p->client.Update(playDt);
        p->game.UpdateSightingMemory(playDt);
      }
      if (onFrame) onFrame();
    }
    return Fail("round playback never finished");
  }

  bool NewMatch() {
    if (Blue().game.Mode() != InputMode::GameOver) {
      return Fail("new_game is only valid after game over in the networked runner");
    }
    Blue().client.RequestNewMatch();
    SendOutgoing(Blue());
    if (!Barrier(Blue()) || !Barrier(Red())) return false;
    for (Player* p : {&Blue(), &Red()}) {
      p->client.ConsumeMatchStarted();
      p->game.UpdateSightingMemory(0.0f);
    }
    return true;
  }

  // Server-side ground truth for the room (control tap).
  bool DumpState(const std::string& room, Json* dump) {
    Json request;
    request.Set("t", "dump_state");
    request.Set("room", room);
    control_->Send(request.Dump());
    const int64_t deadline = NowMs() + kWaitMs;
    std::vector<std::string> messages;
    while (NowMs() < deadline) {
      if (!control_->Poll(20, &messages)) return Fail("control tap closed");
      if (!messages.empty()) {
        if (!Json::Parse(messages.front(), dump) || (*dump)["t"].AsString() != "state_dump") {
          return Fail("dump_state failed: " + messages.front());
        }
        return true;
      }
    }
    return Fail("dump_state timed out");
  }

 private:
  void Dispatch(Player& p, const std::string& text) {
    // NETDBG=1 traces every server message a client receives.
    if (std::getenv("NETDBG")) std::fprintf(stderr, "[%s] %.300s\n", net::TeamName(p.team), text.c_str());
    Json m;
    if (Json::Parse(text, &m)) {
      const std::string& type = m["t"].AsString();
      const int seq = static_cast<int>(m["seq"].AsNumber(0));
      const bool isBarrier = type == "ack" && seq == 1000000 + p.barrierSeq;
      if (type == "ack" && !m["ok"].AsBool(true) && !isBarrier) {
        Fail(std::string("server rejected ") + net::TeamName(p.team) + "'s action: " + m["error"].AsString());
      }
      if (type == "error") Fail(std::string("server error to ") + net::TeamName(p.team) + ": " + m["error"].AsString());
      if (isBarrier) p.barrierAcked = true;
      else if (type == "plans" && p.barrierAcked) p.barrierDone = true;
    }
    p.client.OnMessage(text);
  }

  uint16_t gamePort_;
  WsClient* control_;
  std::unique_ptr<Player> players_[2];
};


}  // namespace tactics::testing
