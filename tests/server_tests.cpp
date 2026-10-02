// Server tests: WebSocket primitives, authoritative session rules (fog
// filtering, ready handshake), lobby pairing, a full client<->server round
// through RemoteClient, and a real-socket loopback through Server.

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cmath>
#include <deque>
#include <map>
#include <string>
#include <vector>

#include "game/GameLogic.h"
#include "game/MapGenerator.h"
#include "net/ActionApply.h"
#include "net/RemoteClient.h"
#include "server/GameSession.h"
#include "server/Lobby.h"
#include "server/Server.h"
#include "server/WebSocket.h"
#include "test_util.h"

using namespace tactics;
using tactics::net::Action;
using tactics::net::ActionKind;
using tactics::net::Json;
namespace ws = tactics::server::ws;

namespace {

std::string Hex(const std::string& raw) {
  static const char* d = "0123456789abcdef";
  std::string out;
  for (unsigned char c : raw) {
    out.push_back(d[c >> 4]);
    out.push_back(d[c & 15]);
  }
  return out;
}

// Client-side (masked) frame, as a browser would send.
std::string MaskedFrame(ws::Opcode op, const std::string& payload, bool fin = true) {
  std::string out;
  out.push_back(static_cast<char>((fin ? 0x80 : 0) | static_cast<uint8_t>(op)));
  const char mask[4] = {0x12, 0x34, 0x56, 0x78};
  if (payload.size() < 126) {
    out.push_back(static_cast<char>(0x80 | payload.size()));
  } else {
    out.push_back(static_cast<char>(0x80 | 126));
    out.push_back(static_cast<char>(payload.size() >> 8));
    out.push_back(static_cast<char>(payload.size() & 0xFF));
  }
  out.append(mask, 4);
  for (size_t i = 0; i < payload.size(); ++i) out.push_back(payload[i] ^ mask[i % 4]);
  return out;
}

void TestWebSocketPrimitives() {
  CHECK(Hex(ws::Sha1("abc")) == "a9993e364706816aba3e25717850c26c9cd0d89d");
  CHECK(Hex(ws::Sha1("")) == "da39a3ee5e6b4b0d3255bfef95601890afd80709");
  CHECK(ws::Base64Encode("f") == "Zg==");
  CHECK(ws::Base64Encode("fo") == "Zm8=");
  CHECK(ws::Base64Encode("foo") == "Zm9v");
  // RFC 6455 section 1.3 example.
  CHECK(ws::AcceptKey("dGhlIHNhbXBsZSBub25jZQ==") == "s3pPLMBiTxaQ9kYGzzhZRbK+xOo=");

  ws::HttpRequest req;
  size_t used = 0;
  const std::string head =
      "GET /play HTTP/1.1\r\nHost: x\r\nUpgrade: WebSocket\r\nConnection: Upgrade\r\n"
      "Sec-WebSocket-Key: abc\r\nSec-WebSocket-Version: 13\r\n\r\nEXTRA";
  CHECK(ws::ParseHttpRequest(head, 8192, &req, &used) == ws::ParseStatus::Ok);
  CHECK(req.upgrade == "websocket" && req.key == "abc" && req.path == "/play" && head.substr(used) == "EXTRA");
  CHECK(ws::ParseHttpRequest("GET / HTTP/1.1\r\nHost", 8192, &req, &used) == ws::ParseStatus::NeedMore);
  CHECK(ws::ParseHttpRequest(std::string(9000, 'a'), 8192, &req, &used) == ws::ParseStatus::Bad);
  CHECK(ws::ParseHttpRequest("garbage\r\n\r\n", 8192, &req, &used) == ws::ParseStatus::Bad);
}

void TestFrameParser() {
  ws::FrameParser p(1024);
  const std::string frame = MaskedFrame(ws::Opcode::Text, "hello");
  p.Feed(frame.data(), 3);  // Partial.
  CHECK(!p.Next());
  p.Feed(frame.data() + 3, frame.size() - 3);
  auto m = p.Next();
  CHECK(m && m->op == ws::Opcode::Text && m->payload == "hello");
  CHECK(!p.Next() && !p.failed());

  // Fragmented message with an interleaved ping.
  const std::string f1 = MaskedFrame(ws::Opcode::Text, "foo", false);
  const std::string ping = MaskedFrame(ws::Opcode::Ping, "p");
  const std::string f2 = MaskedFrame(ws::Opcode::Continuation, "bar", true);
  const std::string all = f1 + ping + f2;
  p.Feed(all.data(), all.size());
  m = p.Next();
  CHECK(m && m->op == ws::Opcode::Ping);
  m = p.Next();
  CHECK(m && m->payload == "foobar");

  // 126-length extended header.
  const std::string big(300, 'z');
  const std::string bf = MaskedFrame(ws::Opcode::Text, big);
  p.Feed(bf.data(), bf.size());
  m = p.Next();
  CHECK(m && m->payload == big);

  // Unmasked client frames are a protocol error.
  ws::FrameParser unmasked(1024);
  const std::string um = ws::EncodeFrame(ws::Opcode::Text, "x");
  unmasked.Feed(um.data(), um.size());
  CHECK(!unmasked.Next() && unmasked.failed() && unmasked.failure_code() == 1002);

  // Oversize message.
  ws::FrameParser small(16);
  const std::string of = MaskedFrame(ws::Opcode::Text, std::string(100, 'a'));
  small.Feed(of.data(), of.size());
  CHECK(!small.Next() && small.failed() && small.failure_code() == 1009);

  // Server encoder shape.
  CHECK(ws::EncodeFrame(ws::Opcode::Text, "hi") == std::string("\x81\x02hi", 4));
  CHECK(ws::EncodeFrame(ws::Opcode::Text, std::string(200, 'a')).size() == 204);
}

Action Plan(ActionKind kind, int unit, int target = -1) {
  Action a;
  a.kind = kind;
  a.unit = unit;
  a.target = target;
  return a;
}

// Plans a short walk for `unit` by trying a few directions through the
// validated action path; returns the accepted action.
bool PlanSomeMove(server::GameSession& s, Team team, int unitId, Action* accepted = nullptr) {
  const glm::vec3 p = s.game().FindUnit(unitId)->position;
  for (int i = 0; i < 16; ++i) {
    const float ang = i * 0.3927f;
    Action mv = Plan(ActionKind::Move, unitId);
    mv.waypoints = {p + glm::vec3(std::cos(ang) * 5.0f, 0.0f, std::sin(ang) * 5.0f)};
    if (s.Apply(team, mv).ok) {
      if (accepted) *accepted = mv;
      return true;
    }
  }
  return false;
}

void PlanEveryone(server::GameSession& s, Team team) {
  for (const Unit& u : s.game().GetScene().units) {
    if (u.team != team) continue;
    if (!PlanSomeMove(s, team, u.id)) s.Apply(team, Plan(ActionKind::Pass, u.id));
  }
}

void TestSessionReadyAndRound() {
  server::GameSession s(1);
  Action commit;
  commit.kind = ActionKind::Commit;
  CHECK(!s.Apply(Team::Blue, commit).ok);  // Nothing planned yet.
  PlanEveryone(s, Team::Blue);
  CHECK(s.Apply(Team::Blue, commit).ok && s.Ready(Team::Blue) && !s.BothReady());

  // Editing a plan un-readies that team.
  CHECK(s.Apply(Team::Blue, Plan(ActionKind::Pass, 0)).ok);
  CHECK(!s.Ready(Team::Blue));
  CHECK(s.Apply(Team::Blue, commit).ok);

  // A team can't act for the other side.
  CHECK(!s.Apply(Team::Blue, Plan(ActionKind::Pass, 3)).ok);

  PlanEveryone(s, Team::Red);
  CHECK(s.Apply(Team::Red, commit).ok && s.BothReady());

  const auto round = s.RunRound();
  CHECK(!s.Ready(Team::Blue) && !s.Ready(Team::Red));
  for (int ti = 0; ti < 2; ++ti) {
    CHECK(round[ti]["t"].AsString() == "round");
    CHECK(round[ti]["frames"].AsArray().size() >= 2);
    CHECK(round[ti]["state"]["round"].AsNumber() == 2 || round[ti]["state"]["over"].AsBool());
    // Rule state only: no animation-interpolation fields anywhere in the message.
    const std::string text = round[ti].Dump();
    for (const char* banned : {"walkPhase", "walkBlend", "idleElapsed", "knockdown", "shootElapsed"}) {
      CHECK(text.find(banned) == std::string::npos);
    }
    // Every unit sample is [id,x,y,z,yaw,alive,moving] and own units always appear.
    for (const Json& f : round[ti]["frames"].AsArray()) {
      int own = 0;
      for (const Json& row : f["u"].AsArray()) {
        CHECK(row.AsArray().size() == 7);
        const int id = static_cast<int>(row.AsArray()[0].AsNumber());
        if ((id < 3) == (ti == 0)) ++own;
      }
      CHECK(own == 3 || round[ti]["state"]["over"].AsBool() || own > 0);
    }
  }
  // Planning reopened (or game over).
  CHECK(s.game().Mode() != InputMode::Executing);
}

void TestSessionFogHidesEnemyPlansAndUnits() {
  server::GameSession s(1);
  PlanEveryone(s, Team::Red);
  const Json plans = s.PlansMessage(Team::Blue);
  CHECK(plans["plans"].AsArray().size() == 3);  // Own team only.
  for (const Json& p : plans["plans"].AsArray()) CHECK(p["unit"].AsNumber() < 3);
  // Squads start out of sight of each other, so no enemy appears in Blue's state.
  const Json state = s.StateMessage(Team::Blue);
  int enemies = 0;
  for (const Json& u : state["units"].AsArray()) enemies += u["id"].AsNumber() >= 3;
  CHECK(enemies == 0 || s.game().ComputeVisibility(Team::Blue).visibleUnit.size() > 0);
  // Enemy figures are only reported if actually visible.
  for (const Json& u : state["units"].AsArray()) {
    const int id = static_cast<int>(u["id"].AsNumber());
    if (id >= 3) CHECK(s.game().ComputeVisibility(Team::Blue).UnitVisible(id));
    if (id >= 3) CHECK(!u.Has("reaction"));  // Enemy rules aren't leaked.
  }
}

// In-memory transport for the lobby.
struct Wire {
  std::map<int, std::deque<std::string>> inbox;
  server::Lobby lobby;
  uint32_t nextSeed = 7;
  Wire()
      : lobby([this](int c, const std::string& t) { inbox[c].push_back(t); },
              [this]() { return nextSeed++; }) {}
  void Send(int conn, const std::string& text) { lobby.OnMessage(conn, text); }
  std::vector<Json> Drain(int conn) {
    std::vector<Json> out;
    for (const auto& t : inbox[conn]) {
      Json j;
      CHECK(Json::Parse(t, &j));
      out.push_back(j);
    }
    inbox[conn].clear();
    return out;
  }
};

bool Has(const std::vector<Json>& msgs, const char* type) {
  for (const Json& m : msgs) if (m["t"].AsString() == type) return true;
  return false;
}

void TestLobbyPairing() {
  Wire w;
  w.Send(1, R"({"t":"pass","unit":0})");
  CHECK(w.Drain(1)[0]["t"].AsString() == "error");  // Must join first.
  w.Send(1, "not json");
  CHECK(w.Drain(1)[0]["t"].AsString() == "error");
  w.Send(1, R"({"t":"join","room":"bad room!"})");
  CHECK(w.Drain(1)[0]["t"].AsString() == "error");

  w.Send(1, R"({"t":"join","room":"r1"})");
  CHECK(Has(w.Drain(1), "waiting"));
  w.Send(1, R"({"t":"pass","unit":0})");
  CHECK(w.Drain(1)[0]["t"].AsString() == "error");  // No opponent yet.

  w.Send(2, R"({"t":"join","room":"other"})");  // Different room: stays waiting.
  CHECK(Has(w.Drain(2), "waiting"));
  w.Send(3, R"({"t":"join","room":"r1"})");
  const auto m1 = w.Drain(1), m3 = w.Drain(3);
  CHECK(Has(m1, "start") && Has(m3, "start"));
  CHECK(m1[0]["team"].AsString() == "blue" && m3[0]["team"].AsString() == "red");
  CHECK(m1[0]["seed"].AsNumber() == m3[0]["seed"].AsNumber());
  CHECK(Has(m1, "plans") && Has(m1, "state"));

  w.Send(4, R"({"t":"join","room":"r1"})");
  CHECK(w.Drain(4)[0]["t"].AsString() == "error");  // Full.

  // Illegal action is nacked with the seq and the authoritative plans follow.
  w.Send(1, R"({"t":"pass","unit":4,"seq":5})");
  auto r = w.Drain(1);
  CHECK(r[0]["t"].AsString() == "ack" && !r[0]["ok"].AsBool() && r[0]["seq"].AsNumber() == 5);
  CHECK(Has(r, "plans"));
  CHECK(w.Drain(3).empty());  // The opponent hears nothing about it.

  // A legal plan reveals nothing to the opponent either.
  w.Send(1, R"({"t":"pass","unit":0,"seq":6})");
  CHECK(w.Drain(1)[0]["ok"].AsBool());
  CHECK(w.Drain(3).empty());

  w.lobby.OnDisconnect(3);
  CHECK(Has(w.Drain(1), "opponent_left"));
  CHECK(w.lobby.RoomCount() == 1);  // Only "other" remains.
  w.Send(1, R"({"t":"join","room":"r1"})");  // Survivor may re-join.
  CHECK(Has(w.Drain(1), "waiting"));
}

// Drives a RemoteClient (with its own mirror game) through the lobby.
struct ClientRig {
  GameLogic game;
  net::RemoteClient client;
  int conn;
  ClientRig(int c, const char* room) : game(GenerateUrbanMap(1)), client(&game, room), conn(c) {}
  void Pump(Wire& w) {
    for (const auto& t : client.TakeOutgoing()) w.Send(conn, t);
    for (const auto& t : w.inbox[conn]) client.OnMessage(t);
    w.inbox[conn].clear();
  }
};

void PlanAllLocally(ClientRig& rig) {
  const Team team = *rig.client.team();
  for (const Unit& u : rig.game.GetScene().units) {
    if (u.team != team) continue;
    bool done = false;
    for (int i = 0; i < 16 && !done; ++i) {
      const float ang = i * 0.3927f;
      const glm::vec3 dest = u.position + glm::vec3(std::cos(ang) * 5.0f, 0.0f, std::sin(ang) * 5.0f);
      rig.game.ClickUnit(u.id, team);
      rig.game.ChooseMove();
      rig.game.ClickGround(dest, team);
      rig.game.FinishMovePlan();
      done = rig.game.Mode() == InputMode::AwaitingSelection;
      if (!done) {
        rig.game.CancelAction();
        rig.game.CancelAction();
      }
    }
    if (!done) {
      rig.game.ClickUnit(u.id, team);
      rig.game.ChoosePass();
    }
  }
}

void TestClientServerRound() {
  Wire w;
  ClientRig blue(1, "t"), red(2, "t");
  blue.client.OnOpen();
  red.client.OnOpen();
  CHECK(blue.client.status() == net::RemoteClient::Status::Connecting);
  blue.Pump(w);
  blue.Pump(w);
  CHECK(blue.client.status() == net::RemoteClient::Status::Waiting);
  red.Pump(w);
  blue.Pump(w);
  red.Pump(w);
  CHECK(blue.client.status() == net::RemoteClient::Status::Playing);
  CHECK(red.client.status() == net::RemoteClient::Status::Playing);
  CHECK(blue.client.team() == Team::Blue && red.client.team() == Team::Red);
  CHECK(blue.client.ConsumeMatchStarted() && !blue.client.ConsumeMatchStarted());
  // Both mirrors adopted the server's seed: identical maps.
  CHECK(blue.game.GetScene().obstacles.size() == red.game.GetScene().obstacles.size());

  // Plan locally through the normal click flow; the client turns that into actions.
  PlanAllLocally(blue);
  PlanAllLocally(red);
  CHECK(blue.game.CanCommitRound());  // Enemy plans are placeholders: only our own gate the button.
  blue.client.Update(0.016f);
  red.client.Update(0.016f);
  blue.Pump(w);
  red.Pump(w);
  blue.Pump(w);
  red.Pump(w);

  // Commit from Blue only: server waits for Red, tells Blue nothing more, Red sees "ready".
  blue.client.RequestCommit();
  blue.Pump(w);
  red.Pump(w);
  blue.Pump(w);
  CHECK(blue.client.selfReady() && !blue.client.peerReady());
  CHECK(red.client.peerReady() && !red.client.selfReady());
  CHECK(!blue.client.playingBack());

  red.client.RequestCommit();
  red.Pump(w);
  blue.Pump(w);
  red.Pump(w);
  CHECK(blue.client.playingBack() && red.client.playingBack());
  CHECK(blue.game.Mode() == InputMode::Executing);

  // Play it out frame by frame.
  const Team bt = Team::Blue;
  bool sawMotion = false;
  const glm::vec3 start = blue.game.FindUnit(0)->position;
  for (int i = 0; i < 600 && blue.client.playingBack(); ++i) {
    blue.client.Update(1.0f / 30.0f);
    red.client.Update(1.0f / 30.0f);
    if (glm::distance(blue.game.FindUnit(0)->position, start) > 0.5f) sawMotion = true;
  }
  CHECK(sawMotion);
  CHECK(!blue.client.playingBack() && !red.client.playingBack());
  (void)bt;

  // Mirror's own units equal the server's authoritative positions after the round.
  // (No handle to the session here; compare the two mirrors' views of the same
  // visible/own units instead: a unit visible to both agrees.)
  CHECK(blue.game.Mode() != InputMode::Executing);
  for (int id = 0; id < 3; ++id) {
    const Unit* mine = blue.game.FindUnit(id);
    CHECK(mine->position.x < 5000.0f);  // Own units never parked.
    const Unit* theirs = red.game.FindUnit(id);
    if (theirs->position.x < 5000.0f) CHECK(glm::distance(mine->position, theirs->position) < 0.05f);
  }
  // Unseen enemies are parked, not leaked.
  for (int id = 3; id < 6; ++id) {
    const Unit* e = blue.game.FindUnit(id);
    if (e->position.x >= 5000.0f) CHECK(!blue.game.ComputeVisibility(Team::Blue).UnitVisible(id));
  }
  CHECK(blue.game.RoundNumber() == 2 || blue.game.Mode() == InputMode::GameOver);

  // Disconnect propagates.
  w.lobby.OnDisconnect(2);
  blue.Pump(w);
  CHECK(blue.client.status() == net::RemoteClient::Status::OpponentLeft);
}

void TestRejectedActionResyncs() {
  Wire w;
  ClientRig blue(1, "t"), red(2, "t");
  blue.client.OnOpen();
  red.client.OnOpen();
  for (int i = 0; i < 3; ++i) { blue.Pump(w); red.Pump(w); }
  // Forge an out-of-range plan locally; the client queues it as an action.
  Unit* u = blue.game.FindUnit(0);
  u->plan.type = PlannedActionType::Move;
  u->plan.movePath = {u->position, u->position + glm::vec3(900.0f, 0.0f, 0.0f)};
  blue.client.Update(0.016f);
  blue.Pump(w);  // Sends the forged action; the server nacks and replies with its plans.
  blue.Pump(w);
  CHECK(blue.game.FindUnit(0)->plan.type == PlannedActionType::None);  // Authoritative plan restored.
}

// ---- Real sockets ----

int ConnectLoopback(uint16_t port) {
  const int fd = socket(AF_INET, SOCK_STREAM, 0);
  sockaddr_in a{};
  a.sin_family = AF_INET;
  a.sin_port = htons(port);
  inet_pton(AF_INET, "127.0.0.1", &a.sin_addr);
  if (connect(fd, reinterpret_cast<sockaddr*>(&a), sizeof a) != 0) return -1;
  return fd;
}

void SendAll(int fd, const std::string& s) { CHECK(send(fd, s.data(), s.size(), 0) == (ssize_t)s.size()); }

// Pumps the server while reading from fd until `needle` shows up (or time out).
std::string ReadUntil(server::Server& srv, int fd, const std::string& needle, std::string* acc) {
  for (int i = 0; i < 200 && acc->find(needle) == std::string::npos; ++i) {
    srv.RunOnce(10);
    char buf[8192];
    const ssize_t n = recv(fd, buf, sizeof buf, MSG_DONTWAIT);
    if (n > 0) acc->append(buf, static_cast<size_t>(n));
  }
  return *acc;
}

void TestRealSocketLoopback() {
  server::ServerConfig cfg;
  cfg.port = 0;
  cfg.bindAddress = "127.0.0.1";
  server::Server srv(cfg);
  std::string err;
  CHECK(srv.Listen(&err));
  const uint16_t port = srv.port();
  CHECK(port != 0);

  // Health endpoint.
  int h = ConnectLoopback(port);
  SendAll(h, "GET /health HTTP/1.1\r\nHost: x\r\n\r\n");
  std::string hb;
  ReadUntil(srv, h, "ok", &hb);
  CHECK(hb.find("200 OK") != std::string::npos);
  close(h);

  const std::string handshake =
      "GET / HTTP/1.1\r\nHost: x\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
      "Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\nSec-WebSocket-Version: 13\r\n\r\n";
  int a = ConnectLoopback(port), b = ConnectLoopback(port);
  SendAll(a, handshake);
  SendAll(b, handshake);
  std::string ra, rb;
  ReadUntil(srv, a, "\r\n\r\n", &ra);
  ReadUntil(srv, b, "\r\n\r\n", &rb);
  CHECK(ra.find("101 Switching Protocols") != std::string::npos);
  CHECK(ra.find("s3pPLMBiTxaQ9kYGzzhZRbK+xOo=") != std::string::npos);

  SendAll(a, MaskedFrame(ws::Opcode::Text, R"({"t":"join","room":"loop"})"));
  ReadUntil(srv, a, "waiting", &ra);
  CHECK(ra.find("waiting") != std::string::npos);
  SendAll(b, MaskedFrame(ws::Opcode::Text, R"({"t":"join","room":"loop"})"));
  ReadUntil(srv, a, "\"team\":\"blue\"", &ra);
  ReadUntil(srv, b, "\"team\":\"red\"", &rb);
  CHECK(ra.find("\"team\":\"blue\"") != std::string::npos);
  CHECK(rb.find("\"team\":\"red\"") != std::string::npos);

  // Ping gets a pong; closing one side tells the other.
  SendAll(a, MaskedFrame(ws::Opcode::Ping, "x"));
  std::string before = ra;
  for (int i = 0; i < 50; ++i) {
    srv.RunOnce(10);
    char buf[4096];
    const ssize_t n = recv(a, buf, sizeof buf, MSG_DONTWAIT);
    if (n > 0) ra.append(buf, static_cast<size_t>(n));
  }
  CHECK(ra.size() > before.size());
  close(a);
  ReadUntil(srv, b, "opponent_left", &rb);
  CHECK(rb.find("opponent_left") != std::string::npos);
  close(b);

  // Non-websocket junk and garbage frames don't take the server down.
  int junk = ConnectLoopback(port);
  SendAll(junk, "\x01\x02\x03\r\n\r\n");
  for (int i = 0; i < 20; ++i) srv.RunOnce(5);
  close(junk);
  int c = ConnectLoopback(port);
  SendAll(c, handshake);
  std::string rc;
  ReadUntil(srv, c, "\r\n\r\n", &rc);
  SendAll(c, ws::EncodeFrame(ws::Opcode::Text, "unmasked"));  // Protocol violation.
  ReadUntil(srv, c, "\x88", &rc);
  CHECK(rc.find('\x88') != std::string::npos);  // Close frame.
  close(c);
  for (int i = 0; i < 20; ++i) srv.RunOnce(5);
}

}  // namespace

int main() {
  TestWebSocketPrimitives();
  TestFrameParser();
  TestSessionReadyAndRound();
  TestSessionFogHidesEnemyPlansAndUnits();
  TestLobbyPairing();
  TestClientServerRound();
  TestRejectedActionResyncs();
  TestRealSocketLoopback();
  if (g_failures == 0) {
    std::printf("All server tests passed.\n");
    return 0;
  }
  std::fprintf(stderr, "%d check(s) failed.\n", g_failures);
  return 1;
}
