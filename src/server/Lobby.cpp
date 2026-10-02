#include "server/Lobby.h"

namespace tactics::server {

using net::Json;

namespace {

bool ValidRoomName(const std::string& s) {
  if (s.empty() || s.size() > 32) return false;
  for (char c : s) {
    const bool ok = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') ||
                    c == '-' || c == '_';
    if (!ok) return false;
  }
  return true;
}

Json Typed(const char* type) {
  Json j;
  j.Set("t", type);
  return j;
}

}  // namespace

void Lobby::SendError(int conn, const std::string& why) {
  Json j = Typed("error");
  j.Set("error", why);
  Send(conn, j);
}

void Lobby::OnMessage(int conn, const std::string& text) {
  Json message;
  if (!Json::Parse(text, &message) || !message.IsObject()) {
    SendError(conn, "malformed message");
    return;
  }
  auto it = members_.find(conn);
  if (it == members_.end()) {
    if (message["t"].IsString() && message["t"].AsString() == "join") {
      Join(conn, message);
    } else {
      SendError(conn, "send join first");
    }
    return;
  }
  HandleAction(conn, it->second, message);
}

void Lobby::Join(int conn, const Json& message) {
  std::string name = "default";
  if (message.Has("room")) {
    if (!message["room"].IsString() || !ValidRoomName(message["room"].AsString())) {
      SendError(conn, "invalid room name (1-32 chars of A-Z a-z 0-9 - _)");
      return;
    }
    name = message["room"].AsString();
  }
  Room& room = rooms_[name];
  if (room.conns[0] >= 0 && room.conns[1] >= 0) {
    SendError(conn, "room is full");
    return;
  }
  if (room.conns[0] < 0) {
    room.conns[0] = conn;
    members_[conn] = Member{name, Team::Blue};
    Send(conn, Typed("waiting"));
    return;
  }
  room.conns[1] = conn;
  members_[conn] = Member{name, Team::Red};
  room.session = std::make_unique<GameSession>(seed_());
  StartMatch(room);
}

void Lobby::StartMatch(Room& room) {
  for (int ti = 0; ti < 2; ++ti) {
    const Team team = static_cast<Team>(ti);
    Json start = Typed("start");
    start.Set("team", net::TeamName(team));
    start.Set("seed", static_cast<double>(room.session->seed()));
    start.Set("protocol", net::kProtocolVersion);
    Send(room.conns[ti], start);
    Send(room.conns[ti], room.session->PlansMessage(team));
    Send(room.conns[ti], room.session->StateMessage(team));
  }
}

void Lobby::HandleAction(int conn, Member& member, const Json& message) {
  auto roomIt = rooms_.find(member.room);
  if (roomIt == rooms_.end() || !roomIt->second.session) {
    SendError(conn, "waiting for an opponent");
    return;
  }
  Room& room = roomIt->second;
  GameSession& session = *room.session;
  const Team team = member.team;
  const Team other = OpposingTeam(team);

  net::Action action;
  std::string error;
  if (!net::ParseAction(message, &action, &error)) {
    SendError(conn, error);
    return;
  }
  const bool wasReady = session.Ready(team);
  const GameSession::Result result = session.Apply(
      team, action, action.kind == net::ActionKind::NewMatch ? seed_() : 0);

  Json ack = Typed("ack");
  ack.Set("seq", action.seq);
  ack.Set("ok", Json(result.ok));
  if (!result.ok) ack.Set("error", result.error);
  Send(conn, ack);

  if (action.kind == net::ActionKind::NewMatch && result.ok) {
    StartMatch(room);
    return;
  }
  // Always answer with the authoritative plans so a rejected action (or a
  // stale client) resyncs; the opponent only hears about readiness.
  Send(conn, session.PlansMessage(team));
  if (session.Ready(team) != wasReady) {
    Json peer = Typed("peer");
    peer.Set("ready", Json(session.Ready(team)));
    Send(room.conns[static_cast<int>(other)], peer);
  }

  if (session.BothReady()) {
    const auto reports = session.RunRound();
    for (int ti = 0; ti < 2; ++ti) {
      const Team t = static_cast<Team>(ti);
      Send(room.conns[ti], reports[ti]);
      Send(room.conns[ti], session.PlansMessage(t));
      Send(room.conns[ti], Json(Json::Object{{"t", Json("peer")}, {"ready", Json(false)}}));
    }
  }
}

void Lobby::OnDisconnect(int conn) {
  auto it = members_.find(conn);
  if (it == members_.end()) return;
  const Member member = it->second;
  members_.erase(it);
  auto roomIt = rooms_.find(member.room);
  if (roomIt == rooms_.end()) return;
  Room& room = roomIt->second;
  const int otherConn = room.conns[static_cast<int>(OpposingTeam(member.team))];
  if (otherConn >= 0) {
    Send(otherConn, Typed("opponent_left"));
    members_.erase(otherConn);  // The match is over; the survivor may join again.
  }
  rooms_.erase(roomIt);
}

}  // namespace tactics::server
