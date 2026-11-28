#include "net/GamesClient.h"

#include <cstdio>

#include "net/Json.h"

namespace tactics::net {

void GamesClient::OnOpen() {
  open_ = true;
  loaded_ = false;
  Json ask;
  ask.Set("t", "my_games");
  ask.Set("token", token_);
  outgoing_.push_back(ask.Dump());
}

std::vector<std::string> GamesClient::TakeOutgoing() {
  std::vector<std::string> out;
  out.swap(outgoing_);
  return out;
}

bool GamesClient::OnMessage(const std::string& text) {
  Json m;
  if (!Json::Parse(text, &m) || !m["t"].IsString()) return false;
  if (m["t"].AsString() == "error") {
    error_ = m["error"].AsString();
    return false;
  }
  if (m["t"].AsString() != "games") return false;
  entries_.clear();
  for (const Json& g : m["games"].AsArray()) {
    Entry e;
    e.id = g["id"].AsString();
    e.finished = g["status"].AsString() == "finished";
    e.team = g["team"].AsString();
    e.round = static_cast<int>(g["round"].AsNumber(1));
    e.winner = g["winner"].AsString();
    e.updatedMs = g["updated"].AsNumber();
    if (!e.id.empty()) entries_.push_back(std::move(e));
  }
  loaded_ = true;
  error_.clear();
  return true;
}

const GamesClient::Entry* GamesClient::LatestActive() const {
  const Entry* best = nullptr;
  for (const Entry& e : entries_) {
    if (!e.finished && (!best || e.updatedMs > best->updatedMs)) best = &e;
  }
  return best;
}

std::string GamesClient::Title(const Entry& e) {
  auto cap = [](std::string s) {
    if (!s.empty()) s[0] = static_cast<char>(s[0] - 'a' + 'A');
    return s;
  };
  if (e.finished) {
    return cap(e.team) + " - finished, " +
           (e.winner == "none" || e.winner.empty() ? std::string("draw") : cap(e.winner) + " won");
  }
  return cap(e.team) + " - round " + std::to_string(e.round) + " - active";
}

std::string GamesClient::NewToken(unsigned long long a, unsigned long long b) {
  char buf[40];
  std::snprintf(buf, sizeof buf, "%016llx%016llx", a, b);
  return buf;
}

}  // namespace tactics::net
