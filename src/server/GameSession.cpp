#include "server/GameSession.h"

#include <cmath>

#include "game/MapGenerator.h"
#include "net/ActionApply.h"

namespace tactics::server {

using net::Json;

namespace {

Json Num(double v) { return Json(v); }

bool Visible(const GameLogic& game, Team viewer, const Unit& unit, const TeamVisibility& vis) {
  (void)game;
  return unit.team == viewer || vis.UnitVisible(unit.id);
}

}  // namespace

GameSession::GameSession(uint32_t seed) : seed_(seed), game_(GenerateUrbanMap(seed)) {}

GameSession::GameSession(const Scene& scene) : seed_(0), fixedScene_(scene), game_(scene) {}

void GameSession::ConfigureForTest(bool friendlyFire, std::vector<float> shotRolls) {
  game_.SetFriendlyFireEnabled(friendlyFire);
  if (shotRolls.empty()) return;
  game_.SetShotRollSource([rolls = std::move(shotRolls), next = size_t{0}]() mutable {
    return rolls[next++ % rolls.size()];
  });
}

GameSession::Result GameSession::Apply(Team team, const net::Action& action, uint32_t newSeed) {
  Result result;
  if (action.kind == net::ActionKind::Commit) {
    if (game_.Mode() == InputMode::Executing || game_.Mode() == InputMode::GameOver) {
      return {false, "planning is closed"};
    }
    for (const Unit& unit : game_.GetScene().units) {
      if (unit.team == team && unit.alive && unit.plan.type == PlannedActionType::None) {
        return {false, "every living figure needs a plan before committing"};
      }
    }
    ready_[static_cast<int>(team)] = true;
    return result;
  }
  if (action.kind == net::ActionKind::NewMatch) {
    if (game_.Mode() != InputMode::GameOver) return {false, "match is still running"};
    seed_ = newSeed;
    game_.Reset(fixedScene_ ? *fixedScene_ : GenerateUrbanMap(seed_));
    ready_ = {false, false};
    return result;
  }
  if (!net::ApplyPlanAction(game_, team, action, &result.error)) {
    result.ok = false;
    return result;
  }
  if (action.kind != net::ActionKind::Focus && action.kind != net::ActionKind::Reaction) {
    ready_[static_cast<int>(team)] = false;
  }
  return result;
}

std::array<Json, 2> GameSession::RunRound() {
  std::array<Json, 2> out;
  std::array<Json, 2> frames{Json(Json::Array{}), Json(Json::Array{})};
  std::array<Json, 2> shots{Json(Json::Array{}), Json(Json::Array{})};
  ready_ = {false, false};
  game_.CommitRound();

  const auto& units = game_.GetScene().units;
  std::vector<float> lastShoot(units.size(), -1.0f);
  float t = 0.0f;
  int step = 0;

  auto sampleFrame = [&]() {
    for (int ti = 0; ti < 2; ++ti) {
      const Team viewer = static_cast<Team>(ti);
      const TeamVisibility vis = game_.ComputeVisibility(viewer);
      Json list{Json::Array{}};
      for (const Unit& u : units) {
        if (!Visible(game_, viewer, u, vis)) continue;
        list.Push(Json(Json::Array{Num(u.id), Num(u.position.x), Num(u.position.y), Num(u.position.z),
                                   Num(u.facingYaw), Num(u.alive ? 1 : 0),
                                   Num(game_.IsUnitMoving(u.id) ? 1 : 0)}));
      }
      Json frame;
      frame.Set("t", Num(std::round(t * 100.0) / 100.0));
      frame.Set("u", std::move(list));
      frames[ti].Push(std::move(frame));
    }
  };

  sampleFrame();
  // Safety cap far above a round's kRoundDuration so a logic bug can't hang the server.
  const int maxSteps = static_cast<int>(constants::kRoundDuration / kSimStep) * 8;
  while (game_.Mode() == InputMode::Executing && step < maxSteps) {
    game_.Update(kSimStep);
    t += kSimStep;
    ++step;
    for (size_t i = 0; i < units.size(); ++i) {
      const float s = units[i].shootElapsed;
      const bool fired = s >= 0.0f && (lastShoot[i] < 0.0f || s < lastShoot[i]);
      lastShoot[i] = s;
      if (!fired) continue;
      for (int ti = 0; ti < 2; ++ti) {
        const Team viewer = static_cast<Team>(ti);
        if (!Visible(game_, viewer, units[i], game_.ComputeVisibility(viewer))) continue;
        Json shot;
        shot.Set("t", Num(std::round(t * 100.0) / 100.0));
        shot.Set("unit", Num(units[i].id));
        shot.Set("yaw", Num(units[i].shootAimYaw));
        shots[ti].Push(std::move(shot));
      }
    }
    if (step % kFrameEvery == 0 || game_.Mode() != InputMode::Executing) sampleFrame();
  }

  for (int ti = 0; ti < 2; ++ti) {
    const Team team = static_cast<Team>(ti);
    Json msg;
    msg.Set("t", "round");
    msg.Set("round", Num(game_.RoundNumber()));
    msg.Set("duration", Num(std::round(t * 100.0) / 100.0));
    msg.Set("frames", std::move(frames[ti]));
    msg.Set("shots", std::move(shots[ti]));
    msg.Set("state", StateBody(team));
    out[ti] = std::move(msg);
  }
  return out;
}

Json GameSession::StateBody(Team team) const {
  const TeamVisibility vis = game_.ComputeVisibility(team);
  Json body;
  body.Set("round", Num(game_.RoundNumber()));
  body.Set("over", Json(game_.Mode() == InputMode::GameOver));
  if (game_.Mode() == InputMode::GameOver) {
    body.Set("winner", game_.Winner() ? net::TeamName(*game_.Winner()) : "none");
  }
  Json list{Json::Array{}};
  for (const Unit& u : game_.GetScene().units) {
    const bool own = u.team == team;
    if (!own && !vis.UnitVisible(u.id)) continue;  // Fog: hidden enemies aren't reported at all.
    Json j;
    j.Set("id", Num(u.id));
    j.Set("pos", net::EncodeVec3(u.position));
    j.Set("yaw", Num(u.facingYaw));
    j.Set("alive", Json(u.alive));
    list.Push(std::move(j));
  }
  body.Set("units", std::move(list));
  body.Set("playbook", net::EncodePlaybook(game_.Playbook(team)));
  return body;
}

Json GameSession::DumpState() const {
  Json dump;
  dump.Set("t", "state_dump");
  dump.Set("round", Num(game_.RoundNumber()));
  dump.Set("over", Json(game_.Mode() == InputMode::GameOver));
  dump.Set("winner", game_.Winner() ? Json(net::TeamName(*game_.Winner())) : Json());
  Json units{Json::Array{}};
  for (const Unit& u : game_.GetScene().units) {
    Json j;
    j.Set("id", Num(u.id));
    j.Set("team", net::TeamName(u.team));
    j.Set("pos", net::EncodeVec3(u.position));
    j.Set("yaw", Num(u.facingYaw));
    j.Set("alive", Json(u.alive));
    units.Push(std::move(j));
  }
  dump.Set("units", std::move(units));
  for (Team team : {Team::Blue, Team::Red}) {
    const TeamVisibility vis = game_.ComputeVisibility(team);
    Json seen{Json::Array{}};
    for (const Unit& u : game_.GetScene().units) {
      if (vis.UnitVisible(u.id)) seen.Push(Num(u.id));
    }
    dump.Set(std::string("visible_") + net::TeamName(team), std::move(seen));
  }
  return dump;
}

Json GameSession::StateMessage(Team team) const {
  Json msg = StateBody(team);
  msg.Set("t", "state");
  return msg;
}

Json GameSession::PlansMessage(Team team) const {
  Json msg;
  msg.Set("t", "plans");
  msg.Set("round", Num(game_.RoundNumber()));
  msg.Set("ready", Json(Ready(team)));
  Json list{Json::Array{}};
  for (const Unit& u : game_.GetScene().units) {
    if (u.team != team || !u.alive) continue;
    list.Push(net::EncodeAction(net::PlanToAction(u)));
  }
  msg.Set("plans", std::move(list));
  msg.Set("playbook", net::EncodePlaybook(game_.Playbook(team)));
  return msg;
}

}  // namespace tactics::server
