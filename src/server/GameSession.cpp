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

// Report slot 2 is the spectator's: unfiltered, so it never needs a TeamVisibility.
constexpr int kSpectatorSlot = 2;

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
  // A commit locks the team's plans until the round runs: accepting later
  // edits would silently un-ready the team (a "did my commit do anything?"
  // trap for the client). Focus stays allowed as a validated no-op.
  if (ready_[static_cast<int>(team)] && action.kind != net::ActionKind::Focus) {
    return {false, "plans are locked in; waiting for the opponent to commit"};
  }
  if (!net::ApplyPlanAction(game_, team, action, &result.error)) {
    result.ok = false;
    return result;
  }
  return result;
}

GameSession::Result GameSession::ApplyBotPlan(Team team, const ai::JevRequest* request,
                                               const std::vector<std::string>& choices) {
  if (!PlanningOpen()) return {false, "planning is closed"};
  if (ready_[static_cast<int>(team)]) return {false, "team is already committed"};
  if (request && !ai::ApplyJevChoice(&game_, *request, choices)) {
    return {false, "Jev's plan is no longer legal"};
  }
  net::Action commit;
  commit.kind = net::ActionKind::Commit;
  return Apply(team, commit);
}

std::array<Json, 3> GameSession::RunRound() {
  std::array<Json, 3> out;
  const Json empty{Json::Array{}};
  std::array<Json, 3> frames{empty, empty, empty};
  std::array<Json, 3> shots{empty, empty, empty};
  std::array<Json, 3> tracers{empty, empty, empty};
  ready_ = {false, false};
  game_.CommitRound();

  const auto& units = game_.GetScene().units;
  std::vector<float> lastShoot(units.size(), -1.0f);
  float t = 0.0f;
  int step = 0;
  // Tracers already forwarded this round.
  size_t tracersSent = 0;

  auto sampleFrame = [&]() {
    for (int ti = 0; ti < 3; ++ti) {
      const bool spectator = ti == kSpectatorSlot;
      const Team viewer = spectator ? Team::Blue : static_cast<Team>(ti);
      const TeamVisibility vis = game_.ComputeVisibility(viewer);
      Json list{Json::Array{}};
      for (const Unit& u : units) {
        if (!spectator && !Visible(game_, viewer, u, vis)) continue;
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
      for (int ti = 0; ti < 3; ++ti) {
        const Team viewer = static_cast<Team>(ti == kSpectatorSlot ? 0 : ti);
        if (ti != kSpectatorSlot &&
            !Visible(game_, viewer, units[i], game_.ComputeVisibility(viewer))) {
          continue;
        }
        Json shot;
        shot.Set("t", Num(std::round(t * 100.0) / 100.0));
        shot.Set("unit", Num(units[i].id));
        shot.Set("yaw", Num(units[i].shootAimYaw));
        shots[ti].Push(std::move(shot));
      }
    }
    {
      const auto& all = game_.Tracers();
      // The game clears its tracers when a turn starts, so all of them are this round's.
      for (; tracersSent < all.size(); ++tracersSent) {
        const Tracer& tr = all[tracersSent];
        for (int ti = 0; ti < 3; ++ti) {
          const Team viewer = static_cast<Team>(ti == kSpectatorSlot ? 0 : ti);
          if (ti != kSpectatorSlot && tr.team != viewer) {
            // Fog: an enemy's lines show only while some figure of theirs is in view.
            const TeamVisibility vis = game_.ComputeVisibility(viewer);
            bool seen = false;
            for (const Unit& u : units) seen = seen || (u.team == tr.team && vis.UnitVisible(u.id));
            if (!seen) continue;
          }
          Json j;
          j.Set("t", Num(std::round((t - tr.age) * 100.0) / 100.0));
          j.Set("team", net::TeamName(tr.team));
          j.Set("from", net::EncodeVec3(tr.from));
          j.Set("to", net::EncodeVec3(tr.to));
          tracers[ti].Push(std::move(j));
        }
      }
    }
    if (step % kFrameEvery == 0 || game_.Mode() != InputMode::Executing) sampleFrame();
  }

  for (int ti = 0; ti < 3; ++ti) {
    Json msg;
    msg.Set("t", "round");
    msg.Set("round", Num(game_.RoundNumber()));
    msg.Set("duration", Num(std::round(t * 100.0) / 100.0));
    msg.Set("frames", std::move(frames[ti]));
    msg.Set("shots", std::move(shots[ti]));
    msg.Set("tracers", std::move(tracers[ti]));
    msg.Set("state", ti == kSpectatorSlot ? StateBody(std::nullopt)
                                          : StateBody(static_cast<Team>(ti)));
    out[ti] = std::move(msg);
  }
  return out;
}

Json GameSession::StateBody(std::optional<Team> viewer) const {
  const Team team = viewer.value_or(Team::Blue);
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
    if (viewer && !own && !vis.UnitVisible(u.id)) continue;  // Fog: hidden enemies aren't reported at all.
    Json j;
    j.Set("id", Num(u.id));
    j.Set("pos", net::EncodeVec3(u.position));
    j.Set("yaw", Num(u.facingYaw));
    j.Set("alive", Json(u.alive));
    list.Push(std::move(j));
  }
  body.Set("units", std::move(list));
  if (viewer) body.Set("playbook", net::EncodePlaybook(game_.Playbook(team)));
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
  const FlagState& flag = game_.Flag();
  if (flag.enabled) {
    Json f;
    f.Set("carrier", Num(flag.carrierId));
    f.Set("dropped", Json(flag.carrierId < 0 && flag.dropElapsed >= 0.0f));
    f.Set("pos", net::EncodeVec3(game_.FlagPosition()));
    dump.Set("flag", std::move(f));
  }
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

Json GameSession::SpectatorStateMessage() const {
  Json msg = StateBody(std::nullopt);
  msg.Set("t", "state");
  return msg;
}

Json GameSession::SpectatorPlansMessage() const {
  Json msg;
  msg.Set("t", "plans");
  msg.Set("round", Num(game_.RoundNumber()));
  msg.Set("ready", Json(false));
  Json list{Json::Array{}};
  for (const Unit& u : game_.GetScene().units) {
    if (!u.alive) continue;
    list.Push(net::EncodeAction(net::PlanToAction(u)));
  }
  msg.Set("plans", std::move(list));
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
