#include "net/RemoteClient.h"

#include <algorithm>
#include <cmath>

#include "game/MapGenerator.h"
#include "net/ActionApply.h"

namespace tactics::net {

using tactics::GameSnapshot;
using tactics::InputMode;
using tactics::Team;
using tactics::Unit;
namespace constants = tactics::constants;

namespace {

constexpr float kPi = 3.14159265358979f;

glm::vec3 HiddenPosition() { return glm::vec3(kHiddenCoord, 0.0f, kHiddenCoord); }
bool IsHidden(const glm::vec3& p) { return p.x >= kHiddenCoord * 0.5f; }

float LerpAngle(float a, float b, float s) {
  const float d = std::remainder(b - a, 2.0f * kPi);
  return a + d * s;
}

std::string Signature(const Action& a) {
  Action copy = a;
  copy.seq = 0;
  return EncodeAction(copy).Dump();
}

}  // namespace

RemoteClient::RemoteClient(tactics::GameLogic* game, std::string room)
    : game_(game), room_(std::move(room)) {}

std::vector<std::string> RemoteClient::TakeOutgoing() {
  std::vector<std::string> out;
  out.swap(outgoing_);
  return out;
}

void RemoteClient::OnOpen() {
  Json join;
  join.Set("t", "join");
  if (!room_.empty()) join.Set("room", room_);
  Queue(join);
}

void RemoteClient::OnClose() {
  if (status_ != Status::OpponentLeft) status_ = Status::Disconnected;
  playback_.reset();
}

std::string RemoteClient::StatusText() const {
  switch (status_) {
    case Status::Connecting: return "Connecting to server...";
    case Status::Waiting: return "Waiting for an opponent to join...";
    case Status::OpponentLeft: return "Opponent left the match. Reload to play again.";
    case Status::Disconnected: return "Disconnected from server. Reload to reconnect.";
    case Status::Playing: break;
  }
  if (playback_) return "Round in progress...";
  if (!lastError_.empty()) return lastError_;
  if (selfReady_) return peerReady_ ? "Both ready..." : "Committed; waiting for opponent...";
  if (peerReady_) return "Opponent is ready.";
  return "";
}

void RemoteClient::OnMessage(const std::string& text) {
  Json m;
  if (!Json::Parse(text, &m) || !m["t"].IsString()) return;
  const std::string& type = m["t"].AsString();
  if (type == "waiting") {
    status_ = Status::Waiting;
  } else if (type == "start") {
    HandleStart(m);
  } else if (type == "plans") {
    HandlePlans(m);
  } else if (type == "state") {
    if (!playback_) ApplyState(m);
  } else if (type == "round") {
    HandleRound(m);
  } else if (type == "peer") {
    peerReady_ = m["ready"].AsBool();
  } else if (type == "ack") {
    lastError_ = m["ok"].AsBool(true) ? "" : m["error"].AsString();
  } else if (type == "opponent_left") {
    status_ = Status::OpponentLeft;
    playback_.reset();
  } else if (type == "error") {
    lastError_ = m["error"].AsString();
  }
}

void RemoteClient::HandleStart(const Json& m) {
  const auto team = ParseTeamName(m["team"].AsString());
  if (!team || !m["seed"].IsNumber()) return;
  team_ = team;
  game_->Reset(tactics::GenerateUrbanMap(static_cast<uint32_t>(m["seed"].AsNumber())));
  playback_.reset();
  pendingPlans_.reset();
  syncedPlan_.clear();
  syncedPlaybook_.reset();
  selfReady_ = peerReady_ = false;
  lastError_.clear();
  status_ = Status::Playing;
  matchStarted_ = true;
  NormalizeEnemyPlans();
}

void RemoteClient::HandlePlans(const Json& m) {
  if (playback_) {
    pendingPlans_ = m;
    return;
  }
  ApplyPlans(m);
}

void RemoteClient::ApplyPlans(const Json& m) {
  if (!team_) return;
  selfReady_ = m["ready"].AsBool();
  for (const Json& entry : m["plans"].AsArray()) {
    Action action;
    std::string error;
    if (!ParseAction(entry, &action, &error)) continue;
    Unit* unit = game_->FindUnit(action.unit);
    if (!unit || unit->team != *team_ || !unit->alive) continue;
    const std::string wanted = Signature(action);
    if (Signature(PlanToAction(*unit)) != wanted) {
      ApplyPlanAction(*game_, *team_, action, &error);
    }
    syncedPlan_[unit->id] = wanted;
  }
  SquadPlaybook playbook;
  if (DecodePlaybook(m["playbook"], &playbook)) {
    game_->SetPlaybook(*team_, playbook);
    syncedPlaybook_ = playbook;
  }
}

void RemoteClient::ApplyState(const Json& state) {
  GameSnapshot snap = game_->ExportState();
  for (auto& u : snap.units) {
    const Unit* unit = game_->FindUnit(u.id);
    const bool own = team_ && unit->team == *team_;
    const Json* entry = nullptr;
    for (const Json& e : state["units"].AsArray()) {
      if (static_cast<int>(e["id"].AsNumber(-1)) == u.id) entry = &e;
    }
    u.moving = false;
    u.walkBlend = 0.0f;
    u.shootElapsed = -1.0f;
    if (!entry) {
      if (!own) {
        u.position = HiddenPosition();
        u.alive = true;
      }
      continue;
    }
    glm::vec3 pos;
    if (DecodeVec3((*entry)["pos"], &pos)) u.position = pos;
    u.facingYaw = static_cast<float>((*entry)["yaw"].AsNumber(u.facingYaw));
    const bool alive = (*entry)["alive"].AsBool(true);
    if (!alive && u.alive) u.knockdownElapsed = constants::kKnockdownDuration;  // Already down.
    u.alive = alive;
    if (alive) u.knockdownElapsed = -1.0f;
  }
  snap.roundNumber = static_cast<int>(state["round"].AsNumber(snap.roundNumber));
  if (state["over"].AsBool()) {
    snap.mode = InputMode::GameOver;
    const auto winner = ParseTeamName(state["winner"].AsString());
    snap.winner = winner ? static_cast<int>(*winner) : -1;
  } else {
    snap.mode = InputMode::AwaitingSelection;
    snap.winner = -1;
  }
  game_->ImportState(snap);
  NormalizeEnemyPlans();
}

void RemoteClient::HandleRound(const Json& m) {
  Playback pb;
  pb.duration = static_cast<float>(m["duration"].AsNumber());
  pb.finalState = m["state"];
  for (const Json& f : m["frames"].AsArray()) {
    Frame frame;
    frame.t = static_cast<float>(f["t"].AsNumber());
    for (const Json& row : f["u"].AsArray()) {
      const auto& a = row.AsArray();
      if (a.size() != 7) continue;
      Sample s;
      s.id = static_cast<int>(a[0].AsNumber());
      s.pos = glm::vec3(a[1].AsNumber(), a[2].AsNumber(), a[3].AsNumber());
      s.yaw = static_cast<float>(a[4].AsNumber());
      s.alive = a[5].AsNumber() != 0;
      s.moving = a[6].AsNumber() != 0;
      frame.units.push_back(s);
    }
    pb.frames.push_back(std::move(frame));
  }
  for (const Json& s : m["shots"].AsArray()) {
    pb.shots.push_back(Shot{static_cast<float>(s["t"].AsNumber()),
                            static_cast<int>(s["unit"].AsNumber(-1)),
                            static_cast<float>(s["yaw"].AsNumber())});
  }
  selfReady_ = peerReady_ = false;
  if (pb.frames.empty()) {
    ApplyState(pb.finalState);
    return;
  }
  playback_ = std::move(pb);
  GameSnapshot snap = game_->ExportState();
  snap.mode = InputMode::Executing;
  game_->ImportState(snap);
}

void RemoteClient::AdvancePlayback(float dt) {
  Playback& pb = *playback_;
  pb.clock += dt;
  if (pb.clock >= pb.frames.back().t) {
    const Json finalState = pb.finalState;
    playback_.reset();
    ApplyState(finalState);
    if (pendingPlans_) {
      const Json plans = *pendingPlans_;
      pendingPlans_.reset();
      ApplyPlans(plans);
    }
    return;
  }
  size_t i = 0;
  while (i + 2 < pb.frames.size() && pb.frames[i + 1].t <= pb.clock) ++i;
  const Frame& a = pb.frames[i];
  const Frame& b = pb.frames[std::min(i + 1, pb.frames.size() - 1)];
  const float span = b.t - a.t;
  const float s = span > 1e-4f ? std::clamp((pb.clock - a.t) / span, 0.0f, 1.0f) : 1.0f;

  GameSnapshot snap = game_->ExportState();
  snap.mode = InputMode::Executing;
  for (auto& u : snap.units) {
    const Sample* sa = nullptr;
    const Sample* sb = nullptr;
    for (const Sample& x : a.units) if (x.id == u.id) sa = &x;
    for (const Sample& x : b.units) if (x.id == u.id) sb = &x;
    const glm::vec3 before = u.position;
    if (!sa) {  // Not visible to us at this instant.
      u.position = HiddenPosition();
      u.moving = false;
      u.walkBlend = 0.0f;
      u.shootElapsed = -1.0f;
      continue;
    }
    glm::vec3 pos = sa->pos;
    float yaw = sa->yaw;
    if (sb) {
      pos = glm::mix(sa->pos, sb->pos, s);
      yaw = LerpAngle(sa->yaw, sb->yaw, s);
    }
    u.position = pos;
    u.facingYaw = yaw;
    u.moving = sa->moving;
    if (u.alive && !sa->alive) {  // Fell this segment: start the knockdown beat.
      u.alive = false;
      u.knockdownElapsed = 0.0f;
      u.knockdownAxis = glm::vec3(std::sin(yaw), 0.0f, -std::cos(yaw));
    }
    if (!u.alive && u.knockdownElapsed >= 0.0f && u.knockdownElapsed < constants::kKnockdownDuration) {
      u.knockdownElapsed += dt;
    }
    // Presentation derived locally: stride follows distance travelled.
    if (!IsHidden(before)) {
      const float moved = glm::length(glm::vec2(pos.x - before.x, pos.z - before.z));
      u.walkPhase += moved / constants::kWalkStrideLength * 2.0f * kPi;
    }
    const float target = (u.alive && sa->moving) ? 1.0f : 0.0f;
    u.walkBlend += (target - u.walkBlend) * (1.0f - std::exp(-constants::kWalkBlendRate * dt));
    u.idleElapsed = std::fmod(u.idleElapsed + dt, constants::kIdleAnimDuration);
    u.shootElapsed = -1.0f;
    for (const Shot& shot : pb.shots) {
      const float age = pb.clock - shot.t;
      if (shot.unit == u.id && age >= 0.0f && age < constants::kShootAnimDuration) {
        u.shootElapsed = age;
        u.shootAimYaw = shot.yaw;
      }
    }
  }
  game_->ImportState(snap);
  NormalizeEnemyPlans();
}

void RemoteClient::NormalizeEnemyPlans() {
  if (!team_) return;
  // Enemy plans are secret; the local mirror only needs them "non-empty" so
  // CanCommitRound() (which drives the HUD's Commit button) reflects whether
  // *our* figures are all planned.
  for (const Unit& u : game_->GetScene().units) {
    if (u.team == *team_ || !u.alive || u.plan.type != PlannedActionType::None) continue;
    game_->FindUnit(u.id)->plan.type = PlannedActionType::Pass;
  }
}

void RemoteClient::QueueAction(Action action) {
  action.seq = nextSeq_++;
  Queue(EncodeAction(action));
}

void RemoteClient::SyncPlans() {
  if (!team_) return;
  const auto selected = game_->SelectedUnitId();
  const bool midPlan = game_->Mode() == InputMode::AwaitingMoveDestination ||
                       game_->Mode() == InputMode::AwaitingShootTarget;
  for (const Unit& u : game_->GetScene().units) {
    if (u.team != *team_ || !u.alive) continue;
    if (midPlan && selected && *selected == u.id) continue;  // Still being planned.
    const Action action = PlanToAction(u);
    const std::string sig = Signature(action);
    auto it = syncedPlan_.find(u.id);
    Action none;
    none.kind = ActionKind::Cancel;
    none.unit = u.id;
    const std::string known = it == syncedPlan_.end() ? Signature(none) : it->second;
    if (sig != known) {
      QueueAction(action);
      syncedPlan_[u.id] = sig;
    }
  }
  const SquadPlaybook& playbook = game_->Playbook(*team_);
  if (playbook != syncedPlaybook_.value_or(SquadPlaybook{})) {
    Action reaction;
    reaction.kind = ActionKind::Reaction;
    reaction.playbook = playbook;
    QueueAction(reaction);
    syncedPlaybook_ = playbook;
  }
}

void RemoteClient::Update(float dt) {
  if (status_ != Status::Playing) return;
  if (playback_) {
    AdvancePlayback(dt);
  } else if (game_->Mode() != InputMode::GameOver && game_->Mode() != InputMode::Executing) {
    SyncPlans();
  }
}

void RemoteClient::RequestCommit() {
  if (status_ != Status::Playing || playback_) return;
  SyncPlans();  // The server must hold our latest plans before it counts the commit.
  Action commit;
  commit.kind = ActionKind::Commit;
  QueueAction(commit);
}

void RemoteClient::RequestNewMatch() {
  if (status_ != Status::Playing || game_->Mode() != InputMode::GameOver) return;
  Action a;
  a.kind = ActionKind::NewMatch;
  QueueAction(a);
}

}  // namespace tactics::net
