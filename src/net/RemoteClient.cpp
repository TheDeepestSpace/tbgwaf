#include "net/RemoteClient.h"

#include <algorithm>
#include <cmath>

#include "game/MapGenerator.h"
#include "net/ActionApply.h"
#include "net/SceneSpec.h"

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

RemoteClient::RemoteClient(tactics::GameLogic* game, std::string room, std::string intent,
                           bool spectate)
    : game_(game), room_(std::move(room)), intent_(std::move(intent)), spectate_(spectate) {}

std::vector<std::string> RemoteClient::TakeOutgoing() {
  std::vector<std::string> out;
  out.swap(outgoing_);
  return out;
}

void RemoteClient::OnOpen() {
  Json join;
  join.Set("t", "join");
  if (!room_.empty()) join.Set("room", room_);
  if (!intent_.empty()) join.Set("intent", intent_);
  if (spectate_) join.Set("spectate", Json(true));
  if (!botMode_.empty()) join.Set("mode", botMode_);
  Queue(join);
}

void RemoteClient::OnClose() {
  if (status_ != Status::OpponentLeft && status_ != Status::Rejected) status_ = Status::Disconnected;
  playback_.reset();
}

std::string RemoteClient::StatusText() const {
  switch (status_) {
    case Status::Connecting: return "Connecting to server...";
    case Status::Waiting:
      if (spectate_) return "Watching room " + room_ + " -- waiting for the players...";
      return room_.empty() ? "Waiting for an opponent to join..."
                           : "Room code " + room_ + " -- waiting for an opponent to join...";
    case Status::Rejected: return lastError_;
    case Status::OpponentLeft: return "Opponent left the match. Reload to play again.";
    case Status::Disconnected: return "Disconnected from server. Reload to reconnect.";
    case Status::Playing: break;
  }
  if (playback_) return "Round in progress...";
  if (!lastError_.empty()) return lastError_;
  if (CommitLocked()) return peerReady_ ? "Both ready..." : "Committed; waiting for opponent...";
  if (peerReady_) return "Opponent is ready.";
  return "";
}

void RemoteClient::OnMessage(const std::string& text) {
  Json m;
  if (!Json::Parse(text, &m) || !m["t"].IsString()) return;
  const std::string& type = m["t"].AsString();
  if (type == "waiting") {
    status_ = Status::Waiting;
    if (m["room"].IsString()) room_ = m["room"].AsString();
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
    const int seq = static_cast<int>(m["seq"].AsNumber(0));
    // Only acks for seqs this client issued advance the in-flight window
    // (test rigs inject barrier actions with out-of-band seqs).
    if (seq > lastAckedSeq_ && seq <= lastSentSeq_) lastAckedSeq_ = seq;
    const bool ok = m["ok"].AsBool(true);
    lastError_ = ok ? "" : m["error"].AsString();
    if (!ok && seq == commitSeq_) commitPending_ = false;  // Commit refused: unlock planning.
  } else if (type == "opponent_left") {
    status_ = Status::OpponentLeft;
    playback_.reset();
  } else if (type == "error") {
    lastError_ = m["error"].AsString();
    // Errors carry no seq; don't let an unacked action wedge the plans window.
    lastAckedSeq_ = lastSentSeq_;
    if (status_ == Status::Connecting) status_ = Status::Rejected;  // The join was refused.
  }
}

void RemoteClient::HandleStart(const Json& m) {
  // A spectator is told "spectator"; it views from Blue's side but is no team.
  const auto team = spectate_ ? std::optional<Team>(Team::Blue) : ParseTeamName(m["team"].AsString());
  if (!team || !m["seed"].IsNumber()) return;
  team_ = team;
  // Rooms loaded through the server's test control tap carry an explicit
  // scene; ordinary matches are rebuilt from the shared deterministic seed.
  tactics::Scene scene;
  std::string sceneError;
  if (m.Has("scene") && tactics::net::SceneFromSpec(m["scene"], &scene, &sceneError)) {
    game_->Reset(scene);
  } else {
    scene = tactics::GenerateUrbanMap(static_cast<uint32_t>(m["seed"].AsNumber()));
    tactics::ApplySpawnHints(&scene);  // Same corner spawns as the server (#226).
    game_->Reset(scene);
  }
  playback_.reset();
  pendingPlans_.reset();
  syncedPlan_.clear();
  syncedPlaybook_.reset();
  selfReady_ = peerReady_ = false;
  commitPending_ = false;
  lastAckedSeq_ = lastSentSeq_;  // Anything still in flight is void now.
  lastError_.clear();
  status_ = Status::Playing;
  matchStarted_ = true;
  NormalizeEnemyPlans();
}

void RemoteClient::HandlePlans(const Json& m) {
  // A "plans" echo that arrives mid-playback, or while our own actions are
  // still in flight (queued/sent but not yet acked), describes a server state
  // older than the local mirror -- applying it would revert fresh local plans
  // and yank the player out of an in-progress edit. Hold the latest echo
  // instead: the server answers every action with a fresh one, so the echo
  // following the final ack supersedes it.
  if (playback_ || lastAckedSeq_ < lastSentSeq_) {
    pendingPlans_ = m;
    return;
  }
  pendingPlans_.reset();
  ApplyPlans(m);
}

void RemoteClient::ApplyPlans(const Json& m) {
  if (!team_) return;
  selfReady_ = m["ready"].AsBool();
  if (selfReady_) commitPending_ = false;  // The lock is carried by selfReady_ now.
  const auto selected = game_->SelectedUnitId();
  const bool editing = game_->Mode() == InputMode::ActionMenu ||
                       game_->Mode() == InputMode::AwaitingMoveDestination ||
                       game_->Mode() == InputMode::AwaitingShootTarget;
  for (const Json& entry : m["plans"].AsArray()) {
    Action action;
    std::string error;
    if (!ParseAction(entry, &action, &error)) continue;
    Unit* unit = game_->FindUnit(action.unit);
    if (!unit || !unit->alive || (!spectate_ && unit->team != *team_)) continue;
    const std::string wanted = Signature(action);
    if (editing && selected && *selected == unit->id) {
      // The player is editing this unit right now: don't stomp the edit (or
      // the selection). Recording the server's plan lets SyncPlans() diff the
      // finished edit against it.
      syncedPlan_[unit->id] = wanted;
      continue;
    }
    if (Signature(PlanToAction(*unit)) != wanted) {
      ApplyPlanAction(*game_, unit->team, action, &error);
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
    const bool own = spectate_ || (team_ && unit->team == *team_);
    const Json* entry = nullptr;
    for (const Json& e : state["units"].AsArray()) {
      if (static_cast<int>(e["id"].AsNumber(-1)) == u.id) entry = &e;
    }
    u.moving = false;
    if (!entry) {
      u.walkBlend = 0.0f;
      u.shootElapsed = -1.0f;
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
    if (alive) {
      u.walkBlend = 0.0f;
      u.shootElapsed = -1.0f;
      u.knockdownElapsed = -1.0f;
    } else if (u.alive) {
      // First seen already down (it fell while hidden): render it settled on
      // the ground, not mid-fall.
      u.knockdownElapsed = constants::kKnockdownMaxSeconds;
      u.knockdownDrop =
          std::max(0.0f, u.position.y -
                             game_->GetScene().ground.HeightAt(u.position.x, u.position.z));
      u.rideTravel = -1.0f;
    }
    // A corpse keeps its frozen animation fields: the ragdoll replays from
    // the pose the figure died in, so stomping them re-seeds its trajectory.
    u.alive = alive;
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
  for (const Json& tj : m["tracers"].AsArray()) {
    TracerEvent ev;
    ev.t = static_cast<float>(tj["t"].AsNumber());
    const auto team = ParseTeamName(tj["team"].AsString());
    if (!team || !DecodeVec3(tj["from"], &ev.from) || !DecodeVec3(tj["to"], &ev.to)) continue;
    ev.team = *team;
    pb.tracers.push_back(ev);
  }
  selfReady_ = peerReady_ = false;
  commitPending_ = false;  // The round ran; planning reopens afterwards.
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
      // Killed above the ground (a zipline rider): the ragdoll falls the
      // whole way down, as GameLogic::KnockDown computes server-side.
      u.knockdownDrop = std::max(
          0.0f, pos.y - game_->GetScene().ground.HeightAt(pos.x, pos.z));
      u.rideTravel = -1.0f;
    }
    if (!u.alive) {
      // Same clock as GameLogic::Update: the whole ragdoll window, not just
      // the rigid tip-over (a long fall froze mid-air at kKnockdownDuration).
      // The other animation clocks freeze: the ragdoll replays from the pose
      // the figure was hit in, so mutating them re-seeds its trajectory.
      if (u.knockdownElapsed >= 0.0f) {
        u.knockdownElapsed =
            std::min(u.knockdownElapsed + dt, constants::kKnockdownMaxSeconds);
      }
      continue;
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
  // Bullet lines, as the single-process game draws them: every tracer fired
  // so far this turn (last turn's are gone), aged by how long ago it left the barrel.
  snap.tracers.clear();
  for (const TracerEvent& ev : pb.tracers) {
    if (ev.t > pb.clock) continue;
    snap.tracers.push_back(Tracer{ev.team, ev.from, ev.to, ev.t, pb.clock - ev.t});
  }
  game_->ImportState(snap);
  NormalizeEnemyPlans();
}

void RemoteClient::NormalizeEnemyPlans() {
  if (!team_ || spectate_) return;
  // Enemy plans are secret; the local mirror only needs them "non-empty" so
  // CanCommitRound() (which drives the HUD's Commit button) reflects whether
  // *our* figures are all planned.
  for (const Unit& u : game_->GetScene().units) {
    if (u.team == *team_ || !u.alive || u.plan.type != PlannedActionType::None) continue;
    game_->FindUnit(u.id)->plan.type = PlannedActionType::Pass;
  }
}

int RemoteClient::QueueAction(Action action) {
  action.seq = nextSeq_++;
  lastSentSeq_ = action.seq;
  Queue(EncodeAction(action));
  return action.seq;
}

void RemoteClient::SyncPlans() {
  if (!team_ || spectate_) return;
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
  } else if (game_->Mode() != InputMode::GameOver && game_->Mode() != InputMode::Executing &&
             !CommitLocked()) {
    // Once committed, the plans are locked in on the server; local mirror
    // noise must not be shipped (a plan edit would be refused anyway).
    SyncPlans();
  }
}

void RemoteClient::RequestCommit() {
  if (spectate_ || status_ != Status::Playing || playback_ || CommitLocked()) return;
  SyncPlans();  // The server must hold our latest plans before it counts the commit.
  Action commit;
  commit.kind = ActionKind::Commit;
  commitSeq_ = QueueAction(commit);
  commitPending_ = true;  // Lock planning (and grey the button) immediately.
}

void RemoteClient::RequestNewMatch() {
  if (spectate_ || status_ != Status::Playing || game_->Mode() != InputMode::GameOver) return;
  Action a;
  a.kind = ActionKind::NewMatch;
  QueueAction(a);
}

}  // namespace tactics::net
