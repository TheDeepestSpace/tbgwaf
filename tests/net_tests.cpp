// Headless tests for the wire protocol layer: JSON, action parsing, and the
// shared plan-application path (the same validation the server runs).

#include <cmath>
#include <string>

#include "game/GameLogic.h"
#include "game/Scene.h"
#include "net/ActionApply.h"
#include "net/Json.h"
#include "net/Protocol.h"
#include "test_util.h"

using namespace tactics;
using tactics::net::Action;
using tactics::net::ActionKind;
using tactics::net::Json;

namespace {

void TestJsonRoundTrip() {
  Json j;
  j.Set("a", 1).Set("s", "he\"llo\n").Set("b", true).Set("n", Json());
  Json arr{Json::Array{}};
  arr.Push(1.5).Push("x");
  j.Set("arr", arr);
  Json back;
  CHECK(Json::Parse(j.Dump(), &back));
  CHECK(back["a"].AsNumber() == 1);
  CHECK(back["s"].AsString() == "he\"llo\n");
  CHECK(back["b"].AsBool());
  CHECK(back["arr"].AsArray().size() == 2);
  CHECK(back["arr"].AsArray()[0].AsNumber() == 1.5);
  CHECK(!back["missing"].IsString());
}

void TestJsonRejectsMalformed() {
  Json out;
  for (const char* bad : {"", "{", "{\"a\":}", "[1,2", "{\"a\" 1}", "nul", "1 2", "\"abc",
                          "{\"a\":1,}", "[1,]", "NaN"}) {
    CHECK(!Json::Parse(bad, &out));
  }
  std::string deep(200, '[');
  deep += std::string(200, ']');
  CHECK(!Json::Parse(deep, &out));  // Depth bomb.
  CHECK(Json::Parse(" { \"a\" : [ 1 , 2 ] } ", &out));
  CHECK(Json::Parse("\"\\u00e9\"", &out) && out.AsString() == "\xC3\xA9");
}

Action Parse(const char* text, bool* ok, std::string* err = nullptr) {
  Json j;
  Action a;
  std::string e;
  *ok = Json::Parse(text, &j) && net::ParseAction(j, &a, &e);
  if (err) *err = e;
  return a;
}

void TestParseAction() {
  bool ok = false;
  Action a = Parse(R"({"t":"move","unit":2,"seq":7,"waypoints":[[1,0,2],[3,0,4]],"facing":1.5})", &ok);
  CHECK(ok && a.kind == ActionKind::Move && a.unit == 2 && a.seq == 7 && a.waypoints.size() == 2);
  CHECK(a.facing && std::fabs(*a.facing - 1.5f) < 1e-6f);
  a = Parse(R"({"t":"shoot","unit":0,"target":4})", &ok);
  CHECK(ok && a.target == 4);
  a = Parse(R"({"t":"commit"})", &ok);
  CHECK(ok && a.kind == ActionKind::Commit);
  a = Parse(R"({"t":"reaction","table":["none","shoot","stop","shoot_continue"]})", &ok);
  CHECK(ok && a.kind == ActionKind::Reaction && a.playbook.At(false, true) == ReactionAction::Shoot &&
        a.playbook.At(true, true) == ReactionAction::ShootContinue &&
        a.playbook.At(true, false) == ReactionAction::Stop);

  // Structural validation.
  for (const char* bad : {
           R"({"t":"move","unit":2})",                                  // no waypoints
           R"({"t":"move","unit":2,"waypoints":[]})",                   // empty
           R"({"t":"move","unit":2,"waypoints":[[1,2]]})",              // bad vec
           R"({"t":"move","unit":2,"waypoints":[[1,2,"x"]]})",          // bad coordinate
           R"({"t":"move","unit":2,"waypoints":[[1e9,0,0]]})",          // out of range
           R"({"t":"shoot","unit":0})",                                 // no target
           R"({"t":"shoot","unit":-1,"target":1})",                     // negative id
           R"({"t":"shoot","unit":1.5,"target":1})",                    // fractional id
           R"({"t":"pass"})",                                           // no unit
           R"({"t":"teleport","unit":1})",                              // unknown
           R"({"t":"reaction","table":["nuke","none","none","none"]})",
           R"({"t":"reaction","table":["none"]})",
           R"({"unit":1})",
           R"([1,2])"}) {
    Parse(bad, &ok);
    CHECK(!ok);
  }
  std::string many = R"({"t":"move","unit":1,"waypoints":[)";
  for (int i = 0; i < 40; ++i) many += (i ? ",[0,0,0]" : "[0,0,0]");
  many += "]}";
  Parse(many.c_str(), &ok);
  CHECK(!ok);  // Too many waypoints.
}

void TestActionEncodeParseRoundTrip() {
  Action a;
  a.kind = ActionKind::Move;
  a.unit = 3;
  a.seq = 9;
  a.waypoints = {glm::vec3(1, 0, 2), glm::vec3(4.25f, 1.2f, -7)};
  a.facing = 0.75f;
  Json j;
  CHECK(Json::Parse(net::EncodeAction(a).Dump(), &j));
  Action b;
  std::string err;
  CHECK(net::ParseAction(j, &b, &err));
  CHECK(b.kind == a.kind && b.unit == 3 && b.seq == 9 && b.waypoints.size() == 2);
  CHECK(glm::distance(b.waypoints[1], a.waypoints[1]) < 1e-4f);
}

Action Make(ActionKind kind, int unit, int target = -1) {
  Action a;
  a.kind = kind;
  a.unit = unit;
  a.target = target;
  return a;
}

void TestApplyPlanActionRules() {
  GameLogic game(BuildDefaultScene());
  std::string err;
  // Units 0..2 Blue, 3..5 Red.
  CHECK(net::ApplyPlanAction(game, Team::Blue, Make(ActionKind::Pass, 0), &err));
  CHECK(game.FindUnit(0)->plan.type == PlannedActionType::Pass);
  CHECK(game.Mode() == InputMode::AwaitingSelection);

  // Can't plan someone else's figure, or a nonexistent one.
  CHECK(!net::ApplyPlanAction(game, Team::Blue, Make(ActionKind::Pass, 3), &err));
  CHECK(game.FindUnit(3)->plan.type == PlannedActionType::None);
  CHECK(!net::ApplyPlanAction(game, Team::Blue, Make(ActionKind::Pass, 99), &err));

  // Fog: the default squads start hidden from each other, so shooting is rejected
  // and the previous plan survives (atomic).
  CHECK(!net::ApplyPlanAction(game, Team::Blue, Make(ActionKind::Shoot, 0, 3), &err));
  CHECK(game.FindUnit(0)->plan.type == PlannedActionType::Pass);
  CHECK(game.Mode() == InputMode::AwaitingSelection);
  // Can't shoot a teammate either.
  CHECK(!net::ApplyPlanAction(game, Team::Blue, Make(ActionKind::Shoot, 0, 1), &err));

  // Move: unreachable waypoint is rejected and rolls back.
  Action far = Make(ActionKind::Move, 1);
  far.waypoints = {glm::vec3(0.0f, 0.0f, 500.0f)};
  CHECK(!net::ApplyPlanAction(game, Team::Blue, far, &err));
  CHECK(game.FindUnit(1)->plan.type == PlannedActionType::None);
  CHECK(game.Mode() == InputMode::AwaitingSelection);

  // Valid move somewhere nearby.
  const glm::vec3 p = game.FindUnit(1)->position;
  bool moved = false;
  for (int i = 0; i < 8 && !moved; ++i) {
    const float ang = i * 0.785398f;
    Action mv = Make(ActionKind::Move, 1);
    mv.waypoints = {p + glm::vec3(std::cos(ang) * 4.0f, 0.0f, std::sin(ang) * 4.0f)};
    mv.facing = 1.0f;
    moved = net::ApplyPlanAction(game, Team::Blue, mv, &err);
  }
  CHECK(moved);
  CHECK(game.FindUnit(1)->plan.type == PlannedActionType::Move);
  CHECK(std::fabs(game.FindUnit(1)->plan.endFacingYaw - 1.0f) < 1e-4f);

  // PlanToAction reproduces the plan; replaying it into a fresh game matches.
  GameLogic other(BuildDefaultScene());
  CHECK(net::ApplyPlanAction(other, Team::Blue, net::PlanToAction(*game.FindUnit(1)), &err));
  CHECK(other.FindUnit(1)->plan.type == PlannedActionType::Move);
  CHECK(glm::distance(other.FindUnit(1)->plan.movePath.back(), game.FindUnit(1)->plan.movePath.back()) < 1e-3f);

  CHECK(net::ApplyPlanAction(game, Team::Blue, Make(ActionKind::Cancel, 1), &err));
  CHECK(game.FindUnit(1)->plan.type == PlannedActionType::None);
  CHECK(net::ApplyPlanAction(game, Team::Blue, Make(ActionKind::Focus, 1), &err));
  CHECK(!net::ApplyPlanAction(game, Team::Blue, Make(ActionKind::Commit, 1), &err));

  // Planning is closed once a round is executing / game over.
  for (int id = 0; id < 6; ++id) game.FindUnit(id)->plan.type = PlannedActionType::Pass;
  game.CommitRound();
  if (game.Mode() == InputMode::Executing) {
    CHECK(!net::ApplyPlanAction(game, Team::Blue, Make(ActionKind::Pass, 0), &err));
  }
}

}  // namespace

int main() {
  TestJsonRoundTrip();
  TestJsonRejectsMalformed();
  TestParseAction();
  TestActionEncodeParseRoundTrip();
  TestApplyPlanActionRules();
  if (g_failures == 0) {
    std::printf("All net tests passed.\n");
    return 0;
  }
  std::fprintf(stderr, "%d check(s) failed.\n", g_failures);
  return 1;
}
