#include <cstdio>

#include "flow/game_flow.h"

using flow::Event;
using flow::Screen;

int main() {
  int failures = 0;
  auto check = [&](bool ok, const char* what) {
    if (!ok) {
      std::fprintf(stderr, "FAIL: %s\n", what);
      ++failures;
    }
  };
  static_assert(flow::kInitialScreen == Screen::Splash);
  check(flow::Next(Screen::Splash, Event::NewGame) == Screen::MapSelect, "Splash -NewGame-> MapSelect");
  check(flow::Next(Screen::MapSelect, Event::SelectUrban) == Screen::Playing, "MapSelect -SelectUrban-> Playing");
  check(flow::Next(Screen::MapSelect, Event::Back) == Screen::Splash, "MapSelect -Back-> Splash");
  check(!flow::Next(Screen::Splash, Event::SelectUrban).has_value(), "Splash has no SelectUrban");
  return failures ? 1 : 0;
}
