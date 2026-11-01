#include "ui/Menu.h"

#include <imgui.h>

#include <algorithm>
#include <cfloat>
#include <cstdint>
#include <cstdio>
#include <cctype>
#include <cstdlib>
#include <random>

namespace tactics::ui {

using tbgwaf_flow::Event;
using tbgwaf_flow::State;

std::optional<Event> DrawMenu(State state, int windowWidth, int windowHeight,
                                  uint32_t* mapSeed, bool onlineAvailable, RoomForm* room,
                                  BuilderMenu* builder) {
  if (builder) {
    builder->openIndex = -1;
    builder->newScene = false;
    builder->deleteIndex = -1;
    builder->downloadLastLog = false;
  }
  if (state != State::Splash && state != State::GameMode && state != State::MapSelect &&
      state != State::Builder && state != State::Room) {
    return std::nullopt;
  }

  const ImGuiWindowFlags flags = ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                                 ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoSavedSettings |
                                 ImGuiWindowFlags_AlwaysAutoResize;
  ImGui::SetNextWindowPos(ImVec2(windowWidth * 0.5f, windowHeight * 0.5f), ImGuiCond_Always,
                          ImVec2(0.5f, 0.5f));
  const ImVec2 button(220.0f, 40.0f);
  std::optional<Event> fired;

  if (state == State::Splash) {
    ImGui::Begin("Firestep", nullptr, flags | ImGuiWindowFlags_NoTitleBar);
    ImGui::SetWindowFontScale(2.5f);
    ImGui::TextUnformatted("Firestep");
    ImGui::SetWindowFontScale(1.0f);
    ImGui::Spacing();
    if (ImGui::Button("New Game", button)) fired = Event::NewGame;
    ImGui::End();
  } else if (state == State::GameMode) {
    ImGui::Begin("Game Mode", nullptr, flags);
    if (ImGui::Button("Regular", button)) fired = Event::SelectRegular;
    if (ImGui::Button("Capture the Flag", button)) fired = Event::SelectCtf;
    if (ImGui::Button("Builder", button)) fired = Event::SelectBuilder;
    if (onlineAvailable && ImGui::Button("Online (2 players)", button)) fired = Event::SelectOnline;
    ImGui::Spacing();
    if (ImGui::Button("Back", ImVec2(100.0f, 0.0f))) fired = Event::Back;
    ImGui::End();
  } else if (state == State::Room) {
    ImGui::Begin("Online match", nullptr, flags);
    if (ImGui::Button("New room", button)) fired = Event::CreateRoom;
    ImGui::Spacing();
    ImGui::TextUnformatted("Room code");
    static char codeText[40] = "";
    ImGui::SetNextItemWidth(button.x - 70.0f);
    const bool entered = ImGui::InputText("##roomcode", codeText, sizeof codeText,
                                          ImGuiInputTextFlags_CharsUppercase |
                                              ImGuiInputTextFlags_CharsNoBlank |
                                              ImGuiInputTextFlags_EnterReturnsTrue);
    ImGui::SameLine();
    if (ImGui::Button("Join") || entered) {
      // Same alphabet and length the server accepts: [A-Z0-9_-]{1,32}.
      std::string code(codeText);
      bool valid = !code.empty() && code.size() <= 32;
      for (char c : code) {
        valid = valid && (std::isalnum(static_cast<unsigned char>(c)) || c == '_' || c == '-');
      }
      if (valid && room) {
        room->code = code;
        room->error.clear();
        fired = Event::JoinRoom;
      } else if (room) {
        room->error = "Enter the room code from the other player.";
      }
    }
    if (room && !room->error.empty()) {
      ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.62f, 0.62f, 1.0f));
      ImGui::PushTextWrapPos(button.x + 20.0f);
      ImGui::TextUnformatted(room->error.c_str());
      ImGui::PopTextWrapPos();
      ImGui::PopStyleColor();
    }
    ImGui::Spacing();
    if (ImGui::Button("Back", ImVec2(100.0f, 0.0f))) fired = Event::Back;
    ImGui::End();
  } else if (state == State::Builder) {
    // One card per player-built scene (thumbnail, a button with its name,
    // Delete) plus the New Scene card, in as many columns as fit.
    ImGui::SetNextWindowSizeConstraints(ImVec2(0.0f, 0.0f),
                                        ImVec2(windowWidth - 20.0f, windowHeight - 20.0f));
    ImGui::Begin("Builder", nullptr, flags);
    ImGui::TextUnformatted("Build a scene, then play it hotseat: plan both teams on one screen.");
    ImGui::Spacing();
    constexpr float kCardWidth = 240.0f;
    constexpr float kThumbHeight = 135.0f;
    constexpr float kCardGap = 16.0f;
    const int cardsPerRow = std::max(
        1, std::min(4, static_cast<int>((windowWidth - 60.0f + kCardGap) / (kCardWidth + kCardGap))));
    const int cardCount = builder ? static_cast<int>(builder->cards.size()) : 0;
    auto placeholder = [&](const char* label) {
      const ImVec2 p = ImGui::GetCursorScreenPos();
      ImDrawList* draw = ImGui::GetWindowDrawList();
      draw->AddRectFilled(p, ImVec2(p.x + kCardWidth, p.y + kThumbHeight),
                          IM_COL32(40, 44, 52, 255));
      if (label) {
        const float fontSize = ImGui::GetFontSize() * 4.0f;
        const ImVec2 size =
            ImGui::GetFont()->CalcTextSizeA(fontSize, FLT_MAX, 0.0f, label);
        draw->AddText(ImGui::GetFont(), fontSize,
                      ImVec2(p.x + (kCardWidth - size.x) * 0.5f,
                             p.y + (kThumbHeight - size.y) * 0.5f),
                      IM_COL32(200, 205, 215, 255), label);
      }
      ImGui::Dummy(ImVec2(kCardWidth, kThumbHeight));
    };
    for (int i = 0; i <= cardCount; ++i) {
      if (i % cardsPerRow != 0) ImGui::SameLine(0.0f, kCardGap);
      ImGui::PushID(i);
      ImGui::BeginGroup();
      if (i == cardCount) {
        placeholder("+");
        if (ImGui::Button("New Scene", ImVec2(kCardWidth, 36.0f))) {
          fired = Event::OpenBuilderScene;
          if (builder) builder->newScene = true;
        }
      } else {
        const BuilderCard& card = builder->cards[i];
        if (card.thumbnailTexture != 0) {
          // Offscreen renders come out bottom-up; flip the V axis.
          ImGui::Image(
              static_cast<ImTextureID>(static_cast<std::uintptr_t>(card.thumbnailTexture)),
              ImVec2(kCardWidth, kThumbHeight), ImVec2(0.0f, 1.0f), ImVec2(1.0f, 0.0f));
        } else {
          placeholder(nullptr);
        }
        if (ImGui::Button(card.name.empty() ? "(unnamed)" : card.name.c_str(),
                          ImVec2(kCardWidth, 36.0f))) {
          fired = Event::OpenBuilderScene;
          builder->openIndex = i;
        }
        if (ImGui::SmallButton("Delete")) builder->deleteIndex = i;
      }
      ImGui::EndGroup();
      ImGui::PopID();
    }
    ImGui::Spacing();
    if (builder && builder->lastLogAvailable) {
      if (ImGui::Button("Download last game's log")) builder->downloadLastLog = true;
    }
    if (builder && !builder->status.empty()) ImGui::TextUnformatted(builder->status.c_str());
    if (ImGui::Button("Back", ImVec2(100.0f, 0.0f))) fired = Event::ExitBuilder;
    ImGui::End();
  } else {
    ImGui::Begin("Map Select", nullptr, flags);
    // Edit buffer persists across frames; refreshed when the seed changes
    // from outside the box (Random button, first show).
    static char seedText[16] = "";
    static uint32_t shownSeed = 0;
    static bool seedShown = false;
    if (!seedShown || shownSeed != *mapSeed) {
      std::snprintf(seedText, sizeof seedText, "%u", static_cast<unsigned>(*mapSeed));
      shownSeed = *mapSeed;
      seedShown = true;
    }
    ImGui::TextUnformatted("Seed");
    ImGui::SetNextItemWidth(button.x - 80.0f);
    if (ImGui::InputText("##seed", seedText, sizeof seedText,
                         ImGuiInputTextFlags_CharsDecimal)) {
      *mapSeed = static_cast<uint32_t>(std::strtoull(seedText, nullptr, 10));
      shownSeed = *mapSeed;
    }
    ImGui::SameLine();
    if (ImGui::Button("Random")) *mapSeed = std::random_device{}();
    ImGui::Spacing();
    if (ImGui::Button("Urban", button)) fired = Event::SelectUrban;
    if (ImGui::Button("Urban x4", button)) fired = Event::SelectUrbanX4;
    if (ImGui::Button("Hills", button)) fired = Event::SelectHills;
    ImGui::Spacing();
    if (ImGui::Button("Back", ImVec2(100.0f, 0.0f))) fired = Event::Back;
    ImGui::End();
  }
  return fired;
}

}  // namespace tactics::ui
