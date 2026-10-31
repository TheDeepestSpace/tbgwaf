#include "gfx/StreetFixtureMesh.h"

#include <cmath>

#include <glm/gtc/constants.hpp>
#include <glm/gtc/matrix_transform.hpp>

namespace gfx {

void AppendTriangle(glm::vec3 a, glm::vec3 b, glm::vec3 c, glm::vec3 normal,
                    LitVertices* vertices, std::vector<GLuint>* indices) {
  const GLuint base = static_cast<GLuint>(vertices->size());
  vertices->push_back({a, normal});
  vertices->push_back({b, normal});
  vertices->push_back({c, normal});
  indices->insert(indices->end(), {base, base + 1, base + 2});
}

glm::mat4 BoxModel(const glm::vec3& minCorner, const glm::vec3& size) {
  return glm::translate(glm::mat4(1.0f), minCorner) * glm::scale(glm::mat4(1.0f), size);
}

void AppendBoxModel(const glm::mat4& model, LitVertices* vertices,
                    std::vector<GLuint>* indices) {
  const glm::mat3 normalMatrix = glm::transpose(glm::inverse(glm::mat3(model)));
  const auto p = [&](float x, float y, float z) {
    return glm::vec3(model * glm::vec4(x, y, z, 1.0f));
  };
  const auto n = [&](float x, float y, float z) {
    return glm::normalize(normalMatrix * glm::vec3(x, y, z));
  };
  auto quad = [&](glm::vec3 a, glm::vec3 b, glm::vec3 c, glm::vec3 d, glm::vec3 normal) {
    AppendTriangle(a, b, c, normal, vertices, indices);
    AppendTriangle(a, c, d, normal, vertices, indices);
  };
  quad(p(0, 1, 0), p(0, 1, 1), p(1, 1, 1), p(1, 1, 0), n(0, 1, 0));
  quad(p(0, 0, 0), p(1, 0, 0), p(1, 0, 1), p(0, 0, 1), n(0, -1, 0));
  quad(p(0, 0, 0), p(0, 1, 0), p(1, 1, 0), p(1, 0, 0), n(0, 0, -1));
  quad(p(0, 0, 1), p(1, 0, 1), p(1, 1, 1), p(0, 1, 1), n(0, 0, 1));
  quad(p(0, 0, 0), p(0, 0, 1), p(0, 1, 1), p(0, 1, 0), n(-1, 0, 0));
  quad(p(1, 0, 0), p(1, 1, 0), p(1, 1, 1), p(1, 0, 1), n(1, 0, 0));
}

namespace {

// Axis-aligned box with outward normals, for an obstacle without a model.
void AppendPlainBox(const tactics::AABB& box, LitVertices* vertices,
                    std::vector<GLuint>* indices) {
  AppendBoxModel(BoxModel(box.min, box.max - box.min), vertices, indices);
}

}  // namespace

// Models use a unit footprint centered on XZ and Y=0..1. All detail stays
// inside the gameplay box; one mesh serves lighting, shadows and FOV receivers.
void AppendStreetFixture(const tactics::Obstacle& obstacle, LitVertices* vertices,
                         std::vector<GLuint>* indices) {
  using Model = tactics::StreetFixtureModel;
  if (obstacle.fixtureModel == Model::None) {
    AppendPlainBox(obstacle.bounds, vertices, indices);
    return;
  }
  const glm::vec3 extent = obstacle.bounds.max - obstacle.bounds.min;
  const bool alongX = std::fabs(std::cos(obstacle.fixtureYaw)) > 0.5f;
  const glm::vec3 size(alongX ? extent.x : extent.z,
                       obstacle.bounds.max.y - obstacle.fixtureBaseY,
                       alongX ? extent.z : extent.x);
  const glm::vec3 center = obstacle.bounds.Center();
  const glm::mat4 frame =
      glm::translate(glm::mat4(1.0f), {center.x, obstacle.fixtureBaseY, center.z}) *
      glm::rotate(glm::mat4(1.0f), obstacle.fixtureYaw, {0.0f, -1.0f, 0.0f}) *
      glm::scale(glm::mat4(1.0f), size);
  const auto point = [&](glm::vec3 p) { return glm::vec3(frame * glm::vec4(p, 1.0f)); };
  const auto triangle = [&](glm::vec3 a, glm::vec3 b, glm::vec3 c) {
    a = point(a);
    b = point(b);
    c = point(c);
    AppendTriangle(a, b, c, glm::normalize(glm::cross(b - a, c - a)), vertices, indices);
  };
  const auto box = [&](glm::vec3 lo, glm::vec3 hi) {
    AppendBoxModel(frame * BoxModel(lo, hi - lo), vertices, indices);
  };
  // Extrude a counter-clockwise XY profile along local Z. Used for sloped
  // windscreens, tapered bodies and lids, rather than stacked rectangular blocks.
  const auto prism = [&](const std::vector<glm::vec2>& profile, float z0, float z1) {
    const auto p = [&](size_t i, float z) { return glm::vec3(profile[i], z); };
    for (size_t i = 1; i + 1 < profile.size(); ++i) {
      triangle(p(0, z0), p(i + 1, z0), p(i, z0));
      triangle(p(0, z1), p(i, z1), p(i + 1, z1));
    }
    for (size_t i = 0; i < profile.size(); ++i) {
      const size_t j = (i + 1) % profile.size();
      triangle(p(i, z0), p(j, z0), p(j, z1));
      triangle(p(i, z0), p(j, z1), p(i, z1));
    }
  };
  // Capped, faceted cylinders: wheel axles run along Z; litter bins along Y.
  const auto cylinder = [&](glm::vec3 c, glm::vec3 radius, bool vertical) {
    constexpr int kSides = 16;
    const auto p = [&](float angle, float end) {
      const glm::vec3 q = vertical ? glm::vec3(std::cos(angle), end, std::sin(angle))
                                   : glm::vec3(std::cos(angle), std::sin(angle), end);
      return c + radius * q;
    };
    const glm::vec3 axis = vertical ? glm::vec3(0, radius.y, 0) : glm::vec3(0, 0, radius.z);
    const float direction = vertical ? -1.0f : 1.0f;
    for (int i = 0; i < kSides; ++i) {
      const float a = direction * glm::two_pi<float>() * i / kSides;
      const float b = direction * glm::two_pi<float>() * (i + 1) / kSides;
      triangle(p(a, -1), p(b, -1), p(b, 1));
      triangle(p(a, -1), p(b, 1), p(a, 1));
      triangle(c - axis, p(b, -1), p(a, -1));
      triangle(c + axis, p(a, 1), p(b, 1));
    }
  };

  switch (obstacle.fixtureModel) {
    case Model::ParkedCar:
      // Low bonnet/trunk, sloped cabin, recessed windows and four round tires.
      prism({{-.49f, .25f}, {.49f, .25f}, {.50f, .43f}, {.43f, .59f},
             {-.42f, .59f}, {-.50f, .43f}}, -.41f, .41f);
      prism({{-.28f, .59f}, {.28f, .59f}, {.12f, .96f}, {-.16f, .96f}}, -.33f, .33f);
      box({-.17f, .96f, -.36f}, {.13f, 1.0f, .36f});  // Roof.
      for (float side : {-1.0f, 1.0f}) {
        const float z = side * .36f;
        box({-.28f, .57f, z - .025f}, {.28f, .62f, z + .025f});  // Window sill.
        box({-.035f, .60f, z - .025f}, {-.005f, .96f, z + .025f});  // B pillar.
        prism({{-.30f, .59f}, {-.27f, .59f}, {-.15f, .97f}, {-.18f, .97f}},
               z - .025f, z + .025f);
        prism({{.26f, .59f}, {.29f, .59f}, {.14f, .97f}, {.11f, .97f}},
               z - .025f, z + .025f);
        for (float x : {-.31f, .31f}) {
          cylinder({x, .25f, side * .43f}, {.077f, .25f, .065f}, false);
          cylinder({x, .25f, side * .497f}, {.040f, .13f, .003f}, false);  // Hub.
        }
        box({.12f, .56f, side * .45f - .04f}, {.18f, .64f, side * .45f + .04f});
        box({-.09f, .49f, z + side * .06f - .008f},
             {-.04f, .52f, z + side * .06f + .008f});  // Door handle.
      }
      for (float end : {-1.0f, 1.0f}) {
        box({end * .487f - .012f, .27f, -.42f}, {end * .487f + .012f, .34f, .42f});
        for (float z : {-.29f, .29f}) {
          box({end * .47f - .015f, .43f, z - .09f},
               {end * .47f + .015f, .51f, z + .09f});  // Head/tail lamps.
        }
      }
      break;
    case Model::Suv:
      // Taller, squarer body than the sedan: high belt line, long cabin with a
      // raked windscreen and near-vertical tailgate, roof rails, bigger tires.
      prism({{-.49f, .22f}, {.49f, .22f}, {.50f, .40f}, {.49f, .58f},
             {-.49f, .58f}, {-.50f, .40f}}, -.42f, .42f);
      box({.24f, .58f, -.40f}, {.49f, .64f, .40f});  // Bonnet.
      prism({{-.46f, .58f}, {.26f, .58f}, {.12f, .93f}, {-.42f, .93f}}, -.37f, .37f);
      box({-.42f, .93f, -.34f}, {.10f, .965f, .34f});  // Roof.
      for (float z : {-.29f, .29f}) {
        box({-.40f, .955f, z - .015f}, {.08f, 1.0f, z + .015f});  // Roof rails.
      }
      for (float side : {-1.0f, 1.0f}) {
        const float z = side * .37f;
        box({-.46f, .56f, z - .025f}, {.26f, .61f, z + .025f});  // Window sill.
        box({-.10f, .59f, z - .025f}, {-.07f, .93f, z + .025f});  // B pillar.
        box({-.30f, .59f, z - .025f}, {-.27f, .93f, z + .025f});  // C pillar.
        box({-.46f, .59f, z - .025f}, {-.43f, .93f, z + .025f});  // D pillar.
        prism({{.24f, .58f}, {.27f, .58f}, {.13f, .94f}, {.10f, .94f}},
               z - .025f, z + .025f);  // A pillar.
        for (float x : {-.33f, .33f}) {
          cylinder({x, .25f, side * .43f}, {.072f, .248f, .06f}, false);
          cylinder({x, .25f, side * .495f}, {.038f, .13f, .003f}, false);  // Hub.
        }
        box({.12f, .60f, side * .44f - .04f}, {.17f, .68f, side * .44f + .04f});  // Mirror.
        box({-.03f, .50f, z + side * .06f - .008f},
             {.02f, .53f, z + side * .06f + .008f});  // Front door handle.
        box({-.25f, .50f, z + side * .06f - .008f},
             {-.20f, .53f, z + side * .06f + .008f});  // Rear door handle.
        box({-.47f, .18f, side * .40f - .04f}, {-.33f, .22f, side * .40f + .04f});  // Step.
      }
      for (float end : {-1.0f, 1.0f}) {
        box({end * .487f - .012f, .24f, -.43f}, {end * .487f + .012f, .33f, .43f});  // Bumper.
        for (float z : {-.30f, .30f}) {
          box({end * .475f - .015f, .44f, z - .08f},
               {end * .475f + .015f, .54f, z + .08f});  // Head/tail lamps.
        }
      }
      for (float y : {.45f, .50f}) {
        box({.488f, y, -.18f}, {.50f, y + .015f, .18f});  // Grille slats.
      }
      break;
    case Model::Pickup:
      // Bonnet, two-seat cab and an open bed with side walls and a tailgate;
      // the box top is the cab roof, the bed walls stop well below it.
      prism({{-.49f, .24f}, {.49f, .24f}, {.50f, .40f}, {.49f, .56f},
             {-.49f, .56f}, {-.50f, .40f}}, -.42f, .42f);
      box({.22f, .56f, -.40f}, {.48f, .62f, .40f});  // Bonnet.
      prism({{-.07f, .56f}, {.24f, .56f}, {.10f, .96f}, {-.04f, .96f}}, -.37f, .37f);
      box({-.05f, .96f, -.35f}, {.08f, 1.0f, .35f});  // Cab roof.
      for (float side : {-1.0f, 1.0f}) {
        const float z = side * .37f;
        box({-.07f, .55f, z - .025f}, {.24f, .60f, z + .025f});  // Window sill.
        box({-.07f, .58f, z - .025f}, {-.04f, .96f, z + .025f});  // B pillar.
        prism({{.22f, .56f}, {.25f, .56f}, {.11f, .97f}, {.08f, .97f}},
               z - .025f, z + .025f);  // A pillar.
        box({-.48f, .56f, side * .40f - .025f}, {-.08f, .72f, side * .40f + .025f});  // Bed wall.
        box({-.48f, .70f, side * .40f - .035f}, {-.08f, .73f, side * .40f + .035f});  // Bed rail.
        for (float x : {-.31f, .31f}) {
          cylinder({x, .26f, side * .43f}, {.0685f, .26f, .06f}, false);
          cylinder({x, .26f, side * .495f}, {.036f, .135f, .003f}, false);  // Hub.
        }
        box({.12f, .60f, side * .44f - .04f}, {.17f, .68f, side * .44f + .04f});  // Mirror.
        box({.03f, .49f, z + side * .06f - .008f},
             {.08f, .52f, z + side * .06f + .008f});  // Door handle.
      }
      box({-.49f, .56f, -.42f}, {-.46f, .72f, .42f});  // Tailgate.
      box({-.09f, .56f, -.37f}, {-.07f, .72f, .37f});  // Bed front wall.
      box({-.47f, .56f, -.38f}, {-.09f, .575f, .38f});  // Bed floor.
      for (float end : {-1.0f, 1.0f}) {
        box({end * .487f - .012f, .26f, -.43f}, {end * .487f + .012f, .35f, .43f});  // Bumper.
        for (float z : {-.30f, .30f}) {
          box({end * .475f - .015f, .43f, z - .07f},
               {end * .475f + .015f, .53f, z + .07f});  // Head/tail lamps.
        }
      }
      box({.488f, .45f, -.18f}, {.50f, .465f, .18f});  // Grille slat.
      break;
    case Model::CoachBus:
      // Full-height coach: a slab body on three axles with a raked windscreen,
      // a window band framed by pillars, luggage bay doors, a front door,
      // mirrors and a roof air-conditioning pod. Taller than a standing eye.
      prism({{-.50f, .10f}, {.49f, .10f}, {.49f, .38f}, {.47f, .97f}, {-.50f, .97f}},
            -.47f, .47f);
      box({-.22f, .97f, -.30f}, {.08f, 1.0f, .30f});  // Air-conditioning pod.
      box({-.47f, .97f, -.44f}, {.44f, .985f, .44f});  // Roof rim.
      box({.458f, .90f, -.30f}, {.478f, .955f, .30f});  // Destination sign.
      for (float side : {-1.0f, 1.0f}) {
        const float z = side * .47f;
        box({-.49f, .44f, z - .02f}, {.44f, .47f, z + .02f});  // Window sill.
        box({-.49f, .88f, z - .02f}, {.44f, .91f, z + .02f});  // Window head.
        for (float x = -.45f; x < .42f; x += .11f) {
          box({x - .01f, .46f, z - .02f}, {x + .01f, .89f, z + .02f});  // Pillars.
        }
        box({-.42f, .20f, z - .015f}, {.22f, .225f, z + .015f});  // Bay door rails.
        box({-.42f, .40f, z - .015f}, {.22f, .425f, z + .015f});
        for (float x : {-.42f, -.10f, .22f}) {
          box({x - .01f, .20f, z - .015f}, {x + .01f, .425f, z + .015f});
        }
        box({.465f, .70f, side * .47f}, {.495f, .80f, side * .50f});  // Mirror.
        box({.455f, .66f, side * .48f - .01f}, {.47f, .80f, side * .48f + .01f});  // Mirror arm.
        for (float x : {.33f, -.26f, -.38f}) {
          cylinder({x, .167f, side * .44f}, {.046f, .167f, .06f}, false);
          cylinder({x, .167f, side * .497f}, {.028f, .10f, .003f}, false);  // Hub.
        }
      }
      box({.40f, .10f, .46f}, {.43f, .88f, .49f});  // Front door frame (curb side).
      box({.30f, .10f, .46f}, {.32f, .88f, .49f});
      box({.30f, .10f, .46f}, {.43f, .13f, .49f});
      box({.485f, .12f, -.46f}, {.50f, .22f, .46f});  // Front bumper.
      box({-.50f, .12f, -.46f}, {-.485f, .22f, .46f});  // Rear bumper.
      for (float z : {-.36f, .36f}) {
        box({.485f, .27f, z - .07f}, {.50f, .35f, z + .07f});  // Headlamps.
        box({-.50f, .26f, z - .06f}, {-.485f, .40f, z + .06f});  // Tail lamp clusters.
      }
      break;
    case Model::Bench:
      // Open space under the slatted seat, two leg frames, back and armrests.
      for (float x : {-.39f, .39f}) {
        box({x - .025f, 0, -.40f}, {x + .025f, .46f, -.28f});
        box({x - .025f, 0, .29f}, {x + .025f, 1.0f, .40f});
        box({x - .03f, .34f, -.43f}, {x + .03f, .40f, .43f});
        box({x - .025f, .45f, -.35f}, {x + .025f, .66f, -.26f});
        box({x - .035f, .64f, -.43f}, {x + .035f, .70f, .43f});
      }
      for (float z : {-.35f, -.10f, .15f}) {
        box({-.5f, .40f, z - .10f}, {.5f, .47f, z + .10f});
      }
      for (float y : {.57f, .73f, .89f}) {
        box({-.5f, y, .32f}, {.5f, y + .11f, .48f});
      }
      break;
    case Model::Dumpster:
      // Tapered steel hopper, split ribbed lid, lifting pockets and casters.
      prism({{-.38f, .12f}, {.38f, .12f}, {.47f, .86f}, {-.47f, .86f}}, -.43f, .43f);
      for (float x : {-.24f, .24f}) {
        box({x - .23f, .86f, -.48f}, {x + .23f, .94f, .48f});
        box({x - .07f, .94f, -.36f}, {x + .07f, 1.0f, -.30f});  // Lid handle.
        for (float z : {-.36f, .36f}) {
          cylinder({x, .07f, z}, {.07f, .07f, .035f}, false);
        }
      }
      for (float z : {-.28f, -.05f, .18f, .38f}) {
        box({-.46f, .94f, z - .015f}, {.46f, .97f, z + .015f});
      }
      for (float side : {-1.0f, 1.0f}) {
        box({side * .47f - .03f, .50f, -.22f}, {side * .47f + .03f, .59f, .22f});
        for (float x : {-.28f, 0.0f, .28f}) {
          box({x - .025f, .22f, side * .44f - .015f},
               {x + .025f, .78f, side * .44f + .015f});
        }
      }
      break;
    case Model::TrashBin:
      // Cylindrical litter basket with vertical ribs and an open slot below
      // its raised rain hood. The gap is geometry, visible even in thermal view.
      cylinder({0, .36f, 0}, {.43f, .36f, .43f}, true);
      cylinder({0, .73f, 0}, {.49f, .035f, .49f}, true);
      cylinder({0, .96f, 0}, {.50f, .04f, .50f}, true);
      for (float x : {-.36f, .36f}) {
        box({x - .045f, .73f, -.045f}, {x + .045f, .95f, .045f});
      }
      for (int i = 0; i < 12; ++i) {
        const float a = glm::two_pi<float>() * i / 12;
        const glm::vec3 c(.43f * std::cos(a), .37f, .43f * std::sin(a));
        box(c - glm::vec3(.025f, .31f, .025f), c + glm::vec3(.025f, .31f, .025f));
      }
      break;
    case Model::Newsstand:
      box({-.44f, 0, -.40f}, {.44f, .12f, .40f});
      box({-.39f, .12f, .27f}, {.39f, .88f, .40f});
      for (float x : {-.43f, .43f}) {
        box({x - .04f, .12f, -.42f}, {x + .04f, .90f, .42f});
      }
      for (float y : {.20f, .46f, .72f}) {
        box({-.40f, y, -.44f}, {.40f, y + .04f, .28f});
        for (float x : {-.22f, .20f}) {
          box({x - .15f, y + .04f, -.33f}, {x + .15f, y + .10f, .18f});
        }
      }
      box({-.50f, .90f, -.50f}, {.50f, 1.0f, .50f});
      break;
    case Model::Planter:
      box({-.46f, 0, -.43f}, {.46f, .15f, .43f});
      for (float x : {-.46f, .46f}) {
        box({x - .04f, .15f, -.50f}, {x + .04f, .67f, .50f});
      }
      for (float z : {-.46f, .46f}) {
        box({-.42f, .15f, z - .04f}, {.42f, .67f, z + .04f});
      }
      box({-.42f, .38f, -.42f}, {.42f, .43f, .42f});  // Recessed soil.
      for (float x : {-.27f, 0.0f, .27f}) {
        cylinder({x, .69f, 0}, {.10f, .27f, .22f}, true);
      }
      break;
    case Model::None: break;  // Handled above.
  }
}

}  // namespace gfx
