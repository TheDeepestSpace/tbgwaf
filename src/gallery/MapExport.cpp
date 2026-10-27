#include "gallery/MapExport.h"

#include <cstdio>

#include "game/MapGenerator.h"

namespace gallery {
namespace {

void Num(std::string* out, float v) {
  char buf[32];
  std::snprintf(buf, sizeof buf, "%.2f", v);
  out->append(buf);
}

void Poly(std::string* out, int kind, const std::vector<glm::vec3>& vertices, bool* first) {
  if (vertices.size() < 3) return;
  if (!*first) out->push_back(',');
  *first = false;
  *out += "{\"t\":" + std::to_string(kind) + ",\"v\":[";
  for (size_t i = 0; i < vertices.size(); ++i) {
    if (i) out->push_back(',');
    Num(out, vertices[i].x);
    out->push_back(',');
    Num(out, vertices[i].y);
    out->push_back(',');
    Num(out, vertices[i].z);
  }
  *out += "]}";
}

}  // namespace

std::string MapToJson(const tactics::Scene& scene) {
  std::string out = "{\"half\":";
  Num(&out, scene.mapHalfExtent);

  const tactics::HeightField& g = scene.ground;
  out += ",\"ground\":";
  if (g.Empty()) {
    out += "null";
  } else {
    out += "{\"minX\":";
    Num(&out, g.minX);
    out += ",\"minZ\":";
    Num(&out, g.minZ);
    out += ",\"step\":";
    Num(&out, g.step);
    out += ",\"nx\":" + std::to_string(g.nx) + ",\"nz\":" + std::to_string(g.nz) + ",\"h\":[";
    for (size_t i = 0; i < g.heights.size(); ++i) {
      if (i) out.push_back(',');
      Num(&out, g.heights[i]);
    }
    out += "]}";
  }

  out += ",\"prisms\":[";
  bool first = true;
  for (const tactics::Obstacle& o : scene.obstacles) {
    if (o.navOnly) continue;
    int kind = 0;
    if (o.building >= 0 && o.building < static_cast<int>(scene.buildings.size())) {
      kind = 1 + static_cast<int>(scene.buildings[o.building].kind);
    }
    if (!first) out.push_back(',');
    first = false;
    out += "{\"k\":" + std::to_string(kind) + ",\"y0\":";
    Num(&out, o.bounds.min.y);
    out += ",\"y1\":";
    Num(&out, o.bounds.max.y);
    out += ",\"fp\":[";
    const std::vector<glm::vec2> fp = tactics::ObstacleFootprint(o);
    for (size_t i = 0; i < fp.size(); ++i) {
      if (i) out.push_back(',');
      Num(&out, fp[i].x);
      out.push_back(',');
      Num(&out, fp[i].y);
    }
    out += "]}";
  }

  out += "],\"polys\":[";
  first = true;
  for (const auto& r : scene.roads) Poly(&out, 0, r.vertices, &first);
  for (const auto& r : scene.sidewalkSurfaces) Poly(&out, 1, r.vertices, &first);
  for (const auto& w : scene.walkSurfaces) Poly(&out, 2, w.vertices, &first);
  out += "]}";
  return out;
}

std::string UrbanMapJson(uint32_t seed, bool x4) {
  tactics::MapGeneratorConfig config;
  if (!x4) {
    config.blocksX = 4;
    config.blocksZ = 4;
  }
  return MapToJson(tactics::GenerateUrbanMap(seed, config));
}

}  // namespace gallery
