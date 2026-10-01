#pragma once

// PNG capture/load/diff helpers shared by the scenario visual runner and the
// map-only visual tests. Header-only (inline) and defines the stb
// implementations, so include it from exactly one TU per executable.

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <vector>

#include <GLES3/gl3.h>

#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_STATIC
#include <stb_image.h>
#define STB_IMAGE_WRITE_IMPLEMENTATION
#define STB_IMAGE_WRITE_STATIC
#include <stb_image_write.h>

namespace visual {

namespace fs = std::filesystem;

struct Image {
  int width = 0;
  int height = 0;
  std::vector<unsigned char> rgb;  // Row-major, top-to-bottom, 3 bytes/pixel.
};

// Reads the current back buffer. GL's origin is bottom-left, so rows are
// flipped into the top-to-bottom order PNG/video expect. RGBA readback is
// the only combination ES 3.0 guarantees; converted to RGB here.
inline Image CaptureFramebuffer(int width, int height) {
  std::vector<unsigned char> rgba(static_cast<size_t>(width) * height * 4);
  glReadPixels(0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE, rgba.data());

  Image image;
  image.width = width;
  image.height = height;
  image.rgb.resize(static_cast<size_t>(width) * height * 3);
  for (int y = 0; y < height; ++y) {
    const unsigned char* src = rgba.data() + static_cast<size_t>(height - 1 - y) * width * 4;
    unsigned char* dst = image.rgb.data() + static_cast<size_t>(y) * width * 3;
    for (int x = 0; x < width; ++x) {
      dst[x * 3 + 0] = src[x * 4 + 0];
      dst[x * 3 + 1] = src[x * 4 + 1];
      dst[x * 3 + 2] = src[x * 4 + 2];
    }
  }
  return image;
}

inline Image CropColumns(const Image& image, int x, int width) {
  Image out;
  out.width = width;
  out.height = image.height;
  out.rgb.resize(static_cast<size_t>(width) * image.height * 3);
  for (int y = 0; y < image.height; ++y) {
    std::memcpy(out.rgb.data() + static_cast<size_t>(y) * width * 3,
                image.rgb.data() + (static_cast<size_t>(y) * image.width + x) * 3,
                static_cast<size_t>(width) * 3);
  }
  return out;
}

inline bool SavePng(const fs::path& path, const Image& image) {
  fs::create_directories(path.parent_path());
  return stbi_write_png(path.string().c_str(), image.width, image.height, 3, image.rgb.data(),
                        image.width * 3) != 0;
}

inline bool LoadPng(const fs::path& path, Image* out) {
  int w = 0, h = 0, channels = 0;
  unsigned char* data = stbi_load(path.string().c_str(), &w, &h, &channels, 3);
  if (!data) return false;
  out->width = w;
  out->height = h;
  out->rgb.assign(data, data + static_cast<size_t>(w) * h * 3);
  stbi_image_free(data);
  return true;
}

struct DiffResult {
  bool sizeMismatch = false;
  long long differingPixels = 0;
  long long totalPixels = 0;
  Image diffImage;  // Golden as dimmed grayscale, differing pixels in red.

  double Fraction() const {
    return totalPixels == 0 ? 1.0 : static_cast<double>(differingPixels) / totalPixels;
  }
};

// Pixelmatch-style tolerance diff: a pixel counts as differing only if some
// channel deviates by more than `threshold`, and the image only fails if
// the differing fraction exceeds the caller's budget. Absorbs cross-driver
// shading/edge noise while still catching object-sized changes (a single
// figure is roughly 0.3% of a pane).
inline DiffResult DiffImages(const Image& golden, const Image& actual, int threshold) {
  DiffResult result;
  if (golden.width != actual.width || golden.height != actual.height) {
    result.sizeMismatch = true;
    return result;
  }
  result.totalPixels = static_cast<long long>(golden.width) * golden.height;
  result.diffImage.width = golden.width;
  result.diffImage.height = golden.height;
  result.diffImage.rgb.resize(golden.rgb.size());
  for (long long i = 0; i < result.totalPixels; ++i) {
    const unsigned char* g = golden.rgb.data() + i * 3;
    const unsigned char* a = actual.rgb.data() + i * 3;
    const int delta = std::max({std::abs(g[0] - a[0]), std::abs(g[1] - a[1]),
                                std::abs(g[2] - a[2])});
    unsigned char* d = result.diffImage.rgb.data() + i * 3;
    if (delta > threshold) {
      ++result.differingPixels;
      d[0] = 255;
      d[1] = 0;
      d[2] = 0;
    } else {
      const unsigned char gray = static_cast<unsigned char>((g[0] + g[1] + g[2]) / 3 / 2);
      d[0] = d[1] = d[2] = gray;
    }
  }
  return result;
}

}  // namespace visual
