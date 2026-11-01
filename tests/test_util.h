#pragma once

#include <cstdio>

// Tiny CHECK harness shared by the net/server test binaries.
inline int g_failures = 0;

#define CHECK(expr)                                                                \
  do {                                                                             \
    if (!(expr)) {                                                                 \
      std::fprintf(stderr, "CHECK FAILED at %s:%d: %s\n", __FILE__, __LINE__, #expr); \
      ++g_failures;                                                                \
    }                                                                              \
  } while (0)
