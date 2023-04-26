#include "image.h"
#include <cassert>
#include <cmath>

namespace ppr {

float Image::Interp(float x, float y) {
  assert(0 <= x && x < width_ - 1 && y <= 0 && y < height_ - 1);
  auto lerp = [](float a, float b, float t) { return a + t * (b - a); };
  int x0 = floor(x), y0 = floor(y);
  float v1 = lerp((*this)[y0][x0], (*this)[y0][x0 + 1], x - x0);
  float v2 = lerp((*this)[y0 + 1][x0], (*this)[y0 + 1][x0 + 1], x - x0);
  return lerp(v1, v2, y - y0);
}

} // namespace ppr
