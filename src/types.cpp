#include "damiao/types.hpp"

#include <algorithm>
#include <cstdint>

namespace dm {

uint16_t float_to_uint(float x, const float x_min, const float x_max,
                       const uint8_t bits) {
  x = std::clamp(x, x_min, x_max);
  const float span = x_max - x_min;
  const float offset = x - x_min;
  return static_cast<uint16_t>(offset / span * static_cast<float>((1 << bits) - 1));
}

float uint_to_float(const uint16_t x_int, const float x_min, const float x_max,
                    const uint8_t bits) {
  const float span = x_max - x_min;
  return static_cast<float>(x_int) / static_cast<float>((1 << bits) - 1) * span +
         x_min;
}

}  // namespace dm
