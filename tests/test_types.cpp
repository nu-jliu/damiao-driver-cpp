#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "damiao/types.hpp"

using Catch::Matchers::WithinAbs;

TEST_CASE("float_to_uint maps correctly", "[types]") {
  SECTION("Zero maps to midpoint for signed range") {
    const uint16_t result = dm::float_to_uint(0.0f, -12.5f, 12.5f, 16);
    REQUIRE(result == 32767);  // (2^16 - 1) / 2
  }

  SECTION("Min maps to 0") {
    const uint16_t result = dm::float_to_uint(-12.5f, -12.5f, 12.5f, 16);
    REQUIRE(result == 0);
  }

  SECTION("Max maps to 2^bits - 1") {
    const uint16_t result = dm::float_to_uint(12.5f, -12.5f, 12.5f, 16);
    REQUIRE(result == 65535);
  }

  SECTION("Values are clamped below min") {
    const uint16_t result = dm::float_to_uint(-100.0f, -12.5f, 12.5f, 16);
    REQUIRE(result == 0);
  }

  SECTION("Values are clamped above max") {
    const uint16_t result = dm::float_to_uint(100.0f, -12.5f, 12.5f, 16);
    REQUIRE(result == 65535);
  }

  SECTION("12-bit unsigned range") {
    const uint16_t result = dm::float_to_uint(250.0f, 0.0f, 500.0f, 12);
    REQUIRE(result == 2047);  // midpoint of 4095
  }
}

TEST_CASE("uint_to_float maps correctly", "[types]") {
  SECTION("Midpoint maps to zero for signed range") {
    const float result = dm::uint_to_float(32767, -12.5f, 12.5f, 16);
    REQUIRE_THAT(result, WithinAbs(0.0f, 0.001f));
  }

  SECTION("Zero maps to min") {
    const float result = dm::uint_to_float(0, -12.5f, 12.5f, 16);
    REQUIRE_THAT(result, WithinAbs(-12.5f, 0.001f));
  }

  SECTION("Max maps to max") {
    const float result = dm::uint_to_float(65535, -12.5f, 12.5f, 16);
    REQUIRE_THAT(result, WithinAbs(12.5f, 0.001f));
  }
}

TEST_CASE("Round-trip float_to_uint -> uint_to_float", "[types]") {
  SECTION("Signed 16-bit round trip") {
    const float original = 3.14f;
    const uint16_t encoded =
        dm::float_to_uint(original, -12.5f, 12.5f, 16);
    const float decoded =
        dm::uint_to_float(encoded, -12.5f, 12.5f, 16);
    REQUIRE_THAT(decoded, WithinAbs(original, 0.001f));
  }

  SECTION("Signed 12-bit round trip") {
    const float original = -5.0f;
    const uint16_t encoded =
        dm::float_to_uint(original, -30.0f, 30.0f, 12);
    const float decoded =
        dm::uint_to_float(encoded, -30.0f, 30.0f, 12);
    REQUIRE_THAT(decoded, WithinAbs(original, 0.02f));
  }

  SECTION("Unsigned 12-bit round trip") {
    const float original = 100.0f;
    const uint16_t encoded =
        dm::float_to_uint(original, 0.0f, 500.0f, 12);
    const float decoded =
        dm::uint_to_float(encoded, 0.0f, 500.0f, 12);
    REQUIRE_THAT(decoded, WithinAbs(original, 0.15f));
  }

  SECTION("Symmetry for signed range") {
    const float pos = 6.0f;
    const float neg = -6.0f;
    const uint16_t pos_enc =
        dm::float_to_uint(pos, -12.5f, 12.5f, 16);
    const uint16_t neg_enc =
        dm::float_to_uint(neg, -12.5f, 12.5f, 16);
    const float pos_dec =
        dm::uint_to_float(pos_enc, -12.5f, 12.5f, 16);
    const float neg_dec =
        dm::uint_to_float(neg_enc, -12.5f, 12.5f, 16);
    REQUIRE_THAT(pos_dec, WithinAbs(-neg_dec, 0.001f));
  }
}
