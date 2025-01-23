#include <catch2/catch_all.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "geometry.hpp"

#include <array>

namespace {

namespace p = pong;
namespace t = p::transform;
namespace u = p::unit;
namespace c = p::constant;

} // namespace

TEST_CASE("pi") {
  CHECK_THAT(c::pi<float>(), Catch::Matchers::WithinRel(3.14159, 5e-6));
}

TEST_CASE("rot") {
  const std::array<pong::vec_t, 4> points{
      p::vec_t{0, 1},  // north
      p::vec_t{-1, 0}, // west
      p::vec_t{0, -1}, // south
      p::vec_t{1, 0},  // east
  };

  for (int i = 0; i < 4; ++i) {
    WHEN("i = " << i) {
      const auto pi = c::pi<float>();
      CHECK((t::rot(-pi * .5f) * points[i % 4]).isApprox(points[(i + 3) % 4]));
      CHECK((t::rot(-pi) * points[i % 4]).isApprox(points[(i + 2) % 4]));
      CHECK((t::rot(-pi * 1.5f) * points[i % 4]).isApprox(points[(i + 1) % 4]));
      CHECK((t::rot(-pi * 2.f) * points[i % 4]).isApprox(points[i % 4]));

      CHECK((t::rot(pi * .5f) * points[i % 4]).isApprox(points[(i + 1) % 4]));
      CHECK((t::rot(pi) * points[i % 4]).isApprox(points[(i + 2) % 4]));
      CHECK((t::rot(pi * 1.5f) * points[i % 4]).isApprox(points[(i + 3) % 4]));
      CHECK((t::rot(pi * 2.f) * points[i % 4]).isApprox(points[i % 4]));
    }
  }
}
