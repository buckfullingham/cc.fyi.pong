#include <catch2/catch_all.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "model.hpp"
#include <cstring>
#include <random>

namespace {
namespace p = pong;
namespace c = Catch;
namespace m = c::Matchers;
namespace u = p::unit;

std::function<p::vec_t()> make_starter() {
  return [prng = std::mt19937{c::rngSeed()},
          theta_dist =
              std::uniform_real_distribution<float>{
                  p::constant::pi<float>() / 8.f,
                  p::constant::pi<float>() * 3.f / 8.f},
          sign_dist = std::uniform_int_distribution<int>{0, 1},
          speed_dist = std::uniform_real_distribution<float>{
              150, 250}]() mutable -> p::vec_t {
    const p::scalar_t theta = theta_dist(prng);
    const p::matrix_t signs{
        {p::scalar_t(sign_dist(prng) * 2 - 1), 0.f},
        {0.f, p::scalar_t(sign_dist(prng) * 2 - 1)},
    };
    return p::transform::rot(theta) * signs * p::unit::i * speed_dist(prng);
  };
}

} // namespace

TEST_CASE("advance time through a horizontal collision with a paddle") {
  p::arena_t a{make_starter()};
  a.puck().velocity() = {1.f, 0.f};
  a.advance_time(a.rhs_paddle().box().min()(0) - a.puck().radius() -
                 a.puck().centre()(0) - 1.f);
  CHECK(a.puck().velocity() == p::vec_t{1.f, 0.f});
  a.advance_time(2.f);
  CHECK(a.puck().velocity().isApprox(p::vec_t{-1.f, 0.f}));
}

TEST_CASE("ensure puck doesn't break through walls") {
  std::mt19937 prng{c::rngSeed()};

  auto random_velocity = [&prng, dist = std::uniform_real_distribution<float>{
                                     -200.f, 200.f}]() mutable {
    auto x = dist(prng);
    auto y = dist(prng);
    return p::vec_t{x, y};
  };

  auto random_dt = [&prng, dist = std::exponential_distribution<float>(
                               60.f)]() mutable { return dist(prng); };

  p::arena_t a{make_starter()};

  const p::vec_t adjustment{a.puck().radius(), a.puck().radius()};

  const p::box_t adjusted_arena{
      a.box().min() + adjustment,
      a.box().max() - adjustment,
  };

  const p::box_t adjusted_lhs_paddle{
      a.lhs_paddle().box().min() - adjustment,
      a.lhs_paddle().box().max() + adjustment,
  };

  const p::box_t adjusted_rhs_paddle{
      a.rhs_paddle().box().min() - adjustment,
      a.rhs_paddle().box().max() + adjustment,
  };

  auto error = [&](p::scalar_t dt, const auto &p) {
    union {
      char chars[sizeof(float) * 4];
      float floats[5];
    };

    floats[0] = dt;
    floats[1] = p.centre()(0);
    floats[2] = p.centre()(1);
    floats[3] = p.velocity()(0);
    floats[4] = p.velocity()(1);

    for (unsigned char c : chars)
      std::cout << unsigned(c) << ", ";

    std::cout << "\n";

    REQUIRE(false);
  };

  for (int i = 0; i < 1 << 10; ++i) {
    a.puck().velocity() = random_velocity();
    REQUIRE(adjusted_arena.contains(a.puck().centre()));
    REQUIRE(!adjusted_lhs_paddle.contains(a.puck().centre()));
    REQUIRE(!adjusted_rhs_paddle.contains(a.puck().centre()));
    for (int j = 0; j < 1 << 10; ++j) {
      const auto copy = a.puck();
      const auto dt = random_dt();
      a.advance_time(random_dt());

      if (!adjusted_arena.contains(a.puck().centre()))
        error(dt, copy);

      if (adjusted_lhs_paddle.contains(a.puck().centre()))
        error(dt, copy);

      if (adjusted_rhs_paddle.contains(a.puck().centre()))
        error(dt, copy);
    }
  }
}

TEST_CASE("known breakout cases") {
  // known bug in an earlier version relating to a collision at -0 time
  p::arena_t a{make_starter()};

  // clang-format off
  using test_t = std::tuple<p::scalar_t , std::array<unsigned char, 16>>;
  std::array tests {
    test_t{1/60.f, {116, 164, 0, 68, 0, 128, 232, 67, 64, 235, 202, 194, 192, 142, 220, 66,}},
    test_t{1/60.f, {93, 9, 28, 68, 10, 29, 86, 67, 80, 126, 209, 66, 166, 70, 225, 66,}},
    test_t{60.f, {160, 141, 10, 66, 217, 76, 102, 67, 248, 149, 195, 193, 96, 172, 228, 65,}},
  };
  // clang-format on

  for (std::size_t i = 0; i < tests.size(); ++i) {
    WHEN("i = " << i) {
      auto &[dt, chars] = tests[i];

      float floats[4]{};
      std::memcpy(floats, chars.begin(), sizeof(floats));

      a.puck().centre()(0) = floats[0];
      a.puck().centre()(1) = floats[1];
      a.puck().velocity()(0) = floats[2];
      a.puck().velocity()(1) = floats[3];

      const p::vec_t adjustment{a.puck().radius(), a.puck().radius()};

      const p::box_t adjusted_arena{
          a.box().min() + adjustment,
          a.box().max() - adjustment,
      };

      const p::box_t adjusted_lhs_paddle{
          a.lhs_paddle().box().min() - adjustment,
          a.lhs_paddle().box().max() + adjustment,
      };

      const p::box_t adjusted_rhs_paddle{
          a.rhs_paddle().box().min() - adjustment,
          a.rhs_paddle().box().max() + adjustment,
      };

      REQUIRE(adjusted_arena.contains(a.puck().centre()));
      REQUIRE(!adjusted_lhs_paddle.contains(a.puck().centre()));
      REQUIRE(!adjusted_rhs_paddle.contains(a.puck().centre()));
      a.advance_time(dt);
      CHECK(adjusted_arena.contains(a.puck().centre()));
      CHECK(!adjusted_lhs_paddle.contains(a.puck().centre()));
      CHECK(!adjusted_rhs_paddle.contains(a.puck().centre()));
    }
  }
}

TEST_CASE("paddle stops at south arena boundary") {
  p::arena_t a{make_starter()};
  a.puck().velocity() = p::vec_t{0, 0};
  a.rhs_paddle().velocity()(1) = 1;
  a.advance_time(a.box().max()(1) - a.rhs_paddle().box().max()(1) - 2);
  // before hitting the boundary
  CHECK(a.rhs_paddle().velocity()(1) == 1.f);
  a.advance_time(2);
  // after hitting the boundary
  CHECK(a.rhs_paddle().velocity()(1) == 0.f);
  CHECK_THAT(a.rhs_paddle().box().max()(1),
             m::WithinRel(a.box().max()(1) - 1.f, 1e-3f));
}

TEST_CASE("paddle stops at north arena boundary") {
  p::arena_t a{make_starter()};
  a.puck().velocity() = p::vec_t{0, 0};
  a.rhs_paddle().velocity()(1) = -1;
  a.advance_time(a.rhs_paddle().box().min()(1) - a.box().min()(1) - 2);
  // before hitting the boundary
  CHECK(a.rhs_paddle().velocity()(1) == -1.f);
  a.advance_time(2);
  // after hitting the boundary
  CHECK(a.rhs_paddle().velocity()(1) == 0.f);
  CHECK_THAT(a.rhs_paddle().box().min()(1),
             m::WithinRel(a.box().min()(1) + 1.f, 1e-3f));
}

TEST_CASE("puck collides with moving paddle") {
  p::arena_t a{make_starter()};
  const auto b = p::bordered(a.rhs_paddle().box(), a.puck().radius());

  const p::vec_t offset{-4, 0};
  const p::vec_t paddle_velocity{0, 1};
  const p::vec_t puck_velocity{1, -1};

  a.rhs_paddle().velocity() = paddle_velocity;

  a.puck().velocity() = puck_velocity;
  a.puck().centre() = b.max() + offset + paddle_velocity - puck_velocity;

  a.advance_time(.9f);
  CHECK(a.puck().velocity() == puck_velocity);
  a.advance_time(.2f);
  CHECK(a.puck().velocity()(1) == -puck_velocity(1));
}

TEST_CASE("estimating next collision stationary paddles") {
  std::size_t start_count = 0;

  // distance between the two paddles
  const p::scalar_t width = []() {
    p::arena_t a{[&]() -> p::vec_t { return {}; }};
    return p::bordered(a.rhs_paddle().box(), a.puck().radius()).min()(0) -
           p::bordered(a.lhs_paddle().box(), a.puck().radius()).max()(0);
  }();

  // distance between the top and bottom arena boundaries
  const p::scalar_t height = []() {
    p::arena_t a{[&]() -> p::vec_t { return {}; }};
    return p::bordered(a.box(), -a.puck().radius()).diagonal()(1);
  }();

  // choose a velocity that results in repeated collisions with the stationary
  // paddles by bouncing off the top / bottom of the arena
  p::arena_t a{[&]() -> p::vec_t {
    ++start_count;
    return {1, 2.f * height / width};
  }};

  {
    auto [when, movement] = p::estimate_next_collision(a, a.rhs_paddle());
    CHECK_THAT(when, m::WithinRel(.5f * width, 1e-3f));
  }

  {
    auto [when, movement] = p::estimate_next_collision(a, a.lhs_paddle());
    CHECK_THAT(when, m::WithinRel(1.5f * width, 1e-3f));
  }

  for (int i = 0; i < 1000; ++i) {
    {
      auto [when, movement] = p::estimate_next_collision(a, a.rhs_paddle());
      CHECK_THAT(movement, m::WithinAbs(0.f, 1.f));
    }

    {
      auto [when, movement] = p::estimate_next_collision(a, a.lhs_paddle());
      CHECK_THAT(movement, m::WithinAbs(0.f, 1.f));
    }

    // advance to within 0.1 * width of the next collision
    a.advance_time(.1f * width);
  }
  CHECK(start_count == 1);

  a.advance_time(3.4f * width);
  CHECK(start_count == 1);
  const p::vec_t v1 = a.puck().velocity();
  // we should get a collision here
  {
    auto [when, movement] = p::estimate_next_collision(a, a.lhs_paddle());
    CHECK_THAT(when, m::WithinRel(.1f * width, 1e-2f));
    CHECK_THAT(movement, m::WithinAbs(0.f, .5f));
  }
  {
    auto [when, movement] = p::estimate_next_collision(a, a.rhs_paddle());
    CHECK_THAT(when, m::WithinRel(1.1f * width, 1e-2f));
    CHECK_THAT(movement, m::WithinAbs(0.f, .5f));
  }
  a.advance_time(.2f * width);
  CHECK(start_count == 1);
  const p::vec_t v2 = a.puck().velocity();
  CHECK(v1(0) != v2(0));
  CHECK(v1(1) == v2(1));
}

TEST_CASE("linear_oscillation") {
  std::vector<double> positions;

  const long cycles = 10;
  const long upper_bound = 10;

  for (long i = 0; i < cycles * upper_bound * 2; ++i) {
    positions.push_back(
        p::linear_oscillation(double(upper_bound), double(i) / 2.));
  }

  // output starts increasing from 0
  CHECK(positions[0] == 0);
  CHECK(positions[1] == .5);

  // range >= 0
  CHECK(std::all_of(positions.begin(), positions.end(),
                    [&](auto p) { return 0 <= p; }));

  // range < upper_bound
  CHECK(std::all_of(positions.begin(), positions.end(),
                    [&](auto p) { return p < upper_bound; }));

  // output is continuous for continuous x
  CHECK(std::adjacent_find(positions.begin(), positions.end(),
                           [](auto l, auto r) {
                             return !(l == r + .5 || r == l + .5);
                           }) == positions.end());
}
