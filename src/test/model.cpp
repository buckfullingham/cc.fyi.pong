#include <catch2/catch_all.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "model.hpp"
#include <cstring>
#include <functional>
#include <iostream>
#include <random>
#include <tuple>

namespace {
namespace p = pong;
namespace c = Catch;
namespace m = c::Matchers;
namespace u = p::unit;

auto make_starter() { return p::make_starter(c::rngSeed()); }

} // namespace

TEST_CASE("advance time through a horizontal collision with a paddle") {
  p::arena_t a{[]() -> std::tuple<p::scalar_t, p::vec_t> {
    return {240.f, {1.f, 0.f}};
  }};
  a.advance_time(a.rhs_paddle().box().min()(0) - a.puck().radius() -
                 a.puck().centre()(0) - 1.f);
  CHECK(a.puck().velocity() == p::vec_t{1.f, 0.f});

  // this takes us to 1 after the collision, when the x component of velocity
  // should have a reversed sign
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
      std::memcpy(floats, &*chars.begin(), sizeof(floats));

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

TEST_CASE("linear_oscillation") {
  std::vector<std::uint64_t> positions;

  const long cycles = 10;
  const long upper_bound = 10;

  for (long i = 0; i < cycles * upper_bound; ++i) {
    positions.push_back(p::linear_oscillation(upper_bound, i));
  }

  // output starts increasing from 0
  CHECK(positions[0] == 0);
  CHECK(positions[1] == 1);

  // range >= 0
  CHECK(std::all_of(positions.begin(), positions.end(),
                    [&](auto p) { return 0 <= p; }));

  // range < upper_bound
  CHECK(std::all_of(positions.begin(), positions.end(),
                    [&](auto p) { return p < upper_bound; }));

  // output is continuous for continuous x
  CHECK(std::adjacent_find(positions.begin(), positions.end(),
                           [](auto l, auto r) {
                             return !(l == r + 1 || r == l + 1);
                           }) == positions.end());
}

TEST_CASE("linear oscillation inverse") {
  // 0 1 2 3 4 5 6
  // 0 1 2 3 2 1 0
  CHECK(p::linear_oscillation_inverse(4, 0, true) == 0);
  CHECK(p::linear_oscillation_inverse(4, 1, true) == 1);
  CHECK(p::linear_oscillation_inverse(4, 2, true) == 2);
  CHECK(p::linear_oscillation_inverse(4, 3, true) == 3);
  CHECK(p::linear_oscillation_inverse(4, 3, false) == 3);
  CHECK(p::linear_oscillation_inverse(4, 2, false) == 4);
  CHECK(p::linear_oscillation_inverse(4, 1, false) == 5);
  CHECK(p::linear_oscillation_inverse(4, 0, false) == 6);
}

TEST_CASE("estimating next paddle collision at rhs, starting height") {
  auto [puck, box] = []() {
    p::arena_t a{
        []() -> std::tuple<p::scalar_t, p::vec_t> { return {240.f, {0.f, 0.f}}; }};
    return std::make_tuple(
        a.puck(),
        p::box_t{
            p::vec_t{a.lhs_paddle().box().max()(0) + a.puck().radius(),
                     a.box().min()(1) + a.puck().radius()},
            p::vec_t{a.rhs_paddle().box().min()(0) - a.puck().radius(),
                     a.box().max()(1) - a.puck().radius()},
        });
  }();

  const p::scalar_t distance_to_rhs = box.max()(0) - puck.centre()(0);

  const p::scalar_t distance_to_bottom = box.max()(1) - puck.centre()(1);

  /*     +--------------------+
   *     |                    | start in the centre, bounce off the bottom and
   *     |                    | hit the paddle at the puck's starting y co-ord
   *     |                 |  |
   *     |         \      /|  |
   *     |          \    / |  |
   *     |           \  /     |
   *     |            \/      |
   *     +--------------------+
   */
  p::arena_t a{[&]() -> std::tuple<p::scalar_t, p::vec_t> {
    return {240.f, p::vec_t{distance_to_rhs, 2 * distance_to_bottom}};
  }};

  const auto [when, where_y] = p::estimate_next_collision(a, a.rhs_paddle());

  CHECK_THAT(when, m::WithinRel(1.f, .01f));
  CHECK(where_y == a.puck().centre()(1));
}

TEST_CASE("estimating next paddle collision at rhs, half height") {
  auto [puck, box] = []() {
    p::arena_t a{
        []() -> std::tuple<p::scalar_t, p::vec_t> { return {240.f, {0.f, 0.f}}; }};
    return std::make_tuple(
        a.puck(),
        p::box_t{
            p::vec_t{a.lhs_paddle().box().max()(0) + a.puck().radius(),
                     a.box().min()(1) + a.puck().radius()},
            p::vec_t{a.rhs_paddle().box().min()(0) - a.puck().radius(),
                     a.box().max()(1) - a.puck().radius()},
        });
  }();

  const p::scalar_t distance_to_rhs = box.max()(0) - puck.centre()(0);

  const p::scalar_t distance_to_bottom = box.max()(1) - puck.centre()(1);

  /*     +--------------+
   *     |              |
   *     |              |
   *     |              |
   *     |      \     | |
   *     |       \    | |
   *     |        \  /| |
   *     |         \/   |
   *     +--------------+
   */
  p::arena_t a{[&]() -> std::tuple<p::scalar_t, p::vec_t> {
    return {240., p::vec_t{distance_to_rhs, 1.5f * distance_to_bottom}};
  }};

  const auto [when, where_y] = p::estimate_next_collision(a, a.rhs_paddle());

  CHECK_THAT(when, m::WithinRel(1.f, .01f));
  CHECK_THAT(
      where_y,
      m::WithinAbs(a.puck().centre()(1) + distance_to_bottom / 2.f, 1.f));
}

TEST_CASE("estimating next but one paddle collision") {
  // same as estimating the next collision on the rhs except this is a collision
  // with the lhs paddle after initially travelling rightwards

  auto [puck, box] = []() {
    p::arena_t a{
        []() -> std::tuple<p::scalar_t, p::vec_t> { return {240.f, {0.f, 0.f}}; }};
    return std::make_tuple(
        a.puck(),
        p::box_t{
            p::vec_t{a.lhs_paddle().box().max()(0) + a.puck().radius(),
                     a.box().min()(1) + a.puck().radius()},
            p::vec_t{a.rhs_paddle().box().min()(0) - a.puck().radius(),
                     a.box().max()(1) - a.puck().radius()},
        });
  }();

  const p::scalar_t x_range = box.max()(0) - box.min()(0);
  const p::scalar_t distance_to_rhs = box.max()(0) - puck.centre()(0);
  const p::scalar_t distance_to_lhs = distance_to_rhs + x_range;
  const p::scalar_t distance_to_bottom = box.max()(1) - puck.centre()(1);

  p::arena_t a{[&]() -> std::tuple<p::scalar_t, p::vec_t> {
    return {240.f, p::vec_t{distance_to_lhs, 2 * distance_to_bottom}};
  }};

  const auto [when, where_y] = p::estimate_next_collision(a, a.lhs_paddle());

  CHECK_THAT(when, m::WithinRel(1.f, .01f));
  CHECK(where_y == a.puck().centre()(1));
}

TEST_CASE("estimating next paddle collision at lhs, starting height") {
  auto [puck, box] = []() {
    p::arena_t a{
        []() -> std::tuple<p::scalar_t, p::vec_t> { return {240.f, {0.f, 0.f}}; }};
    return std::make_tuple(
        a.puck(),
        p::box_t{
            p::vec_t{a.lhs_paddle().box().max()(0) + a.puck().radius(),
                     a.box().min()(1) + a.puck().radius()},
            p::vec_t{a.rhs_paddle().box().min()(0) - a.puck().radius(),
                     a.box().max()(1) - a.puck().radius()},
        });
  }();

  const p::scalar_t distance_to_lhs = puck.centre()(0) - box.min()(0);
  const p::scalar_t distance_to_top = puck.centre()(1) - box.min()(1);

  p::arena_t a{[&]() -> std::tuple<p::scalar_t, p::vec_t> {
    return {240.f, p::vec_t{-distance_to_lhs, -2 * distance_to_top}};
  }};

  const auto [when, where_y] = p::estimate_next_collision(a, a.lhs_paddle());

  CHECK_THAT(when, m::WithinRel(1.f, .01f));
  CHECK(where_y == a.puck().centre()(1));
}

TEST_CASE("estimating next paddle collision at lhs, 1.5 height") {
  auto [puck, box] = []() {
    p::arena_t a{
        []() -> std::tuple<p::scalar_t, p::vec_t> { return {240.f, {0.f, 0.f}}; }};
    return std::make_tuple(
        a.puck(),
        p::box_t{
            p::vec_t{a.lhs_paddle().box().max()(0) + a.puck().radius(),
                     a.box().min()(1) + a.puck().radius()},
            p::vec_t{a.rhs_paddle().box().min()(0) - a.puck().radius(),
                     a.box().max()(1) - a.puck().radius()},
        });
  }();

  const p::scalar_t distance_to_lhs = puck.centre()(0) - box.min()(0);
  const p::scalar_t distance_to_top = puck.centre()(1) - box.min()(1);

  p::arena_t a{[&]() -> std::tuple<p::scalar_t, p::vec_t> {
    return {240.f, p::vec_t{-distance_to_lhs, -1.5f * distance_to_top}};
  }};

  {
    const auto [when, where_y] = p::estimate_next_collision(a, a.lhs_paddle());

    CHECK_THAT(when, m::WithinRel(1.f, .01f));
    CHECK_THAT(where_y,
               m::WithinAbs(box.min()(1) + .5f * distance_to_top, 1.f));
  }

  a.advance_time(0.1f);

  {
    const auto [when, where_y] = p::estimate_next_collision(a, a.lhs_paddle());

    CHECK_THAT(when, m::WithinRel(.9f, .01f));
    CHECK_THAT(where_y,
               m::WithinAbs(box.min()(1) + .5f * distance_to_top, 1.f));
  }

  a.advance_time(0.1f);

  {
    const auto [when, where_y] = p::estimate_next_collision(a, a.lhs_paddle());

    CHECK_THAT(when, m::WithinRel(.8f, .01f));
    CHECK_THAT(where_y,
               m::WithinAbs(box.min()(1) + .5f * distance_to_top, 1.f));
  }
}

TEST_CASE("estimate_next_collision stability") {
  std::mt19937 prng{c::rngSeed()};
  std::exponential_distribution<float> dt_dist{60.f};
  p::arena_t a{make_starter()};

  // Only going past or colliding with a paddle should change the estimated
  // point of the next collision.  Check that as we advance time, the estimate
  // doesn't change.
  for (int i = 0; i < 1 << 10; ++i) {
    const auto old_dx_sign = std::signbit(a.puck().velocity()(0));
    const auto [old_lhs_when, old_lhs_estimate] =
        p::estimate_next_collision(a, a.lhs_paddle());
    const auto [old_rhs_when, old_rhs_estimate] =
        p::estimate_next_collision(a, a.rhs_paddle());
    a.advance_time(dt_dist(prng));
    const auto new_dx_sign = std::signbit(a.puck().velocity()(0));
    const auto [new_lhs_when, new_lhs_estimate] =
        p::estimate_next_collision(a, a.lhs_paddle());
    const auto [new_rhs_when, new_rhs_estimate] =
        p::estimate_next_collision(a, a.rhs_paddle());
    const auto do_not_check = old_lhs_when == 0 || old_rhs_when == 0 ||
                              new_lhs_when == 0 || new_rhs_when == 0 ||
                              old_dx_sign != new_dx_sign;
    if (!do_not_check) {
      CHECK_THAT(new_lhs_estimate, m::WithinAbs(old_lhs_estimate, 2.f));
      CHECK_THAT(new_rhs_estimate, m::WithinAbs(old_rhs_estimate, 2.f));
    }
  }
}

TEST_CASE("perfect ai vs perfect ai") {
  std::mt19937 prng{c::rngSeed()};
  std::exponential_distribution<float> dt_dist(60.f);

  p::arena_t a{make_starter()};
  p::ai_t ai{c::rngSeed(), 0.f};

  for (int i = 0; i < 1 << 14; ++i) {
    if (const auto y_speed = ai.paddle_speed(a, a.lhs_paddle())) {
      a.lhs_paddle().velocity()(1) = *y_speed;
    }
    if (const auto y_speed = ai.paddle_speed(a, a.rhs_paddle())) {
      a.rhs_paddle().velocity()(1) = *y_speed;
    }
    a.advance_time(dt_dist(prng));
  }

  CHECK(a.lhs_score() == 0);
  CHECK(a.rhs_score() == 0);
  CHECK(a.box().contains(a.puck().centre()));
  CHECK(a.box().contains(a.lhs_paddle().box()));
  CHECK(a.box().contains(a.rhs_paddle().box()));
}

TEST_CASE("imperfect ai") {
  std::mt19937 prng{c::rngSeed()};
  const int attempts = 1 << 14;

  for (int setting : {25, 50, 75}) {
    std::uint32_t score = 0;
    WHEN("setting = " << setting) {
      for (int i = 0; i < attempts; ++i) {
        p::arena_t a{[]() -> std::tuple<p::scalar_t, p::vec_t> {
          return {240.f, {1.f, 0.f}};
        }};
        p::ai_t ai{prng(), 25.f / p::z_scores[setting]};
        const auto result = ai.paddle_speed(a, a.rhs_paddle());
        REQUIRE(!!result);
        a.rhs_paddle().velocity()(1) = *result;
        a.advance_time(a.box().max()(0) - a.puck().centre()(0) -
                       a.puck().radius() + 10);
        auto b = p::bordered(a.rhs_paddle().box(), a.puck().radius());
        score += a.lhs_score();
      }

      CHECK_THAT(score, m::WithinAbs(attempts * (100 - setting) / 100.f,
                                     attempts * .05f));
    }
  }
}

TEST_CASE("make_starter") {
  std::mt19937 prng{c::rngSeed()};
  auto starter = p::make_starter(prng());

  int quadrants = 0;

  for (int i = 0; i < 1 << 14; ++i) {
    const auto [y, vel] = starter();
    REQUIRE(vel(0) != 0.f);
    REQUIRE(vel(1) != 0.f);
    quadrants |= vel(0) < 0.f ? 1 : 2;
    quadrants |= vel(1) < 0.f ? 4 : 8;
    CHECK(y >= 20.f);
    CHECK(y <= 460.f);
    CHECK(vel.norm() >= 150.f);
    CHECK(vel.norm() <= 250.f);
  }

  // puck has been fired into all four quadrants
  CHECK(quadrants == 15);
}
