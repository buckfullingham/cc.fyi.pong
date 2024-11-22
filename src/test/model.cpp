#include <catch2/catch_all.hpp>

#include "model.hpp"
#include <cstring>
#include <random>

TEST_CASE("horizontal and vertical collisions with arena") {
  pong::arena_t a;
  using pong::box_t;
  using pong::vec_t;
  a.box().min() = {-100.f, -100.f};
  a.box().max() = {100.f, 100.f};
  a.puck().centre() = {};
  a.puck().radius() = 5.f;

  // place paddles out of the way
  a.lhs_paddle().box() = box_t{vec_t{0.f, 0.f}, vec_t{1.f, 1.f}}.translated(
      a.box().min() + vec_t{25.f, 10.f});
  a.rhs_paddle().box() = box_t{vec_t{0.f, 0.f}, vec_t{1.f, 1.f}}.translated(
      a.box().max() - vec_t{25.f, 10.f});

  const auto x = GENERATE(-1.f, 0.f, +1.f);
  const auto y = GENERATE(-1.f, 0.f, +1.f);

  if (x != 0.f || y != 0.f) {
    WHEN("x == " << x << " and y = " << y) {
      a.puck().velocity() = {x, y};
      CHECK(!a.next_collision(94.f));
      auto collision = a.next_collision(96.f);
      REQUIRE(!!collision);
      CHECK(std::get<0>(*collision).isApprox(95.f * a.puck().velocity()));
    }
  }
}

TEST_CASE("advance time through a horizontal collision with a paddle") {
  pong::arena_t a;
  a.box().min() = {-100.f, -100.f};
  a.box().max() = {100.f, 100.f};
  a.puck().centre() = {};
  a.puck().radius() = 5.f;
  a.puck().velocity() = {1.f, 0.f};
  a.lhs_paddle().box().min() = {10.f, 10.f};
  a.lhs_paddle().box().max() = {11.f, 11.f};
  a.rhs_paddle().box().min() = {50.f, -50.f};
  a.rhs_paddle().box().max() = {60.f, 50.f};
  a.advance_time(44.f);
  CHECK(a.puck().velocity() == pong::vec_t{1.f, 0.f});
  a.advance_time(2.f);
  CHECK(a.puck().velocity().isApprox(pong::vec_t{-1.f, 0.f}));
}

TEST_CASE("ensure puck doesn't break through walls") {
  auto random_velocity =
      [prng = std::mt19937{Catch::rngSeed()},
       dist = std::uniform_real_distribution<float>{-200.f, 200.f}]() mutable {
        auto x = dist(prng);
        auto y = dist(prng);
        return pong::vec_t{x, y};
      };

  pong::arena_t a;
  a.init();

  const pong::vec_t adjustment{a.puck().radius(), a.puck().radius()};

  const pong::box_t adjusted_arena{
      a.box().min() + adjustment,
      a.box().max() - adjustment,
  };

  const pong::box_t adjusted_lhs_paddle{
      a.lhs_paddle().box().min() - adjustment,
      a.lhs_paddle().box().max() + adjustment,
  };

  const pong::box_t adjusted_rhs_paddle{
      a.rhs_paddle().box().min() - adjustment,
      a.rhs_paddle().box().max() + adjustment,
  };

  auto error = [&](const auto &p) {
    union {
      char chars[sizeof(float) * 4];
      float floats[4];
    };

    floats[0] = p.centre()(0);
    floats[1] = p.centre()(1);
    floats[2] = p.velocity()(0);
    floats[3] = p.velocity()(1);

    for (unsigned char c : chars)
      std::cout << unsigned(c) << ", ";

    std::cout << "\n";

    REQUIRE(false);
  };

  for (int i = 0; i < 1 << 3; ++i) {
    a.puck().velocity() = random_velocity();
    REQUIRE(adjusted_arena.contains(a.puck().centre()));
    REQUIRE(!adjusted_lhs_paddle.contains(a.puck().centre()));
    REQUIRE(!adjusted_rhs_paddle.contains(a.puck().centre()));
    for (int j = 0; j < 1 << 7; ++j) {
      const auto copy = a.puck();
      a.advance_time(1.f / 60.f);

      if (!adjusted_arena.contains(a.puck().centre()))
        error(copy);

      if (adjusted_lhs_paddle.contains(a.puck().centre()))
        error(copy);

      if (adjusted_rhs_paddle.contains(a.puck().centre()))
        error(copy);
    }
  }
}

TEST_CASE("known breakout case") {
  // known bug in an earlier version relating to a collision at -0 time
  pong::arena_t a;
  a.init();

  unsigned char chars[] = {116, 164, 0,   68,  0,   128, 232, 67,
                           64,  235, 202, 194, 192, 142, 220, 66};

  float floats[4]{};
  std::memcpy(floats, chars, sizeof(floats));

  a.puck().centre()(0) = floats[0];
  a.puck().centre()(1) = floats[1];
  a.puck().velocity()(0) = floats[2];
  a.puck().velocity()(1) = floats[3];

  const pong::vec_t adjustment{a.puck().radius(), a.puck().radius()};

  const pong::box_t adjusted_arena{
      a.box().min() + adjustment,
      a.box().max() - adjustment,
  };

  const pong::box_t adjusted_lhs_paddle{
      a.lhs_paddle().box().min() - adjustment,
      a.lhs_paddle().box().max() + adjustment,
  };

  const pong::box_t adjusted_rhs_paddle{
      a.rhs_paddle().box().min() - adjustment,
      a.rhs_paddle().box().max() + adjustment,
  };

  REQUIRE(adjusted_arena.contains(a.puck().centre()));
  REQUIRE(!adjusted_lhs_paddle.contains(a.puck().centre()));
  REQUIRE(!adjusted_rhs_paddle.contains(a.puck().centre()));
  a.advance_time(1.f / 60.f);
  CHECK(adjusted_arena.contains(a.puck().centre()));
  CHECK(!adjusted_lhs_paddle.contains(a.puck().centre()));
  CHECK(!adjusted_rhs_paddle.contains(a.puck().centre()));
}
