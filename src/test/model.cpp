#include <catch2/catch_all.hpp>

#include "model.hpp"
#include <cstring>
#include <random>

namespace {
namespace p = pong;

std::function<p::vec_t()> make_starter() {
  return [prng = std::mt19937{Catch::rngSeed()},
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
  std::mt19937 prng{Catch::rngSeed()};

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
//    test_t{1/60.f, {116, 164, 0, 68, 0, 128, 232, 67, 64, 235, 202, 194, 192, 142, 220, 66,}},
//    test_t{1/60.f, {93, 9, 28, 68, 10, 29, 86, 67, 80, 126, 209, 66, 166, 70, 225, 66,}},
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

TEST_CASE("translated_within_limits") {}
