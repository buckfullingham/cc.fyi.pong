#include <catch2/catch_all.hpp>

#include <random>

TEST_CASE("generating random numbers") {
  std::mt19937 prng{std::random_device{}()};
  std::uniform_int_distribution<unsigned int> dist(0u, 1u);
  CHECK(dist(prng) < 2u);
}
