#ifndef PONG_MODEL_HPP
#define PONG_MODEL_HPP

#include "geometry.hpp"

#include <array>
#include <concepts>
#include <iostream>
#include <numeric>
#include <optional>
#include <random>

namespace pong {

class circle_t;
class rectangle_t;
class puck_t;
class paddle_t;
class arena_t;
using colour_t = Eigen::Matrix<std::uint8_t, 4, 1>;

/**
 * A function that oscillates linearly from 0 to upper_bound - 1.
 *
 * e.g. for upper_bound == 4, this will yield (for 0 <= x << 12):
 *     0 1 2 3 2 1 0 1 2 3 2 1
 */
inline std::uint64_t linear_oscillation(const std::uint64_t upper_bound,
                                        std::uint64_t x) {
  x += x / (upper_bound - 1);
  return (x / upper_bound) % 2 == 0 ? x % upper_bound
                                    : upper_bound - 1 - x % upper_bound;
}

/**
 * Find the (first) inverse of linear_oscillation given output from the
 * function and the direction of its derivative (positive true, negative false).
 */
inline std::uint64_t linear_oscillation_inverse(const std::uint64_t upper_bound,
                                                std::uint64_t x,
                                                bool dx_positive) {
  return dx_positive ? x : 2 * upper_bound - x - 2;
}

inline std::tuple<scalar_t, scalar_t> estimate_next_collision(const arena_t &,
                                                              const paddle_t &);

class circle_t {
public:
  circle_t() = default;
  circle_t(vec_t centre, vec_t velocity, const scalar_t radius, colour_t colour)
      : centre_(std::move(centre)), velocity_(std::move(velocity)),
        radius_(radius), colour_(std::move(colour)) {}

  [[nodiscard]] auto &centre() const { return centre_; }
  auto &centre() { return centre_; }

  [[nodiscard]] auto &velocity() const { return velocity_; }
  auto &velocity() { return velocity_; }

  [[nodiscard]] auto &radius() const { return radius_; }
  auto &radius() { return radius_; }

  [[nodiscard]] auto &colour() const { return colour_; }
  auto &colour() { return colour_; }

private:
  vec_t centre_{};
  vec_t velocity_{};
  scalar_t radius_{};
  colour_t colour_{};
};

class rectangle_t {
public:
  rectangle_t() = default;
  rectangle_t(const box_t &box, vec_t velocity, colour_t colour)
      : box_(box), velocity_(std::move(velocity)), colour_(std::move(colour)) {}

  [[nodiscard]] auto &box() const { return box_; }
  auto &box() { return box_; }

  [[nodiscard]] auto &velocity() const { return velocity_; }
  auto &velocity() { return velocity_; }

  [[nodiscard]] auto &colour() const { return colour_; }
  auto &colour() { return colour_; }

  [[nodiscard]] vec_t centre() const { return box_.min() + box_.diagonal() * .5f; };


private:
  box_t box_{};
  vec_t velocity_{};
  colour_t colour_{};
};

class puck_t : public circle_t {
public:
  using circle_t::circle_t;
  void advance_time(scalar_t dt) { centre() = centre() + velocity() * dt; }
};

class paddle_t : public rectangle_t {
public:
  template <typename... Args>
  explicit paddle_t(arena_t &arena, Args &&...args)
      : rectangle_t{std::forward<Args>(args)...}, arena_{arena} {}

  std::optional<std::tuple<scalar_t, std::function<void()>>> next_action(
      scalar_t dt,
      std::optional<std::tuple<scalar_t, std::function<void()>>> result);

  void advance_time(scalar_t dt);

private:
  arena_t &arena_;
};

class arena_t : public rectangle_t {
public:
  explicit arena_t(std::function<vec_t()> next_puck_velocity)
      : rectangle_t{box_t{vec_t{10, 10}, vec_t{630, 470}}, vec_t{0, 0},
                    colour_t{0, 0, 0, 0}},
        next_puck_velocity_{std::move(next_puck_velocity)},
        puck_{
            vec_t{320, 240},
            next_puck_velocity_(),
            5,
            colour_t{0, 255, 0, 255},
        },
        lhs_paddle_{
            *this,
            box_t{vec_t{18, 220}, vec_t{22, 260}},
            vec_t{0, 0},
            colour_t{0, 0, 255, 255},
        },
        rhs_paddle_{
            *this,
            box_t{vec_t{618, 220}, vec_t{622, 260}},
            vec_t{0, 0},
            colour_t{255, 0, 0, 255},
        },
        lhs_score_{}, rhs_score_{} {}

  [[nodiscard]] auto &puck() const { return puck_; }
  auto &puck() { return puck_; }

  [[nodiscard]] auto &lhs_paddle() const { return lhs_paddle_; }
  auto &lhs_paddle() { return lhs_paddle_; }

  [[nodiscard]] auto &rhs_paddle() const { return rhs_paddle_; }
  auto &rhs_paddle() { return rhs_paddle_; }

  [[nodiscard]] auto &lhs_score() const { return lhs_score_; }
  auto &lhs_score() { return lhs_score_; }

  [[nodiscard]] auto &rhs_score() const { return rhs_score_; }
  auto &rhs_score() { return rhs_score_; }

  void restart_puck() {
    puck().centre() = vec_t{320, 240};
    puck().velocity() = next_puck_velocity_();
  }

  std::optional<std::tuple<scalar_t, std::function<void()>>> next_action(
      scalar_t dt,
      std::optional<std::tuple<scalar_t, std::function<void()>>> result);

  void advance_time(scalar_t dt) {
    while (dt > 0) {
      std::optional<std::tuple<scalar_t, std::function<void()>>> next;

      next = lhs_paddle().next_action(dt, std::move(next));
      next = rhs_paddle().next_action(dt, std::move(next));
      next = next_action(dt, std::move(next));

      if (next) {
        auto &[when, action] = *next;
        puck().advance_time(when);
        lhs_paddle().advance_time(when);
        rhs_paddle().advance_time(when);
        action();
        dt -= when;
      } else {
        puck().advance_time(dt);
        lhs_paddle().advance_time(dt);
        rhs_paddle().advance_time(dt);
        dt = 0;
      }
    }
  }

private:
  std::function<vec_t()> next_puck_velocity_;
  puck_t puck_;
  paddle_t lhs_paddle_;
  paddle_t rhs_paddle_;
  std::uint32_t lhs_score_;
  std::uint32_t rhs_score_;
};

class ai_t {
public:
  explicit ai_t(std::mt19937::result_type seed, scalar_t prob_reaction,
                scalar_t stdev)
      : prng_(seed), prob_reaction_(prob_reaction), react_dist_(0., 1.),
        error_dist_(0.f, stdev) {}

  ai_t(const ai_t &) = delete;
  ai_t &operator=(const ai_t &) = delete;

  std::optional<scalar_t> paddle_speed(arena_t &, paddle_t &);

private:
  bool react() { return react_dist_(prng_) < prob_reaction_; }

  std::mt19937 prng_;
  scalar_t prob_reaction_;
  std::uniform_real_distribution<double> react_dist_;
  std::normal_distribution<scalar_t> error_dist_;
};

std::optional<std::tuple<scalar_t, std::function<void()>>>
paddle_t::next_action(
    pong::scalar_t dt,
    std::optional<std::tuple<scalar_t, std::function<void()>>> result) {
  // paddle can only move north <-> south
  assert(velocity()(0) == 0.f);

  if (velocity()(1) != 0.f) {
    // paddle hits top or bottom of arena
    const scalar_t when =
        velocity()(1) > 0.f
            ? (arena_.box().max()(1) - box().max()(1) - 1.f) / velocity()(1)
            : (arena_.box().min()(1) - box().min()(1) + 1.f) / velocity()(1);

    if (when > -0.f && when <= dt && (!result || when < std::get<0>(*result))) {
      result.emplace(
          std::forward_as_tuple(when, [this]() {
            velocity() = vec_t{0, 0};
          }));
    }
  }

  const box_t b = bordered(box(), arena_.puck().radius());

  // north / south surfaces
  {
    const scalar_t ds = velocity()(1) - arena_.puck().velocity()(1);

    if (ds == ds && ds != 0.f) {
      const scalar_t y0 = arena_.puck().centre()(1);
      const scalar_t when = arena_.puck().velocity()(1) > -0.f
                                ? (y0 - b.min()(1)) / ds  // heading south
                                : (y0 - b.max()(1)) / ds; // heading north
      const scalar_t x =
          arena_.puck().centre()(0) + arena_.puck().velocity()(0) * when;

      // if when is in (0, dt) and x is within the bounds of the paddle and
      // this is the earliest found collision, then set the current result to
      // this collision
      if (when >= -0.f && when <= dt && x >= b.min()(0) && x <= b.max()(0) &&
          (!result || when < std::get<0>(*result))) {
        result.emplace(std::forward_as_tuple(
            when, [this]() { arena_.puck().velocity()(1) *= -1; }));
      }
    }
  }

  // east / west surfaces
  {
    const scalar_t x0 = arena_.puck().centre()(0);
    const scalar_t s = arena_.puck().velocity()(0);
    const scalar_t when = arena_.puck().velocity()(0) > -0.f
                              ? (b.min()(0) - x0) / s  // heading east
                              : (b.max()(0) - x0) / s; // heading west
    const scalar_t y =
        arena_.puck().centre()(1) + arena_.puck().velocity()(1) * when;

    // if when is in (0, dt) and y is within the bounds of the paddle and this
    // is the earliest found collision, then set the current result to this
    // collision
    if (when >= -0.f && when <= dt && y >= b.min()(1) && y <= b.max()(1) &&
        (!result || when < std::get<0>(*result))) {
      result.emplace(std::forward_as_tuple(
          when, [this]() { arena_.puck().velocity()(0) *= -1; }));
    }
  }

  return result;
}

void paddle_t::advance_time(scalar_t dt) {
  assert(velocity()(0) == 0.f);
  assert(box().diagonal()(1) > 0.f);

  const scalar_t min_y = arena_.box().min()(1) + 1;
  const scalar_t max_y =
      arena_.box().max()(1) - (box().max()(1) - box().min()(1)) - 1;

  assert(min_y < max_y);

  const scalar_t y =
      std::max(min_y, std::min(max_y, box().min()(1) + velocity()(1) * dt));

  box().translate(vec_t{0, y - box().min()(1)});
}

std::optional<std::tuple<scalar_t, std::function<void()>>> arena_t::next_action(
    scalar_t dt,
    std::optional<std::tuple<scalar_t, std::function<void()>>> result) {

  const box_t b = bordered(box(), -puck().radius());

  // north / south
  {
    const scalar_t s = -puck().velocity()(1);

    if (s == s && s != 0.f) {
      const scalar_t y0 = puck().centre()(1);

      const scalar_t when = puck().velocity()(1) > -0.f
                                ? (y0 - b.max()(1)) / s  // heading south
                                : (y0 - b.min()(1)) / s; // heading north

      if (when >= -0.f && when <= dt &&
          (!result || when < std::get<0>(*result))) {
        result.emplace(std::forward_as_tuple(
            when, [this]() { puck().velocity()(1) *= -1; }));
      }
    }
  }

  // east / west
  {
    const scalar_t x0 = puck().centre()(0);
    const scalar_t s = puck().velocity()(0);
    if (puck().velocity()(0) > -0.f) {
      // heading east
      const scalar_t when = (b.max()(0) - x0) / s;
      if (when >= -0.f && when <= dt &&
          (!result || when < std::get<0>(*result))) {
        result.emplace(std::forward_as_tuple(when, [this]() {
          ++lhs_score_;
          restart_puck();
        }));
      }
    } else {
      // heading west
      const scalar_t when = (b.min()(0) - x0) / s;
      if (when >= -0.f && when <= dt &&
          (!result || when < std::get<0>(*result))) {
        result.emplace(std::forward_as_tuple(when, [this]() {
          ++rhs_score_;
          restart_puck();
        }));
      }
    }
  }

  return result;
}

inline std::tuple<scalar_t, scalar_t>
estimate_next_collision(const arena_t &a, const paddle_t &p) {
  assert(&a.lhs_paddle() == &p || &a.rhs_paddle() == &p);

  assert(a.puck().velocity()(0) != 0.f);

  const bool is_lhs = &p == &a.lhs_paddle();
  const bool is_going_right = a.puck().velocity()(0) > 0.f;

  const box_t box{
      vec_t{a.lhs_paddle().box().max()(0) + a.puck().radius(),
            a.box().min()(1) + a.puck().radius()},
      vec_t{a.rhs_paddle().box().min()(0) - a.puck().radius(),
            a.box().max()(1) - a.puck().radius()},
  };

  if (!box.contains(a.puck().centre())) {
    // don't move if the puck has already left the box
    return {0.f, p.centre()(1)};
  }

  const plane_t plane =
      is_lhs
          ? plane_t::Through(box.min(), box.min() + unit::j)
          : plane_t::Through(box.max(), box.max() + unit::j);

  const line_t trajectory{a.puck().centre(), a.puck().velocity()};

  const auto y_range = std::uint64_t(box.max()(1) - box.min()(1));

  const auto width = box.max()(0) - box.min()(0);
  const auto x = a.puck().centre()(0) - box.min()(0);
  const auto y = a.puck().centre()(1) - box.min()(1);

  // how far in x-terms until we hit the boundary
  const scalar_t x_to_go =
      [&]() {
        if (is_lhs) {
          return is_going_right ? 2 * width - x : x;
        } else {
          return is_going_right ? width - x : width + x;
        }
      }();

  assert(x_to_go >= 0.f);

  const scalar_t when = x_to_go / std::abs(a.puck().velocity()(0));

  // where are we in the y oscillation when we've gone x_to_go in y
  const std::uint64_t estimated_y =
      box.min()(1) +
      linear_oscillation(
          y_range + 1,
          linear_oscillation_inverse(y_range + 1,
                                     std::uint64_t(y),
                                     a.puck().velocity()(1) > 0) +
              x_to_go * std::abs(a.puck().velocity()(1) / a.puck().velocity()(0)));

  return {when, estimated_y};
}

std::optional<scalar_t> ai_t::paddle_speed(arena_t &a, paddle_t &p) {
  if (!react())
    return {};

  const auto [when, target] = estimate_next_collision(a, p);

  return when == 0.f ? 0.f : (target - p.centre()(1) + error_dist_(prng_)) / when;
}

} // namespace pong

#endif // PONG_MODEL_HPP
