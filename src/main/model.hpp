#ifndef PONG_MODEL_HPP
#define PONG_MODEL_HPP

#include "geometry.hpp"

#include <array>
#include <concepts>
#include <iostream>
#include <numeric>
#include <optional>

namespace pong {

class circle_t;
class rectangle_t;
class puck_t;
class paddle_t;
class arena_t;
using colour_t = Eigen::Matrix<std::uint8_t, 4, 1>;

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

  std::optional<std::tuple<scalar_t, std::function<void()>>> next_collision(
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

  void on_lhs_goal() {
    ++rhs_score();
    restart_puck();
  }

  void on_rhs_goal() {
    ++lhs_score();
    restart_puck();
  }

  std::optional<std::tuple<scalar_t, std::function<void()>>> next_collision(
      scalar_t dt,
      std::optional<std::tuple<scalar_t, std::function<void()>>> result);

  void advance_time(scalar_t dt) {
    while (dt > 0) {
      std::optional<std::tuple<scalar_t, std::function<void()>>> next;

      next = lhs_paddle().next_collision(dt, std::move(next));
      next = rhs_paddle().next_collision(dt, std::move(next));
      next = next_collision(dt, std::move(next));

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

std::optional<std::tuple<scalar_t, std::function<void()>>>
paddle_t::next_collision(
    pong::scalar_t dt,
    std::optional<std::tuple<scalar_t, std::function<void()>>> result) {
  // paddle can only move north <-> south
  assert(velocity()(0) == 0.f);

  const box_t b = bordered(box(), arena_.puck().radius());

  // north / south
  {
    const scalar_t ds = velocity()(1) - arena_.puck().velocity()(1);

    if (ds == ds && ds != 0.f) {
      const scalar_t y0 = arena_.puck().centre()(1);
      const scalar_t when = arena_.puck().velocity()(1) > -0.f
                                ? (y0 - b.min()(1)) / ds  // heading south
                                : (y0 - b.max()(1)) / ds; // heading north
      const scalar_t x =
          arena_.puck().centre()(0) + arena_.puck().velocity()(0) * when;

      // if when is in (0, dt) and x is within the bounds of the paddle and this
      // is the earliest found collision, then set the current result to this
      // collision
      if (when >= -0.f && when <= dt && x >= b.min()(0) && x <= b.max()(0) &&
          (!result || when < std::get<0>(*result))) {
        result.emplace(std::forward_as_tuple(
            when, [this]() { arena_.puck().velocity()(1) *= -1; }));
      }
    }
  }

  // east / west
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
  const scalar_t max_y = arena_.box().max()(1) - box().diagonal()(1) - 1;

  assert(min_y < max_y);

  const scalar_t y =
      std::max(min_y, std::min(max_y, box().min()(1) + velocity()(1) * dt));

  box().translate(vec_t{0, y - box().min()(1)});
}

std::optional<std::tuple<scalar_t, std::function<void()>>>
arena_t::next_collision(
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

      if (when == when && when >= -0.f && when <= dt &&
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
      if (when == when && when >= -0.f && when <= dt &&
          (!result || when < std::get<0>(*result))) {
        result.emplace(
            std::forward_as_tuple(when, [this]() { on_rhs_goal(); }));
      }
    } else {
      // heading west
      const scalar_t when = (b.min()(0) - x0) / s;
      if (when == when && when >= -0.f && when <= dt &&
          (!result || when < std::get<0>(*result))) {
        result.emplace(
            std::forward_as_tuple(when, [this]() { on_lhs_goal(); }));
      }
    }
  }

  return result;
}

} // namespace pong

#endif // PONG_MODEL_HPP
