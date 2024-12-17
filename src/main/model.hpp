#ifndef PONG_MODEL_HPP
#define PONG_MODEL_HPP

#include "geometry.hpp"

#include <array>
#include <concepts>
#include <iostream>
#include <numeric>
#include <optional>

#define MEMBER_GETTER_SETTER(TYPE, NAME)                                       \
public:                                                                        \
  [[nodiscard]] auto &NAME() const { return NAME##_; }                         \
  auto &NAME() { return NAME##_; }                                             \
                                                                               \
private:                                                                       \
  TYPE NAME##_{};

namespace pong {

class circle_t;
class rectangle_t;
class puck_t;
class paddle_t;
class arena_t;
using colour_t = Eigen::Matrix<std::uint8_t, 4, 1>;

class circle_t {
  MEMBER_GETTER_SETTER(vec_t, centre);
  MEMBER_GETTER_SETTER(vec_t, velocity);
  MEMBER_GETTER_SETTER(scalar_t, radius);
  MEMBER_GETTER_SETTER(colour_t, colour);
};

class wall_t {
  MEMBER_GETTER_SETTER(vec_t, origin);
  MEMBER_GETTER_SETTER(vec_t, diagonal);
  MEMBER_GETTER_SETTER(vec_t, velocity);
  MEMBER_GETTER_SETTER(vec_t, normal);
};

class rectangle_t {
  MEMBER_GETTER_SETTER(box_t, box);
  MEMBER_GETTER_SETTER(vec_t, velocity);
  MEMBER_GETTER_SETTER(colour_t, colour);
};

class puck_t : public circle_t {
  using circle_t::circle_t;
};

class paddle_t : public rectangle_t {
  using rectangle_t::rectangle_t;
};

class arena_t : public rectangle_t {
  MEMBER_GETTER_SETTER(puck_t, puck);
  MEMBER_GETTER_SETTER(paddle_t, lhs_paddle);
  MEMBER_GETTER_SETTER(paddle_t, rhs_paddle);
  MEMBER_GETTER_SETTER(std::uint32_t, lhs_score);
  MEMBER_GETTER_SETTER(std::uint32_t, rhs_score);

private:
  std::function<vec_t()> next_puck_velocity_;

public:
  explicit arena_t(std::function<vec_t()> next_puck_velocity)
      : next_puck_velocity_(std::move(next_puck_velocity)) {}

  void init() {
    vec_t top_left{10, 10};
    vec_t size{620, 460};
    box() = {top_left, top_left + size};
    puck().radius() = 5;
    puck().centre() = {320, 240};
    puck().velocity() = next_puck_velocity_();
    puck().colour() = {UINT8_C(0), UINT8_C(255), UINT8_C(0), UINT8_C(255)};
    lhs_paddle().box() = {vec_t{18, 220}, vec_t{22, 260}};
    lhs_paddle().velocity() = {0, 0};
    lhs_paddle().colour() = {UINT8_C(0), UINT8_C(0), UINT8_C(255),
                             UINT8_C(255)};
    rhs_paddle().box() = {vec_t{618, 220}, vec_t{622, 260}};
    rhs_paddle().velocity() = {0, 0};
    rhs_paddle().colour() = {UINT8_C(255), UINT8_C(0), UINT8_C(0),
                             UINT8_C(255)};
  }

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

  void reflect(const vec_t &normal) {
    puck().velocity() =
        puck().velocity() - 2 * puck().velocity().dot(normal) * normal;
  }

  [[nodiscard]] std::optional<std::tuple<scalar_t, std::function<void()>>>
  next_collision_with_arena(const scalar_t dt) {
    std::optional<std::tuple<scalar_t, std::function<void()>>> result{};

    const line_t trajectory{puck().centre(), puck().velocity()};
    const box_t box = bordered(this->box(), -puck().radius());

    using TT = std::tuple<plane_t, vec_t, std::function<void()>>;

    std::array<TT, 12> planes_and_normals{
        TT{plane_t::Through(box.min(), box.min() + unit::i), unit::j,
           [this]() { reflect(unit::j); }},
        TT{plane_t::Through(box.min(), box.min() + unit::j), unit::i,
           [this]() { on_lhs_goal(); }},
        TT{plane_t::Through(box.max(), box.max() + unit::i), -unit::j,
           [this]() { reflect(-unit::j); }},
        TT{plane_t::Through(box.max(), box.max() + unit::j), -unit::i,
           [this]() { on_rhs_goal(); }},
    };

    for (auto &[plane, normal, action] : planes_and_normals) {
      if (trajectory.direction().dot(normal) >= -0.f)
        continue;

      auto when = trajectory.intersectionParameter(plane);

      if (when != when || when < -0.f || when > dt)
        continue;

      if (result && when >= std::get<0>(*result))
        continue;

      result.emplace(when, std::move(action));
    }

    return result;
  }

  /**
   * @arg dt the time period over which to search for the next collision
   * @return optional tuple of:
   * 0: where the centre of the puck is at the next collision
   * 1: the transformation to apply to the puck velocity at the next collision
   * 2: (0 <= when <= dt) the next collision occurs
   */
  [[nodiscard]] std::optional<std::tuple<scalar_t, std::function<void()>>>
  next_collision(const scalar_t dt) {
    assert(!puck().velocity().isZero());

    const line_t trajectory{puck().centre(), puck().velocity()};

    const box_t box = bordered(this->box(), -puck().radius());
    const box_t lhs = bordered(lhs_paddle().box(), puck().radius());
    const box_t rhs = bordered(rhs_paddle().box(), puck().radius());

    if (!bordered(box, 1).contains(puck().centre())) {
      return {};
    }

    assert(bordered(box, 1).contains(puck().centre()));
    assert(!bordered(lhs, -1).contains(puck().centre()));
    assert(!bordered(rhs, -1).contains(puck().centre()));

    // a box within which there are no planes to collide with
    auto empty_box = [&]() {
      vec_t min{lhs.max()(0), box.min()(1)};
      vec_t max{rhs.min()(0), box.max()(1)};
      return bordered(box_t{min, max}, -1);
    }();

    if (empty_box.contains(trajectory.pointAt(0.f)) &&
        empty_box.contains(trajectory.pointAt(dt))) {
      return {};
    }

    std::optional<std::tuple<scalar_t, std::function<void()>>> result;
    result = next_collision_with_arena(dt);

    using TT = std::tuple<box_t, plane_t, vec_t>;
    const std::array<TT, 12> planes_and_normals{
        TT{lhs, plane_t::Through(lhs.min(), lhs.min() + unit::i), -unit::j},
        TT{lhs, plane_t::Through(lhs.min(), lhs.min() + unit::j), -unit::i},
        TT{lhs, plane_t::Through(lhs.max(), lhs.max() + unit::i), unit::j},
        TT{lhs, plane_t::Through(lhs.max(), lhs.max() + unit::j), unit::i},

        TT{rhs, plane_t::Through(rhs.min(), rhs.min() + unit::i), -unit::j},
        TT{rhs, plane_t::Through(rhs.min(), rhs.min() + unit::j), -unit::i},
        TT{rhs, plane_t::Through(rhs.max(), rhs.max() + unit::i), unit::j},
        TT{rhs, plane_t::Through(rhs.max(), rhs.max() + unit::j), unit::i},
    };

    for (const auto &[b, plane, normal] : planes_and_normals) {
      // points in the wrong direction
      if (trajectory.direction().dot(normal) >= -0.f)
        continue;

      const auto when = trajectory.intersectionParameter(plane);

      // collides never, in the past or in the future
      if (when != when || when < -0.f || when > dt)
        continue;

      // FIXME can end up beyond the plane
      const auto where = trajectory.intersectionPoint(plane);

      // isn't within the box's bounds
      // FIXME can end up outside the arena box, check only non-plane extremes?
      if (!b.contains(vec_t{std::round(where(0)), std::round(where(1))}))
        continue;

      // first collision found; record it
      if (!result) {
        result.emplace(when, [=, this]() { reflect(normal); });
        continue;
      }

      const auto last_when = std::get<0>(*result);

      // subsequent collision found; record if earlier than the earliest found
      // so far
      if (when < last_when) {
        result.emplace(when, [=, this]() { reflect(normal); });
      } else if (when == last_when) {
        assert(false); // FIXME
      }
    }

    // FIXME
    //    assert(!result || box.contains(std::get<0>(*result)));
    //    assert(!result || std::get<2>(*result) >= -0.f);
    //    assert(!result || std::get<2>(*result) <= dt);

    return result;
  }

  void advance_time(scalar_t dt) {
    // first move the paddle within its bounds according to its velocity
    auto move_paddle = [&](paddle_t &p) {
      if (p.velocity().isZero())
        return;
      p.box().translate(p.velocity() * dt);
      if (p.box().min()(1) < box().min()(1)) {
        p.box().translate(vec_t{0, box().min()(1) - p.box().min()(1) + 1});
        p.velocity() = {0, 0};
      }
      if (p.box().max()(1) > box().max()(1)) {
        p.box().translate(vec_t{0, box().max()(1) - p.box().max()(1) - 1});
        p.velocity() = {0, 0};
      }
    };
    move_paddle(lhs_paddle());
    move_paddle(rhs_paddle());

    // now perform collisions for dt time
    while (dt > 0) {
      //      std::cout << "dt = " << dt << "\n";
      if (auto collision = next_collision(dt)) {
        auto &[when, action] = *collision;
        puck().centre() = puck().centre() + puck().velocity() * when;
        action();
        dt -= when;
      } else {
        puck().centre() += puck().velocity() * dt;
        dt = 0;
      }
    }
  }
};

} // namespace pong

#undef MEMBER_GETTER_SETTER

#endif // PONG_MODEL_HPP
