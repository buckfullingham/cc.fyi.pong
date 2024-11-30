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

public:
  void init() {
    vec_t top_left{10, 10};
    vec_t size{620, 460};
    box() = {top_left, top_left + size};
    puck().radius() = 5;
    puck().centre() = {320, 240};
    puck().velocity() = {197, 87};
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

  /**
   * @arg dt the time period over which to search for the next collision
   * @return optional tuple of:
   * 0: where the centre of the puck is at the next collision
   * 1: the transformation to apply to the puck velocity at the next collision
   * 2: (0 <= when <= dt) the next collision occurs
   */
  [[nodiscard]] std::optional<std::tuple<vec_t, matrix_t, scalar_t>>
  next_collision(const scalar_t dt) const {
    assert(!puck().velocity().isZero());

    const line_t trajectory{puck().centre(), puck().velocity()};

    // enlarge (shrink) a box
    auto enlarge = [](box_t result, const scalar_t adjustment) {
      const vec_t v{adjustment, adjustment};
      result.min() -= v;
      result.max() += v;
      return result;
    };

    const box_t box = enlarge(this->box(), -puck().radius());
    const box_t lhs = enlarge(lhs_paddle().box(), puck().radius());
    const box_t rhs = enlarge(rhs_paddle().box(), puck().radius());

    assert(enlarge(box, 1).contains(puck().centre()));
    assert(!enlarge(lhs, -1).contains(puck().centre()));
    assert(!enlarge(rhs, -1).contains(puck().centre()));

    // a box within which there are no planes to collide with
    auto empty_box = [&]() {
      vec_t min{lhs.max()(0), box.min()(1)};
      vec_t max{rhs.min()(0), box.max()(1)};
      return enlarge(box_t{min, max}, -1);
    }();

    if (empty_box.contains(trajectory.pointAt(0.f)) &&
        empty_box.contains(trajectory.pointAt(dt))) {
      return {};
    }

    std::optional<std::tuple<vec_t, matrix_t, scalar_t>> result;

    for (const auto &b : {box, lhs, rhs}) {
      const std::array planes_and_transforms{
          std::make_tuple(plane_t::Through(b.min(), b.min() + unit::i),
                          transform::flip_y),
          std::make_tuple(plane_t::Through(b.min(), b.min() + unit::j),
                          transform::flip_x),
          std::make_tuple(plane_t::Through(b.max(), b.max() + unit::i),
                          transform::flip_y),
          std::make_tuple(plane_t::Through(b.max(), b.max() + unit::j),
                          transform::flip_x),
      };

      for (const auto &[plane, transform] : planes_and_transforms) {
        const auto when = trajectory.intersectionParameter(plane);
        const auto where = trajectory.intersectionPoint(plane);

        // collides never, in the past or in the future
        if (when != when || when < -0.f || when > dt)
          continue;

        // isn't within the box's bounds
        if (!b.contains(where))
          continue;

        if (!result) {
          result.emplace(where, transform, when);
          continue;
        }

        const auto last_when = std::get<2>(*result);

        if (when < last_when) { // this collision is earlier than the last
          result.emplace(where, transform, when);
        } else if (when == last_when) { // concurrent collisions
          result.emplace(where, transform * std::get<1>(*result), when);
        }
      }
    }

    assert(!result || box.contains(std::get<0>(*result)));
    assert(!result || std::get<2>(*result) >= -0.f);
    assert(!result || std::get<2>(*result) <= dt);

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
      if (auto collision = next_collision(dt)) {
        auto &[where, transformation, when] = *collision;
        // if we arrive at exactly the time of a collision then we can end up
        // in an infinite loop; avoid this by ensuring we always increment time
        // some small amount whenever there is a collision
        constexpr auto time_incr = float(1/1000.f);
        puck().velocity() = transformation * puck().velocity();
        puck().centre() = where + puck().velocity() * time_incr;
        dt -= when + time_incr;
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
