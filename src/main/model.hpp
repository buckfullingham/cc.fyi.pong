#ifndef PONG_MODEL_HPP
#define PONG_MODEL_HPP

#include "geometry.hpp"

#include <array>
#include <concepts>
#include <iostream>
#include <numeric>

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
  MEMBER_GETTER_SETTER(Eigen::Vector2f, centre);
  MEMBER_GETTER_SETTER(Eigen::Vector2f, velocity);
  MEMBER_GETTER_SETTER(float, radius);
  MEMBER_GETTER_SETTER(colour_t, colour);
};

class rectangle_t {
  MEMBER_GETTER_SETTER(Eigen::AlignedBox2f, box);
  MEMBER_GETTER_SETTER(Eigen::Vector2f, velocity);
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

  template <std::floating_point F> static bool is_approx(F lhs, F rhs) {
    return std::abs(lhs - rhs) < .5f;
  }

public:
  void init() {
    vec_t top_left{10.f, 10.f};
    vec_t size{620.f, 460.f};
    box() = {top_left, top_left + size};
    puck().radius() = 5.f;
    puck().centre() = {320.f, 240.f};
    puck().velocity() = {197.f, 87.f};
    puck().colour() = {UINT8_C(0), UINT8_C(255), UINT8_C(0), UINT8_C(255)};
    lhs_paddle().box() = {vec_t{18.f, 220.f}, vec_t{22.f, 260.f}};
    lhs_paddle().velocity() = {0.f, 0.f};
    lhs_paddle().colour() = {UINT8_C(0), UINT8_C(0), UINT8_C(255),
                             UINT8_C(255)};
    rhs_paddle().box() = {vec_t{618.f, 220.f}, vec_t{622.f, 260.f}};
    rhs_paddle().velocity() = {0.f, 0.f};
    rhs_paddle().colour() = {UINT8_C(255), UINT8_C(0), UINT8_C(0),
                             UINT8_C(255)};
  }

  [[nodiscard]] std::optional<std::tuple<vec_t, matrix_t, float>>
  next_collision(const float dt) const {
    using namespace unit;

    // only call this when the puck is moving
    assert(puck().velocity().norm() != 0.f);

    std::optional<std::tuple<vec_t, matrix_t, float>> result{};

    // add (or subtract) a border around a box to account for puck radius when
    // calculating collisions: positive r means a bigger box
    const auto adjust = [](box_t result, float r) {
      result.min() -= vec_t{r, r};
      result.max() += vec_t{r, r};
      return result;
    };

    const auto adjusted_box = adjust(box(), -puck().radius());
    const auto adjusted_lhs_paddle = adjust(lhs_paddle().box(), puck().radius());
    const auto adjusted_rhs_paddle = adjust(rhs_paddle().box(), puck().radius());

    // there are no walls to collide with in the empty box
    auto empty_box = adjust(box_t{
        vec_t{adjusted_lhs_paddle.max()(0), adjusted_box.min()(1)},
        vec_t{adjusted_rhs_paddle.min()(0), adjusted_box.max()(1)},
    }, -1.f);

    assert(adjusted_box.contains(empty_box));
    assert(!empty_box.contains(adjusted_lhs_paddle));
    assert(!empty_box.contains(adjusted_rhs_paddle));
    assert(!empty_box.intersects(adjusted_lhs_paddle));
    assert(!empty_box.intersects(adjusted_rhs_paddle));

    // if the trajectory is contained within the empty box, there was no
    // collision in dt
    if (empty_box.contains(puck().centre()) &&
        empty_box.contains(puck().centre() + puck().velocity() * dt)) {
      return {};
    }

    // boxes that the puck can collide with (arena, paddle), adjusted to take
    // into account the puck's radius
    const std::array boxes{
        adjusted_box,
        adjusted_lhs_paddle,
        adjusted_rhs_paddle,
    };

    // a parametrized line describing the puck's trajectory
    const line_t trajectory{puck().centre(), puck().velocity().normalized()};

    for (auto &b : boxes) {
      // the planes representing the sides of the box in question and the
      // transformation that occurs upon collision with the plane
      auto plane_and_transform = [](auto &p0, auto &p1, auto &t) {
        return std::tuple<plane_t, matrix_t>(plane_t::Through(p0, p1), t);
      };

      const std::array planes_and_transforms{
          plane_and_transform(b.min(), b.min() + i, flip_y),
          plane_and_transform(b.min(), b.min() + j, flip_x),
          plane_and_transform(b.max(), b.max() + i, flip_y),
          plane_and_transform(b.max(), b.max() + j, flip_x),
      };

      for (auto &[plane, transform] : planes_and_transforms) {
        // where on trajectory collision occurs
        const auto where = trajectory.intersectionPoint(plane);
        // when it occurs
        const auto when =
            trajectory.intersectionParameter(plane) / puck().velocity().norm();
        // must happen within the box (ie on one of its sides), not before time
        // -0f and not after time dt
        if (!b.contains(where) || when > dt || when < -0.f)
          continue;
        // if this is the first collision we've found, or it happens before any
        // other collision we already found, make this collision the result
        if (!result || when < std::get<2>(*result))
          result = {where, transform, when};
      }
    }

    assert(!result || std::get<2>(*result) <= dt);
    return result;
  }

  void advance_time(float dt) {
    // first move the paddle within its bounds according to its velocity
    auto move_paddle = [&](paddle_t &p) {
      if (p.velocity().isZero())
        return;
      p.box().translate(p.velocity() * dt);
      if (p.box().min()(1) < box().min()(1)) {
        p.box().translate(vec_t{0.f, box().min()(1) - p.box().min()(1) + 1.f});
        p.velocity() = {0.f, 0.f};
      }
      if (p.box().max()(1) > box().max()(1)) {
        p.box().translate(vec_t{0.f, box().max()(1) - p.box().max()(1) - 1.f});
        p.velocity() = {0.f, 0.f};
      }
    };
    move_paddle(lhs_paddle());
    move_paddle(rhs_paddle());

    // now perform collisions for dt time
    while (dt > 0.f) {
      if (auto collision = next_collision(dt)) {
        auto &[where, transformation, when] = *collision;
        // if we arrive at exactly the time of a collision then we can end up
        // in an infinite loop; avoid this by ensuring we always increment time
        // some small amount whenever there is a collision
        dt -= when;
        puck().centre() = where;
        puck().velocity() = transformation * puck().velocity();
      } else {
        puck().centre() += puck().velocity() * dt;
        dt = 0.f;
      }
    }
  }
};

} // namespace pong

#undef MEMBER_GETTER_SETTER

#endif // PONG_MODEL_HPP
