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

public:
  circle_t() = default;
  circle_t(const vec_t &centre, const vec_t &velocity, const scalar_t radius,
           const colour_t &colour)
      : centre_(centre), velocity_(velocity), radius_(radius), colour_(colour) {
  }
};

class rectangle_t {
  MEMBER_GETTER_SETTER(box_t, box);
  MEMBER_GETTER_SETTER(vec_t, velocity);
  MEMBER_GETTER_SETTER(colour_t, colour);

public:
  rectangle_t() = default;
  rectangle_t(const box_t &box, vec_t velocity, colour_t colour)
      : box_(box), velocity_(std::move(velocity)), colour_(std::move(colour)) {}
};

class puck_t : public circle_t {
  using circle_t::circle_t;
};

class paddle_t : public rectangle_t {
  using rectangle_t::rectangle_t;
};

class arena_t : public rectangle_t {

  std::function<vec_t()> next_puck_velocity_;

  MEMBER_GETTER_SETTER(puck_t, puck);
  MEMBER_GETTER_SETTER(paddle_t, lhs_paddle);
  MEMBER_GETTER_SETTER(paddle_t, rhs_paddle);
  MEMBER_GETTER_SETTER(std::uint32_t, lhs_score);
  MEMBER_GETTER_SETTER(std::uint32_t, rhs_score);

private:
  using arena_surface_t = std::tuple<plane_t, vec_t, std::function<void()>>;
  std::array<arena_surface_t, 4> arena_surfaces_;

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
            box_t{vec_t{18, 220}, vec_t{22, 260}},
            vec_t{0, 0},
            colour_t{0, 0, 255, 255},
        },
        rhs_paddle_{
            box_t{vec_t{618, 220}, vec_t{622, 260}},
            vec_t{0, 0},
            colour_t{255, 0, 0, 255},
        },
        arena_surfaces_{
            arena_surface_t{
                plane_t::Through(bordered(box(), -puck().radius()).min(),
                                 bordered(box(), -puck().radius()).min() +
                                     unit::i),
                unit::j, [this]() { reflect(unit::j); }},
            arena_surface_t{
                plane_t::Through(bordered(box(), -puck().radius()).max(),
                                 bordered(box(), -puck().radius()).max() +
                                     unit::i),
                -unit::j, [this]() { reflect(-unit::j); }},
            arena_surface_t{
                plane_t::Through(bordered(box(), -puck().radius()).min(),
                                 bordered(box(), -puck().radius()).min() +
                                     unit::j),
                unit::i, [this]() { on_lhs_goal(); }},
            arena_surface_t{
                plane_t::Through(bordered(box(), -puck().radius()).max(),
                                 bordered(box(), -puck().radius()).max() +
                                     unit::j),
                -unit::i, [this]() { on_rhs_goal(); }},
        } {}

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
    // Arena is made up of surfaces with inward facing normals.  The puck is
    // within the arena so will always be facing at least one of these normals
    // (meaning it has a negative dot product with the normal).  Furthermore, it
    // will collide with the same surface(s) in the future since we know it's
    // inside the arena.
    //
    //    +---------------+---------------+   Here P faces the top and rhs
    //    |         ^     |               |   surfaces.
    //    |        /      v               |
    //    +-->    P                    <--+   Of those two surfaces, the top
    //    |               ^               |   surface will be hit first.
    //    |               |               |
    //    +---------------+---------------+
    //
    std::optional<std::tuple<scalar_t, std::function<void()>>> result{};

    const line_t trajectory{puck().centre(), puck().velocity()};
    const box_t box = bordered(this->box(), -puck().radius());

    for (auto &[plane, normal, action] : arena_surfaces_) {
      // If the trajectory isn't heading towards the surface, ignore it.
      if (trajectory.direction().dot(normal) >= -0.f)
        continue;

      // This is the time at which the collision with the surface happens.
      const auto when = trajectory.intersectionParameter(plane);

      // The trajectory is pointing towards the surface and therefore it must
      // intersect in the future.
      assert(when == when);
      assert(when >= -0.f);

      // Ignore collisions after dt.
      if (when > dt)
        continue;

      // If when is equal to the current collision time, then we're in a corner.
      // We fall through this case and overwrite; this favours scoring a goal
      // since the goal surfaces are ordered after the other surfaces, which
      // results in the puck position being reset and means we don't need to
      // consider applying a different normal.
      if (result && when > std::get<0>(*result))
        continue;

      result.emplace(when, action);
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

    std::optional<std::tuple<scalar_t, std::function<void()>>> result =
        next_collision_with_arena(dt);

    const line_t trajectory{puck().centre(), puck().velocity()};
    const box_t lhs = bordered(lhs_paddle().box(), puck().radius());
    const box_t rhs = bordered(rhs_paddle().box(), puck().radius());

    using surface_t = std::tuple<box_t, plane_t, vec_t>;
    const std::array surfaces{
        surface_t{lhs, plane_t::Through(lhs.min(), lhs.min() + unit::i),
                  -unit::j},
        surface_t{lhs, plane_t::Through(lhs.min(), lhs.min() + unit::j),
                  -unit::i},
        surface_t{lhs, plane_t::Through(lhs.max(), lhs.max() + unit::i),
                  unit::j},
        surface_t{lhs, plane_t::Through(lhs.max(), lhs.max() + unit::j),
                  unit::i},

        surface_t{rhs, plane_t::Through(rhs.min(), rhs.min() + unit::i),
                  -unit::j},
        surface_t{rhs, plane_t::Through(rhs.min(), rhs.min() + unit::j),
                  -unit::i},
        surface_t{rhs, plane_t::Through(rhs.max(), rhs.max() + unit::i),
                  unit::j},
        surface_t{rhs, plane_t::Through(rhs.max(), rhs.max() + unit::j),
                  unit::i},
    };

    for (const auto &[b, plane, normal] : surfaces) {
      // Continue if we're not heading in the opposite direction to the normal.
      if (trajectory.direction().dot(normal) >= -0.f)
        continue;

      // When the collision with the surface will occur.
      const auto when = trajectory.intersectionParameter(plane);

      // This should be a real number since there's a negative dot product.
      assert(when == when);

      // If we're the other side of the surface (collision is in the past), or
      // if the collision is after dt, continue.
      if (when < -0.f || when > dt)
        continue;

      // If this collision isn't earlier than any other collision we already
      // found, continue.
      if (result && when > std::get<0>(*result))
        continue;

      const auto where = trajectory.intersectionPoint(plane);

      // There's no collision if we're outside the bounds of the box.
      if (!b.contains(where))
        continue;

      if (when == std::get<0>(*result)) {
        // We hit two surfaces at the same time (a corner); reflect about both
        // normals.
        std::get<1>(*result) = [=, this,
                                delegate = std::move(std::get<1>(*result))]() {
          delegate();
          reflect(normal);
        };
      } else {
        // This is the earliest collision found so far, store it in result.
        result.emplace(when, [=, this]() { reflect(normal); });
      }
    }

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
