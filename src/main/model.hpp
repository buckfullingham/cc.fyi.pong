#ifndef PONG_MODEL_HPP
#define PONG_MODEL_HPP

#include "geometry.hpp"

#include <numeric>
#include <optional>
#include <random>
#include <utility>

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
     * e.g. for upper_bound == 4, this will yield (for 0 <= x < 12):
     *     0 1 2 3 2 1 0 1 2 3 2 1
     */
    inline std::uint64_t linear_oscillation(const std::uint64_t upper_bound,
                                            std::uint64_t x) {
        x += x / (upper_bound - 1);
        return (x / upper_bound) % 2 == 0
                   ? x % upper_bound
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

    constexpr float z_scores[100]{
        0.f, 0.01253347f, 0.025068908f, 0.037608288f, 0.050153583f,
        0.062706778f, 0.075269862f, 0.087844838f, 0.100433721f, 0.113038541f,
        0.125661347f, 0.138304208f, 0.150969215f, 0.163658486f, 0.176374165f,
        0.189118426f, 0.201893479f, 0.214701568f, 0.227544977f, 0.240426031f,
        0.253347103f, 0.266310613f, 0.279319034f, 0.292374896f, 0.305480788f,
        0.318639364f, 0.331853346f, 0.345125531f, 0.358458793f, 0.371856089f,
        0.385320466f, 0.398855066f, 0.412463129f, 0.426148008f, 0.439913166f,
        0.45376219f, 0.467698799f, 0.48172685f, 0.495850347f, 0.510073457f,
        0.524400513f, 0.53883603f, 0.55338472f, 0.568051498f, 0.582841507f,
        0.597760126f, 0.612812991f, 0.628006014f, 0.643345405f, 0.658837693f,
        0.67448975f, 0.690308824f, 0.706302563f, 0.722479052f, 0.738846849f,
        0.755415026f, 0.772193214f, 0.789191653f, 0.806421247f, 0.82389363f,
        0.841621234f, 0.859617364f, 0.877896295f, 0.896473364f, 0.915365088f,
        0.934589291f, 0.954165253f, 0.974113877f, 0.994457883f, 1.015222033f,
        1.036433389f, 1.058121618f, 1.080319341f, 1.103062556f, 1.126391129f,
        1.15034938f, 1.174986792f, 1.200358858f, 1.22652812f, 1.253565438f,
        1.281551566f, 1.310579112f, 1.340755034f, 1.372203809f, 1.40507156f,
        1.439531471f, 1.475791028f, 1.514101888f, 1.554773595f, 1.59819314f,
        1.644853627f, 1.69539771f, 1.750686071f, 1.811910673f, 1.880793608f,
        1.959963985f, 2.053748911f, 2.170090378f, 2.326347874f, 2.575829304f,
    };

    inline std::tuple<scalar_t, scalar_t> estimate_next_collision(const arena_t &,
                                                                  const paddle_t &);

    class circle_t {
    public:
        circle_t() = default;

        circle_t(vec_t centre, vec_t velocity, const scalar_t radius, colour_t colour)
            : centre_(std::move(centre)), velocity_(std::move(velocity)),
              radius_(radius), colour_(std::move(colour)) {
        }

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
            : box_(box), velocity_(std::move(velocity)), colour_(std::move(colour)) {
        }

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
        template<typename... Args>
        explicit paddle_t(arena_t &arena, Args &&... args)
            : rectangle_t{std::forward<Args>(args)...}, arena_{arena} {
        }

        std::optional<std::tuple<scalar_t, std::function<void()> > > next_action(
            scalar_t dt,
            std::optional<std::tuple<scalar_t, std::function<void()> > > result);

        void advance_time(scalar_t dt);

    private:
        arena_t &arena_;
    };

    class arena_t : public rectangle_t {
    public:
        explicit arena_t(
            std::function<std::tuple<scalar_t, vec_t>()> next_puck_velocity)
            : rectangle_t{
                  box_t{vec_t{10, 10}, vec_t{630, 470}}, vec_t{0, 0},
                  colour_t{0, 0, 0, 0}
              },
              next_puck_velocity_{std::move(next_puck_velocity)}, puck_([this]() {
                  const auto [y, vel] = next_puck_velocity_();
                  return puck_t{
                      vec_t{320, y},
                      vel,
                      5,
                      colour_t{0, 255, 0, 255},
                  };
              }()),
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
              lhs_score_{}, rhs_score_{} {
        }

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
            const auto [y, vel] = next_puck_velocity_();
            puck().centre() = vec_t{320, y};
            puck().velocity() = vel;
        }

        std::optional<std::tuple<scalar_t, std::function<void()> > > next_action(
            scalar_t dt,
            std::optional<std::tuple<scalar_t, std::function<void()> > > result);

        void advance_time(scalar_t dt) {
            auto do_advance = [this](scalar_t t) {
                puck().advance_time(t);
                lhs_paddle().advance_time(t);
                rhs_paddle().advance_time(t);
            };

            while (dt > 0) {
                std::optional<std::tuple<scalar_t, std::function<void()> > > next;

                next = lhs_paddle().next_action(dt, std::move(next));
                next = rhs_paddle().next_action(dt, std::move(next));
                next = next_action(dt, std::move(next));

                if (next) {
                    auto &[when, action] = *next;
                    do_advance(when);
                    action();
                    dt -= when;
                } else {
                    do_advance(dt);
                    dt = 0;
                }
            }
        }

    private:
        std::function<std::tuple<scalar_t, vec_t>()> next_puck_velocity_;
        puck_t puck_;
        paddle_t lhs_paddle_;
        paddle_t rhs_paddle_;
        std::uint32_t lhs_score_;
        std::uint32_t rhs_score_;
    };

    class ai_t {
    public:
        explicit ai_t(std::mt19937::result_type seed, scalar_t stdev)
            : prng_(seed), error_dist_(0.f, stdev),
              last_estimate_{std::numeric_limits<scalar_t>::max()} {
        }

        ai_t(const ai_t &) = delete;

        ai_t &operator=(const ai_t &) = delete;

        std::optional<scalar_t> paddle_speed(arena_t &, paddle_t &);

    private:
        std::mt19937 prng_;
        std::normal_distribution<scalar_t> error_dist_;
        scalar_t last_estimate_;
    };

    std::optional<std::tuple<scalar_t, std::function<void()> > >
    inline paddle_t::next_action(
        pong::scalar_t dt,
        std::optional<std::tuple<scalar_t, std::function<void()> > > result) {
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
                                          ? (y0 - b.min()(1)) / ds // heading south
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
                                      ? (b.min()(0) - x0) / s // heading east
                                      : (b.max()(0) - x0) / s; // heading west
            const scalar_t y =
                    arena_.puck().centre()(1) + arena_.puck().velocity()(1) * when;

            const scalar_t min_y = b.min()(1) + when * velocity()(1);
            const scalar_t max_y = b.max()(1) + when * velocity()(1);

            // if when is in (0, dt) and y is within the bounds of the paddle and this
            // is the earliest found collision, then set the current result to this
            // collision
            if (when >= -0.f && when <= dt && y >= min_y && y <= max_y &&
                (!result || when < std::get<0>(*result))) {
                result.emplace(std::forward_as_tuple(
                    when, [this]() { arena_.puck().velocity()(0) *= -1; }));
            }
        }

        return result;
    }

    inline void paddle_t::advance_time(scalar_t dt) {
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

    inline std::optional<std::tuple<scalar_t, std::function<void()> > > arena_t::next_action(
        scalar_t dt,
        std::optional<std::tuple<scalar_t, std::function<void()> > > result) {
        const box_t b = bordered(box(), -puck().radius());

        // north / south
        {
            const scalar_t s = -puck().velocity()(1);

            if (s == s && s != 0.f) {
                const scalar_t y0 = puck().centre()(1);

                const scalar_t when = puck().velocity()(1) > -0.f
                                          ? (y0 - b.max()(1)) / s // heading south
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
            vec_t{
                a.lhs_paddle().box().max()(0) + a.puck().radius(),
                a.box().min()(1) + a.puck().radius()
            },
            vec_t{
                a.rhs_paddle().box().min()(0) - a.puck().radius(),
                a.box().max()(1) - a.puck().radius()
            },
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

    inline std::optional<scalar_t> ai_t::paddle_speed(arena_t &a, paddle_t &p) {
        const auto [when, target] = estimate_next_collision(a, p);

        if (std::abs(target - std::exchange(last_estimate_, target)) < 2.f)
            return {};

        return when == 0.f ? 0.f : (target - p.centre()(1) + error_dist_(prng_)) / when;
    }

    std::function<std::tuple<scalar_t, vec_t>()>
    make_starter(std::mt19937::result_type);
} // namespace pong

#endif // PONG_MODEL_HPP
