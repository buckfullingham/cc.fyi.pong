#include "geometry.hpp"
#include "model.hpp"

#include <functional>
#include <random>
#include <tuple>

std::function<std::tuple<pong::scalar_t, pong::vec_t>()>
pong::make_starter(std::mt19937::result_type seed){
    return
            [prng = std::mt19937{seed},
                theta_dist =
                std::uniform_real_distribution<float>{
                    constant::pi<float>() / 8.f, constant::pi<float>() * 3.f / 8.f
                },
                y_dist = std::uniform_real_distribution<scalar_t>{20, 460},
                sign_dist = std::uniform_int_distribution<int>{0, 1},
                speed_dist = std::uniform_real_distribution<float>{
                    150, 250
                }]() mutable -> std::tuple<scalar_t, vec_t> {
        const scalar_t theta = theta_dist(prng);
        const matrix_t signs{
            {scalar_t(sign_dist(prng) * 2 - 1), 0.f},
            {0.f, scalar_t(sign_dist(prng) * 2 - 1)},
        };
        const scalar_t y = y_dist(prng);
        return {y, transform::rot(theta) * signs * unit::i * speed_dist(prng)};
    };
}
