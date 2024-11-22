#ifndef PONG_GEOMETRY_HPP
#define PONG_GEOMETRY_HPP

#include <Eigen/Dense>

namespace pong {

using plane_t = Eigen::Hyperplane<float, 2>;
using line_t = Eigen::ParametrizedLine<float, 2>;
using vec_t = Eigen::Vector2f;
using box_t = Eigen::AlignedBox<float, 2>;
using matrix_t = Eigen::Matrix2f;

namespace unit {
const vec_t i = {1.f, 0.f};
const vec_t j = {0.f, 1.f};
} // namespace unit

const auto flip_x = matrix_t{{-1.f, 0.f}, {0.f, 1.f}};
const auto flip_y = matrix_t{{1.f, 0.f}, {0.f, -1.f}};

} // namespace pong

#endif // PONG_GEOMETRY_HPP
