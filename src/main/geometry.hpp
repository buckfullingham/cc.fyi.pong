#ifndef PONG_GEOMETRY_HPP
#define PONG_GEOMETRY_HPP

#include <Eigen/Dense>
#include <concepts>

namespace pong {

using scalar_t = float;

using vec_t = Eigen::Vector<scalar_t, 2>;
using matrix_t = Eigen::Matrix<scalar_t, 2, 2>;
using plane_t = Eigen::Hyperplane<scalar_t, 2>;
using line_t = Eigen::ParametrizedLine<scalar_t, 2>;
using box_t = Eigen::AlignedBox<scalar_t, 2>;

namespace unit {
const vec_t i = {scalar_t{1}, scalar_t{0}};
const vec_t j = {scalar_t{0}, scalar_t{1}};

} // namespace unit

namespace transform {
const auto flip_x = matrix_t{{scalar_t{-1}, scalar_t{0}}, {scalar_t{0}, scalar_t{1}}};
const auto flip_y = matrix_t{{scalar_t{1}, scalar_t{0}}, {scalar_t{0}, scalar_t{-1}}};
}

} // namespace pong

static_assert(std::is_signed_v<pong::scalar_t>);

#endif // PONG_GEOMETRY_HPP
