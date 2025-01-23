#ifndef PONG_GEOMETRY_HPP
#define PONG_GEOMETRY_HPP

#include <Eigen/Dense>
#include <cmath>
#include <concepts>

namespace pong {

using scalar_t = float;

using vec_t = Eigen::Vector<scalar_t, 2>;
using matrix_t = Eigen::Matrix<scalar_t, 2, 2>;
using plane_t = Eigen::Hyperplane<scalar_t, 2>;
using line_t = Eigen::ParametrizedLine<scalar_t, 2>;
using box_t = Eigen::AlignedBox<scalar_t, 2>;

namespace constant {
template <std::floating_point T> inline T pi() { return std::acos(T(-1)); }
} // namespace constant

namespace unit {

/**
 * unit vector in the x direction
 */
const vec_t i = {scalar_t{1}, scalar_t{0}};

/**
 * unit vector in the y direction
 */
const vec_t j = {scalar_t{0}, scalar_t{1}};

} // namespace unit

namespace transform {

/**
 * transformation matrix to rotate anti-clockwise by theta radians
 */
inline matrix_t rot(scalar_t theta) {
  return matrix_t{
      {std::cos(theta), -std::sin(theta)},
      {std::sin(theta), std::cos(theta)},
  };
}

} // namespace transform

/**
 * add (subtract) a border around a(n immutable) box
 */
box_t bordered(box_t result, const scalar_t border_size) {
  const vec_t v = {border_size, border_size};
  result.min() -= v;
  result.max() += v;
  return result;
};

} // namespace pong

static_assert(std::is_signed_v<pong::scalar_t>);

#endif // PONG_GEOMETRY_HPP
