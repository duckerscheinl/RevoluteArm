#pragma once
#include <array>
#include <cmath>
#include <numbers>
#include <concepts>
#include <optional>
#include "geometry/transforms.hpp"
#include "geometry/triangle.hpp"
#include "kinematics/analytic/analytic_solutions.hpp"

namespace kinematics {

namespace planar {

// Slot 0: elbow down, slot 1: elbow up.
// Both slots hold the same angles at full stretch or full fold.
template <std::floating_point T>
using Solutions = std::array<std::optional<std::array<T, 2>>, 2>;

template<std::floating_point T>
linalg::Vector<T, 3> forward(T R1, T phi1, T R2, T phi2)
{
    linalg::Vector<T,3> v{{{T(1),T(0),R2}}};
    v = se2(phi2,T(0),R1)*v;
    v = se2(phi1,T(0),T(0))*v;
    return v;
}

/*
Planar inverse!
*/
template<std::floating_point T>
Solutions<T> inverse(T x, T y, T R1, T R2)
{
    constexpr T pi  = std::numbers::pi_v<T>;
    T norm_t        = std::sqrt(x*x+y*y);
    Solutions<T> angles{};
    if (norm_t > R1+R2 || norm_t < std::abs(R1-R2))
        return angles;
    T s_cos_a       = triangle::two_times_dot(R1, norm_t, R2);
    T s_sin         = triangle::two_times_cross(R1, norm_t, R2);
    T alpha         = std::atan2(s_sin, s_cos_a);
    T theta         = std::atan2(y, x);
    T s_cos_b       = triangle::two_times_dot(R1, R2, norm_t);
    T beta          = std::atan2(s_sin, s_cos_b);
    std::size_t flip = x < 0 ? 1 : 0;
    for (std::size_t side = 0; side < 2; ++side) {
        T s    = side == 1 ? T(1) : T(-1);
        T phi1 = std::remainder(theta + s * alpha - pi / 2, 2 * pi);
        T phi2 = s * (beta - pi);
        if (!(std::abs(phi1) <= pi / 2)) continue;      // elbow below the table
        angles[side ^ flip] = std::array{phi1, phi2};   // the formulas for elbow up/down flip at x = 0.
    }

    return angles;
}


template<std::floating_point T>
Solutions<T> naive_inverse(T x, T y, T R1, T R2)
{
    constexpr T pi  = std::numbers::pi_v<T>;
    T norm_t        = std::sqrt(x*x+y*y);
    T cos_a         = triangle::cosine(R1, norm_t, R2);
    T alpha         = std::acos(cos_a);
    T theta         = std::atan2(y, x);
    T cos_b         = triangle::cosine(R1, R2, norm_t);
    T beta          = std::acos(cos_b);
    std::size_t flip = x < 0 ? 1 : 0;
    Solutions<T> angles{};
    for (std::size_t side = 0; side < 2; ++side) {
        T s    = side == 1 ? T(1) : T(-1);
        T phi1 = std::remainder(theta + s * alpha - pi / 2, 2 * pi);
        T phi2 = s * (beta - pi);
        if (std::abs(phi1) > pi / 2) continue;         // elbow below the table
        angles[side ^ flip] = std::array{phi1, phi2};   // the formulas for elbow up/down flip at x = 0.
    }

    return angles;
}
}
}