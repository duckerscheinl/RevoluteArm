#pragma once
#include "linalg/matrix.hpp"
#include <cmath>
#include <concepts>

template<std::floating_point T>
linalg::Matrix<T,3,3> se2(T phi, T dx, T dy)
{
    const T c = std::cos(phi), s = std::sin(phi);
    return linalg::Matrix<T,3,3>{{{
        {1,     0,              0},
        {dx,    c,  -s},
        {dy,    s,  c}
    }}};
}

template <std::floating_point T>
linalg::Matrix<T, 4, 4> se3_x(T phi, T dx, T dy, T dz)
{
    const T c = std::cos(phi), s = std::sin(phi);
    return linalg::Matrix<T, 4, 4>{{{
        {1, 0, 0,  0},
        {dx, 1, 0,  0},
        {dy, 0, c, -s},
        {dz, 0, s,  c}
    }}};
}

template <std::floating_point T>
linalg::Matrix<T, 4, 4> se3_y(T phi, T dx, T dy, T dz)
{
    const T c = std::cos(phi), s = std::sin(phi);
    return linalg::Matrix<T, 4, 4>{{{
        {1,  0, 0, 0},
        {dx,  c, 0, -s},
        {dy,  0, 1, 0},
        {dz, s, 0, c}
    }}};
}

template <std::floating_point T>
linalg::Matrix<T, 4, 4> se3_z(T phi, T dx, T dy, T dz)
{
    const T c = std::cos(phi), s = std::sin(phi);
    return linalg::Matrix<T, 4, 4>{{{
        {1, 0,  0, 0},
        {dx, c, -s, 0},
        {dy, s,  c, 0},
        {dz, 0,  0, 1}
    }}};
}

template <std::floating_point T>
constexpr linalg::Matrix<T, 4, 4> se3_x(T c, T s, T dx, T dy, T dz)
{
    return linalg::Matrix<T, 4, 4>{{{
        {1, 0, 0,  0},
        {dx, 1, 0,  0},
        {dy, 0, c, -s},
        {dz, 0, s,  c}
    }}};
}

template <std::floating_point T>
constexpr linalg::Matrix<T, 4, 4> se3_z(T s, T c, T dx, T dy, T dz)
{
    return linalg::Matrix<T, 4, 4>{{{
        {1, 0,  0, 0},
        {dx, c, -s, 0},
        {dy, s,  c, 0},
        {dz, 0,  0, 1}
    }}};
}