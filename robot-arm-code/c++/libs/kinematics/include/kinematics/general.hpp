#pragma once
#include <concepts>
#include <array>
#include <cstddef>
#include "kinematics/arm_geometry.hpp"
#include "linalg/matrix.hpp"

namespace kinematics {

template<std::floating_point T, Twist tool_twist, Twist... twists>
linalg::Matrix<T,4,4> forward(std::type_identity_t<const std::array<T, sizeof...(twists)>&> phi, const RevoluteArm<T, twists...>& arm, const ToolLink<T,tool_twist>& tool)
{
    constexpr std::size_t n = sizeof...(twists);
    linalg::Matrix<T,4,4> res = linalg::identity<T,4>();
    res = tool.bridge_trafo()*res;
    for (std::size_t i=n; i-->0;){
        res = se3_z(phi[i],T(0),T(0),T(0))*res;
        res = arm.links()[i].bridge_trafo()*res;
    }
    res = arm.world_trafo()*res;
    return res;
}

}