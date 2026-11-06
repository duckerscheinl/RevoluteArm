#pragma once
#include <concepts>
#include <iostream>
#include "kinematics/arm_geometry.hpp"
#include "linalg/matrix_io.hpp"

namespace kinematics {

inline std::ostream& operator<<(std::ostream& os, Twist t) {
    switch (t) {
        case Twist::Zero:    return os << "Zero";
        case Twist::Plus90:  return os << "Plus90";
        case Twist::Minus90: return os << "Minus90";
    }
    std::unreachable();
}

template<std::floating_point T>
std::ostream& operator<<(std::ostream& os, const Link<T>& link){
    os << "offset along rotation axis: " << link.transl_ra() << "offset along common normal: " << link.transl_cn() <<  " twist angle: " << link.twist_angle() << "\n";
    return os;
}

template<std::floating_point T, Twist... twists>
std::ostream& operator<<(std::ostream& os, const RevoluteArm<T, twists...>& arm){
    os << "world transform: " << arm.world_trafo() << "\n";
    std::size_t ind = 0;
    for (const auto& joint : arm.links()){
        os << "Link " << ind << ": " << joint;
        ind++;
    }
    return os;
}

}