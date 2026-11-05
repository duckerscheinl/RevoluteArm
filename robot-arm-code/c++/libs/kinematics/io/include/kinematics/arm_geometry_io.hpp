#pragma once
#include <concepts>
#include <iostream>
#include "kinematics/arm_geometry.hpp"

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
std::ostream& operator<<(std::ostream& os, const Link<T>& joint){
    os << "link length: " << joint.link_length() << " link offset: " << joint.link_offset() <<  " twist angle: " << joint.twist_angle() << "\n";
    return os;
}

template<std::floating_point T, Twist... twists>
std::ostream& operator<<(std::ostream& os, const RevoluteArm<T, twists...>& arm){
    os << "base height: " << arm.base_height() << "\n";
    std::size_t ind = 0;
    for (const auto& joint : arm.joints()){
        os << "Link " << ind << ": " << joint;
        ind++;
    }
    return os;
}

}