#pragma once
#include <array>
#include <cmath>
#include <numbers>
#include <concepts>
#include <cstddef>
#include "geometry/transforms.hpp"
#include "geometry/triangle.hpp"
#include "kinematics/arm_geometry.hpp"
#include "linalg/matrix.hpp"
#include "kinematics/analytic/planar.hpp"
#include "kinematics/analytic/analytic_solutions.hpp"

// Rotation about the y-axis, is actually rotation about the negative y-axis.
// This choice is simply a strong preference of my mind.
// This specific 4R arm has no twists, since base rotation is treated seperately.

namespace kinematics{

template <>
struct BranchFlags<Twist::Zero, Twist::Plus90, Twist::Zero, Twist::Zero> {
    enum Flag : std::size_t {
        reach_front_i,
        elbow_up_i,
        count
    };
};

// 4R with a base and a planar arm attached to it.
namespace arm4R_bnp {

using Flags = BranchFlags<Twist::Zero, Twist::Plus90, Twist::Zero, Twist::Zero>;

template<std::floating_point T>
using Solutions = RevoluteAnalyticSolutions<T, Twist::Zero, Twist::Plus90, Twist::Zero, Twist::Zero>;

template<std::floating_point T>
using Arm = RevoluteArm<T, Twist::Zero, Twist::Plus90, Twist::Zero, Twist::Zero>;

template<std::floating_point T>
using Branch = RevoluteBranch<T, Twist::Zero, Twist::Plus90, Twist::Zero, Twist::Zero>;

}

/*
std::floating_point theta: angle of vertical approach 0 corresponds to approach from above. 
*/
template<std::floating_point T>
arm4R_bnp::Solutions<T> inverse(std::type_identity_t<std::array<T, 3>> pos, std::type_identity_t<T> theta, const arm4R_bnp::Arm<T>& arm, const ToolLink<T, Twist::Plus90>& tool)
{
    constexpr T pi = std::numbers::pi_v<T>;
    T total_offset = -tool.translation()[1];
    for (std::size_t i = 1; i < 4; i++)
        total_offset -= arm.links()[i].transl_ra(); // z-axis points in global -y direction.
    const T norm_xy = std::sqrt(pos[0]*pos[0]+pos[1]*pos[1]);
    arm4R_bnp::Solutions<T> solutions{};
    if (norm_xy < std::abs(total_offset))
        return solutions;

    const T phi_t     = std::atan2(pos[1],pos[0]);
    const T proj_x_t  = std::sqrt(pos[0]*pos[0]+pos[1]*pos[1]-total_offset*total_offset);
    const T dphi0     = std::atan2(total_offset, proj_x_t);

    const T sin_th = std::sin(theta); const T cos_th  = std::cos(theta);
    const T t_ra = tool.translation()[2];
    
    for (int reach_front = 0; reach_front < 2; reach_front++){
        const T t_cn = (reach_front ? 1 : -1) * tool.translation()[0];
        const T proj_x_p3  = proj_x_t - t_ra*sin_th - t_cn*cos_th - arm.links()[1].transl_cn();
        const T z_p3 = pos[2] + t_ra*cos_th - t_cn*sin_th - arm.links()[0].transl_ra();
        const planar::Solutions<T> phi1_2 = planar::inverse<T>(proj_x_p3, z_p3, 
        arm.links()[2].transl_cn(), arm.links()[3].transl_cn());
        const T phi0 = std::remainder(phi_t - reach_front * dphi0 + (1-reach_front) * (dphi0 + pi), 2*pi);
        for(int elbow_up = 0; elbow_up < 2; elbow_up++){
            if (!phi1_2[elbow_up]) continue;
            int sign    = (2*reach_front-1);
            const T phi1    = sign * (*phi1_2[elbow_up])[0];
            const T phi2    = sign * (*phi1_2[elbow_up])[1];
            const T phi3    = std::remainder(sign*pi + sign*theta - phi1 - phi2, 2*pi);
            std::bitset<arm4R_bnp::Flags::count> branch_i(0);
            if (reach_front == 1)
                branch_i.flip(arm4R_bnp::Flags::reach_front_i);
            if (elbow_up == 1)
                branch_i.flip(arm4R_bnp::Flags::elbow_up_i);
            const std::array<T,4> phi = {phi0,phi1+pi/2,phi2,phi3+pi/2};
            solutions[branch_i.to_ulong()].emplace(branch_i,phi);
        }
    }

    return solutions;
}

}