#include <array>
#include <iostream>
#include "kinematics/arm_geometry.hpp"
#include "kinematics/arm_geometry_io.hpp"
using namespace kinematics;

constexpr float base_height = 0.1;
constexpr std::array<float, 2> link_lengths{3,1};
constexpr std::array<float, 2> link_offsets{0,0};
constexpr RevoluteArm<float, Twist::Zero, Twist::Zero> arm{base_height, link_lengths, link_offsets};
static_assert(arm.base_height() == float(0.1));

int main()
{
    std::cout << arm;
    return 0;
}