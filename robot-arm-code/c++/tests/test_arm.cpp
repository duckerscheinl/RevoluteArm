#include <array>
#include <iostream>
#include "linalg/matrix.hpp"
#include "kinematics/arm_geometry.hpp"
#include "kinematics/arm_geometry_io.hpp"
using namespace kinematics;

constexpr linalg::Matrix<float,4,4> world_trafo{{{{1,0,0,0},{2,3,4,5},{6,7,8,9},{10,11,12,13}}}};
constexpr std::array<float, 3> offsets_ra{0,1,2};
constexpr std::array<float, 3> offsets_cn{0,3,0};
constexpr RevoluteArm<float, Twist::Plus90, Twist::Zero, Twist::Zero> arm{offsets_cn, offsets_ra, world_trafo};
static_assert(linalg::norm_el(arm.world_trafo()-world_trafo) < float(1e-16));

int main()
{
    std::cout << arm;
    return 0;
}