#pragma once
#include "kinematics/arm_geometry.hpp"
#include <string_view>
#include <concepts>
#include <bitset>
#include <optional>
#include <cstddef>
#include <array>

namespace kinematics {

template <Twist... twists>
struct BranchFlags;                                   // no flags for unknown geometries

template <Twist... twists>
constexpr std::size_t n_branch_flags = BranchFlags<twists...>::count;

template <std::floating_point T, Twist... twists>
class RevoluteBranch {
public:
    static constexpr std::size_t n_flags = n_branch_flags<twists...>;
    static_assert(n_flags > 0, "no analytic branches defined for this geometry");
    static constexpr std::size_t n_branches = std::size_t(1) << n_flags;
    static constexpr std::size_t n_joints = sizeof...(twists);
    static_assert(n_joints > 0, "zero joints not legal");
private:
    std::bitset<n_flags> flags_;
    std::array<T, n_joints> angles_;
public:
    RevoluteBranch(const std::bitset<n_flags>& flags,
                             const std::array<T, n_joints>& angles)
        : flags_(flags), angles_(angles) {}
    const auto& flags() const { return flags_; }
    const auto& angles() const { return angles_; }
    std::size_t index() const { return flags_.to_ulong(); }
};

template <std::floating_point T, Twist... twists>
using RevoluteAnalyticSolutions =
    std::array<std::optional<RevoluteBranch<T, twists...>>, RevoluteBranch<T, twists...>::n_branches>;

}