#pragma once
#include <cstddef>
#include <cstdint>
#include <algorithm>
#include <array>
#include <cassert>
#include <utility>
#include <cmath>
#include <concepts>
#include <type_traits>
#include "linalg/matrix.hpp"
#include "geometry/transforms.hpp"

namespace kinematics {

template<std::size_t K>
inline constexpr std::integral_constant<std::size_t, K> at{};

enum class Twist : std::uint8_t {Zero, Plus90, Minus90};

template<std::floating_point T>
constexpr std::pair<T, T> sin_cos(Twist twist) {
    switch (twist) {
        case Twist::Zero:    return { T(0),  T(1) };
        case Twist::Plus90:  return { T(1),  T(0) };
        case Twist::Minus90: return { T(-1), T(0) };
    }
    std::unreachable();
}

template<std::floating_point T>
class Link
{
    T transl_cn_{};                          // translation along common normal(cn).
    T transl_ra_{};                          // translation along rotation axis(ra).
    Twist twist_angle_{Twist::Zero};
    public:
    constexpr Link(T transl_cn, T transl_ra, Twist angle): transl_cn_(transl_cn), transl_ra_(transl_ra), twist_angle_(angle)
    {
        assert(transl_cn_ >= T(0) && "translation along common normal must be positive!");
    }
    constexpr T transl_cn() const {return transl_cn_;}
    constexpr T transl_ra() const {return transl_ra_;}
    constexpr Twist twist_angle() const {return twist_angle_;}
    constexpr linalg::Matrix<T,4,4> bridge_trafo() const;
};


// T_tool = Rot_x(twist) * Trans(q), the same order as Link:
// q is the tool point in the tool's own (twisted) frame:
// q_z along the approach direction, q_x along the common normal, q_y the third direction.
template<std::floating_point T, Twist tw>
class ToolLink
{
    std::array<T, 3> q_{};
public:
    static constexpr Twist twist = tw;

    constexpr ToolLink(T axial, T radial) requires (tw == Twist::Zero)
        : q_{radial, T(0), axial} {}

    constexpr ToolLink(T qx, T qy, T qz) requires (tw != Twist::Zero)
        : q_{qx, qy, qz} {}

    constexpr const std::array<T, 3>& translation() const { return q_; }

    constexpr linalg::Matrix<T, 4, 4> bridge_trafo() const
    {
        const auto [s, c] = sin_cos<T>(tw);
        // translation column R_x(twist) * q
        return se3_x(c, s, q_[0], c * q_[1] - s * q_[2], s * q_[1] + c * q_[2]);
    }
};


// Order: Rot(twist_i)*Trans(x_i)*Trans(z_i)*Rot(theta_i).
template<std::floating_point T>
constexpr linalg::Matrix<T,4,4> Link<T>::bridge_trafo() const {
    const auto [s,c] = sin_cos<T>(twist_angle_);
    T dy = -transl_ra_*s;
    T dz = transl_ra_*c;
    return se3_x(c, s, transl_cn_, dy, dz);
}

// The twist angles describe how the frames are rotated about the x-axis going outwards.
// Joint i-1 -> i. Where joint 0 is the base.
// The first twist should be given with regards to a rotation around a global vertical axis.
// Afterwards twist is always in regards to prev rotation axis.
// Order: Rot(twist_i)*Trans(x_i)*Trans(z_i)*Rot(theta_i).
// If the world frame has as a z axis the base joints rotation axis and phi0 is measured in the
// world frame then the world frame is the identity.
template<std::floating_point T, Twist... twists>
class RevoluteArm
{
    public:
    static constexpr std::size_t n_joints = sizeof...(twists);
    static constexpr std::array<Twist, n_joints> twist_angles{twists...};
    static_assert(n_joints > 0, "A RevoluteArm needs at least one joint");

    private:
    std::array<Link<T>, n_joints> links_;
    linalg::Matrix<T, 4, 4> world_trafo_;
    constexpr RevoluteArm(  const std::array<Link<T>, n_joints>& links,
                            const linalg::Matrix<T,4,4>& world_trafo = linalg::identity<T,4>())
        : links_(links), world_trafo_(world_trafo){}

    public:
    constexpr RevoluteArm(  std::array<T, n_joints> transl_cn,
                            std::array<T, n_joints> transl_ra,
                            const linalg::Matrix<T,4,4>& world_trafo = linalg::identity<T,4>())
        : RevoluteArm(create_links(transl_cn, transl_ra), world_trafo) {}
    constexpr const std::array<Link<T>, n_joints>& links()  const {return links_;}
    constexpr const linalg::Matrix<T, 4, 4>& world_trafo()  const {return world_trafo_;}
    template<std::size_t K>
    constexpr auto prefix(std::integral_constant<std::size_t,K>) const requires (K > 0 && K < n_joints);
    template<std::size_t K>
    constexpr auto prefix_tool(std::integral_constant<std::size_t,K>) const requires (K > 0 && K < n_joints);
    template<std::size_t K>
    constexpr auto suffix(std::integral_constant<std::size_t,K>, const linalg::Matrix<T, 4, 4>&) const requires (K > 0 && K < n_joints);

    private:
    static constexpr std::array<Link<T>, n_joints> create_links(std::array<T, n_joints> transl_cn,
                                                                std::array<T, n_joints> transl_ra)
    {
        return [&]<std::size_t... Is>(std::index_sequence<Is...>){
            return std::array<Link<T>,n_joints>{{Link<T>(transl_cn[Is],transl_ra[Is],twist_angles[Is])...}};
        }(std::make_index_sequence<n_joints>{});
    }
};

// The first K joints as an arm of their own.
template<std::floating_point T, Twist... twists>
template<std::size_t K>
constexpr auto RevoluteArm<T,twists...>::prefix(std::integral_constant<std::size_t,K>) const requires (K > 0 && K < n_joints)
{
    return [&]<std::size_t... Is>(std::index_sequence<Is...>) {
        return RevoluteArm<T, twist_angles[Is]...>({links_[Is].transl_cn()...},
                                                {links_[Is].transl_ra()...}, world_trafo_);
    }(std::make_index_sequence<K>{});
}

// Link K as a tool for prefix(at<K>): its tool frame is joint K's frame at angle zero.
template<std::floating_point T, Twist... twists>
template<std::size_t K>
constexpr auto RevoluteArm<T,twists...>::prefix_tool(std::integral_constant<std::size_t,K>) const requires (K > 0 && K < n_joints)
{
    constexpr Twist tw = twist_angles[K];
    const auto& link = links_[K];
    if constexpr (tw == Twist::Zero)
        return ToolLink<T, tw>(link.transl_ra(), link.transl_cn());      // (axial, radial)
    else
        return ToolLink<T, tw>(link.transl_cn(), T(0), link.transl_ra());
}

// Joints K..n-1 as an arm of their own, mounted at base.
template<std::floating_point T, Twist... twists>
template<std::size_t K>
constexpr auto RevoluteArm<T,twists...>::suffix(std::integral_constant<std::size_t,K>, const linalg::Matrix<T, 4, 4>& base) const requires (K > 0 && K < n_joints)
{
    return [&]<std::size_t... Is>(std::index_sequence<Is...>) {
        return RevoluteArm<T, twist_angles[K + Is]...>({links_[K + Is].transl_cn()...},
                                                    {links_[K + Is].transl_ra()...}, base);
    }(std::make_index_sequence<n_joints - K>{});
}

}