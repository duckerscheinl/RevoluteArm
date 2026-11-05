#pragma once
#include <cmath>

namespace triangle {

template<std::floating_point T>
T cosine(T adj1, T adj2, T opp)
{
    return (adj1*adj1+adj2*adj2-opp*opp)/(2*adj1*adj2);
}

template<std::floating_point T>
T sine(T adj1, T adj2, T opp)
{
    return std::sqrt((adj1+adj2-opp)*(adj1+adj2+opp)*(adj1-adj2+opp)*(opp-adj1+adj2))/(2*adj1*adj2);
}

// dot product of the two adjacent vectors.
// symmetric in first two entries.
template<std::floating_point T>
T two_times_dot(T adj1, T adj2, T opp)
{
    return (adj1*adj1+adj2*adj2-opp*opp);
}

// magnitude of cross product of the two adjacent vectors.
// fully symmetric.
template<std::floating_point T>
T two_times_cross(T adj1, T adj2, T opp)
{
    T prod = (adj1+adj2-opp)*(adj1+adj2+opp)*(adj1-adj2+opp)*(opp-adj1+adj2);
    return std::sqrt(std::max(T(0),prod));
}

}