#pragma once
#include <cstddef>
#include <array>

template <std::size_t N>
constexpr unsigned to_uint(const std::array<bool, N>& bits) {
    unsigned value = 0;
    for (std::size_t i = 0; i < N; ++i)
        value |= unsigned(bits[i]) << i;   // bits[0] is the least significant bit
    return value;
}