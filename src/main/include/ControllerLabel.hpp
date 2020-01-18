// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <string>
#include <tuple>

#include <wpi/StringRef.h>

namespace frc3512 {

struct ControllerLabel {

};

// Convert array into a tuple
template <typename Array, std::size_t... I>
auto a2t_impl(const Array& a, std::index_sequence<I...>) {
    return std::make_tuple(a[I]...);
}

template <typename T, size_t N, typename Indices = std::make_index_sequence<N>>
auto a2t(const std::array<T, N>& a) {
    return a2t_impl(a, Indices{});
}

}  // namespace frc3512
