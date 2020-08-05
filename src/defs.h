/* Copyright 2020 mc_rtc development team */

#pragma once

#include <chrono>

/** This header defines some useful time types */

namespace mc_franka
{

using duration_ms = std::chrono::duration<double, std::milli>;
using duration_us = std::chrono::duration<double, std::micro>;
/** Always pick a steady clock */
using clock = typename std::conditional<std::chrono::high_resolution_clock::is_steady,
                                        std::chrono::high_resolution_clock,
                                        std::chrono::steady_clock>::type;

} // namespace mc_franka
