/* Copyright 2020 mc_rtc development team */

#pragma once

#include <stdint.h>

namespace mc_franka
{

// Called in non real-time context to initialize the application
void * init(int argc, char * argv[], uint64_t & cycle_ns);

// Main control-loop, when waiting for the next loop one should simply call sched_yield()
void run(void * data);

} // namespace mc_franka
