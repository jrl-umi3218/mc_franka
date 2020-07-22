cmake_minimum_required(VERSION 3.1)

project(mc_franka LANGUAGES CXX VERSION 1.0.0)

find_package(mc_rtc REQUIRED)
find_package(Franka REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)
find_package(mc_panda REQUIRED)

add_executable(MCFrankaControl src/MCFrankaControl.cpp)
target_link_libraries(MCFrankaControl PUBLIC mc_rtc::mc_control Franka::Franka Boost::program_options mc_panda::devices)

install(TARGETS MCFrankaControl DESTINATION bin)
