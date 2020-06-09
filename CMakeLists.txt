cmake_minimum_required(VERSION 3.1)

project(mc_franka LANGUAGES CXX VERSION 1.0.0)

find_package(mc_rtc REQUIRED)
find_package(Franka REQUIRED)

add_executable(MCFrankaControl src/MCFrankaControl.cpp)
target_link_libraries(MCFrankaControl PUBLIC mc_rtc::mc_control Franka::Franka)

install(TARGETS MCFrankaControl DESTINATION bin)
