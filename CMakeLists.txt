cmake_minimum_required(VERSION 3.1)

project(mc_franka LANGUAGES CXX VERSION 1.0.0)

find_package(Boost REQUIRED COMPONENTS program_options)
find_package(mc_panda REQUIRED) # brings Franka and mc_rtc

set(MCFrankaControl_SRC
  src/ControlMode.h
  src/defs.h
  src/main.cpp
  src/MCFrankaControl.cpp
  src/PandaControlLoop.h
  src/PandaControlType.h
  src/thread.h
)

add_executable(MCFrankaControl ${MCFrankaControl_SRC})
target_link_libraries(MCFrankaControl PUBLIC mc_rtc::mc_control Franka::Franka Boost::program_options mc_panda::devices)

install(TARGETS MCFrankaControl DESTINATION bin)
