cmake_minimum_required(VERSION 3.1)

project(mc_franka LANGUAGES CXX VERSION 1.0.0)

find_package(mc_rtc REQUIRED)
find_package(Franka REQUIRED)
get_target_property(Franka_INCLUDE_DIRS ${Franka_LIBRARIES} INTERFACE_INCLUDE_DIRECTORIES)
include_directories(${Franka_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/../libfranka/src
    ${CMAKE_SOURCE_DIR}/../libfranka/common/include
)
#message(STATUS "Franka_LIBRARIES: ${Franka_LIBRARIES}")
#message(STATUS "Franka_DIR: ${Franka_DIR}")
#message(STATUS "Franka_INCLUDE_DIRS: ${Franka_INCLUDE_DIRS}")
#message(STATUS "CMAKE_SOURCE_DIR: ${CMAKE_SOURCE_DIR}")
link_directories(${Franka_LIBRARY_DIRS})

add_executable(MCFrankaControl src/MCFrankaControl.cpp)
add_executable(MCFrankaControlMulti src/MCFrankaControlMulti.cpp src/Controller.hpp src/Controller.cpp)
target_link_libraries(MCFrankaControl PUBLIC mc_rtc::mc_control Franka::Franka)
target_link_libraries(MCFrankaControlMulti PUBLIC mc_rtc::mc_control ${Franka_LIBRARIES} PocoNet) #PocoFoundation 
install(TARGETS MCFrankaControl DESTINATION bin)
install(TARGETS MCFrankaControlMulti DESTINATION bin)
