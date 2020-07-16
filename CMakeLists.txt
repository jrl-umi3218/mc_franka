cmake_minimum_required(VERSION 3.1)

project(mc_franka LANGUAGES CXX VERSION 1.0.0)

find_package(mc_rtc 1.4.0 REQUIRED)
find_package(Franka REQUIRED)
get_target_property(Franka_INCLUDE_DIRS ${Franka_LIBRARIES} INTERFACE_INCLUDE_DIRECTORIES)
include_directories(${Franka_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/../libfranka/src
    ${CMAKE_SOURCE_DIR}/../libfranka/common/include
)
message(STATUS "Franka_LIBRARIES: ${Franka_LIBRARIES}")
message(STATUS "Franka_DIR: ${Franka_DIR}")
message(STATUS "Franka_INCLUDE_DIRS: ${Franka_INCLUDE_DIRS}")
message(STATUS "CMAKE_SOURCE_DIR: ${CMAKE_SOURCE_DIR}")
link_directories(${Franka_LIBRARY_DIRS})
find_package(mc_panda REQUIRED)

add_executable(PrintPandaState src/PrintPandaState.cpp)
add_executable(TestCobotPump src/TestCobotPump.cpp)
add_executable(GravComp src/GravComp.cpp)
add_executable(MCFrankaControl src/MCFrankaControl.cpp)
add_executable(MCFrankaTorqueControl src/MCFrankaTorqueControl.cpp)
add_executable(MCFrankaControlMulti src/MCFrankaControlMulti.cpp src/Controller.hpp src/Controller.cpp)
add_executable(MCFrankaControlTest src/MCFrankaControlMultiTest.cpp)
target_link_libraries(PrintPandaState PUBLIC mc_rtc::mc_control Franka::Franka)
target_link_libraries(TestCobotPump PUBLIC mc_rtc::mc_control Franka::Franka)
target_link_libraries(GravComp PUBLIC mc_rtc::mc_control Franka::Franka)
target_link_libraries(MCFrankaControl PUBLIC mc_rtc::mc_control Franka::Franka mc_panda::devices)
target_link_libraries(MCFrankaTorqueControl PUBLIC mc_rtc::mc_control Franka::Franka)
target_link_libraries(MCFrankaControlMulti PUBLIC mc_rtc::mc_control ${Franka_LIBRARIES} PocoNet) #PocoFoundation 
target_link_libraries(MCFrankaControlTest PUBLIC mc_rtc::mc_control ${Franka_LIBRARIES} PocoNet) #PocoFoundation 
install(TARGETS PrintPandaState DESTINATION bin)
install(TARGETS TestCobotPump DESTINATION bin)
install(TARGETS GravComp DESTINATION bin)
install(TARGETS MCFrankaControl DESTINATION bin)
install(TARGETS MCFrankaTorqueControl DESTINATION bin)
install(TARGETS MCFrankaControlMulti DESTINATION bin)
install(TARGETS MCFrankaControlTest DESTINATION bin)
