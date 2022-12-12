cmake_minimum_required(VERSION 3.1)

project(mc_franka LANGUAGES CXX VERSION 1.0.0)

set(CMAKE_INSTALL_RPATH "$ENV{LD_LIBRARY_PATH}")

find_program(SUDO sudo)
if(NOT SUDO)
  message(FATAL_ERROR "sudo must be available on the system")
else()
  message("-- sudo command: ${SUDO}")
endif()

find_program(SETCAP setcap)
if(NOT SETCAP)
  message(FATAL_ERROR "setcap must be available on the system")
else()
  message("-- setcap command: ${SETCAP}")
endif()

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
# CAP_SYS_NICE is required to set the SCHED_DEADLINE policy, we set it on the generated executable and the installed one
set(JOB_POOL_OPTION)
if(${CMAKE_VERSION} VERSION_GREATER "3.14")
  set(JOB_POOL_OPTION JOB_POOL console)
endif()
add_custom_target(setcap_MCFrankaControl ALL COMMAND ${SUDO} -S ${SETCAP} cap_sys_nice+eip $<TARGET_FILE:MCFrankaControl> DEPENDS MCFrankaControl ${JOB_POOL_OPTION})

install(TARGETS MCFrankaControl DESTINATION bin)
install(CODE "execute_process(COMMAND ${SUDO} -S ${SETCAP} cap_sys_nice+eip ${CMAKE_INSTALL_PREFIX}/bin/MCFrankaControl)")

macro(add_panda_util NAME)
  add_executable(${NAME} src/${NAME}.cpp)
  target_link_libraries(${NAME} PUBLIC Franka::Franka)
  install(TARGETS ${NAME} DESTINATION bin)
endmacro()

add_panda_util(PrintPandaState)
add_panda_util(StopPandaPump)
