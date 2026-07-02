cmake_minimum_required(VERSION 3.22)

set(CMAKE_CXX_VERSION 17)
project(mc_franka LANGUAGES CXX VERSION 1.0.0)

option(USE_REALTIME "Enable real-time scheduling (requires setcap)" OFF)

if(USE_REALTIME)
    find_program(SETCAP setcap)
    if(NOT SETCAP)
        message(
            FATAL_ERROR
            "setcap must be available on the system for real-time support"
        )
    else()
        message("-- setcap command: ${SETCAP}")
    endif()
endif()

set(CMAKE_INSTALL_RPATH "$ENV{LD_LIBRARY_PATH}")

if(USE_REALTIME)
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
endif()

find_package(mc_panda REQUIRED) # brings Franka and mc_rtc

# Try to find system CLI11 first
find_package(CLI11 CONFIG QUIET)
if(NOT CLI11_FOUND)
    include(FetchContent)
    FetchContent_Declare(
        cli11_proj
        QUIET
        GIT_REPOSITORY https://github.com/CLIUtils/CLI11.git
        GIT_TAG v2.5.0
    )
    FetchContent_MakeAvailable(cli11_proj)
    # Alias target for compatibility if needed
    if(NOT TARGET CLI11::CLI11 AND TARGET cli11)
        add_library(CLI11::CLI11 ALIAS cli11)
    endif()
endif()

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
target_link_libraries(
    MCFrankaControl
    PUBLIC mc_rtc::mc_control Franka::Franka mc_panda::devices
    PRIVATE CLI11::CLI11
)
# CAP_SYS_NICE is required to set the SCHED_DEADLINE policy, we set it on the generated executable and the installed one
set(JOB_POOL_OPTION)
if(${CMAKE_VERSION} VERSION_GREATER "3.14")
    set(JOB_POOL_OPTION JOB_POOL console)
endif()
if(USE_REALTIME)
    add_custom_target(
        setcap_MCFrankaControl
        ALL
        COMMAND
            ${SUDO} -S ${SETCAP} cap_sys_nice+eip $<TARGET_FILE:MCFrankaControl>
        DEPENDS MCFrankaControl ${JOB_POOL_OPTION}
    )
    add_compile_definitions(USE_REALTIME)
endif()

install(TARGETS MCFrankaControl DESTINATION bin)
if(USE_REALTIME)
    install(
        CODE
            "execute_process(COMMAND ${SUDO} -S ${SETCAP} cap_sys_nice+eip ${CMAKE_INSTALL_PREFIX}/bin/MCFrankaControl)"
    )
endif()

macro(add_panda_util NAME)
    add_executable(${NAME} src/${NAME}.cpp)
    target_link_libraries(${NAME} PUBLIC Franka::Franka)
    install(TARGETS ${NAME} DESTINATION bin)
endmacro()

add_panda_util(PrintPandaState)
add_panda_util(StopPandaPump)
