cmake_minimum_required(VERSION 3.12)

project(kaanhbin VERSION 2.5.7 DESCRIPTION "")

configure_file(${PROJECT_SOURCE_DIR}/config/kaanhbin_version.h.in kaanhbin_version.h)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY bin)

if(UNIX)
    if(MYIR)
        find_package(kaanhbot REQUIRED PATHS /opt/poky/3.1.20/sysroots/aarch64-poky-linux/usr/kaanhbot)
        message(STATUS "MYIR : ${MYIR}")
    elseif(MYIR_J1028)
        find_package(kaanhbot REQUIRED PATHS /opt/nxp-real-time-edge/2.0/sysroots/aarch64-fsl-linux/usr/kaanhbot)
        message(STATUS "MYIR_J1028 : ${MYIR_J1028}")
    else()
        find_package(kaanhbot REQUIRED PATHS /usr/kaanhbot)
    endif()
elseif(WIN32)
    add_definitions(-DWIN32)
    add_definitions(-DNOMINMAX)
    add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/source-charset:utf-8>")
    add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/execution-charset:utf-8>")
    find_package(kaanhbot REQUIRED PATHS C:/kaanhbot)
endif()
message("kaanhbot dir: ${kaanhbot_DIR}")

set(KAANHBIN_SOURCES
    main.cpp
    plan.cpp
    robot.cpp
    a10_tcp_server.cpp
    a10_policy_tcp_plan.cpp
    a10_teleop_tcp_plan.cpp
    cmd_ft_unit_test.cpp
    gripper.cpp
    gravcomp.cpp
    curve.cpp
    searchDir.cpp
    movel.cpp
    assemcomd.cpp
    datacomd.cpp
    rivetcomd.cpp
    master_slave.cpp
    test.cpp
    tactile_collect_real.cpp
)

find_package(rtb QUIET PATHS 
    /usr/local/rtb/rtb-1.0.0
    /usr/rtb/rtb-1.0.0
)

include_directories(${PROJECT_SOURCE_DIR})
add_executable(kaanhbin ${KAANHBIN_SOURCES})
target_include_directories(kaanhbin PUBLIC 
    ${PROJECT_BINARY_DIR}
    ${rtb_INCLUDE_DIRS}
)
target_link_libraries(kaanhbin PUBLIC 
    ${kaanhbot_LIBRARIES}
    ${rtb_LIBRARIES}
)

add_executable(client ${PROJECT_SOURCE_DIR}/client.cpp)
target_include_directories(client PUBLIC ${PROJECT_BINARY_DIR})
target_link_libraries(client PUBLIC ${kaanhbot_LIBRARIES})
