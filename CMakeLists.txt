cmake_minimum_required(VERSION 3.12)

project(kaanhbin VERSION 2.5.7 DESCRIPTION "")

# 生成 compile_commands.json，供 clangd / C++ 扩展实现跳转、补全
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

configure_file(${PROJECT_SOURCE_DIR}/config/kaanhbin_version.h.in kaanhbin_version.h)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# 使用 /usr/*/lib/release（带 IgH EtherCAT）。未指定时默认 Release。
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
    a10_anchor_protocol.cpp
    a10_policy_tcp_plan.cpp
    a10_vr_plan.cpp
    a10_vr_vel_plan.cpp
    a10_gripper_bridge.cpp
    a10_gripper_plan.cpp
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
    ${PROJECT_SOURCE_DIR}/vendor/rtb-1.0.0
    /usr/local/rtb/rtb-1.0.0
    /usr/rtb/rtb-1.0.0
)
if(NOT rtb_FOUND)
    message(FATAL_ERROR "rtb not found. Place rtb-1.0.0 under vendor/rtb-1.0.0")
endif()
# rtb 安装包只有 Release 导入配置；Debug 工程也映射到 Release
if(TARGET rtb::rtb)
    set_property(TARGET rtb::rtb PROPERTY MAP_IMPORTED_CONFIG_DEBUG RELEASE)
endif()

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
set_target_properties(kaanhbin PROPERTIES
    BUILD_RPATH "${PROJECT_SOURCE_DIR}/vendor/rtb-1.0.0/lib;/opt/etherlab/lib;/usr/kaanhbot/kaanhbot-2.5.7/lib/release;/usr/kaanh/kaanh-2.5.7/lib/release;/usr/aris/aris-2.5.7/lib/release"
    INSTALL_RPATH "${PROJECT_SOURCE_DIR}/vendor/rtb-1.0.0/lib;/opt/etherlab/lib;/usr/kaanhbot/kaanhbot-2.5.7/lib/release;/usr/kaanh/kaanh-2.5.7/lib/release;/usr/aris/aris-2.5.7/lib/release"
)
set(KAANHBIN_RUNTIME_DIR "${CMAKE_CURRENT_BINARY_DIR}/bin")
add_custom_target(sync_kaanh_xml ALL
    COMMAND ${CMAKE_COMMAND} -E make_directory
        ${KAANHBIN_RUNTIME_DIR}
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${PROJECT_SOURCE_DIR}/kaanh.xml
        ${KAANHBIN_RUNTIME_DIR}/kaanh.xml
    DEPENDS ${PROJECT_SOURCE_DIR}/kaanh.xml
    COMMENT "Syncing kaanh.xml to the runtime directory"
    VERBATIM
)
add_dependencies(kaanhbin sync_kaanh_xml)

add_executable(client ${PROJECT_SOURCE_DIR}/client.cpp)
target_include_directories(client PUBLIC ${PROJECT_BINARY_DIR})
target_link_libraries(client PUBLIC ${kaanhbot_LIBRARIES})
