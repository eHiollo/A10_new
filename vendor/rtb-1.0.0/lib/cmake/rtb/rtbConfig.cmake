# RTB Library Configuration File
    get_filename_component(RTB_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
    include("${RTB_CMAKE_DIR}/rtbTargets.cmake")

    set(rtb_LIBRARIES rtb::rtb)
    set(rtb_INCLUDE_DIRS "${RTB_CMAKE_DIR}/../../../include")