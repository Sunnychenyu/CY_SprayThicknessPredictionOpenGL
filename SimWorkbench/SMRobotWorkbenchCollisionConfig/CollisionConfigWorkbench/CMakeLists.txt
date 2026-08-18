cmake_minimum_required(VERSION 3.20)

set(TARGET_NAME CollisionConfigWorkbench)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

add_subdirectory(CollisionLinkModelSetup)
add_subdirectory(CollisionRuntimeResults)
add_subdirectory(CollisionDetectorConfig)

add_library(${TARGET_NAME} INTERFACE)
add_library(SMRobotWorkbenchCollisionConfig::${TARGET_NAME} ALIAS ${TARGET_NAME})

target_link_libraries(${TARGET_NAME}
    INTERFACE
        SMRobotWorkbenchCommon::WorkbenchCommon
        $<BUILD_INTERFACE:CollisionDetectorConfig>
        $<BUILD_INTERFACE:CollisionLinkModelSetup>
        $<BUILD_INTERFACE:CollisionRuntimeResults>
        $<INSTALL_INTERFACE:SMRobotWorkbenchCollisionConfig::CollisionDetectorConfig>
        $<INSTALL_INTERFACE:SMRobotWorkbenchCollisionConfig::CollisionLinkModelSetup>
        $<INSTALL_INTERFACE:SMRobotWorkbenchCollisionConfig::CollisionRuntimeResults>
)

target_compile_features(${TARGET_NAME}
    INTERFACE
        cxx_std_17
)

set(${TARGET_NAME}_INSTALL_PUBLIC_DEPENDENCIES
    SMRobotWorkbenchCommon::WorkbenchCommon
    SMRobotWorkbenchCollisionConfig::CollisionDetectorConfig
    SMRobotWorkbenchCollisionConfig::CollisionLinkModelSetup
    SMRobotWorkbenchCollisionConfig::CollisionRuntimeResults
)

set_target_properties(${TARGET_NAME}
    PROPERTIES
        FOLDER SMRobotWorkbenchCollisionConfig
)

set(${TARGET_NAME}_INSTALL_HEADER_STAGING_DIR ${CMAKE_CURRENT_BINARY_DIR}/install_headers)
file(MAKE_DIRECTORY ${${TARGET_NAME}_INSTALL_HEADER_STAGING_DIR})

function_InstallTarget(
    TARGET_NAME ${TARGET_NAME}
    PACKAGE_NAME ${PACKAGE_NAME}
    PACKAGE_INTERFACE_HEADERS ${${TARGET_NAME}_INSTALL_HEADER_STAGING_DIR}
)
