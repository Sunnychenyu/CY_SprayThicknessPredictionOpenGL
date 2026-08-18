cmake_minimum_required(VERSION 3.20)

set(TARGET_NAME ProjectAssemblyWorkbench)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

add_subdirectory(Editors)
add_subdirectory(SceneExplorer)
add_subdirectory(ToolSetup)

add_library(${TARGET_NAME} INTERFACE)
add_library(SMRobotWorkbenchProjectAssembly::${TARGET_NAME} ALIAS ${TARGET_NAME})

target_link_libraries(${TARGET_NAME}
    INTERFACE
        SMRobotWorkbenchCommon::WorkbenchCommon
        $<BUILD_INTERFACE:ProjectAssemblyEditors>
        $<BUILD_INTERFACE:ProjectAssemblySceneExplorer>
        $<BUILD_INTERFACE:ProjectAssemblyToolSetup>
        $<INSTALL_INTERFACE:SMRobotWorkbenchProjectAssembly::ProjectAssemblyEditors>
        $<INSTALL_INTERFACE:SMRobotWorkbenchProjectAssembly::ProjectAssemblySceneExplorer>
        $<INSTALL_INTERFACE:SMRobotWorkbenchProjectAssembly::ProjectAssemblyToolSetup>
)

target_compile_features(${TARGET_NAME}
    INTERFACE
        cxx_std_17
)

set(${TARGET_NAME}_INSTALL_PUBLIC_DEPENDENCIES
    SMRobotWorkbenchCommon::WorkbenchCommon
    SMRobotWorkbenchProjectAssembly::ProjectAssemblyEditors
    SMRobotWorkbenchProjectAssembly::ProjectAssemblySceneExplorer
    SMRobotWorkbenchProjectAssembly::ProjectAssemblyToolSetup
)

set_target_properties(${TARGET_NAME}
    PROPERTIES
        FOLDER SMRobotWorkbenchProjectAssembly
)

set(${TARGET_NAME}_INSTALL_HEADER_STAGING_DIR ${CMAKE_CURRENT_BINARY_DIR}/install_headers)
file(MAKE_DIRECTORY ${${TARGET_NAME}_INSTALL_HEADER_STAGING_DIR})

function_InstallTarget(
    TARGET_NAME ${TARGET_NAME}
    PACKAGE_NAME ${PACKAGE_NAME}
    PACKAGE_INTERFACE_HEADERS ${${TARGET_NAME}_INSTALL_HEADER_STAGING_DIR}
)
