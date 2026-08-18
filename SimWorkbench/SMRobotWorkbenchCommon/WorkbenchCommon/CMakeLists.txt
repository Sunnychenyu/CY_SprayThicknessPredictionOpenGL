cmake_minimum_required(VERSION 3.20)

set(TARGET_NAME WorkbenchCommon)

add_library(${TARGET_NAME} INTERFACE)
add_library(SMRobotWorkbenchCommon::${TARGET_NAME} ALIAS ${TARGET_NAME})

target_link_libraries(${TARGET_NAME}
    INTERFACE
        $<BUILD_INTERFACE:RobotQtModulesShared>
        $<BUILD_INTERFACE:RobotQtModulesCoreWidgets>
        $<INSTALL_INTERFACE:SMRobotWorkbenchCommon::RobotQtModulesShared>
        $<INSTALL_INTERFACE:SMRobotWorkbenchCommon::RobotQtModulesCoreWidgets>
)

target_compile_features(${TARGET_NAME}
    INTERFACE
        cxx_std_17
)

set(${TARGET_NAME}_INSTALL_PUBLIC_DEPENDENCIES
    Qt5::Widgets
    SMRobotWorkbenchCommon::RobotQtModulesShared
    SMRobotWorkbenchCommon::RobotQtModulesCoreWidgets
)

set_target_properties(${TARGET_NAME}
    PROPERTIES
        FOLDER SMRobotWorkbenchCommon
)

set(${TARGET_NAME}_INSTALL_HEADER_STAGING_DIR ${CMAKE_CURRENT_BINARY_DIR}/install_headers)
file(MAKE_DIRECTORY ${${TARGET_NAME}_INSTALL_HEADER_STAGING_DIR})

function_InstallTarget(
    TARGET_NAME ${TARGET_NAME}
    PACKAGE_NAME ${PACKAGE_NAME}
    PACKAGE_INTERFACE_HEADERS ${${TARGET_NAME}_INSTALL_HEADER_STAGING_DIR}
)
