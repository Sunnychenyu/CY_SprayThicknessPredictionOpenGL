cmake_minimum_required(VERSION 3.20)

set(TARGET_NAME RobotQtModulesStatus)

set(SOURCES
    StatusPanelWidget.cpp
    StatusPanelWidget.h
)

add_library(${TARGET_NAME} STATIC ${SOURCES})
add_library(RobotQtViewerStatus ALIAS ${TARGET_NAME})

target_include_directories(${TARGET_NAME}
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(${TARGET_NAME}
    PRIVATE
        SMRobotWorkbenchCommon::RobotQtModulesCoreWidgets
)

target_compile_features(${TARGET_NAME}
    PUBLIC
        cxx_std_17
)

set_target_properties(${TARGET_NAME}
    PROPERTIES
        FOLDER SMRobotApps/RobotQtModules
        DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX}
        RELEASE_POSTFIX ${CMAKE_RELEASE_POSTFIX}
)
