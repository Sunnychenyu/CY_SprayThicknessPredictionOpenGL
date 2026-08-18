cmake_minimum_required(VERSION 3.20)

set(TARGET_NAME ProjectAssemblyEditors)

set(SOURCES
    ToolAssetEditorWidget.cpp
    ToolAssetEditorWidget.h
    ToolTransformEditorWidget.cpp
    ToolTransformEditorWidget.h
)

add_library(${TARGET_NAME} STATIC ${SOURCES})
add_library(SMRobotWorkbenchProjectAssembly::${TARGET_NAME} ALIAS ${TARGET_NAME})

target_include_directories(${TARGET_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:${PACKAGE_NAME}/include/${TARGET_NAME}>
)

target_link_libraries(${TARGET_NAME}
    PUBLIC
        Qt5::Widgets
        Eigen3::Eigen
        SMRobotPlatform::SimulationProject
    PRIVATE
        $<BUILD_INTERFACE:RobotQtModulesCoreWidgets>
        $<INSTALL_INTERFACE:SMRobotWorkbenchCommon::RobotQtModulesCoreWidgets>
)

target_compile_features(${TARGET_NAME}
    PUBLIC
        cxx_std_17
)

set_target_properties(${TARGET_NAME}
    PROPERTIES
        FOLDER SMRobotWorkbenchProjectAssembly/ProjectAssemblyWorkbench
        DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX}
        RELEASE_POSTFIX ${CMAKE_RELEASE_POSTFIX}
)

if(ExportPacakge${PACKAGE_NAME})
    set(${TARGET_NAME}_INSTALL_PUBLIC_DEPENDENCIES
        Qt5::Widgets
        Eigen3::Eigen
        SMRobotPlatform::SimulationProject
        SMRobotWorkbenchCommon::RobotQtModulesCoreWidgets
    )

    set(${TARGET_NAME}_INSTALL_HEADER_STAGING_DIR
        ${CMAKE_CURRENT_BINARY_DIR}/install_headers)
    file(REMOVE_RECURSE ${${TARGET_NAME}_INSTALL_HEADER_STAGING_DIR})
    file(MAKE_DIRECTORY ${${TARGET_NAME}_INSTALL_HEADER_STAGING_DIR}/${TARGET_NAME})

    foreach(header_ IN LISTS SOURCES)
        if(header_ MATCHES "\\.h$")
            configure_file(
                ${CMAKE_CURRENT_SOURCE_DIR}/${header_}
                ${${TARGET_NAME}_INSTALL_HEADER_STAGING_DIR}/${TARGET_NAME}/${header_}
                COPYONLY)
        endif()
    endforeach()
    unset(header_)

    function_InstallTarget(
        TARGET_NAME ${TARGET_NAME}
        PACKAGE_NAME ${PACKAGE_NAME}
        PACKAGE_INTERFACE_HEADERS ${${TARGET_NAME}_INSTALL_HEADER_STAGING_DIR}
    )
endif()
