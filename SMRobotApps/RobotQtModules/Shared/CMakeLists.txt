cmake_minimum_required(VERSION 3.20)

set(TARGET_NAME RobotQtModulesShared)

set(SOURCES
    CollisionRuntimeViewModel.h
    InspectorContext.cpp
    InspectorContext.h
    ProjectSessionWorkflowController.cpp
    ProjectSessionWorkflowController.h
    RobotQtViewerDocumentContext.cpp
    RobotQtViewerDocumentContext.h
    RobotQtViewerDocumentController.cpp
    RobotQtViewerDocumentController.h
    RobotQtViewerDocumentViewRegistry.cpp
    RobotQtViewerDocumentViewRegistry.h
    RobotQtViewerEventHub.cpp
    RobotQtViewerEventHub.h
    RobotQtViewerEvents.h
    RobotQtViewerAppController.cpp
    RobotQtViewerAppController.h
    RobotQtViewerSelectionModel.cpp
    RobotQtViewerSelectionModel.h
    RobotQtViewerViewportPreviewState.cpp
    RobotQtViewerViewportPreviewState.h
    RobotQtViewerViewportServices.h
    RobotQtViewerWorkbench.cpp
    RobotQtViewerWorkbench.h
    RobotQtViewerWorkbenchPackageRegistry.cpp
    RobotQtViewerWorkbenchPackageRegistry.h
    RobotQtViewerWindowConfig.cpp
    RobotQtViewerWindowConfig.h
    SceneEntityWorkflowController.cpp
    SceneEntityWorkflowController.h
    ViewportReloadWorkflowController.cpp
    ViewportReloadWorkflowController.h
)

add_library(${TARGET_NAME} STATIC ${SOURCES})
add_library(RobotQtViewerShared ALIAS ${TARGET_NAME})
if(NOT TARGET SMRobotWorkbenchCommon::${TARGET_NAME})
    add_library(SMRobotWorkbenchCommon::${TARGET_NAME} ALIAS ${TARGET_NAME})
endif()

target_include_directories(${TARGET_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:SMRobotWorkbenchCommon/include/${TARGET_NAME}>
)

target_link_libraries(${TARGET_NAME}
    PUBLIC
        Qt5::Widgets
        Common::Utility
        SMRobotPlatform::SimulationProject
        SMRobotPlatform::VisualizationSDK
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

if(ExportPacakgeSMRobotWorkbenchCommon)
    set(${TARGET_NAME}_INSTALL_PUBLIC_DEPENDENCIES
        Qt5::Widgets
        Common::Utility
        SMRobotPlatform::SimulationProject
        SMRobotPlatform::VisualizationSDK
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
        PACKAGE_NAME SMRobotWorkbenchCommon
        PACKAGE_INTERFACE_HEADERS ${${TARGET_NAME}_INSTALL_HEADER_STAGING_DIR}
    )
endif()
