cmake_minimum_required(VERSION 3.20)

set(TARGET_NAME ProjectAssemblySceneExplorer)

set(SOURCES
    ProjectAssemblyDialogService.cpp
    ProjectAssemblyDialogService.h
    SceneCollisionTargetResolver.cpp
    SceneCollisionTargetResolver.h
    SceneExplorerModuleController.cpp
    SceneExplorerModuleController.h
    SceneExplorerViewModel.cpp
    SceneExplorerViewModelBuilder.cpp
    SceneExplorerViewModelBuilder.h
    SceneExplorerTaskWidget.cpp
    SceneExplorerTaskWidget.h
    SceneExplorerWidget.cpp
    SceneExplorerWidget.h
    SceneExplorerViewModel.h
    SceneSelectionController.cpp
    SceneSelectionController.h
    SceneTreeIntentController.cpp
    SceneTreeIntentController.h
)

add_library(${TARGET_NAME} STATIC ${SOURCES})
add_library(RobotQtModulesSceneExplorer ALIAS ${TARGET_NAME})
add_library(RobotQtViewerSceneExplorer ALIAS ${TARGET_NAME})

target_include_directories(${TARGET_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:${PACKAGE_NAME}/include/${TARGET_NAME}>
)

target_link_libraries(${TARGET_NAME}
    PUBLIC
        $<BUILD_INTERFACE:RobotQtModulesShared>
        $<INSTALL_INTERFACE:SMRobotWorkbenchCommon::RobotQtModulesShared>
    PRIVATE
        $<BUILD_INTERFACE:ProjectAssemblyEditors>
        $<INSTALL_INTERFACE:SMRobotWorkbenchProjectAssembly::ProjectAssemblyEditors>
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

if(BuildTests)
    add_subdirectory(
        regression/SceneExplorerCollisionProjectionSmokeTest
        ${PROJECT_BINARY_DIR}/tests/scene_explorer_collision_projection)
endif()

if(ExportPacakge${PACKAGE_NAME})
    set(${TARGET_NAME}_INSTALL_PUBLIC_DEPENDENCIES
        SMRobotWorkbenchCommon::RobotQtModulesShared
        SMRobotWorkbenchCommon::RobotQtModulesCoreWidgets
        SMRobotWorkbenchProjectAssembly::ProjectAssemblyEditors
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
