cmake_minimum_required(VERSION 3.20)

set(TARGET_NAME RobotViewerCore)

set(${TARGET_NAME}_REQUIRED_TARGETS
    Qt5::Widgets
    Qt5::OpenGL
    Common::GLRuntime
    Common::CustomLog
    Common::Utility
    SMRobotPlatform::AssetCore
    SMRobotPlatform::CameraCore
    SMRobotCore::Collision
    SMRobotCore::Kinematics
    SMRobotPlatform::RenderCore
    SMRobotCore::RobotCore
    SMRobotCore::RobotIO
    SMRobotCore::RobotInstance
    SMRobotPlatform::RobotRenderBridge
    SMRobotCore::RobotSDK
    SMRobotCore::RobotRuntime
    SMRobotPlatform::SceneCore
    SMRobotPlatform::SimulationProject
    SMRobotPlatform::SimulationRuntime
    SMRobotPlatform::SensorCore
    SMRobotPlatform::SensorSimulation
    SMRobotPlatform::VisualizationSDK
)

set(${TARGET_NAME}_MISSING_TARGETS)
foreach(required_target_ IN LISTS ${TARGET_NAME}_REQUIRED_TARGETS)
    if(NOT TARGET ${required_target_})
        list(APPEND ${TARGET_NAME}_MISSING_TARGETS ${required_target_})
    endif()
endforeach()

if(${TARGET_NAME}_MISSING_TARGETS)
    message(STATUS
        "Skip ${TARGET_NAME} because required targets are missing: "
        "${${TARGET_NAME}_MISSING_TARGETS}"
    )
    return()
endif()

add_library(${TARGET_NAME} STATIC
    ProjectCollisionDetectorRuntime.cpp
    ProjectCollisionDetectorRuntime.h
    ProjectRuntimeBuilder.cpp
    ProjectRuntimeBuilder.h
    ProjectRuntimeTypes.h
    ProjectScene.cpp
    ProjectScene.h
    ProjectScenePickingService.cpp
    ProjectScenePickingService.h
    RobotCollisionOverrideApplier.cpp
    RobotCollisionOverrideApplier.h
    RobotCollisionModelInspector.cpp
    RobotCollisionModelInspector.h
    RobotCollisionProxyGenerator.cpp
    RobotCollisionProxyGenerator.h
    SceneObjectHoverPass.cpp
    SceneObjectHoverPass.h
    RobotViewport.cpp
    RobotViewport.h
)

add_library(SMRobotApps::${TARGET_NAME} ALIAS ${TARGET_NAME})
add_library(Applications::${TARGET_NAME} ALIAS ${TARGET_NAME})

target_compile_features(${TARGET_NAME}
    PUBLIC
        cxx_std_17
)

target_link_libraries(${TARGET_NAME}
    PUBLIC
        SMRobotPlatform::VisualizationSDK
)

target_include_directories(${TARGET_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/include>
        $<INSTALL_INTERFACE:include/RobotViewerCore>
)

set(${TARGET_NAME}_INSTALL_PUBLIC_DEPENDENCIES ${${TARGET_NAME}_REQUIRED_TARGETS})

foreach(required_target_ IN LISTS ${TARGET_NAME}_REQUIRED_TARGETS)
    target_include_directories(${TARGET_NAME}
        PRIVATE
            $<TARGET_PROPERTY:${required_target_},INTERFACE_INCLUDE_DIRECTORIES>
    )

    target_compile_definitions(${TARGET_NAME}
        PRIVATE
            $<TARGET_PROPERTY:${required_target_},INTERFACE_COMPILE_DEFINITIONS>
    )

    target_compile_options(${TARGET_NAME}
        PRIVATE
            $<TARGET_PROPERTY:${required_target_},INTERFACE_COMPILE_OPTIONS>
    )
endforeach()

set_target_properties(${TARGET_NAME}
    PROPERTIES
        FOLDER SMRobotApps
        AUTOMOC ON
)

if(ExportPacakge${PACKAGE_NAME})
    set(${TARGET_NAME}_INSTALL_HEADER_STAGING_DIR ${CMAKE_CURRENT_BINARY_DIR}/install_headers)
    file(MAKE_DIRECTORY ${${TARGET_NAME}_INSTALL_HEADER_STAGING_DIR})

    function_InstallTarget(
        TARGET_NAME ${TARGET_NAME}
        PACKAGE_NAME ${PACKAGE_NAME}
        PACKAGE_INTERFACE_HEADERS ${${TARGET_NAME}_INSTALL_HEADER_STAGING_DIR}
    )

    install(FILES
        ProjectCollisionDetectorRuntime.h
        ProjectRuntimeBuilder.h
        ProjectRuntimeTypes.h
        ProjectScene.h
        ProjectScenePickingService.h
        RobotCollisionOverrideApplier.h
        RobotCollisionModelInspector.h
        RobotCollisionProxyGenerator.h
        RobotViewport.h
        DESTINATION ${PACKAGE_NAME}/include/RobotViewerCore
    )
endif()

rs_add_example_category_roots()
