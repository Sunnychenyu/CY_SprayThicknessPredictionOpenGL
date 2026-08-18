cmake_minimum_required(VERSION 3.20)

set(APP_TARGET_NAME RobotQtViewer)

if(NOT TARGET Qt5::Widgets OR NOT TARGET Qt5::OpenGL)
    message(FATAL_ERROR "${APP_TARGET_NAME} requires Qt5::Widgets and Qt5::OpenGL.")
endif()

if(NOT TARGET SMRobotCore::RobotSDK)
    message(FATAL_ERROR "${APP_TARGET_NAME} requires SMRobotCore::RobotSDK.")
endif()

set(${APP_TARGET_NAME}_REQUIRED_TARGETS
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
    SMRobotApps::RobotViewerCore
    SMRobotWorkbenchProjectAssembly::ProjectAssemblyWorkbench
    SMRobotWorkbenchCollisionConfig::CollisionConfigWorkbench
    SMRobotWorkbenchRobotRun::RobotRunWorkbench
    SMRobotWorkbenchMotionPlanning::MotionPlanningWorkbench
    SMRobotWorkbenchPaintingAnalysis::PaintingAnalysisWorkbench
)

set(${APP_TARGET_NAME}_MISSING_TARGETS)
foreach(required_target_ IN LISTS ${APP_TARGET_NAME}_REQUIRED_TARGETS)
    if(NOT TARGET ${required_target_})
        list(APPEND ${APP_TARGET_NAME}_MISSING_TARGETS ${required_target_})
    endif()
endforeach()

if(${APP_TARGET_NAME}_MISSING_TARGETS)
    message(STATUS
        "Skip ${APP_TARGET_NAME} because required targets are missing: "
        "${${APP_TARGET_NAME}_MISSING_TARGETS}"
    )
    return()
endif()

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

set(${APP_TARGET_NAME}_SHELL_SOURCES
    main.cpp
    MainWindow.cpp
    MainWindow.h
    RobotQtViewerRibbonModel.cpp
    RobotQtViewerRibbonModel.h
    RobotQtViewerResources.qrc
    RobotQtViewerSceneExplorerActionRouter.cpp
    RobotQtViewerSceneExplorerActionRouter.h
    RobotQtViewerToolbarController.cpp
    RobotQtViewerToolbarController.h
    RobotQtViewerCollisionWorkbenchServicesAdapter.cpp
    RobotQtViewerCollisionWorkbenchServicesAdapter.h
    RobotQtViewerToolSetupAppServicesAdapter.cpp
    RobotQtViewerToolSetupAppServicesAdapter.h
    RobotQtViewerViewportEventController.cpp
    RobotQtViewerViewportEventController.h
    RobotQtViewerViewportPresentationController.cpp
    RobotQtViewerViewportPresentationController.h
    RobotQtViewerViewportServicesAdapter.cpp
    RobotQtViewerViewportServicesAdapter.h
    RobotQtViewerLanguage.cpp
    RobotQtViewerLanguage.h
    RobotQtViewerTheme.cpp
    RobotQtViewerTheme.h
)

if(WIN32)
    list(APPEND ${APP_TARGET_NAME}_SHELL_SOURCES
        RobotQtViewer.rc
        RobotQtViewerResource.h
    )
endif()

add_executable(${APP_TARGET_NAME}
    ${${APP_TARGET_NAME}_SHELL_SOURCES}
)

add_executable(SMRobotApps::${APP_TARGET_NAME} ALIAS ${APP_TARGET_NAME})
add_executable(Applications::${APP_TARGET_NAME} ALIAS ${APP_TARGET_NAME})

target_link_libraries(${APP_TARGET_NAME}
    PRIVATE
        SMRobotWorkbenchCommon::RobotQtModulesCoreWidgets
        SMRobotWorkbenchCommon::RobotQtModulesShared
        SMRobotWorkbenchProjectAssembly::ProjectAssemblyWorkbench
        SMRobotWorkbenchCollisionConfig::CollisionConfigWorkbench
        SMRobotWorkbenchRobotRun::RobotRunWorkbench
        SMRobotWorkbenchMotionPlanning::MotionPlanningWorkbench
        SMRobotWorkbenchPaintingAnalysis::PaintingAnalysisWorkbench
        RobotQtModulesStatus
        ${${APP_TARGET_NAME}_REQUIRED_TARGETS}
)

if(WIN32)
    target_link_libraries(${APP_TARGET_NAME}
        PRIVATE
            dwmapi
    )
endif()

target_compile_features(${APP_TARGET_NAME}
    PRIVATE
        cxx_std_17
)

target_compile_definitions(${APP_TARGET_NAME}
    PRIVATE
        ROBOT_QT_VIEWER_DEFAULT_PROJECT_PATH="projects/420-red4600.sys.json"
)

set_target_properties(${APP_TARGET_NAME}
    PROPERTIES
        FOLDER SMRobotApps
        DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX}
        RELEASE_POSTFIX ${CMAKE_RELEASE_POSTFIX}
)

if(CMAKE_SYSTEM_NAME MATCHES "Windows")
    set_property(TARGET ${APP_TARGET_NAME} PROPERTY
        VS_DEBUGGER_ENVIRONMENT $<$<CONFIG:Debug>:${VSDebugEnvPath}>)
endif()

#############################################################
#	Copy the runtime files to release directory for current target
#############################################################
copy_runtime_dlls( ${APP_TARGET_NAME} )

if(TARGET CoACD::coacd AND COMMAND CoACD_CopyRuntimeDependencies)
    CoACD_CopyRuntimeDependencies(${APP_TARGET_NAME})
endif()

deploy_qt( ${APP_TARGET_NAME} )

rs_example_category_enabled(_build_regression Regression ${PACKAGE_NAME})
if(_build_regression)
    add_subdirectory(regression)
endif()

if(ExportPacakge${PACKAGE_NAME})
    set(${APP_TARGET_NAME}_INSTALL_HEADER_STAGING_DIR ${CMAKE_CURRENT_BINARY_DIR}/install_headers)
    file(MAKE_DIRECTORY ${${APP_TARGET_NAME}_INSTALL_HEADER_STAGING_DIR})

    function_InstallTarget(
        TARGET_NAME ${APP_TARGET_NAME}
        PACKAGE_NAME ${PACKAGE_NAME}
        PACKAGE_INTERFACE_HEADERS ${${APP_TARGET_NAME}_INSTALL_HEADER_STAGING_DIR}
    )

    install(FILES
        $<TARGET_RUNTIME_DLLS:${APP_TARGET_NAME}>
        DESTINATION ${PACKAGE_NAME}/bin
        CONFIGURATIONS Release
    )

    install(DIRECTORY
        $<TARGET_FILE_DIR:${APP_TARGET_NAME}>/
        DESTINATION ${PACKAGE_NAME}/bin
        CONFIGURATIONS Release
        FILES_MATCHING
            PATTERN "*.dll"
            PATTERN "iconengines/*"
            PATTERN "imageformats/*"
            PATTERN "platforms/*"
            PATTERN "styles/*"
    )
endif()

