cmake_minimum_required(VERSION 3.20)

set(APP_TARGET_NAME RobotSDKVisualQuickStart)

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
    SMRobotApps::RobotViewerCore
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

function(robot_sdk_visual_quickstart_deploy_runtime TARGET_NAME)
    add_custom_command(TARGET ${TARGET_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_RUNTIME_DLLS:${TARGET_NAME}>
            $<TARGET_FILE_DIR:${TARGET_NAME}>
        COMMAND_EXPAND_LISTS
    )

    if(WIN32)
        get_target_property(_qmake Qt5::qmake IMPORTED_LOCATION)
        get_filename_component(_qt_bin_dir "${_qmake}" DIRECTORY)

        find_program(WINDEPLOYQT_EXECUTABLE
            windeployqt
            HINTS "${_qt_bin_dir}"
        )

        if(WINDEPLOYQT_EXECUTABLE)
            add_custom_command(TARGET ${TARGET_NAME} POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E env
                    "PATH=$<TARGET_FILE_DIR:${TARGET_NAME}>\;${_qt_bin_dir}"
                    "${WINDEPLOYQT_EXECUTABLE}"
                    --no-translations
                    --no-compiler-runtime
                    "$<TARGET_FILE:${TARGET_NAME}>"
                COMMENT "Deploying Qt runtime for ${TARGET_NAME}"
            )
        endif()
    endif()
endfunction()

add_executable(${APP_TARGET_NAME}
    main.cpp
    VisualQuickStartWindow.cpp
    VisualQuickStartWindow.h
)

add_executable(SMRobotApps::${APP_TARGET_NAME} ALIAS ${APP_TARGET_NAME})
add_executable(Applications::${APP_TARGET_NAME} ALIAS ${APP_TARGET_NAME})

target_compile_features(${APP_TARGET_NAME}
    PRIVATE
        cxx_std_17
)

target_include_directories(${APP_TARGET_NAME}
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/include>
)

target_link_libraries(${APP_TARGET_NAME}
    PRIVATE
        ${${APP_TARGET_NAME}_REQUIRED_TARGETS}
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

robot_sdk_visual_quickstart_deploy_runtime(${APP_TARGET_NAME})

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

