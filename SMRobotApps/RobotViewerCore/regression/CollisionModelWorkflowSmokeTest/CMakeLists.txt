#############################################################
# CMake requirement
#############################################################
cmake_minimum_required(VERSION 3.20)

rs_example_category_enabled(_build_example Regression ${PACKAGE_NAME})
if(NOT _build_example)
    return()
endif()

set(EXAMPLE_TARGET_NAME RobotViewerCore-CollisionModelWorkflowSmokeTest)

add_executable(${EXAMPLE_TARGET_NAME}
    main.cpp
)

target_include_directories(${EXAMPLE_TARGET_NAME}
    PRIVATE
        $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/include>
)

target_link_libraries(${EXAMPLE_TARGET_NAME}
    PRIVATE
        SMRobotApps::RobotViewerCore
        ${RobotViewerCore_REQUIRED_TARGETS}
)

target_compile_features(${EXAMPLE_TARGET_NAME}
    PRIVATE
        cxx_std_17
)

set_target_properties(${EXAMPLE_TARGET_NAME}
    PROPERTIES
        FOLDER          "Regression/SMRobotApps/RobotViewerCore"
        DEBUG_POSTFIX   ${CMAKE_DEBUG_POSTFIX}
        RELEASE_POSTFIX ${CMAKE_RELEASE_POSTFIX}
)

if(CMAKE_SYSTEM_NAME MATCHES "Windows")
    set_property(TARGET ${EXAMPLE_TARGET_NAME} PROPERTY
        VS_DEBUGGER_ENVIRONMENT $<$<CONFIG:Debug>:${VSDebugEnvPath}>)
endif()

copy_runtime_dlls(${EXAMPLE_TARGET_NAME})

if(BuildTests)
    add_test(
        NAME SMRobotGen2RobotViewerCoreCollisionModelWorkflowSmokeTest
        COMMAND ${EXAMPLE_TARGET_NAME}
    )

    set_tests_properties(SMRobotGen2RobotViewerCoreCollisionModelWorkflowSmokeTest PROPERTIES
        LABELS "SMRobotGen2;RobotViewerCore;Regression"
        WORKING_DIRECTORY $<TARGET_FILE_DIR:${EXAMPLE_TARGET_NAME}>
    )
endif()
