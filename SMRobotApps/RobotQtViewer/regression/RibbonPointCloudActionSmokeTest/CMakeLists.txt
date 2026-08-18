cmake_minimum_required(VERSION 3.20)

rs_example_category_enabled(_build_example Regression ${PACKAGE_NAME})
if(NOT _build_example)
    return()
endif()

set(EXAMPLE_TARGET_NAME RobotQtViewer-RibbonPcdSmoke)
set(ROBOT_QT_VIEWER_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../..")

add_executable(${EXAMPLE_TARGET_NAME}
    main.cpp
    "${ROBOT_QT_VIEWER_SOURCE_DIR}/RobotQtViewerRibbonModel.cpp"
    "${ROBOT_QT_VIEWER_SOURCE_DIR}/RobotQtViewerRibbonModel.h"
    "${ROBOT_QT_VIEWER_SOURCE_DIR}/RobotQtViewerResources.qrc"
)

target_include_directories(${EXAMPLE_TARGET_NAME}
    PRIVATE
        "${ROBOT_QT_VIEWER_SOURCE_DIR}"
)

target_link_libraries(${EXAMPLE_TARGET_NAME}
    PRIVATE
        Qt5::Widgets
        SMRobotWorkbenchCommon::RobotQtModulesShared
)

file(TO_CMAKE_PATH "${PROJECT_SOURCE_DIR}" _smrobot_test_source_root)
target_compile_definitions(${EXAMPLE_TARGET_NAME}
    PRIVATE
        SMROBOT_TEST_SOURCE_ROOT="${_smrobot_test_source_root}"
)

target_compile_features(${EXAMPLE_TARGET_NAME} PRIVATE cxx_std_17)

set_target_properties(${EXAMPLE_TARGET_NAME}
    PROPERTIES
        AUTORCC ON
        FOLDER "Regression/SMRobotApps/RobotQtViewer"
        DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX}
        RELEASE_POSTFIX ${CMAKE_RELEASE_POSTFIX}
)

copy_runtime_dlls(${EXAMPLE_TARGET_NAME})
if(COMMAND smrobot_deploy_runtime_licenses)
    smrobot_deploy_runtime_licenses(${EXAMPLE_TARGET_NAME})
endif()

if(BuildTests)
    add_test(
        NAME SMRobotGen2RobotQtViewerRibbonPointCloudActionSmokeTest
        COMMAND ${EXAMPLE_TARGET_NAME}
    )
    set_tests_properties(SMRobotGen2RobotQtViewerRibbonPointCloudActionSmokeTest PROPERTIES
        LABELS "SMRobotGen2;RobotQtViewer;Regression;PointCloud"
        WORKING_DIRECTORY $<TARGET_FILE_DIR:${EXAMPLE_TARGET_NAME}>
    )
endif()
