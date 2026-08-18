cmake_minimum_required(VERSION 3.20)

rs_example_category_enabled(_build_example Regression ${PACKAGE_NAME})
if(NOT _build_example)
    return()
endif()

set(EXAMPLE_TARGET_NAME RobotQtViewer-ViewportPresentationSmoke)
set(ROBOT_QT_VIEWER_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../..")

add_executable(${EXAMPLE_TARGET_NAME}
    main.cpp
    "${ROBOT_QT_VIEWER_SOURCE_DIR}/RobotQtViewerViewportPresentationController.cpp"
    "${ROBOT_QT_VIEWER_SOURCE_DIR}/RobotQtViewerViewportPresentationController.h"
)

target_include_directories(${EXAMPLE_TARGET_NAME}
    PRIVATE
        "${ROBOT_QT_VIEWER_SOURCE_DIR}"
)

target_link_libraries(${EXAMPLE_TARGET_NAME}
    PRIVATE
        Qt5::Widgets
)

target_compile_features(${EXAMPLE_TARGET_NAME} PRIVATE cxx_std_17)

set_target_properties(${EXAMPLE_TARGET_NAME}
    PROPERTIES
        FOLDER "Regression/SMRobotApps/RobotQtViewer"
        DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX}
        RELEASE_POSTFIX ${CMAKE_RELEASE_POSTFIX}
)

copy_runtime_dlls(${EXAMPLE_TARGET_NAME})
deploy_qt(${EXAMPLE_TARGET_NAME})

if(BuildTests)
    add_test(
        NAME SMRobotGen2RobotQtViewerViewportPresentationSmokeTest
        COMMAND ${EXAMPLE_TARGET_NAME}
    )
    set_tests_properties(SMRobotGen2RobotQtViewerViewportPresentationSmokeTest PROPERTIES
        LABELS "SMRobotGen2;RobotQtViewer;Regression;Fullscreen"
        WORKING_DIRECTORY $<TARGET_FILE_DIR:${EXAMPLE_TARGET_NAME}>
    )
endif()
