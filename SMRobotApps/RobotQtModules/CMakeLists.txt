cmake_minimum_required(VERSION 3.20)

if(NOT TARGET Qt5::Widgets)
    message(STATUS "Skip RobotQtModules because Qt5::Widgets is missing.")
    return()
endif()

set(ROBOT_QT_MODULES_REQUIRED_TARGETS
    Qt5::Widgets
    Common::Utility
    SMRobotPlatform::SimulationProject
    SMRobotPlatform::SimulationRuntime
)

set(ROBOT_QT_MODULES_MISSING_TARGETS)
foreach(required_target_
    IN LISTS
        ROBOT_QT_MODULES_REQUIRED_TARGETS
)
    if(NOT TARGET ${required_target_})
        list(APPEND ROBOT_QT_MODULES_MISSING_TARGETS ${required_target_})
    endif()
endforeach()

if(ROBOT_QT_MODULES_MISSING_TARGETS)
    message(STATUS
        "Skip RobotQtModules because required targets are missing: "
        "${ROBOT_QT_MODULES_MISSING_TARGETS}"
    )
    return()
endif()

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

if(TARGET SMRobotWorkbenchCommon::RobotQtModulesShared
   AND TARGET SMRobotWorkbenchCommon::RobotQtModulesCoreWidgets)
    message(STATUS "Use SMRobotWorkbenchCommon prebuilt RobotQtModules shared/core widgets.")
else()
    add_subdirectory(Shared)
    add_subdirectory(CoreWidgets)
endif()
add_subdirectory(Status)
