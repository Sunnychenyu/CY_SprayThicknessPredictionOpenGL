cmake_minimum_required(VERSION 3.20)

set(${TARGET_NAME}_RequiredLibsPublic
    SMRobotMotionPlanning::MotionPlanningCore
    SMRobotPlatform::SimulationProject
    SMRobotPlatform::SimulationRuntime
)

set(${TARGET_NAME}_RequiredLibsPrivate
    SMRobotMotionPlanning::MotionPlanningOmpl
)
