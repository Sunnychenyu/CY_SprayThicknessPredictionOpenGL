cmake_minimum_required(VERSION 3.20)

set(${TARGET_NAME}_RequiredLibsPublic
    SMRobotWorkbenchCommon::WorkbenchCommon
    SMRobotWorkbenchRobotRun::RobotRunMotionControl
    SMRobotWorkbenchCollisionConfig::CollisionRuntimeResults
)

set(${TARGET_NAME}_RequiredLibsPrivate)
