cmake_minimum_required(VERSION 3.20)

set(${TARGET_NAME}_RequiredLibsPublic
    SMRobotWorkbenchCommon::WorkbenchCommon
    SMRobotSpray::SprayCore
    SMRobotSpray::SprayPathPlanning
    SMRobotSpray::SprayTrajectoryCore
)

set(${TARGET_NAME}_RequiredLibsPrivate)
