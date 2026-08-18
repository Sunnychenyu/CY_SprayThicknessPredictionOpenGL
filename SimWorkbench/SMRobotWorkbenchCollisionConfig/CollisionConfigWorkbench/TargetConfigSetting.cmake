cmake_minimum_required(VERSION 3.20)

set(${TARGET_NAME}_RequiredLibsPublic
    SMRobotWorkbenchCommon::WorkbenchCommon
    SMRobotWorkbenchCollisionConfig::CollisionDetectorConfig
    SMRobotWorkbenchCollisionConfig::CollisionLinkModelSetup
    SMRobotWorkbenchCollisionConfig::CollisionRuntimeResults
)

set(${TARGET_NAME}_RequiredLibsPrivate)
