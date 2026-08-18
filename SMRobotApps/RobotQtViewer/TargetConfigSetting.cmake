##########################################################################
# CMake requirement
##########################################################################
cmake_minimum_required(VERSION 3.20)

set(${TARGET_NAME}_RequiredLibsPublic
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
    SMRobotWorkbenchProjectAssembly::ProjectAssemblyWorkbench
    SMRobotWorkbenchCollisionConfig::CollisionConfigWorkbench
    SMRobotWorkbenchRobotRun::RobotRunWorkbench
    SMRobotWorkbenchMotionPlanning::MotionPlanningWorkbench
    SMRobotWorkbenchPaintingAnalysis::PaintingAnalysisWorkbench
)

set(${TARGET_NAME}_RequiredLibsPrivate)
