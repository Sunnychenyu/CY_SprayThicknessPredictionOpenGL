cmake_minimum_required(VERSION 3.20)

set(${TARGET_NAME}_RequiredLibsPublic
    SMRobotWorkbenchCommon::WorkbenchCommon
    SMRobotWorkbenchProjectAssembly::ProjectAssemblyEditors
    SMRobotWorkbenchProjectAssembly::ProjectAssemblySceneExplorer
    SMRobotWorkbenchProjectAssembly::ProjectAssemblyToolSetup
)

set(${TARGET_NAME}_RequiredLibsPrivate)
