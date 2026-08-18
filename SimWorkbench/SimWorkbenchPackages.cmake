set(SimWorkbenchPackages
    SMRobotWorkbenchCommon
    SMRobotWorkbenchProjectAssembly
    SMRobotWorkbenchCollisionConfig
    SMRobotWorkbenchRobotRun
    SMRobotWorkbenchMotionPlanning
    SMRobotWorkbenchSprayProcess
    SMRobotWorkbenchPaintingAnalysis
    SMRobotWorkbenchDigitalTwin
)

foreach(_sim_workbench_package ${SimWorkbenchPackages})
    set(PackageSourceDir_${_sim_workbench_package}
        "SimWorkbench/${_sim_workbench_package}"
    )
endforeach()

unset(_sim_workbench_package)
