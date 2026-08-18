##########################################################################
# CMake requirement
##########################################################################
cmake_minimum_required( VERSION 3.20 )

set( ${TARGET_NAME}_RequiredLibsPublic
    Eigen3::Eigen
    SMRobotCore::RobotTrajectoryCore
    SMRobotSpray::SprayCore
)

set( ${TARGET_NAME}_RequiredLibsPrivate
)
