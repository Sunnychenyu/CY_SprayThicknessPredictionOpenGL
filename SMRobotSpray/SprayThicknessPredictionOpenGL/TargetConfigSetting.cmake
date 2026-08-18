cmake_minimum_required(VERSION 3.20)

set(${TARGET_NAME}_RequiredLibsPublic
    SMRobotSpray::SprayThicknessPrediction
)

set(${TARGET_NAME}_RequiredLibsPrivate
    Eigen3::Eigen
    Common::GLRuntime
    OpenGL::GL
)
