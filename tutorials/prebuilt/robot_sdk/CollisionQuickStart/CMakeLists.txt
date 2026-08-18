#############################################################
# CMake requirement
#############################################################
cmake_minimum_required(VERSION 3.20)

set(EXAMPLE_TARGET_NAME RobotSDKCollisionQuickStart)

add_executable(${EXAMPLE_TARGET_NAME} main.cpp)

target_compile_features(${EXAMPLE_TARGET_NAME} PRIVATE cxx_std_17)

target_link_libraries(${EXAMPLE_TARGET_NAME}
    PRIVATE
        SMRobotCore::RobotSDK
)

set_target_properties(${EXAMPLE_TARGET_NAME} PROPERTIES
    FOLDER "Tutorials/Prebuilt/RobotSDK"
)

rs_robot_sdk_deploy_tutorial_runtime(${EXAMPLE_TARGET_NAME})

