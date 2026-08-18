cmake_minimum_required(VERSION 3.20)

project(RS2026PrebuiltTutorials LANGUAGES CXX)

find_package(SMRobotCore CONFIG REQUIRED COMPONENTS RobotSDK)

add_subdirectory(robot_sdk)
