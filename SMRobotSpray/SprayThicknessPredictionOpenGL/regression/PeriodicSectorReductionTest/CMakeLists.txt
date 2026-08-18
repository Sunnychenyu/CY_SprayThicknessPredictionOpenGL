cmake_minimum_required(VERSION 3.20)

add_executable(PeriodicSectorReductionTest main.cpp)
target_include_directories(PeriodicSectorReductionTest
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../src)
target_link_libraries(PeriodicSectorReductionTest
    PRIVATE SMRobotSpray::SprayThicknessPredictionOpenGL)
target_compile_features(PeriodicSectorReductionTest PRIVATE cxx_std_17)
set_target_properties(PeriodicSectorReductionTest PROPERTIES FOLDER tests/SMRobotSpray)
add_test(NAME PeriodicSectorReductionTest COMMAND $<TARGET_FILE:PeriodicSectorReductionTest>)
