add_executable(SpatialInfluenceFilteringTest main.cpp)
target_link_libraries(
    SpatialInfluenceFilteringTest
    PRIVATE SMRobotSpray::SprayThicknessPredictionOpenGL)
target_compile_features(SpatialInfluenceFilteringTest PRIVATE cxx_std_17)
set_target_properties(
    SpatialInfluenceFilteringTest
    PROPERTIES FOLDER tests/SMRobotSpray)
add_test(
    NAME SpatialInfluenceFilteringTest
    COMMAND $<TARGET_FILE:SpatialInfluenceFilteringTest>)
