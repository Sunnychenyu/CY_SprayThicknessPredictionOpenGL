add_executable(ThicknessBvhOcclusionTest main.cpp)
target_link_libraries(ThicknessBvhOcclusionTest PRIVATE
    SMRobotSpray::SprayThicknessPredictionOpenGL
    Common::GLRuntime
    Qt5::Gui)
target_compile_features(ThicknessBvhOcclusionTest PRIVATE cxx_std_17)
set_target_properties(ThicknessBvhOcclusionTest PROPERTIES
    FOLDER tests/SMRobotSpray)
copy_runtime_dlls(ThicknessBvhOcclusionTest)
add_test(NAME ThicknessBvhOcclusionTest
    COMMAND $<TARGET_FILE:ThicknessBvhOcclusionTest>)
set_tests_properties(ThicknessBvhOcclusionTest PROPERTIES
    SKIP_RETURN_CODE 77
    WORKING_DIRECTORY $<TARGET_FILE_DIR:ThicknessBvhOcclusionTest>)
