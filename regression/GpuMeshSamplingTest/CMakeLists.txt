add_executable(GpuMeshSamplingTest main.cpp)
target_link_libraries(GpuMeshSamplingTest PRIVATE
    SMRobotSpray::SprayThicknessPredictionOpenGL
    Common::GLRuntime Qt5::Gui)
target_compile_features(GpuMeshSamplingTest PRIVATE cxx_std_17)
set_target_properties(GpuMeshSamplingTest PROPERTIES FOLDER tests/SMRobotSpray)
copy_runtime_dlls(GpuMeshSamplingTest)
add_test(NAME GpuMeshSamplingTest COMMAND $<TARGET_FILE:GpuMeshSamplingTest>)
set_tests_properties(GpuMeshSamplingTest PROPERTIES
    SKIP_RETURN_CODE 77
    WORKING_DIRECTORY $<TARGET_FILE_DIR:GpuMeshSamplingTest>)
