cmake_minimum_required(VERSION 3.20)

set(TEST_TARGET_NAME SceneExplorerCollisionProjectionSmokeTest)

add_executable(${TEST_TARGET_NAME}
    main.cpp
)

target_link_libraries(${TEST_TARGET_NAME}
    PRIVATE
        ProjectAssemblySceneExplorer
        SMRobotPlatform::SimulationProject
)

target_compile_features(${TEST_TARGET_NAME}
    PRIVATE
        cxx_std_17
)

set_target_properties(${TEST_TARGET_NAME}
    PROPERTIES
        FOLDER "Regression/SMRobotWorkbenchProjectAssembly"
        DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX}
        RELEASE_POSTFIX ${CMAKE_RELEASE_POSTFIX}
)

copy_runtime_dlls(${TEST_TARGET_NAME})

add_test(
    NAME SMRobotGen2SceneExplorerCollisionProjectionSmokeTest
    COMMAND ${TEST_TARGET_NAME}
)

set_tests_properties(SMRobotGen2SceneExplorerCollisionProjectionSmokeTest PROPERTIES
    LABELS "SMRobotGen2;ProjectAssembly;CollisionConfig;Regression"
    WORKING_DIRECTORY $<TARGET_FILE_DIR:${TEST_TARGET_NAME}>
)
