cmake_minimum_required(VERSION 3.20)

set(TEST_TARGET_NAME CollisionLinkModelsControllerSmokeTest)

add_executable(${TEST_TARGET_NAME}
    main.cpp
)

target_link_libraries(${TEST_TARGET_NAME}
    PRIVATE
        CollisionLinkModelSetup
        SMRobotPlatform::SimulationProject
)

target_compile_definitions(${TEST_TARGET_NAME}
    PRIVATE
        PROJECT_SOURCE_PATH="${PROJECT_SOURCE_DIR}"
)

target_compile_features(${TEST_TARGET_NAME}
    PRIVATE
        cxx_std_17
)

set_target_properties(${TEST_TARGET_NAME}
    PROPERTIES
        FOLDER "Regression/SMRobotWorkbenchCollisionConfig"
        DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX}
        RELEASE_POSTFIX ${CMAKE_RELEASE_POSTFIX}
)

copy_runtime_dlls(${TEST_TARGET_NAME})

add_test(
    NAME SMRobotGen2CollisionLinkModelsControllerSmokeTest
    COMMAND ${TEST_TARGET_NAME}
)

set_tests_properties(SMRobotGen2CollisionLinkModelsControllerSmokeTest PROPERTIES
    LABELS "SMRobotGen2;CollisionConfig;Regression"
    WORKING_DIRECTORY $<TARGET_FILE_DIR:${TEST_TARGET_NAME}>
)
