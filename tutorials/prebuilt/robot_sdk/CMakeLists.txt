#############################################################
# CMake requirement
#############################################################
cmake_minimum_required(VERSION 3.20)

function(rs_robot_sdk_deploy_tutorial_runtime EXAMPLE_TARGET_NAME)
    if(WIN32)
        add_custom_command(TARGET ${EXAMPLE_TARGET_NAME} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                $<TARGET_RUNTIME_DLLS:${EXAMPLE_TARGET_NAME}>
                $<TARGET_FILE_DIR:${EXAMPLE_TARGET_NAME}>
            COMMAND ${CMAKE_COMMAND} -E copy_directory
                $<TARGET_FILE_DIR:SMRobotCore::RobotSDK>
                $<TARGET_FILE_DIR:${EXAMPLE_TARGET_NAME}>
            COMMAND_EXPAND_LISTS
        )

        foreach(_runtime_target
            urdfdom::urdfdom_model
            console_bridge::console_bridge
        )
            if(TARGET ${_runtime_target})
                add_custom_command(TARGET ${EXAMPLE_TARGET_NAME} POST_BUILD
                    COMMAND ${CMAKE_COMMAND} -E copy_if_different
                        $<TARGET_FILE:${_runtime_target}>
                        $<TARGET_FILE_DIR:${EXAMPLE_TARGET_NAME}>
                )
            endif()
        endforeach()
    endif()
endfunction()

add_subdirectory(QuickStart)
add_subdirectory(CollisionQuickStart)
