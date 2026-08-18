#######################################################
#   User Configuration
#   Make sure only one user is defined in the program
#######################################################

message( STATUS "\n-------------------------------------------------------" )
message( STATUS "\tUser Configuration" )
message( STATUS "-------------------------------------------------------\n" )


#   Define all possible users
list( APPEND USER_GROUP
    USER_TANGTANG_3090_Ubuntu
    USER_TANGTANG_3090_Windows

    USER_TANGTANG_4090_Windows
    USER_TANGTANG_4090_Ubuntu2204

    USER_TANGTANG_14S_Ubuntu
    USER_TANGTANG_14S_Windows

    USER_TANGTANG_p15v3_Windows
    USER_TANGTANG_p15v3_Ubuntu2204
)



foreach(user ${USER_GROUP})
    string(REPLACE "_" " " user_desc ${user})
    option(${user} "Enable build config for ${user_desc}" OFF)
    list(APPEND User_Options ${user})
endforeach()


set(enabled_users "")
foreach(opt IN LISTS User_Options)
    if(${opt})
        list(APPEND enabled_users ${opt})
    endif()
endforeach()

list(LENGTH enabled_users enabled_count)

if(enabled_count EQUAL 0)
    message(FATAL_ERROR
        "Error: No user selected.\n"
        "Please enable exactly one of the following options:\n"
        "${User_Options}"
    )
elseif(enabled_count GREATER 1)
    message(FATAL_ERROR
        "Error: Multiple users selected:\n"
        "${enabled_users}\n\n"
        "Please enable ONLY ONE of the following options:\n"
        "${User_Options}"
    )
endif()

foreach(each_user IN LISTS User_Options)
    if(${each_user})
        message(STATUS "Current User is ${each_user}")
        string(REGEX REPLACE "USER_" "" user_config_filename ${each_user})
        message(STATUS "User's config filename is ${user_config_filename}")

        include(UserConfigs/${user_config_filename})
        break()  # Since only one user can be enabled, we can break after finding it
    endif()
endforeach()


if(IS_DIRECTORY "${SMROBOT_THIRDPARTY_ROOT}/tinyxml2")
    add_subdirectory("${SMROBOT_THIRDPARTY_ROOT}/tinyxml2" "${CMAKE_BINARY_DIR}/thirdparty/tinyxml2")
else()
    message(FATAL_ERROR
        "tinyxml2 third-party source was not found under SMROBOT_THIRDPARTY_ROOT: "
        "${SMROBOT_THIRDPARTY_ROOT}")
endif()
