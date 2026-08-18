include_guard(GLOBAL)

function(_smrobot_resolve_runtime_license_directory out_directory)
    set(_license_directory)
    if(SMROBOT_RUNTIME_LICENSE_OVERRIDE_DIR)
        set(_license_directory "${SMROBOT_RUNTIME_LICENSE_OVERRIDE_DIR}")
    elseif(SMROBOT_PACKAGED_LICENSE_DIR)
        set(_license_directory "${SMROBOT_PACKAGED_LICENSE_DIR}")
    elseif(SMROBOT_RUNTIME_LICENSE_SOURCE_DIR)
        set(_license_directory "${SMROBOT_RUNTIME_LICENSE_SOURCE_DIR}")
    endif()

    if(_license_directory)
        get_filename_component(_license_directory
            "${_license_directory}" ABSOLUTE BASE_DIR "${CMAKE_SOURCE_DIR}")
    endif()
    set(${out_directory} "${_license_directory}" PARENT_SCOPE)
endfunction()

function(_smrobot_collect_runtime_license_files out_files)
    _smrobot_resolve_runtime_license_directory(_license_directory)
    if(NOT _license_directory)
        set(${out_files} "" PARENT_SCOPE)
        return()
    endif()

    if(NOT IS_DIRECTORY "${_license_directory}")
        message(FATAL_ERROR
            "Configured runtime license directory does not exist: "
            "${_license_directory}")
    endif()

    file(GLOB _license_files
        CONFIGURE_DEPENDS
        LIST_DIRECTORIES FALSE
        "${_license_directory}/*.LIC"
        "${_license_directory}/*.lic"
    )
    list(REMOVE_DUPLICATES _license_files)
    list(SORT _license_files)
    if(NOT _license_files)
        message(FATAL_ERROR
            "Configured runtime license directory does not contain a .LIC file: "
            "${_license_directory}")
    endif()

    set(${out_files} "${_license_files}" PARENT_SCOPE)
endfunction()

function(_smrobot_runtime_output_directory_for_config config out_directory)
    string(TOUPPER "${config}" _config_upper)
    if(DEFINED CMAKE_RUNTIME_OUTPUT_DIRECTORY_${_config_upper}
       AND NOT CMAKE_RUNTIME_OUTPUT_DIRECTORY_${_config_upper} STREQUAL "")
        set(_runtime_directory
            "${CMAKE_RUNTIME_OUTPUT_DIRECTORY_${_config_upper}}")
    else()
        set(_runtime_directory "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
    endif()

    if(NOT _runtime_directory)
        set(_runtime_directory "${CMAKE_BINARY_DIR}")
    endif()
    string(REPLACE "$<CONFIG>" "${config}"
        _runtime_directory "${_runtime_directory}")

    if(_runtime_directory MATCHES "\\$<")
        set(_runtime_directory "")
    else()
        get_filename_component(_runtime_directory
            "${_runtime_directory}" ABSOLUTE BASE_DIR "${CMAKE_BINARY_DIR}")
    endif()
    set(${out_directory} "${_runtime_directory}" PARENT_SCOPE)
endfunction()

function(smrobot_prepare_standard_runtime_licenses)
    _smrobot_collect_runtime_license_files(_license_files)
    if(NOT _license_files)
        return()
    endif()

    get_property(_is_multi_config GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)
    if(_is_multi_config)
        set(_license_configs ${CMAKE_CONFIGURATION_TYPES})
    elseif(CMAKE_BUILD_TYPE)
        set(_license_configs "${CMAKE_BUILD_TYPE}")
    else()
        set(_license_configs "")
    endif()

    foreach(_license_config IN LISTS _license_configs)
        _smrobot_runtime_output_directory_for_config(
            "${_license_config}" _runtime_directory)
        if(NOT _runtime_directory)
            message(STATUS
                "Skip configure-time runtime license deployment for "
                "${_license_config}: output directory contains an unsupported "
                "generator expression.")
            continue()
        endif()

        set(_runtime_license_directory "${_runtime_directory}/license")
        file(MAKE_DIRECTORY "${_runtime_license_directory}")
        foreach(_license_file IN LISTS _license_files)
            get_filename_component(_license_name "${_license_file}" NAME)
            configure_file(
                "${_license_file}"
                "${_runtime_license_directory}/${_license_name}"
                COPYONLY
            )
        endforeach()
        message(STATUS
            "Prepared runtime licenses for ${_license_config}: "
            "${_runtime_license_directory}")
    endforeach()
endfunction()

function(smrobot_deploy_runtime_licenses target_name)
    if(NOT TARGET "${target_name}")
        message(FATAL_ERROR
            "Runtime license deployment target does not exist: ${target_name}")
    endif()

    get_target_property(_deployment_attached
        "${target_name}" SMROBOT_RUNTIME_LICENSE_DEPLOYMENT_ATTACHED)
    if(_deployment_attached)
        return()
    endif()

    _smrobot_collect_runtime_license_files(_license_files)
    if(NOT _license_files)
        return()
    endif()

    add_custom_command(TARGET "${target_name}" POST_BUILD
        COMMAND "${CMAKE_COMMAND}" -E make_directory
            "$<TARGET_FILE_DIR:${target_name}>/license"
        COMMENT "Preparing runtime license directory for ${target_name}"
        VERBATIM
    )
    foreach(_license_file IN LISTS _license_files)
        add_custom_command(TARGET "${target_name}" POST_BUILD
            COMMAND "${CMAKE_COMMAND}" -E copy_if_different
                "${_license_file}"
                "$<TARGET_FILE_DIR:${target_name}>/license/"
            COMMENT "Deploying runtime license for ${target_name}"
            VERBATIM
        )
    endforeach()

    set_property(TARGET "${target_name}" PROPERTY
        SMROBOT_RUNTIME_LICENSE_DEPLOYMENT_ATTACHED TRUE)
endfunction()
