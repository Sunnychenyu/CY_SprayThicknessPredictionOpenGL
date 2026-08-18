############################################################
#   Define target copy functions for projects
#   1. Copy_ReleaseRuntime2WorkingDir
#   2. Copy_ReleaseRuntime2WorkingDir_byIMPLIB
#   3. Copy_DebugRuntime2WorkingDir
#   4. Copy_QTPlatform2WorkingDir
#   Written by Tang Qing in Jan. 2025.
############################################################


function(_smrobot_collect_private_runtime_dlls TARGET_NAME OUT_DEBUG OUT_RELEASE)
    set(_runtime_queue "${TARGET_NAME}")
    set(_runtime_visited)
    set(_runtime_files_debug)
    set(_runtime_files_release)

    while(_runtime_queue)
        list(POP_FRONT _runtime_queue _runtime_target)

        if(_runtime_target MATCHES "^\\$<LINK_ONLY:([^>]+)>$")
            set(_runtime_target "${CMAKE_MATCH_1}")
        elseif(_runtime_target MATCHES "^\\$<(BUILD|INSTALL)_INTERFACE:([^>]+)>$")
            set(_runtime_target "${CMAKE_MATCH_2}")
        elseif(_runtime_target MATCHES "^\\$<")
            continue()
        endif()

        if(NOT TARGET "${_runtime_target}")
            continue()
        endif()

        list(FIND _runtime_visited "${_runtime_target}" _runtime_visited_index)
        if(NOT _runtime_visited_index EQUAL -1)
            continue()
        endif()
        list(APPEND _runtime_visited "${_runtime_target}")

        get_target_property(_runtime_is_imported "${_runtime_target}" IMPORTED)
        if(_runtime_is_imported)
            foreach(_runtime_config DEBUG RELEASE)
                get_target_property(
                    _runtime_names
                    "${_runtime_target}"
                    "SMROBOT_PRIVATE_RUNTIME_DLLS_${_runtime_config}"
                )
                if(NOT _runtime_names OR _runtime_names MATCHES "-NOTFOUND$")
                    continue()
                endif()

                get_target_property(
                    _runtime_location
                    "${_runtime_target}"
                    "IMPORTED_LOCATION_${_runtime_config}"
                )
                if(NOT _runtime_location OR _runtime_location MATCHES "-NOTFOUND$")
                    get_target_property(
                        _runtime_location
                        "${_runtime_target}"
                        IMPORTED_LOCATION
                    )
                endif()
                if(NOT _runtime_location OR _runtime_location MATCHES "-NOTFOUND$")
                    message(FATAL_ERROR
                        "${_runtime_target} declares private runtime DLLs for "
                        "${_runtime_config}, but has no imported runtime location.")
                endif()

                get_filename_component(_runtime_directory "${_runtime_location}" DIRECTORY)
                string(TOLOWER "${_runtime_config}" _runtime_config_lower)
                foreach(_runtime_name IN LISTS _runtime_names)
                    set(_runtime_file "${_runtime_directory}/${_runtime_name}")
                    if(NOT EXISTS "${_runtime_file}")
                        message(FATAL_ERROR
                            "${_runtime_target} requires private runtime DLL "
                            "'${_runtime_name}', but it is missing from "
                            "'${_runtime_directory}'.")
                    endif()
                    list(APPEND
                        _runtime_files_${_runtime_config_lower}
                        "${_runtime_file}"
                    )
                endforeach()
            endforeach()
        endif()

        get_target_property(_runtime_links "${_runtime_target}" LINK_LIBRARIES)
        if(_runtime_links AND NOT _runtime_links MATCHES "-NOTFOUND$")
            list(APPEND _runtime_queue ${_runtime_links})
        endif()
        get_target_property(
            _runtime_interface_links
            "${_runtime_target}"
            INTERFACE_LINK_LIBRARIES
        )
        if(_runtime_interface_links AND
           NOT _runtime_interface_links MATCHES "-NOTFOUND$")
            list(APPEND _runtime_queue ${_runtime_interface_links})
        endif()
    endwhile()

    list(REMOVE_DUPLICATES _runtime_files_debug)
    list(REMOVE_DUPLICATES _runtime_files_release)
    set(${OUT_DEBUG} "${_runtime_files_debug}" PARENT_SCOPE)
    set(${OUT_RELEASE} "${_runtime_files_release}" PARENT_SCOPE)
endfunction()


function(copy_runtime_dlls TARGET_NAME)
    option(Copy_RuntimeFor_${TARGET_NAME}
        "Copy runtime DLLs of ${TARGET_NAME} to the target output directory"
        ON
    )

    if(Copy_RuntimeFor_${TARGET_NAME})
        message(STATUS "Copying runtime DLLs for ${TARGET_NAME} to target output directory ...")
        add_custom_command(TARGET ${TARGET_NAME} POST_BUILD
            COMMAND "$<$<BOOL:$<TARGET_RUNTIME_DLLS:${TARGET_NAME}>>:${CMAKE_COMMAND};-E;copy_if_different;$<TARGET_RUNTIME_DLLS:${TARGET_NAME}>;$<TARGET_FILE_DIR:${TARGET_NAME}>>"
            COMMAND_EXPAND_LISTS
        )

        _smrobot_collect_private_runtime_dlls(
            "${TARGET_NAME}"
            _private_runtime_dlls_debug
            _private_runtime_dlls_release
        )

        if(_private_runtime_dlls_debug)
            message(STATUS
                "Copying declared private Debug runtime DLLs for ${TARGET_NAME}: "
                "${_private_runtime_dlls_debug}")
            add_custom_command(TARGET ${TARGET_NAME} POST_BUILD
                COMMAND "$<$<CONFIG:Debug>:${CMAKE_COMMAND};-E;copy_if_different;${_private_runtime_dlls_debug};$<TARGET_FILE_DIR:${TARGET_NAME}>>"
                COMMAND_EXPAND_LISTS
            )
        endif()

        if(_private_runtime_dlls_release)
            message(STATUS
                "Copying declared private Release runtime DLLs for ${TARGET_NAME}: "
                "${_private_runtime_dlls_release}")
            add_custom_command(TARGET ${TARGET_NAME} POST_BUILD
                COMMAND "$<$<CONFIG:Release,RelWithDebInfo,MinSizeRel>:${CMAKE_COMMAND};-E;copy_if_different;${_private_runtime_dlls_release};$<TARGET_FILE_DIR:${TARGET_NAME}>>"
                COMMAND_EXPAND_LISTS
            )
        endif()
    endif()
endfunction()


function(deploy_qt TARGET_NAME)
    option(Deploy_QtRuntimeFor_${TARGET_NAME}
        "Deploy Qt runtime files for ${TARGET_NAME} to the target output directory"
        ON
    )

    if(Deploy_QtRuntimeFor_${TARGET_NAME} AND WIN32)
        get_target_property(_qmake Qt5::qmake IMPORTED_LOCATION)
        get_filename_component(_qt_bin_dir "${_qmake}" DIRECTORY)

        find_program(WINDEPLOYQT_EXECUTABLE
            windeployqt
            HINTS "${_qt_bin_dir}"
        )

        if(WINDEPLOYQT_EXECUTABLE)
            add_custom_command(TARGET ${TARGET_NAME} POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E env
                    "PATH=$<TARGET_FILE_DIR:${TARGET_NAME}>\;${_qt_bin_dir}"
                    "${WINDEPLOYQT_EXECUTABLE}"
                    $<$<CONFIG:Debug>:--debug>
                    $<$<CONFIG:Release>:--release>
                    --no-translations
                    --no-compiler-runtime
                    "$<TARGET_FILE:${TARGET_NAME}>"
                COMMENT "Deploying Qt runtime for ${TARGET_NAME}"
            )
        endif()
    endif()
endfunction()







#   Copy release runtime to working directory by query LOCATION.
function( Copy_ReleaseRuntime2WorkingDir target_ )
    #   Check whether the target exist!
    if( TARGET ${target_} )
        #   Get the target's IMPORTED_LOCATION_RELEASE property
        get_target_property( ${target_}_RUNTIME_RELEASE ${target_} IMPORTED_LOCATION_RELEASE )
        #   Copy the runtime to the release binary directory
        file( COPY ${${target_}_RUNTIME_RELEASE} DESTINATION ${PROJECT_BINARY_DIR}/Release/bin )
        
        #   Copy the release configuration runtime file to the release output directory
        message( STATUS "Copying ${target_}'s runtime library ${${target_}_RUNTIME_RELEASE} to binary release directory ${PROJECT_BINARY_DIR}/Release/bin." )
    else()
        message( WARNING "Target ${target_} does not exist!" )
    endif()
endfunction()

#   Copy release runtime to working directory by query IMPLIB.
#   Rename the IMPLIB by replacing the ext name from lib to dll in Windows.
#   Setup LD_LIBRARY_PATH in Ubuntu.
#   This is useful for packages such as boost.
function( Copy_ReleaseRuntime2WorkingDir_byIMPLIB target_ )
    #   Check whether the target exist!
    if( TARGET ${target_} )
        #   Get the target's IMPORTED_IMPLIB_RELEASE property
        get_target_property( ${target_}_IMPLIB_RELEASE ${target_} IMPORTED_IMPLIB_RELEASE )
        #   Replace the ext name from lib to dll
        string( REGEX REPLACE "\\.[^.]*$" ".dll" ${target_}_RUNTIME_RELEASE ${${target_}_IMPLIB_RELEASE} )
        #   Copy the runtime to the release binary directory
        file( COPY ${${target_}_RUNTIME_RELEASE} DESTINATION ${PROJECT_BINARY_DIR}/Release/bin )

        #   Copy the release configuration runtime file to the release output directory
        message( STATUS "Copying ${target_}'s runtime library ${${target_}_RUNTIME_RELEASE} to binary release directory ${PROJECT_BINARY_DIR}/Release/bin." )
    else()
        message( WARNING "Target ${target_} does not exist!" )
    endif()
endfunction()

function( Copy_DebugRuntime2WorkingDir_byIMPLIB target_ )
    #   Check whether the target exist!
    if( TARGET ${target_} )
        #   Get the target's IMPORTED_IMPLIB_DEBUG property
        get_target_property( ${target_}_IMPLIB_DEBUG ${target_} IMPORTED_IMPLIB_DEBUG )
        #   Replace the ext name from lib to dll
        string( REGEX REPLACE "\\.[^.]*$" ".dll" ${target_}_RUNTIME_DEBUG ${${target_}_IMPLIB_DEBUG} )
        #   Copy the runtime to the debug binary directory
        file( COPY ${${target_}_RUNTIME_DEBUG} DESTINATION ${PROJECT_BINARY_DIR}/Debug/bin )

        #   Copy the debug configuration runtime file to the debug output directory
        message( STATUS "Copying ${target_}'s runtime library ${${target_}_RUNTIME_DEBUG} to binary debug directory ${PROJECT_BINARY_DIR}/Debug/bin." )
    else()
        message( WARNING "Target ${target_} does not exist!" )
    endif()
endfunction()


#   Copy debug runtime to working directory by query LOCATION.
#   In fact, no copy is needed in visual studio, since debug package could be found by working environment.
function( Copy_DebugRuntime2WorkingDir target_ )
    #   Check whether the target exist!
    if( TARGET ${target_} )
        #   Get the target's IMPORTED_LOCATION_DEBUG property
        get_target_property( ${target_}_RUNTIME_LOCATION_DEBUG ${target_} IMPORTED_LOCATION_DEBUG )


        if( ${target_}_RUNTIME_LOCATION_DEBUG )
            #   Copy the debug configuration runtime file to the debug output directory
            message( STATUS "Copying ${target_}'s debug runtime library ${${target_}_RUNTIME_LOCATION_DEBUG} to binary debug directory ${PROJECT_BINARY_DIR}/Debug/bin." )
            file( COPY ${${target_}_RUNTIME_LOCATION_DEBUG} DESTINATION ${PROJECT_BINARY_DIR}/Debug/bin )
        else()
            #   Copy the release configuration runtime file in case that the debug runtime file does not exist
            get_target_property( ${target_}_RUNTIME_LOCATION_RELEASE ${target_} IMPORTED_LOCATION_RELEASE )

            #   Copy the debug configuration runtime file to the debug output directory
            #   Since the debug runtime does not exist when using distribution
            message( STATUS "Copying ${target_}'s release runtime library ${${target_}_RUNTIME_LOCATION_RELEASE} to binary debug directory ${PROJECT_BINARY_DIR}/Debug/bin." )
            file( COPY ${${target_}_RUNTIME_LOCATION_RELEASE} DESTINATION ${PROJECT_BINARY_DIR}/Debug/bin )
        endif()
    else()
        message( WARNING "Target ${target_} does not exist!" )
    endif()
endfunction()



function( Copy_QTPlatform2WorkingDir )
    #   Copy Qt Platform to the release/debug directory (Debug is not needed by setting up )
    if( ${Copy_QtRuntimePlatform2ReleaseDir} )
        #   Copy Qt platform to release directory.
        message( STATUS "Copying Qt platform directory from ${Qt5_PLATFORM_DIR}/platforms to release binary directory." )

        file( COPY ${Qt5_PLATFORM_DIR}/platforms DESTINATION ${PROJECT_BINARY_DIR}/Release/bin )
        file( COPY ${Qt5_PLATFORM_DIR}/platforms DESTINATION ${PROJECT_BINARY_DIR}/Debug/bin )
    endif()
endfunction()

