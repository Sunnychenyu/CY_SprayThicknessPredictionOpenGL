############################################################
#   Function - collect_subdirs
#   Usage:
#   collect_subdirs(<root_dir> <out_var> [ONLY_CMAKE_PROJECTS] [EXCLUDE dir1 dir2 ...])
############################################################


function(collect_subdirs ROOT_DIR OUT_VAR)

    set(options ONLY_CMAKE_PROJECTS)
    set(oneValueArgs)
    set(multiValueArgs EXCLUDE)

    cmake_parse_arguments(CSD "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    # Scan the contents of the current directory
    # message( STATUS "ROOT_DIR is: ${ROOT_DIR}" )
    file( GLOB children RELATIVE "${ROOT_DIR}" "${ROOT_DIR}/*" )
    # message( STATUS "children is: ${children}" )

    set(result)

    foreach(child ${children})
        set(child_path ${ROOT_DIR}/${child})

        if(IS_DIRECTORY ${child_path})

            # whether to exclude
            list(FIND CSD_EXCLUDE ${child} exclude_index)
            if(NOT exclude_index EQUAL -1)
                continue()
            endif()

            # Only need the project directory containing CMakeLists.txt?
            if(CSD_ONLY_CMAKE_PROJECTS)
                if(NOT EXISTS ${child_path}/CMakeLists.txt)
                    continue()
                endif()
            endif()

            list(APPEND result ${child})

        endif()
    endforeach()

    # Return the result to the caller's scope
    set(${OUT_VAR} ${result} PARENT_SCOPE)

endfunction()

# #   Using example
# collect_subdirs(
#     ${CMAKE_CURRENT_SOURCE_DIR}
#     AllPackageNames
#     EXCLUDE cmake data build .git
# )

# message(STATUS "Packages: ${AllPackageNames}")
