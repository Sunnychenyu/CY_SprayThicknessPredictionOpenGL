############################################################
#   Define three install functions for installing custom targets
#   The install functions include
#   --  InstallTarget
#       group_name_ target_name_ headers_
#
#   --  function_InstallTarget          (Currently Used)
#       TARGET_NAME PACKAGE_NAME PACKAGE_INTERFACE_HEADERS

#   --  find_prebuilt_package           (Currently Used)
#       PACKAGE_NAME COMPONENTS

#   Written by Tang Qing in Jan. 2026.
############################################################


include( CMakePackageConfigHelpers )
# include( CMakeMessageColorSetting )

option(SMROBOT_SDK_INSTALL_PDB
    "Install MSVC PDB files for shared SDK targets when they are generated."
    ON
)
set(SMROBOT_SDK_PDB_CONFIGURATIONS "Debug;RelWithDebInfo;Release" CACHE STRING
    "CMake configurations whose generated MSVC PDB files are installed into the SDK.")


function( InstallTarget group_name_ target_name_ headers_ )
    message( "${BoldYellow}-- Install ${group_name_}::${target_name_} ${ColourReset}" )
    message( "Configurations: $<CONFIG>" )

    #   1. Install the target for different configurations
    foreach( CONFIG ${CMAKE_CONFIGURATION_TYPES} )
        #   1.1. Install the library files
        install( TARGETS ${target_name_}
            EXPORT ${target_name_}_${CONFIG}_target
            CONFIGURATIONS ${CONFIG}
            ARCHIVE DESTINATION ${group_name_}/lib
                COMPONENT smrobot_sdk_${group_name_}_${target_name_}
            LIBRARY DESTINATION ${group_name_}/lib
                COMPONENT smrobot_sdk_${group_name_}_${target_name_}
            RUNTIME DESTINATION ${group_name_}/bin
                COMPONENT smrobot_sdk_${group_name_}_${target_name_}
            INCLUDES DESTINATION ${group_name_}/include
            # PUBLIC_HEADER DESTINATION ${group_name_}/include
        )

        # 1.2. Install the "Targets.cmake"
        install( 
            EXPORT ${target_name_}_${CONFIG}_target
            CONFIGURATIONS ${CONFIG}
            FILE ${target_name_}Targets.cmake
            NAMESPACE ${group_name_}::
            DESTINATION  ${group_name_}/lib/cmake/${target_name_}
        )
    endforeach()

    #   2. Install the headers
    if( EXISTS ${headers_} )
        message( STATUS "Directory exists: ${headers_}" )
        install( 
            DIRECTORY ${headers_}
            DESTINATION ${group_name_}/include/${target_name_}
        )
    else()
        message( STATUS "Directory does not exist: ${headers_}" )
    endif()


    #   3. Install pdb files on windows platform for debug configuration
    if( CMAKE_SYSTEM_NAME MATCHES "Linux")
        #   Needed only in Windows System
        message( STATUS "No pdb file install is needed!" )
    elseif ( CMAKE_SYSTEM_NAME MATCHES "Windows")
        if( MSVC )
            get_target_property( target_type ${target_name_} TYPE )
            if ( target_type STREQUAL SHARED_LIBRARY )
                if(SMROBOT_SDK_INSTALL_PDB AND SMROBOT_SDK_PDB_CONFIGURATIONS)
                    install( FILES $<TARGET_PDB_FILE:${target_name_}>
                        DESTINATION ${group_name_}/bin
                        CONFIGURATIONS ${SMROBOT_SDK_PDB_CONFIGURATIONS}
                        OPTIONAL
                    )
                endif()
            endif()
        endif()
    endif ()
endfunction()




function(find_prebuilt_package)
    # Define parameters
    set(oneValueArgs PACKAGE_NAME)
    set(multiValueArgs COMPONENTS)
    
    # Parse arguments
    cmake_parse_arguments(
        ARG
        ""
        "${oneValueArgs}"
        "${multiValueArgs}"
        ${ARGN}
    )
    
    if(NOT ARG_PACKAGE_NAME)
        message(FATAL_ERROR "PACKAGE_NAME parameter is required")
    endif()
    
    if(NOT ARG_COMPONENTS)
        message(WARNING "No components specified, will try to find default components")
    endif()
    
    # Use variables
    set(packagename "${ARG_PACKAGE_NAME}")
    set(components_list "${ARG_COMPONENTS}")
    list(REMOVE_ITEM components_list "${packagename}")
    
    # Construct variable names
    set(using_var "UsingPrebuilt_${packagename}")
    set(build_var "BuildPackage_${packagename}")
    set(dir_var "${packagename}_Dir")
    set(components_var "${packagename}_Components")
    set(found_var "${packagename}_Found_Components")
    
    message(STATUS "=== Processing package: ${packagename} ===")
    
    if(${using_var})
        message(STATUS "Using prebuilt package: ${packagename}")
        
        # Set variables for parent scope
        set(${build_var} OFF PARENT_SCOPE)
        set(${components_var} "${components_list}" PARENT_SCOPE)
        
        if(SMROBOT_RESOLVED_PREBUILT_PACKAGE_ROOT)
            set(prebuilt_package_root "${SMROBOT_RESOLVED_PREBUILT_PACKAGE_ROOT}")
        elseif(SMROBOT_PREBUILT_PACKAGE_ROOT)
            set(prebuilt_package_root "${SMROBOT_PREBUILT_PACKAGE_ROOT}")
        else()
            set(prebuilt_package_root "${CMAKE_INSTALL_PREFIX}")
        endif()

        # Set search path - also create a local copy
        set(search_path "${prebuilt_package_root}/${packagename}/lib/cmake")
        set(${dir_var} "${search_path}" PARENT_SCOPE)  # Set to parent scope
        
        message(STATUS "Prebuilt package root: ${prebuilt_package_root}")
        message(STATUS "Search path: ${search_path}")
        message(STATUS "Search components: ${components_list}")

        set(missing_components_list)
        foreach(component IN LISTS components_list)
            if((NOT DEFINED ${packagename}_ConfigOnlyPrebuiltPackage
                    OR NOT ${${packagename}_ConfigOnlyPrebuiltPackage})
               AND NOT EXISTS "${search_path}/${component}/${component}Targets.cmake")
                list(APPEND missing_components_list "${component}")
            endif()
        endforeach()

        if(missing_components_list)
            message(FATAL_ERROR
                "Prebuilt package ${packagename} is missing required components: "
                "${missing_components_list}. "
                "Regenerate and install ${packagename} with the selected export profile before using "
                "UsingPrebuilt_${packagename}=ON."
            )
        endif()
        
        # Add package directory to CMAKE_PREFIX_PATH before calling find_package
        # This ensures find_package can locate the configuration files correctly
        list(APPEND CMAKE_PREFIX_PATH "${CMAKE_INSTALL_PREFIX}")
        list(APPEND CMAKE_PREFIX_PATH "${search_path}")
        
        # Find the package
        find_package(${packagename} 
            CONFIG 
            COMPONENTS ${components_list}
            REQUIRED 
            PATHS "${search_path}"
            NO_DEFAULT_PATH
        )
        
        if(${packagename}_FOUND)
            message(STATUS "[OK]${packagename} found!")
            
            # Initialize found components list
            set(found_components_list "")
            
            foreach(component IN LISTS components_list)
                set(target_name "${packagename}::${component}")
                
                if(TARGET ${target_name})
                    message(STATUS "  [OK]Component: ${target_name}")
                    list(APPEND found_components_list "${target_name}")
                    
                    # Print target properties
                    cmake_print_properties(
                        TARGETS ${target_name}
                        PROPERTIES 
                            POSITION_INDEPENDENT_CODE
                            INTERFACE_INCLUDE_DIRECTORIES
                            IMPORTED_LOCATION_DEBUG
                            IMPORTED_IMPLIB_DEBUG
                            IMPORTED_LOCATION_RELEASE
                            IMPORTED_IMPLIB_RELEASE
                            IMPORTED_CONFIGURATIONS
                    )
                else()
                    message(STATUS "  [NO]Component not found: ${target_name}")
                endif()
            endforeach()
            
            # Set found components list to parent scope
            set(${found_var} "${found_components_list}" PARENT_SCOPE)
            
        else()
            message(WARNING "${packagename} not found!")
        endif()
        
    else()
        message(STATUS "Not using prebuilt package: ${packagename}, will build from source")
        set(${build_var} ON PARENT_SCOPE)
    endif()
    
    message(STATUS "")
endfunction()


function(_smrobot_export_private_runtime_dependencies TARGET_NAME)
    set(_runtime_queue ${ARGN})
    set(_runtime_visited)
    set(_runtime_names_debug)
    set(_runtime_names_release)

    while(_runtime_queue)
        list(POP_FRONT _runtime_queue _runtime_candidate)

        if(_runtime_candidate MATCHES "^\\$<LINK_ONLY:([^>]+)>$")
            set(_runtime_candidate "${CMAKE_MATCH_1}")
        elseif(_runtime_candidate MATCHES "^\\$<(BUILD|INSTALL)_INTERFACE:([^>]+)>$")
            set(_runtime_candidate "${CMAKE_MATCH_2}")
        elseif(_runtime_candidate MATCHES "^\\$<")
            continue()
        endif()

        if(NOT TARGET "${_runtime_candidate}")
            continue()
        endif()

        list(FIND _runtime_visited "${_runtime_candidate}" _runtime_visited_index)
        if(NOT _runtime_visited_index EQUAL -1)
            continue()
        endif()
        list(APPEND _runtime_visited "${_runtime_candidate}")

        get_target_property(_runtime_type "${_runtime_candidate}" TYPE)
        if(_runtime_type STREQUAL "SHARED_LIBRARY" OR
           _runtime_type STREQUAL "MODULE_LIBRARY")
            foreach(_runtime_config DEBUG RELEASE)
                get_target_property(
                    _runtime_file
                    "${_runtime_candidate}"
                    "IMPORTED_LOCATION_${_runtime_config}"
                )
                if(NOT _runtime_file OR _runtime_file MATCHES "-NOTFOUND$")
                    get_target_property(
                        _runtime_file
                        "${_runtime_candidate}"
                        "IMPORTED_IMPLIB_${_runtime_config}"
                    )
                endif()

                if(_runtime_file AND NOT _runtime_file MATCHES "-NOTFOUND$")
                    get_filename_component(_runtime_extension "${_runtime_file}" EXT)
                    if(_runtime_extension STREQUAL ".lib")
                        string(REGEX REPLACE "\\.lib$" ".dll" _runtime_file "${_runtime_file}")
                    endif()
                    get_filename_component(_runtime_name "${_runtime_file}" NAME)
                    string(TOLOWER "${_runtime_config}" _runtime_config_lower)
                    list(APPEND
                        _runtime_names_${_runtime_config_lower}
                        "${_runtime_name}"
                    )
                endif()
            endforeach()
        endif()

        get_target_property(
            _runtime_interface_dependencies
            "${_runtime_candidate}"
            INTERFACE_LINK_LIBRARIES
        )
        if(_runtime_interface_dependencies AND
           NOT _runtime_interface_dependencies MATCHES "-NOTFOUND$")
            list(APPEND _runtime_queue ${_runtime_interface_dependencies})
        endif()
    endwhile()

    list(REMOVE_DUPLICATES _runtime_names_debug)
    list(REMOVE_DUPLICATES _runtime_names_release)

    if(_runtime_names_debug OR _runtime_names_release)
        set_property(TARGET "${TARGET_NAME}" PROPERTY
            SMROBOT_PRIVATE_RUNTIME_DLLS_DEBUG "${_runtime_names_debug}")
        set_property(TARGET "${TARGET_NAME}" PROPERTY
            SMROBOT_PRIVATE_RUNTIME_DLLS_RELEASE "${_runtime_names_release}")
        set_property(TARGET "${TARGET_NAME}" APPEND PROPERTY EXPORT_PROPERTIES
            SMROBOT_PRIVATE_RUNTIME_DLLS_DEBUG
            SMROBOT_PRIVATE_RUNTIME_DLLS_RELEASE
        )
        message(STATUS
            " Export private runtime DLL metadata for ${TARGET_NAME}: "
            "Debug=[${_runtime_names_debug}], Release=[${_runtime_names_release}]")
    endif()
endfunction()


function(function_InstallTarget)
    # Define function arguments
    set(oneValueArgs TARGET_NAME PACKAGE_NAME PACKAGE_INTERFACE_HEADERS)
    set(multiValueArgs PRIVATE_RUNTIME_DEPENDENCIES)
    
    # Parse arguments
    cmake_parse_arguments(
        ARG
        ""
        "${oneValueArgs}"
        "${multiValueArgs}"
        ${ARGN}
    )
    
    # Validate required arguments
    if(NOT ARG_TARGET_NAME)
        message(FATAL_ERROR "TARGET_NAME parameter is required for function_InstallTarget")
    endif()
    
    if(NOT ARG_PACKAGE_NAME)
        message(FATAL_ERROR "PACKAGE_NAME parameter is required for function_InstallTarget")
    endif()

    if(NOT ARG_PACKAGE_INTERFACE_HEADERS)
        message(WARNING "PACKAGE_INTERFACE_HEADERS parameter is NOT found for function_InstallTarget")
    endif()
    
    # Use local variables
    set(TARGET_NAME "${ARG_TARGET_NAME}")
    set(PACKAGE_NAME "${ARG_PACKAGE_NAME}")
    set(INTERFACE_HEADERS "${ARG_PACKAGE_INTERFACE_HEADERS}")
    set(SDK_TARGET_INSTALL_COMPONENT "smrobot_sdk_${PACKAGE_NAME}_${TARGET_NAME}")
    
    if(TARGET ${TARGET_NAME})
        message(STATUS " Target ${TARGET_NAME} exists")
        
        set(INSTALL_VAR_NAME "ExportPacakge${PACKAGE_NAME}_${TARGET_NAME}")
        if(DEFINED ${PACKAGE_NAME}_ExportComponents)
            set(PACKAGE_ALLOWED_COMPONENTS ${${PACKAGE_NAME}_ExportComponents})
        else()
            set(PACKAGE_ALLOWED_COMPONENTS ${${PACKAGE_NAME}_PublicComponents} ${${PACKAGE_NAME}_CompatibilityComponents})
        endif()
        
        if(NOT DEFINED ${INSTALL_VAR_NAME})
            list(FIND PACKAGE_ALLOWED_COMPONENTS ${TARGET_NAME} TARGET_MANIFEST_INDEX)
            if(TARGET_MANIFEST_INDEX EQUAL -1)
                message(STATUS " ${INSTALL_VAR_NAME} is undefined, setting to OFF because ${TARGET_NAME} is not in ${PACKAGE_NAME} release manifest")
                set(${INSTALL_VAR_NAME} OFF)
            else()
                message(STATUS " ${INSTALL_VAR_NAME} is undefined, setting to ON because ${TARGET_NAME} is in ${PACKAGE_NAME} release manifest")
                set(${INSTALL_VAR_NAME} ON)
            endif()
            set(${INSTALL_VAR_NAME} ${${INSTALL_VAR_NAME}} PARENT_SCOPE)
        endif()
        
        set(INSTALL_VAR_VALUE ${${INSTALL_VAR_NAME}})

        if(INSTALL_VAR_VALUE)
            list(FIND PACKAGE_ALLOWED_COMPONENTS ${TARGET_NAME} TARGET_MANIFEST_INDEX)
            if(TARGET_MANIFEST_INDEX EQUAL -1)
                message(WARNING " ${PACKAGE_NAME}::${TARGET_NAME} is requested for export but is not listed in ${PACKAGE_NAME}_ExportComponents. Skip installation.")
                set(INSTALL_VAR_VALUE OFF)
            endif()
        endif()
        
        message(STATUS " ${INSTALL_VAR_NAME} = '${INSTALL_VAR_VALUE}'")
        
        if(INSTALL_VAR_VALUE)
            message(STATUS " Installation conditions met, proceeding with installation")

            if(ARG_PRIVATE_RUNTIME_DEPENDENCIES)
                _smrobot_export_private_runtime_dependencies(
                    "${TARGET_NAME}"
                    ${ARG_PRIVATE_RUNTIME_DEPENDENCIES}
                )
            endif()
            
            # Install target files
            install(TARGETS ${TARGET_NAME}
                EXPORT ${TARGET_NAME}Targets
                ARCHIVE DESTINATION ${PACKAGE_NAME}/lib
                    COMPONENT ${SDK_TARGET_INSTALL_COMPONENT}
                LIBRARY DESTINATION ${PACKAGE_NAME}/lib
                    COMPONENT ${SDK_TARGET_INSTALL_COMPONENT}
                RUNTIME DESTINATION ${PACKAGE_NAME}/bin
                    COMPONENT ${SDK_TARGET_INSTALL_COMPONENT}
                INCLUDES DESTINATION ${PACKAGE_NAME}/include
            )
            
            # Fix: Use correct header file source path
            # set(HEADER_SOURCE_DIR "${PROJECT_SOURCE_DIR}/include/${PACKAGE_NAME}/${TARGET_NAME}")
            set(HEADER_SOURCE_DIR "${INTERFACE_HEADERS}")            
            message(STATUS " Looking for headers in: ${HEADER_SOURCE_DIR}")
            
            if(IS_DIRECTORY "${HEADER_SOURCE_DIR}")
                message(STATUS " Installing headers from: ${HEADER_SOURCE_DIR}")
                
                # Install header files - using correct source and destination paths
                install(DIRECTORY "${HEADER_SOURCE_DIR}/"
                    # DESTINATION "${PACKAGE_NAME}/include/${TARGET_NAME}"
                    DESTINATION "${PACKAGE_NAME}/include"
                    COMPONENT ${SDK_TARGET_INSTALL_COMPONENT}
                )
            else()
                message(WARNING " Header source directory not found: ${HEADER_SOURCE_DIR}")
                
                # Create empty directory to avoid installation failure
                install(CODE "
                    message(STATUS \" Creating empty header directory for ${TARGET_NAME}\")
                    execute_process(
                        COMMAND \${CMAKE_COMMAND} -E make_directory 
                        \"\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/${PACKAGE_NAME}/include/${TARGET_NAME}\"
                    )
                " COMPONENT ${SDK_TARGET_INSTALL_COMPONENT})
            endif()
            
            # Install export files
            install(EXPORT ${TARGET_NAME}Targets
                FILE ${TARGET_NAME}Targets.cmake
                NAMESPACE ${PACKAGE_NAME}::
                DESTINATION "${PACKAGE_NAME}/lib/cmake/${TARGET_NAME}"
                COMPONENT ${SDK_TARGET_INSTALL_COMPONENT}
            )
            
            # Generate dependency configuration file
            get_target_property(ALL_DEPS ${TARGET_NAME} INTERFACE_LINK_LIBRARIES)
            message(STATUS " Interface dependencies for ${TARGET_NAME}: ${ALL_DEPS}")
            
            if(NOT ALL_DEPS)
                set(ALL_DEPS "")
            endif()

            set(PUBLIC_DEPS)
            if(DEFINED ${TARGET_NAME}_INSTALL_PUBLIC_DEPENDENCIES)
                set(PUBLIC_DEPS ${${TARGET_NAME}_INSTALL_PUBLIC_DEPENDENCIES})
            elseif(ALL_DEPS)
                set(INSTALL_DEPS "${ALL_DEPS}")
                string(REPLACE "$<BUILD_INTERFACE:" "" INSTALL_DEPS "${INSTALL_DEPS}")
                string(REPLACE "$<INSTALL_INTERFACE:" "" INSTALL_DEPS "${INSTALL_DEPS}")
                string(REPLACE "$<LINK_ONLY:" "" INSTALL_DEPS "${INSTALL_DEPS}")
                string(REPLACE ">" "" INSTALL_DEPS "${INSTALL_DEPS}")
                foreach(DEP IN LISTS INSTALL_DEPS)
                    if(DEP MATCHES "::" AND NOT DEP MATCHES "NOTFOUND")
                        list(APPEND PUBLIC_DEPS "${DEP}")
                    endif()
                endforeach()
                list(REMOVE_DUPLICATES PUBLIC_DEPS)
            endif()
            
            # Generate dependency configuration file content
            set(DEPENDENCY_CONFIG_CONTENT "
# Auto-generated dependencies for ${TARGET_NAME}
set(${TARGET_NAME}_PUBLIC_DEPENDENCIES \"${PUBLIC_DEPS}\")
set(${TARGET_NAME}_DEPENDENCY_FILE_DIR \"\${CMAKE_CURRENT_LIST_DIR}\")

function(_${TARGET_NAME}_include_smrobot_dependency_bootstrap)
    if(COMMAND smrobot_dependency_bootstrap)
        smrobot_dependency_bootstrap()
        set(CMAKE_PREFIX_PATH \"\${CMAKE_PREFIX_PATH}\" PARENT_SCOPE)
        return()
    endif()

    set(_smrobot_bootstrap_search_dir \"\${CMAKE_CURRENT_LIST_DIR}\")
    foreach(_smrobot_bootstrap_depth RANGE 0 8)
        if(EXISTS \"\${_smrobot_bootstrap_search_dir}/cmake/SMRobotDependencyBootstrap.cmake\")
            include(\"\${_smrobot_bootstrap_search_dir}/cmake/SMRobotDependencyBootstrap.cmake\")
            if(COMMAND smrobot_dependency_bootstrap)
                smrobot_dependency_bootstrap()
                set(CMAKE_PREFIX_PATH \"\${CMAKE_PREFIX_PATH}\" PARENT_SCOPE)
            endif()
            return()
        endif()

        get_filename_component(_smrobot_next_bootstrap_search_dir \"\${_smrobot_bootstrap_search_dir}/..\" ABSOLUTE)
        if(_smrobot_next_bootstrap_search_dir STREQUAL _smrobot_bootstrap_search_dir)
            break()
        endif()
        set(_smrobot_bootstrap_search_dir \"\${_smrobot_next_bootstrap_search_dir}\")
    endforeach()
endfunction()

if(DEFINED ${TARGET_NAME}_PUBLIC_DEPENDENCIES AND NOT \"\${${TARGET_NAME}_PUBLIC_DEPENDENCIES}\" STREQUAL \"\")
    set(${TARGET_NAME}_DEPENDENCY_NAMESPACES)
    foreach(dep IN LISTS ${TARGET_NAME}_PUBLIC_DEPENDENCIES)
        string(REGEX REPLACE \"::.*\" \"\" namespace \"\${dep}\")
        list(APPEND ${TARGET_NAME}_DEPENDENCY_NAMESPACES \"\${namespace}\")
    endforeach()
    list(REMOVE_DUPLICATES ${TARGET_NAME}_DEPENDENCY_NAMESPACES)
endif()

function(resolve_${TARGET_NAME}_dependencies)
    if(${TARGET_NAME}_RESOLVING_DEPENDENCIES)
        return()
    endif()
    set(${TARGET_NAME}_RESOLVING_DEPENDENCIES TRUE)
    _${TARGET_NAME}_include_smrobot_dependency_bootstrap()

    if(DEFINED ${TARGET_NAME}_PUBLIC_DEPENDENCIES AND NOT \"\${${TARGET_NAME}_PUBLIC_DEPENDENCIES}\" STREQUAL \"\")
        foreach(dep IN LISTS ${TARGET_NAME}_PUBLIC_DEPENDENCIES)
            string(REGEX REPLACE \"::.*\" \"\" dep_namespace \"\${dep}\")
            string(REGEX REPLACE \".*::\" \"\" dep_component \"\${dep}\")
            if(NOT TARGET \${dep})
                set(_smrobot_dependency_from_source FALSE)
                if(DEFINED BuildPackage_\${dep_namespace}
                   AND BuildPackage_\${dep_namespace}
                   AND (NOT DEFINED UsingPrebuilt_\${dep_namespace}
                        OR NOT UsingPrebuilt_\${dep_namespace}))
                    set(_smrobot_dependency_from_source TRUE)
                endif()

                if(_smrobot_dependency_from_source)
                    # The superproject will define this target in a later add_subdirectory call.
                elseif(dep_namespace STREQUAL \"OpenGL\" AND dep_component STREQUAL \"GL\")
                    find_package(OpenGL QUIET COMPONENTS GL)
                    if(NOT TARGET OpenGL::GL AND WIN32)
                        add_library(OpenGL::GL INTERFACE IMPORTED)
                        set_target_properties(OpenGL::GL PROPERTIES
                            INTERFACE_LINK_LIBRARIES \"opengl32\")
                    endif()
                    if(NOT TARGET OpenGL::GL)
                        find_dependency(OpenGL REQUIRED COMPONENTS GL)
                    endif()
                elseif(dep_namespace STREQUAL \"${PACKAGE_NAME}\")
                    set(dep_dir \"\${${TARGET_NAME}_DEPENDENCY_FILE_DIR}/../\${dep_component}\")
                    if(EXISTS \"\${dep_dir}/\${dep_component}Dependencies.cmake\")
                        include(\"\${dep_dir}/\${dep_component}Dependencies.cmake\")
                        set(dep_resolve_func \"resolve_\${dep_component}_dependencies\")
                        if(COMMAND \${dep_resolve_func})
                            cmake_language(CALL \${dep_resolve_func})
                        endif()
                    endif()
                    if(EXISTS \"\${dep_dir}/\${dep_component}Targets.cmake\")
                        include(\"\${dep_dir}/\${dep_component}Targets.cmake\")
                    endif()
                else()
                    if(dep_namespace STREQUAL dep_component)
                        find_dependency(\${dep_namespace} REQUIRED)
                        if(NOT TARGET \${dep} AND TARGET \${dep_component})
                            add_library(\${dep} ALIAS \${dep_component})
                        endif()
                    else()
                        find_dependency(\${dep_namespace} REQUIRED COMPONENTS \${dep_component})
                    endif()
                endif()
                unset(_smrobot_dependency_from_source)
            endif()
        endforeach()
    endif()

    set(${TARGET_NAME}_RESOLVING_DEPENDENCIES FALSE)
endfunction()
")
            
            # Write dependency configuration file
            set(DEPENDENCY_FILE "${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAME}Dependencies.cmake")
            file(WRITE "${DEPENDENCY_FILE}" "${DEPENDENCY_CONFIG_CONTENT}")
            
            # Install dependency configuration file
            install(FILES
                "${DEPENDENCY_FILE}"
                DESTINATION "${PACKAGE_NAME}/lib/cmake/${TARGET_NAME}"
                COMPONENT ${SDK_TARGET_INSTALL_COMPONENT}
            )
            
            #   3. Install pdb files on windows platform for debug configuration
            if( CMAKE_SYSTEM_NAME MATCHES "Linux")
                #   Needed only in Windows System
                message( STATUS "No pdb file install is needed!" )
            elseif ( CMAKE_SYSTEM_NAME MATCHES "Windows")
                if( MSVC )
                    get_target_property( target_type ${TARGET_NAME} TYPE )
                    if ( target_type STREQUAL SHARED_LIBRARY )
                        if(SMROBOT_SDK_INSTALL_PDB AND SMROBOT_SDK_PDB_CONFIGURATIONS)
                            install( FILES $<TARGET_PDB_FILE:${TARGET_NAME}>
                                DESTINATION ${PACKAGE_NAME}/bin
                                CONFIGURATIONS ${SMROBOT_SDK_PDB_CONFIGURATIONS}
                                OPTIONAL
                                COMPONENT ${SDK_TARGET_INSTALL_COMPONENT}
                            )
                        endif()
                        install( FILES $<TARGET_RUNTIME_DLLS:${TARGET_NAME}>
                            DESTINATION ${PACKAGE_NAME}/bin
                            OPTIONAL
                            COMPONENT ${SDK_TARGET_INSTALL_COMPONENT}
                        )
                    endif()
                endif()
            endif ()

            message(STATUS " Installation completed for ${TARGET_NAME}")
            
        else()
            message(STATUS " Installation conditions not met: ${INSTALL_VAR_NAME}='${INSTALL_VAR_VALUE}'")
        endif()

        
    else()
        message(STATUS " Target ${TARGET_NAME} does not exist")
    endif()
endfunction()


