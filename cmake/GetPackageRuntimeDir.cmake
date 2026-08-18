#############################################################
#	Find the runtime directory for given targets.
#   Written by Tang Qing in Mar. 2024.
#############################################################


if ( CMAKE_SYSTEM_NAME MATCHES "Windows" )
    message( STATUS "\n\n runtime_targets are ${runtime_targets}" )

    if( NOT VSDebugEnvPath )
        message( STATUS "VSDebugEnvPath is NOT defined!" )
        set( VSDebugEnvPath "PATH=%PATH%" )
    else()
        message( STATUS "VSDebugEnvPath is defined as ${VSDebugEnvPath}" )
    endif()

    foreach( _target ${runtime_targets} )
        get_target_property( ${_target}_RUMTIME_FILE ${_target} IMPORTED_LOCATION_DEBUG )
        string( REGEX REPLACE "(.+/)(.+)\\..*"  "\\1"  ${_target}_RUNTIME_DIR  ${${_target}_RUMTIME_FILE} )
        string( APPEND VSDebugEnvPath ";" ${${_target}_RUNTIME_DIR} )
        message( STATUS "${_target}_RUNTIME_DIR is ${${_target}_RUNTIME_DIR}" )
    endforeach()
    message( STATUS "VSDebugEnvPath is ${VSDebugEnvPath}" )
else ()
    message( STATUS "Skip config VSDebugEnvPath for platform: ${CMAKE_SYSTEM_NAME}" )
endif ()



