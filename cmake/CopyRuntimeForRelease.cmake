######################################################################################
#   Copy runtime for release configuration
######################################################################################


#   1. Define the copy function
include( FuncDef_Copy2Release )

#   2. Designate the copy targets for each type.

######################################################################################
#   2.1. Copy boost
######################################################################################
# #   Include all the boost targets
# list( APPEND boost_targets 
#     Boost::date_time
#     Boost::locale
#     Boost::filesystem
#     Boost::timer
#     Boost::thread
#     Boost::chrono
#     Boost::atomic
#     Boost::log
#     Boost::log_setup
# )

# #   Whether copy the release_targets' runtime to binary directory
# option( Copy_BoostRuntime2ReleaseDir "Whether copy packages with lib directory runtime file to release directory. (Such as Boost)" OFF )
# if( ${Copy_BoostRuntime2ReleaseDir} )

#     message( STATUS "\n\n-----------------------------------------------------------------------------" )
#     message( STATUS "Copying Boost release runtime files to release directory ..." )
#     message( STATUS "-----------------------------------------------------------------------------" )

#     #   Copy each boost targets' runtime to release binary directory.
#     foreach( target_ ${boost_targets} )
#         Copy_ReleaseRuntime2WorkingDir_byIMPLIB( ${target_} )
#         Copy_DebugRuntime2WorkingDir_byIMPLIB( ${target_} )
#     endforeach()

#     set( Copy_BoostRuntime2ReleaseDir OFF CACHE BOOL "Override Copy_BoostRuntime2ReleaseDir to OFF" FORCE )

# endif()

# Whether copy boost runtime to release directory
option(Copy_BoostRuntime2ReleaseDir
       "Copy Boost runtime dlls to release directory"
       OFF)

if(Copy_BoostRuntime2ReleaseDir)
    message(STATUS "")
    message(STATUS "-----------------------------------------------------------------------------")
    message(STATUS "Copying Boost runtime DLLs to release directory ...")
    message(STATUS "-----------------------------------------------------------------------------")

    foreach(lib ${Boost_DepLibs})
        # construct Boost target name
        set(target_ "Boost::${lib}")
        if(TARGET ${target_})
            # get release runtime dll
            get_target_property(_dll ${target_} IMPORTED_LOCATION_RELEASE)
            if(_dll)
                message(STATUS "Copy Boost runtime: ${_dll}")
                add_custom_command(
                    TARGET ${PROJECT_NAME}
                    POST_BUILD
                    COMMAND ${CMAKE_COMMAND} -E copy_if_different
                        "${_dll}"
                        "${PROJECT_BINARY_DIR}/Release/bin"
                )
            endif()
        else()
            message(WARNING "Boost target not found: ${target_}")
        endif()
    endforeach()

    # reset option
    set(Copy_BoostRuntime2ReleaseDir OFF
        CACHE BOOL
        "Override Copy_BoostRuntime2ReleaseDir to OFF"
        FORCE)

endif()

######################################################################################
#   2.2. Copy Qt
######################################################################################
list( APPEND qt_targets 
    Qt5::Core
    Qt5::Widgets
    Qt5::Gui
    Qt5::OpenGL
)

option( Copy_QtRuntimePlatform2ReleaseDir "Whether copy Qt5 platform files to release directory" OFF )
if( ${Copy_QtRuntimePlatform2ReleaseDir} )
    message( STATUS "\n\n-----------------------------------------------------------------------------" )
    message( STATUS "Copying Qt release runtime files and platform folder to release directory ..." )
    message( STATUS "-----------------------------------------------------------------------------" )

    #   Copy each QT targets' runtime to release binary directory.
    foreach( target_ ${qt_targets} )
        Copy_ReleaseRuntime2WorkingDir( ${target_} )
    endforeach()

    #   Copy the Qt Platform to release binary directory.
    Copy_QTPlatform2WorkingDir()

    set( Copy_QtRuntimePlatform2ReleaseDir OFF CACHE BOOL "Override Copy_QtRuntimePlatform2ReleaseDir to OFF" FORCE )
endif()

######################################################################################
#   2.3. Copy runtime in target IMPORTED_LOCATION_RELEASE
######################################################################################
list( APPEND location_targets 
    FreeGLUT::freeglut
)

#   Copy all the third-party runtimes to the release/debug directory
option( Copy_AllTargetRuntime2ReleaseDir "Copy dependant runtime files to release directory" OFF )
if( ${Copy_AllTargetRuntime2ReleaseDir} )
	foreach( target_ ${location_targets} )
		Copy_ReleaseRuntime2WorkingDir( ${target_} ) 
	endforeach()

    set( Copy_AllTargetRuntime2ReleaseDir OFF CACHE BOOL "Override Copy_AllTargetRuntime2ReleaseDir to OFF" FORCE )
endif()

# ######################################################################################
# #   2.4. Copy shader files to the release/debug directory. Important for deployment
# ######################################################################################
# option( Copy_ShaderFiles2BuildDir "Copy shader files to build directory" OFF )
# if( ${Copy_ShaderFiles2BuildDir} )
#     file( COPY ${PROJECT_SOURCE_DIR}/data/shader_gen1 DESTINATION ${PROJECT_BINARY_DIR} )
#     file( COPY ${PROJECT_SOURCE_DIR}/data/shader_gen2 DESTINATION ${PROJECT_BINARY_DIR} )

#     set( Copy_ShaderFiles2BuildDir OFF CACHE BOOL "Override Copy_ShaderFiles2BuildDir to OFF" FORCE )
# endif()


# ######################################################################################
# #   2.5. Copy license files to build directory! Do once for deployment!
# ######################################################################################
# option( Copy_License2BuildDir "Copy license files to build directory! Do once for deployment!" OFF )
# if( ${Copy_License2BuildDir} )
#     file( COPY ${PROJECT_SOURCE_DIR}/license DESTINATION ${PROJECT_BINARY_DIR} )

#     set( Copy_License2BuildDir OFF CACHE BOOL "Override Copy_License2BuildDir to OFF" FORCE )
# endif()

