######################################################################################
#   Copy runtime for rs distibution (Important for deployment)
######################################################################################

#   1. Define the copy function
include( FuncDef_Copy2Release )

#   2. Designate the copy targets for each type.
######################################################################################
#   2.1. Copy custom packages to the release/debug directory. 
######################################################################################
#   Include the customized package working directories
if( ${UsingPrebuild_SMRobotGen1} )
    list( APPEND custom_targets
        Common::CustomLog
        SMRobotGen1::CameraTangQing
        SMRobotGen1::ModelTangQing
        SMRobotGen1::SMRobotTangQing
        SMRobotGen1::BasicSystemTangQing
    )
endif()

if( ${UsingPrebuild_SMRobotGen2} )
    list( APPEND custom_targets
        SMRobotGen2::rs_camera
        SMRobotGen2::rs_color
        SMRobotGen2::rs_shader
        SMRobotGen2::rs_model_data
        SMRobotGen2::rs_glmodel
    )
endif()

if( ${UsingPrebuild_QtComponents} )
    list( APPEND custom_targets
        QtComponents::QtAssemblyTreeView
        QtComponents::QtGLGMMenu
        QtComponents::QtOpenGLView
        QtComponents::QtRobotControlMenu
        QtComponents::QtRobotTreeView
    )
endif()

option( Copy_CustomRuntime2BuildDir "Copy license files to build directory! Do once for deployment!" OFF )
if( ${Copy_CustomRuntime2BuildDir} )
    message( STATUS "\n\n-----------------------------------------------------------------------------" )
    message( STATUS "Copying custom runtime files to release/debug directory ..." )
    message( STATUS "-----------------------------------------------------------------------------" )

    #   Copy each QT targets' runtime to release binary directory.
    foreach( target_ ${qt_targets} )
        Copy_ReleaseRuntime2WorkingDir( ${target_} )
        Copy_DebugRuntime2WorkingDir( ${target_} )
    endforeach()

    set( Copy_CustomRuntime2BuildDir OFF CACHE BOOL "Override Copy_CustomRuntime2BuildDir to OFF" FORCE )
endif()

######################################################################################
#   2.2. Copy shader files to the release/debug directory. Do once for deployment!
######################################################################################
option( Copy_ShaderFiles2BuildDir "Copy shader files to build directory" OFF )
if( ${Copy_ShaderFiles2BuildDir} )
    file( COPY ${SMROBOT_DATA_ROOT}/shader_gen1 DESTINATION ${PROJECT_BINARY_DIR} )
    file( COPY ${SMROBOT_DATA_ROOT}/shader_gen2 DESTINATION ${PROJECT_BINARY_DIR} )

    set( Copy_ShaderFiles2BuildDir OFF CACHE BOOL "Override Copy_ShaderFiles2BuildDir to OFF" FORCE )
endif()


######################################################################################
#   2.3. Copy license files to build directory! Do once for deployment!
######################################################################################
option( Copy_License2BuildDir "Copy license files to build directory! Do once for deployment!" OFF )
if( ${Copy_License2BuildDir} )
    #   Send copy license message
    message( STATUS "Copying license from ${PROJECT_SOURCE_DIR}/license to build directory ${PROJECT_BINARY_DIR}." )

    #   Copy license to build directory
    file( COPY ${PROJECT_SOURCE_DIR}/license DESTINATION ${PROJECT_BINARY_DIR} )

    #   Unset the variable
    set( Copy_License2BuildDir OFF CACHE BOOL "Override Copy_License2BuildDir to OFF" FORCE )
endif()

