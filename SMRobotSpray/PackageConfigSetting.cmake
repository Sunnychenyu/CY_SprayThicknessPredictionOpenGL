##########################################################################
# CMake requirement
##########################################################################
cmake_minimum_required( VERSION 3.20 )

##########################################################################
# ${PACKAGE_NAME} Configuration
##########################################################################
message( STATUS "!!!!!!Package ${PACKAGE_NAME} configuration is set here!!!!!!" )

##########################################################################
# Package ${PACKAGE_NAME} export components definition
##########################################################################
set( ${PACKAGE_NAME}_PublicComponents
    WorkpieceCore
    SprayCore
    SprayTrajectoryCore
    SprayThicknessPrediction
    SprayThicknessPredictionOpenGL
    SprayPathPlanning
    SprayTrajectoryOptimization
)

set( ${PACKAGE_NAME}_CompatibilityComponents )
set( ${PACKAGE_NAME}_TutorialDirs )
set( ${PACKAGE_NAME}_InstallDocs )

set( ${PACKAGE_NAME}_ExportComponents
    ${${PACKAGE_NAME}_PublicComponents}
    ${${PACKAGE_NAME}_CompatibilityComponents}
)

##########################################################################
# Package ${PACKAGE_NAME} components definition
##########################################################################
set( ${PACKAGE_NAME}_Components
    WorkpieceCore
    SprayCore
    SprayTrajectoryCore
    SprayThicknessPrediction
    SprayThicknessPredictionOpenGL
    SprayPathPlanning
    SprayTrajectoryOptimization
)
message( STATUS "==================================================================" )
message( STATUS "All Targets in Package ${PACKAGE_NAME}: ${${PACKAGE_NAME}_Components}" )
message( STATUS "==================================================================\n" )

set( ${PACKAGE_NAME}_InternalComponents ${${PACKAGE_NAME}_Components} )
foreach( public_comp_ ${${PACKAGE_NAME}_ExportComponents} )
    list( REMOVE_ITEM ${PACKAGE_NAME}_InternalComponents ${public_comp_} )
endforeach()

foreach( comp_ ${${PACKAGE_NAME}_Components} )
    list( FIND ${PACKAGE_NAME}_ExportComponents ${comp_} export_index_ )
    if( export_index_ EQUAL -1 )
        set( ExportPacakge${PACKAGE_NAME}_${comp_} OFF CACHE BOOL "Do not export internal ${PACKAGE_NAME}::${comp_}" FORCE )
    else()
        set( ExportPacakge${PACKAGE_NAME}_${comp_} ON CACHE BOOL "Export public ${PACKAGE_NAME}::${comp_}" FORCE )
    endif()
endforeach()

##########################################################################
# Package ${PACKAGE_NAME} required targets
##########################################################################
foreach( TARGET_NAME ${${PACKAGE_NAME}_Components} )
    include( ${PACKAGE_NAME}/${TARGET_NAME}/TargetConfigSetting )

    list( APPEND ${PACKAGE_NAME}_RequiredLibsPublic ${${TARGET_NAME}_RequiredLibsPublic} )
    list( APPEND ${PACKAGE_NAME}_RequiredLibsPrivate ${${TARGET_NAME}_RequiredLibsPrivate} )

    set( ${PACKAGE_NAME}${TARGET_NAME}_RequiredLibsPublic ${${TARGET_NAME}_RequiredLibsPublic} )
    set( ${PACKAGE_NAME}${TARGET_NAME}_RequiredLibsPrivate ${${TARGET_NAME}_RequiredLibsPrivate} )
endforeach()

list( REMOVE_DUPLICATES ${PACKAGE_NAME}_RequiredLibsPublic )
list( REMOVE_DUPLICATES ${PACKAGE_NAME}_RequiredLibsPrivate )
list( APPEND ${PACKAGE_NAME}_RequiredLibs ${${PACKAGE_NAME}_RequiredLibsPublic} ${${PACKAGE_NAME}_RequiredLibsPrivate} )
list( REMOVE_DUPLICATES ${PACKAGE_NAME}_RequiredLibs )
resolve_dependencies( ${PACKAGE_NAME}_RequiredLibs )

message( STATUS "AllPackagesRequired is ${AllPackagesRequired}" )
foreach( pkg ${AllPackagesRequired} )
    message( STATUS "Finding ${pkg} with components: ${${pkg}_Components}" )
endforeach()
