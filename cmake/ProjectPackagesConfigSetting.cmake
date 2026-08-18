############################################################
#   Package Config Setting
############################################################

############################################################
#   1. Get all package names
############################################################
set( ExcludeBuildPackages
    cmake data build out .git .idea Prune doc .cache .codex .planning
    PrebuiltPackages PackagesInstallation cmake_bk
    cmake_upgrade config include license thirdparty
    external archives archive
    patchs plans docs log generated_assets
    tests
    tutorials
    scripts
    SimWorkbench
    # Algorithm
    Applications
    # Common
    # QtComponents
    RadiationTherapySystem
    # SMRobotGen1
    SMRobotGen2
    Test
    # Tutorials
    # Verification

    #   For Package Build
    PackageBuildFromSource

    #   For Ubuntu Qt
    .qtcreator
    .agents
    .codex
    phase1_release
)

message( STATUS "ExcludeBuildPackages is : ${ExcludeBuildPackages}" )
collect_subdirs(
    ${CMAKE_CURRENT_SOURCE_DIR}
    AllPackageNames
    EXCLUDE ${ExcludeBuildPackages}
)

set( AllBuildPackages ${AllPackageNames} )
list( APPEND AllBuildPackages ${AdditionalBuildPackages} )
list( REMOVE_DUPLICATES AllBuildPackages )
message( STATUS "------------------------------------------------------------------" )
message( STATUS "All Build Packages: ${AllBuildPackages}" )
message( STATUS "------------------------------------------------------------------\n" )

############################################################
#   2. Get all prebuilt package names
############################################################
set(SMROBOT_PREBUILT_PACKAGE_SELECTION_MODE "Explicit" CACHE STRING
    "Prebuilt package selection mode: Explicit or LegacyAuto.")
set_property(CACHE SMROBOT_PREBUILT_PACKAGE_SELECTION_MODE PROPERTY STRINGS Explicit LegacyAuto)
if(NOT SMROBOT_PREBUILT_PACKAGE_SELECTION_MODE MATCHES "^(Explicit|LegacyAuto)$")
    message(FATAL_ERROR
        "SMROBOT_PREBUILT_PACKAGE_SELECTION_MODE must be Explicit or LegacyAuto.")
endif()

set(SMROBOT_PREBUILT_PACKAGE_ROOT "" CACHE PATH
    "Explicit SDK install prefix used when resolving UsingPrebuilt_<Package>=ON.")

set(_smrobot_default_legacy_prebuilt_root "${CMAKE_CURRENT_SOURCE_DIR}/PrebuiltPackages")
set(_smrobot_prebuilt_package_root "")
if(SMROBOT_PREBUILT_PACKAGE_ROOT)
    get_filename_component(_smrobot_prebuilt_package_root
        "${SMROBOT_PREBUILT_PACKAGE_ROOT}" ABSOLUTE)
elseif(SMROBOT_PREBUILT_PACKAGE_SELECTION_MODE STREQUAL "LegacyAuto")
    set(_smrobot_prebuilt_package_root "${_smrobot_default_legacy_prebuilt_root}")
else()
    get_filename_component(_smrobot_current_install_prefix
        "${CMAKE_INSTALL_PREFIX}" ABSOLUTE)
    get_filename_component(_smrobot_default_install_prefix
        "${_smrobot_default_legacy_prebuilt_root}" ABSOLUTE)
    if(NOT _smrobot_current_install_prefix STREQUAL _smrobot_default_install_prefix)
        set(_smrobot_prebuilt_package_root "${_smrobot_current_install_prefix}")
    endif()
    unset(_smrobot_current_install_prefix)
    unset(_smrobot_default_install_prefix)
endif()

if(_smrobot_prebuilt_package_root)
    set(SMROBOT_RESOLVED_PREBUILT_PACKAGE_ROOT
        "${_smrobot_prebuilt_package_root}" CACHE INTERNAL
        "Resolved SDK install prefix used for prebuilt package consumption." FORCE)
else()
    unset(SMROBOT_RESOLVED_PREBUILT_PACKAGE_ROOT CACHE)
endif()

set( ExcludePrebuiltPackages
    bin
    cmake
    include
    lib
    thirdparty
)
message( STATUS "ExcludePrebuiltPackages is : ${ExcludePrebuiltPackages}" )
message( STATUS "SMROBOT_PREBUILT_PACKAGE_SELECTION_MODE is ${SMROBOT_PREBUILT_PACKAGE_SELECTION_MODE}" )
message( STATUS "SMROBOT_PREBUILT_PACKAGE_ROOT is ${SMROBOT_PREBUILT_PACKAGE_ROOT}" )
message( STATUS "SMROBOT_RESOLVED_PREBUILT_PACKAGE_ROOT is ${SMROBOT_RESOLVED_PREBUILT_PACKAGE_ROOT}" )

if(_smrobot_prebuilt_package_root AND IS_DIRECTORY "${_smrobot_prebuilt_package_root}")
    collect_subdirs(
        ${_smrobot_prebuilt_package_root}
        AllPackageNames
        EXCLUDE ${ExcludePrebuiltPackages}
    )
else()
    set(AllPackageNames)
endif()

set( AllPrebuiltPackages ${AllPackageNames} )
message( STATUS "------------------------------------------------------------------" )
message( STATUS "All Prebuilt Packages: ${AllPrebuiltPackages}" )
message( STATUS "------------------------------------------------------------------\n" )

set( PrebuiltPackagesOptions )


#   2.1. Get prebuilt packages' components
foreach( PACKAGE_NAME ${AllPrebuiltPackages} )
    #   Get all components for each package
    set( ExcludeComponents ${PACKAGE_NAME} )
    collect_subdirs(
        ${_smrobot_prebuilt_package_root}/${PACKAGE_NAME}/lib/cmake
        AllComponentNames
        EXCLUDE ${ExcludeComponents}
    )

    set( ${PACKAGE_NAME}_PrebuiltComponents ${AllComponentNames} )
    message( STATUS "------------------------------------------------------------------" )
    message( STATUS "All Prebuilt Components of package ${PACKAGE_NAME}: ${${PACKAGE_NAME}_PrebuiltComponents}" )
    message( STATUS "------------------------------------------------------------------\n" )

    #   Set prebuilt package option
    option( UsingPrebuilt_${PACKAGE_NAME} "Use prebuilt ${PACKAGE_NAME}" OFF )

    set( PrebuiltPackagesOptions ${PrebuiltPackagesOptions} ${PACKAGE_NAME} )

endforeach()

set(SMRobotCore_PrebuiltComponents
    RobotCore
    RobotRuntime
    RobotInstance
    RobotIO
    Kinematics
    Collision
    RobotSDK
    RobotTrajectoryCore
)

set(SMRobotPlatform_PrebuiltComponents
    AssetCore
    CameraCore
    ProjectSimulationSDK
    SceneCore
    RenderCore
    RenderCoreShaderResources
    RobotRenderBridge
    SimulationProject
    SimulationRuntime
    SensorCore
    SensorSimulation
    VisualizationSDK
)

set(SMRobotApps_PrebuiltComponents
    RobotViewerCore
    RobotQtViewer
    RobotSDKVisualQuickStart
)

set(SMRobotCore_ConfigOnlyPrebuiltPackage ON)
set(Applications_ConfigOnlyPrebuiltPackage ON)

message( STATUS "PrebuiltPackagesOptions is ${PrebuiltPackagesOptions}" )

rs_apply_logical_prebuilt_alias_options()


############################################################
#   3. Get all package configuration
############################################################
foreach( PACKAGE_NAME ${AllBuildPackages} )
    rs_get_package_source_dir(${PACKAGE_NAME} _package_source_dir)
    option( UsingPrebuilt_${PACKAGE_NAME} "Use prebuilt ${PACKAGE_NAME}" OFF )

    option( BuildPackage_${PACKAGE_NAME} "Build package ${PACKAGE_NAME}" ON )

    set( _auto_disabled_var AutoDisabledBuildPackage_${PACKAGE_NAME} )

    if( UsingPrebuilt_${PACKAGE_NAME} )
        list( FIND AllPrebuiltPackages ${PACKAGE_NAME} _prebuilt_package_index )
        if( _prebuilt_package_index EQUAL -1 )
            if(SMROBOT_RESOLVED_PREBUILT_PACKAGE_ROOT)
                set(_prebuilt_package_hint "${SMROBOT_RESOLVED_PREBUILT_PACKAGE_ROOT}/${PACKAGE_NAME}")
            else()
                set(_prebuilt_package_hint
                    "no explicit SDK prefix was selected; set SMROBOT_PREBUILT_PACKAGE_ROOT or use LegacyAuto mode")
            endif()
            message( FATAL_ERROR
                "UsingPrebuilt_${PACKAGE_NAME}=ON but no prebuilt package was found at "
                "${_prebuilt_package_hint}."
            )
        endif()

        if( BuildPackage_${PACKAGE_NAME} )
            message( STATUS
                "UsingPrebuilt_${PACKAGE_NAME}=ON conflicts with BuildPackage_${PACKAGE_NAME}=ON. "
                "Disable source build for package ${PACKAGE_NAME}."
            )
            set( BuildPackage_${PACKAGE_NAME} OFF CACHE BOOL "Build package ${PACKAGE_NAME}" FORCE )
            set( ${_auto_disabled_var} ON CACHE INTERNAL
                "BuildPackage_${PACKAGE_NAME} was disabled automatically by UsingPrebuilt_${PACKAGE_NAME}"
                FORCE
            )
        endif()
    elseif( ${_auto_disabled_var} AND NOT BuildPackage_${PACKAGE_NAME} )
        message( STATUS
            "UsingPrebuilt_${PACKAGE_NAME}=OFF; restore BuildPackage_${PACKAGE_NAME}=ON "
            "because it was disabled automatically."
        )
        set( BuildPackage_${PACKAGE_NAME} ON CACHE BOOL "Build package ${PACKAGE_NAME}" FORCE )
        set( ${_auto_disabled_var} OFF CACHE INTERNAL
            "BuildPackage_${PACKAGE_NAME} was disabled automatically by UsingPrebuilt_${PACKAGE_NAME}"
            FORCE
        )
    else()
        set( ${_auto_disabled_var} OFF CACHE INTERNAL
            "BuildPackage_${PACKAGE_NAME} was disabled automatically by UsingPrebuilt_${PACKAGE_NAME}"
            FORCE
        )
    endif()
    unset( _auto_disabled_var )

    if( BuildPackage_${PACKAGE_NAME} )
        #   Get package configuration
        message( STATUS "\n\n==================================================================" )
        message( STATUS "[PackageMode] ${PACKAGE_NAME}: source" )
        include( ${_package_source_dir}/PackageConfigSetting )

        #   Print the required public and private libraries for each package
        message( STATUS "----   [Packages Dependencies] ${PACKAGE_NAME}_RequiredLibsPublic: " )
        message( STATUS "----   [Packages Dependencies] ${${PACKAGE_NAME}_RequiredLibsPublic}" )
        message( STATUS "----   [Packages Dependencies] ${PACKAGE_NAME}_RequiredLibsPrivate: " )
        message( STATUS "----   [Packages Dependencies] ${${PACKAGE_NAME}_RequiredLibsPrivate}" )
        message( STATUS "\n\n==================================================================" )

        #   Collect all required libraries for this package
        list( APPEND ProjectRequiredLibs ${${PACKAGE_NAME}_RequiredLibsPrivate} ${${PACKAGE_NAME}_RequiredLibsPublic} )
    elseif( UsingPrebuilt_${PACKAGE_NAME} )
        message( STATUS "[PackageMode] ${PACKAGE_NAME}: prebuilt" )
        if( EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${_package_source_dir}/PackageConfigSetting.cmake" )
            include( ${_package_source_dir}/PackageConfigSetting )
        else()
            message( STATUS
                "[PackageMode] ${PACKAGE_NAME}: prebuilt source package config is absent; "
                "installed package config will resolve components during find_package."
            )
        endif()
    else()
        message( STATUS "[PackageMode] ${PACKAGE_NAME}: disabled" )
    endif()
    unset(_package_source_dir)

endforeach()

############################################################
#   4. Catalog all required libraries for each package and display them in the end of this section.
############################################################
#   Get all required libraries for each package and display them in the end of this section.
message( STATUS "Handle all packages for the whole project" )
message( STATUS "ProjectRequiredLibs: ${ProjectRequiredLibs}" )
list( REMOVE_DUPLICATES ProjectRequiredLibs )
resolve_dependencies( ProjectRequiredLibs )

#   4. Display the required libs for this package

message( STATUS "*************************************************************************" )
message( STATUS "All packages required for this project are listed : ${AllPackagesRequired}" )
foreach( pkg ${AllPackagesRequired} )
    message( STATUS "Project requires ${pkg} with components: ${${pkg}_DepLibs}" )
endforeach()
message( STATUS "*************************************************************************" )

############################################################
#   5. Build Example
############################################################
option( BuildExample "Build examples!" ON )
option( BuildDiagnostics "Build diagnostic programs" OFF )
option( BuildFeatureProbes "Build feature probe programs" OFF )
option( BuildRegression "Build regression executables" OFF )
option( BuildExternalValidation "Build external validation programs" OFF )
option( BuildTutorials "Build tutorial programs" OFF )

foreach( PACKAGE_NAME ${AllBuildPackages} )
    option( BuildExample_${PACKAGE_NAME} "Build examples of package ${PACKAGE_NAME}" OFF )
    option( BuildDiagnostics_${PACKAGE_NAME} "Build diagnostic programs of package ${PACKAGE_NAME}" OFF )
    option( BuildFeatureProbes_${PACKAGE_NAME} "Build feature probe programs of package ${PACKAGE_NAME}" OFF )
    option( BuildRegression_${PACKAGE_NAME} "Build regression executables of package ${PACKAGE_NAME}" OFF )
    option( BuildExternalValidation_${PACKAGE_NAME} "Build external validation programs of package ${PACKAGE_NAME}" OFF )
    option( BuildTutorials_${PACKAGE_NAME} "Build tutorial programs of package ${PACKAGE_NAME}" OFF )
endforeach()





