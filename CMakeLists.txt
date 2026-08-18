######################################################################################
#   Robot Simulator 2025
#   Written by Tang Qing in Jan, 2023
#   Rewrittern by Tang Qing in Feb, 2026
######################################################################################
cmake_minimum_required( VERSION 3.20 )

######################################################################################
#	Project Setting
######################################################################################
#   Define the project
project( RobotSimulator2026
    VERSION 1.0.2
    LANGUAGES CXX C
    DESCRIPTION "Robot Simulator!"
)

option( BuildTests "Build headless tests" OFF )
if( BuildTests )
    enable_testing()
endif()
option( BuildRobotQtViewer "Build SMRobotApps Qt desktop viewer prototype" ON )
set(RS2026_LICENSE_ENFORCEMENT_MODE "Enforce" CACHE STRING
    "License enforcement mode: Enforce, ReportOnly, or Disabled.")
set_property(CACHE RS2026_LICENSE_ENFORCEMENT_MODE PROPERTY STRINGS Enforce ReportOnly Disabled)
if(NOT RS2026_LICENSE_ENFORCEMENT_MODE MATCHES "^(Enforce|ReportOnly|Disabled)$")
    message(FATAL_ERROR
        "RS2026_LICENSE_ENFORCEMENT_MODE must be Enforce, ReportOnly, or Disabled.")
endif()
message(STATUS "RS2026 license enforcement mode: ${RS2026_LICENSE_ENFORCEMENT_MODE}")
set(SMROBOT_RUNTIME_LICENSE_SOURCE_DIR "" CACHE PATH
    "Optional directory containing .LIC files to deploy beside built applications.")
set(SMROBOT_RUNTIME_LICENSE_OVERRIDE_DIR "" CACHE PATH
    "Optional explicit runtime license directory overriding packaged defaults.")

######################################################################################
#   Project Information
######################################################################################
#   The CMake message of RobotSimulator 2025
get_filename_component( this_cmake_dir ${CMAKE_CURRENT_LIST_DIR} ABSOLUTE )
message( STATUS "\n==================== Building ${Project_Name} ${PROJECT_VERSION} in ${this_cmake_dir} ====================\n" )
set( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_LIST_DIR} ${CMAKE_CURRENT_LIST_DIR}/cmake )
set(_smrobot_business_source_defaults
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/BusinessSourcePackageDefaults.cmake")
if(EXISTS "${_smrobot_business_source_defaults}")
    include("${_smrobot_business_source_defaults}")
endif()
unset(_smrobot_business_source_defaults)

set(SMROBOT_THIRDPARTY_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty" CACHE PATH
    "Root directory for SMRobot third-party source/prebuilt dependency helpers.")
set(SMROBOT_DATA_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/data" CACHE PATH
    "Root directory for SMRobot runtime data assets.")
get_filename_component(SMROBOT_THIRDPARTY_ROOT "${SMROBOT_THIRDPARTY_ROOT}" ABSOLUTE)
get_filename_component(SMROBOT_DATA_ROOT "${SMROBOT_DATA_ROOT}" ABSOLUTE)

######################################################################################
#   Project Setting
######################################################################################
#   Compile language, output directory setting.
include( ProjectSetting )
include( SMRobotRuntimeLicense )
smrobot_prepare_standard_runtime_licenses()

include( FuncDef_GetAllPackageNames )
include( FuncDef_GetPackageDeps )
include( FuncDef_PackageInstall )
include( FuncDef_ExampleCategory )
include( PackageCompatibility )
include( PackageSourceLayout )

#   Get all configurations from each package
#   From the 

include( ProjectPackagesConfigSetting )
list( REMOVE_DUPLICATES _all_pkgs )

# message( STATUS "[RS2025] Package Common's Components include: ${Common_Components}" )

######################################################################################
#   CMake tools included
######################################################################################
include( CMakeDependentOption )
include( CMakePrintHelpers )

######################################################################################
#	User Configuration
######################################################################################
#   Define the current user.
option( DisplayTargetProperties "Whether display the targets' properties" OFF )

#   Define user and find third-party packages
include( UserConfig )

# OpenGL is a direct source-build dependency of Common targets. Some user
# profiles used to obtain OpenGL::GL indirectly through PCL/VTK, so ensure the
# imported target exists independently of optional third-party providers.
if("OpenGL" IN_LIST RS2026_REQUIRED_PACKAGES AND NOT TARGET OpenGL::GL)
    find_package( OpenGL REQUIRED )
endif()

if(NOT UsingPrebuilt_Common)
    include(SourceLicenseFacadeProvider)
endif()

#   Define prebuilt package discovery. Packages are loaded later in SeqPackages order.
include( FindPrebuiltPackages )

configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/SMRobotDependencyBootstrap.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/SMRobotDependencyBootstrap.cmake
    @ONLY
)
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/SMRobotDependencyBootstrap.cmake
    DESTINATION cmake
    COMPONENT smrobot_sdk_bootstrap
)
foreach(_smrobot_embedded_thirdparty Eigen340 glm nlohmann_json)
    if(IS_DIRECTORY "${SMROBOT_THIRDPARTY_ROOT}/${_smrobot_embedded_thirdparty}")
        install(DIRECTORY
            "${SMROBOT_THIRDPARTY_ROOT}/${_smrobot_embedded_thirdparty}"
            DESTINATION thirdparty
            COMPONENT smrobot_sdk_thirdparty_header_only
        )
    endif()
endforeach()
set(SMROBOT_PREBUILT_THIRDPARTY_ROOT "" CACHE PATH "Optional root containing prebuilt third-party SDKs to embed into the installed SMRobot SDK.")
set(_smrobot_prebuilt_thirdparty_install_roots)
if(SMROBOT_PREBUILT_THIRDPARTY_ROOT)
    list(APPEND _smrobot_prebuilt_thirdparty_install_roots "${SMROBOT_PREBUILT_THIRDPARTY_ROOT}")
endif()
if(DEFINED PREBUILD_DIR AND IS_DIRECTORY "${PREBUILD_DIR}")
    list(APPEND _smrobot_prebuilt_thirdparty_install_roots "${PREBUILD_DIR}")
endif()
list(REMOVE_DUPLICATES _smrobot_prebuilt_thirdparty_install_roots)

set(_smrobot_qt_sdk_root)
if(DEFINED Qt5_DIR AND IS_DIRECTORY "${Qt5_DIR}")
    get_filename_component(_smrobot_qt_sdk_root "${Qt5_DIR}/../../.." ABSOLUTE)
endif()
if(NOT IS_DIRECTORY "${_smrobot_qt_sdk_root}")
    foreach(_smrobot_prebuilt_thirdparty_root IN LISTS _smrobot_prebuilt_thirdparty_install_roots)
        if(IS_DIRECTORY "${_smrobot_prebuilt_thirdparty_root}/vs_indep/Qt5155_x64")
            set(_smrobot_qt_sdk_root "${_smrobot_prebuilt_thirdparty_root}/vs_indep/Qt5155_x64")
            break()
        endif()
    endforeach()
endif()

if(IS_DIRECTORY "${_smrobot_qt_sdk_root}")
    foreach(_smrobot_qt_sdk_directory plugins include lib mkspecs)
        if(IS_DIRECTORY "${_smrobot_qt_sdk_root}/${_smrobot_qt_sdk_directory}")
            install(DIRECTORY
                "${_smrobot_qt_sdk_root}/${_smrobot_qt_sdk_directory}"
                DESTINATION thirdparty/Qt5
                COMPONENT smrobot_sdk_qt
                PATTERN "*.pdb" EXCLUDE
            )
        endif()
    endforeach()
    foreach(_smrobot_qt_sdk_binary
        qmake.exe
        moc.exe
        rcc.exe
        uic.exe
        Qt5Core.dll
        Qt5Cored.dll
        Qt5Gui.dll
        Qt5Guid.dll
        Qt5Widgets.dll
        Qt5Widgetsd.dll
        Qt5OpenGL.dll
        Qt5OpenGLd.dll
    )
        if(EXISTS "${_smrobot_qt_sdk_root}/bin/${_smrobot_qt_sdk_binary}")
            install(FILES
                "${_smrobot_qt_sdk_root}/bin/${_smrobot_qt_sdk_binary}"
                DESTINATION thirdparty/Qt5/bin
                COMPONENT smrobot_sdk_qt
            )
        endif()
    endforeach()
endif()

set(_smrobot_boost_sdk_root)
if(DEFINED Boost_DIR AND IS_DIRECTORY "${Boost_DIR}")
    get_filename_component(_smrobot_boost_sdk_root "${Boost_DIR}/../../.." ABSOLUTE)
endif()
if(NOT IS_DIRECTORY "${_smrobot_boost_sdk_root}")
    foreach(_smrobot_prebuilt_thirdparty_root IN LISTS _smrobot_prebuilt_thirdparty_install_roots)
        if(IS_DIRECTORY "${_smrobot_prebuilt_thirdparty_root}/vs2019/boost_1_78_0")
            set(_smrobot_boost_sdk_root "${_smrobot_prebuilt_thirdparty_root}/vs2019/boost_1_78_0")
            break()
        endif()
    endforeach()
endif()

if(IS_DIRECTORY "${_smrobot_boost_sdk_root}/boost")
    install(DIRECTORY
        "${_smrobot_boost_sdk_root}/boost"
        DESTINATION thirdparty/Boost
        COMPONENT smrobot_sdk_boost
    )

    file(GLOB _smrobot_boost_config_dirs LIST_DIRECTORIES TRUE
        "${_smrobot_boost_sdk_root}/lib*/cmake"
    )
    list(LENGTH _smrobot_boost_config_dirs _smrobot_boost_config_dir_count)
    if(_smrobot_boost_config_dir_count GREATER 0)
        list(GET _smrobot_boost_config_dirs 0 _smrobot_boost_config_dir)
        get_filename_component(_smrobot_boost_library_dir "${_smrobot_boost_config_dir}/.." ABSOLUTE)
        get_filename_component(_smrobot_boost_library_dir_name "${_smrobot_boost_library_dir}" NAME)
        install(DIRECTORY
            "${_smrobot_boost_config_dir}"
            DESTINATION "thirdparty/Boost/${_smrobot_boost_library_dir_name}"
            COMPONENT smrobot_sdk_boost
        )

        set(_smrobot_boost_sdk_components
            atomic
            chrono
            date_time
            filesystem
            log
            log_setup
            regex
            system
            thread
        )
        set(_smrobot_boost_sdk_libraries)
        foreach(_smrobot_boost_sdk_component IN LISTS _smrobot_boost_sdk_components)
            file(GLOB _smrobot_boost_component_libraries
                "${_smrobot_boost_library_dir}/*boost_${_smrobot_boost_sdk_component}-*.lib"
                "${_smrobot_boost_library_dir}/*boost_${_smrobot_boost_sdk_component}-*.dll"
            )
            list(APPEND _smrobot_boost_sdk_libraries ${_smrobot_boost_component_libraries})
        endforeach()
        list(REMOVE_DUPLICATES _smrobot_boost_sdk_libraries)
        if(_smrobot_boost_sdk_libraries)
            install(FILES
                ${_smrobot_boost_sdk_libraries}
                DESTINATION "thirdparty/Boost/${_smrobot_boost_library_dir_name}"
                COMPONENT smrobot_sdk_boost
            )
        endif()
    endif()
endif()

set(_smrobot_assimp_sdk_root)
if(DEFINED assimp_DIR AND IS_DIRECTORY "${assimp_DIR}")
    get_filename_component(_smrobot_assimp_sdk_root "${assimp_DIR}/../../.." ABSOLUTE)
endif()
if(NOT IS_DIRECTORY "${_smrobot_assimp_sdk_root}")
    foreach(_smrobot_prebuilt_thirdparty_root IN LISTS _smrobot_prebuilt_thirdparty_install_roots)
        foreach(_smrobot_assimp_relative_root vs2019/assimp vs2019/Assimp6.0.2)
            if(IS_DIRECTORY "${_smrobot_prebuilt_thirdparty_root}/${_smrobot_assimp_relative_root}")
                set(_smrobot_assimp_sdk_root "${_smrobot_prebuilt_thirdparty_root}/${_smrobot_assimp_relative_root}")
                break()
            endif()
        endforeach()
        if(IS_DIRECTORY "${_smrobot_assimp_sdk_root}")
            break()
        endif()
    endforeach()
endif()

if(IS_DIRECTORY "${_smrobot_assimp_sdk_root}")
    foreach(_smrobot_assimp_sdk_directory bin include lib)
        if(IS_DIRECTORY "${_smrobot_assimp_sdk_root}/${_smrobot_assimp_sdk_directory}")
            install(DIRECTORY
                "${_smrobot_assimp_sdk_root}/${_smrobot_assimp_sdk_directory}"
                DESTINATION thirdparty/assimp
                COMPONENT smrobot_sdk_assimp
                PATTERN "*.pdb" EXCLUDE
            )
        endif()
    endforeach()
endif()

foreach(_smrobot_prebuilt_thirdparty_root IN LISTS _smrobot_prebuilt_thirdparty_install_roots)
    foreach(_smrobot_prebuilt_thirdparty
        vs2019/urdfdom_install
        vs_indep/urdfdom_headers
    )
        if(IS_DIRECTORY "${_smrobot_prebuilt_thirdparty_root}/${_smrobot_prebuilt_thirdparty}")
            install(DIRECTORY
                "${_smrobot_prebuilt_thirdparty_root}/${_smrobot_prebuilt_thirdparty}"
                DESTINATION thirdparty
                COMPONENT smrobot_sdk_robotio_thirdparty
            )
        endif()
    endforeach()

    if(IS_DIRECTORY "${_smrobot_prebuilt_thirdparty_root}/vs2019/fcl_static_x64")
        install(DIRECTORY
            "${_smrobot_prebuilt_thirdparty_root}/vs2019/fcl_static_x64"
            DESTINATION thirdparty/vs2019
            COMPONENT smrobot_sdk_fcl
        )
    endif()
endforeach()




message( STATUS "[RS2025] ${PACKAGE_NAME}_RequiredLibsPublic: ${${PACKAGE_NAME}_RequiredLibsPublic}" )
message( STATUS "[RS2025] ${PACKAGE_NAME}_RequiredLibsPriavate: ${${PACKAGE_NAME}_RequiredLibsPriavate}" )

# ######################################################################################
# #   Set project package build configuration. 
# ######################################################################################
# #	Whether using prebuild library
# option( UsingPrebuild_ALL "Using prebuild target" OFF )
# include( UsingPrebuildSetting )

# # #   Whether build on site or using prebuild
# # include( PackageBuildConfig )

# #   Define all the third party packages for each users.
# # include( ImportThirdPartyLibs )

# #   Define the installation directory.
# set( CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR}/PackagesInstallation CACHE STRING "Install Directory" FORCE )

######################################################################################
#	CMake Info to Program
######################################################################################
#   Get compile time information
include( GetCompileTimeInfo )
#   Get the source directory
include( SourcePathConfig )

######################################################################################
#	Copy the release runtime files to release directory
######################################################################################
# Enable copying the release runtime files to release directory
option( Copy_RuntimeForRelease "Whether copy the runtime files to release directory for current project" OFF )
include( FuncDef_CopyRuntimeForRelease )


# ######################################################################################
# #	Define GLEW or GLAD usage
# ######################################################################################
# #   Define which OpenGL package will be used.
# option( USING_GLEW "Option: Whether use GLEW or GLAD" ON )
# if( NOT ${USING_GLEW} )
#     set( USING_GLAD ON CACHE BOOL "Using GLAD ON" FORCE )
#     # option( USING_GLAD "Option: Whether use GLEW or GLAD" ON )
# else()
#     set( USING_GLAD OFF CACHE BOOL "Using GLAD OFF" FORCE)
#     # option( USING_GLAD "Option: Whether use GLEW or GLAD" OFF )
# endif()



######################################################################################
#	VS debug env configuration : Get the runtime directory for visual studio
######################################################################################

set( _runtime_target_candidates
   FreeGLUT::freeglut
   assimp::assimp
   Qt5::Widgets

   #    For URDF Loading
   urdfdom::urdfdom_model
   console_bridge::console_bridge
)
list( APPEND _runtime_target_candidates CoACD::_coacd )
foreach( _runtime_target IN LISTS _runtime_target_candidates )
    if( TARGET ${_runtime_target} )
        list( APPEND runtime_targets ${_runtime_target} )
    else()
        message( STATUS
            "Skip VS runtime path for unavailable optional target: ${_runtime_target}" )
    endif()
endforeach()
unset( _runtime_target )
unset( _runtime_target_candidates )
include( GetPackageRuntimeDir )


# #   Get target runtime directory for VS Debug
# set( VSDebugEnvPath "PATH=%PATH%" )

# #   Get the target related function
# include( FuncDef_GetTargetPropertyDir )

# #   Collect IMPLIB_DEBUG target
# set( Dir_in_target_IMPLIB_DEBUG 
#     Boost::filesystem
# )

# #   Add directory from IMPLIB_DEBUG into VS Debug Env Path
# foreach( target_ ${Dir_in_target_IMPLIB_DEBUG} )
#     message( STATUS "Append IMPLIB_DEBUG directory from target ${target_} ..." )

#     #   Get the property directory
#     GetTargetPropertyDir( ${target_} "IMPORTED_IMPLIB_DEBUG" )

#     #   Append the property directory
#     string( APPEND VSDebugEnvPath ";" ${Target_RUNTIME_DIR} )

#     #   Display the value of VSDebugEnvPath
#     message( STATUS "VSDebugEnvPath is: ${VSDebugEnvPath}\n" )
# endforeach()

# #   Collect LOCATION_DEBUG target
# set( Dir_in_target_LOCATION_DEBUG 
#     Qt5::Widgets
#     FreeGLUT::freeglut
#     GLEW::glew

#     console_bridge::console_bridge
#     urdfdom::urdfdom_sensor
#     pinocchio::pinocchio
# )

# #   Add directory from LOCATION_DEBUG into VS Debug Env Path
# foreach( target_ ${Dir_in_target_LOCATION_DEBUG} )
#     message( STATUS "Append LOCATION_DEBUG directory from target ${target_} ..." )

#     #   Get the property directory
#     GetTargetPropertyDir( ${target_} "IMPORTED_LOCATION_DEBUG" )

#     #   Append the property directory
#     string( APPEND VSDebugEnvPath ";" ${Target_RUNTIME_DIR} )

#     #   Display the value of VSDebugEnvPath
#     message( STATUS "VSDebugEnvPath is: ${VSDebugEnvPath}\n" )
# endforeach()


# message( STATUS "---------------------------------------------------------------------------" )
# message( STATUS "Final VSDebugEnvPath is : ${VSDebugEnvPath}" )
# message( STATUS "---------------------------------------------------------------------------\n\n\n" )

# ######################################################################################
# #	Include packages either built from source or prebuilt already
# ######################################################################################
# #   Actual loading packages
# include( UsingPrebuildLoading )


######################################################################################
#	Main Project Inclusion
######################################################################################
message( STATUS "\n\n\n" )

set( SeqPackages Common Verification Algorithm 
    QtComponents
    SMRobotGen1
    SMRobotCore
    SMRobotPlatform
    SMRobotSpray
    SMRobotMotionPlanning
    Tutorials 
    SMRobotWorkbenchCommon
    SMRobotWorkbenchProjectAssembly
    SMRobotWorkbenchCollisionConfig
    SMRobotWorkbenchRobotRun
    SMRobotWorkbenchMotionPlanning
    SMRobotWorkbenchPaintingAnalysis
    SMRobotApps
    SMRobotWorkbenchSprayProcess
    SMRobotWorkbenchDigitalTwin
)

foreach( PACKAGE_NAME ${SeqPackages} )
    rs_get_package_source_dir(${PACKAGE_NAME} _package_source_dir)
    if( UsingPrebuilt_${PACKAGE_NAME} )
        rs_find_requested_prebuilt_package(${PACKAGE_NAME})
        rs_create_logical_package_alias_targets()
    endif()
    if( BuildPackage_${PACKAGE_NAME} )
        message( STATUS "\n--------------------------------------------------------" )
        message( STATUS "\tAdd subdirectory : ${PACKAGE_NAME} (${_package_source_dir})" )
        message( STATUS "--------------------------------------------------------\n" )
        add_subdirectory( ${_package_source_dir} )
    elseif( UsingPrebuilt_${PACKAGE_NAME} )
        message( STATUS "\n--------------------------------------------------------" )
        message( STATUS "\tUse prebuilt package : ${PACKAGE_NAME}" )
        message( STATUS "--------------------------------------------------------\n" )
        if( (PACKAGE_NAME STREQUAL "Common" OR PACKAGE_NAME STREQUAL "SMRobotCore" OR PACKAGE_NAME STREQUAL "SMRobotPlatform")
            AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${_package_source_dir}/CMakeLists.txt" )
            add_subdirectory( ${_package_source_dir} )
        elseif( PACKAGE_NAME STREQUAL "Common" OR PACKAGE_NAME STREQUAL "SMRobotCore" OR PACKAGE_NAME STREQUAL "SMRobotPlatform" )
            message( STATUS "Skip prebuilt package directory for ${PACKAGE_NAME}; no source directory exists." )
        endif()
    else()
        message( STATUS "\n--------------------------------------------------------" )
        message( STATUS "\tSkip package : ${PACKAGE_NAME}" )
        message( STATUS "--------------------------------------------------------\n" )
    endif()
    unset(_package_source_dir)
endforeach()

foreach( PACKAGE_NAME ${PrebuiltPackagesOptions} )
    list(FIND SeqPackages ${PACKAGE_NAME} _prebuilt_sequence_index)
    if(_prebuilt_sequence_index EQUAL -1 AND UsingPrebuilt_${PACKAGE_NAME})
        rs_find_requested_prebuilt_package(${PACKAGE_NAME})
    endif()
endforeach()

rs_create_logical_package_alias_targets()




# #	1. Robot Simulator Tutorials
# option( BuildPackage_Tutorials "Build Package Tutorials" ON )
# if( ${BuildPackage_Tutorials} )
#     add_subdirectory( Tutorials )
# endif()

# #	2. Applications
# option( BuildPackage_Applications "Build Package Applications" ON )
# if( ${BuildPackage_Applications} )
#     add_subdirectory( Applications )
# endif()

# #	3. Test
# option( BuildPackage_Test "Build Package Test" OFF )
# if( ${BuildPackage_Test} )
#     add_subdirectory( Test )
# endif()


######################################################################################
#	Copy Release Runtime Files to Output Directory
######################################################################################
#	The VS Debug Environment Setting is useless during release configuration
get_filename_component( RELEASE_TARGET_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY} ABSOLUTE )
message( STATUS "RELEASE_TARGET_DIRECTORY is ${RELEASE_TARGET_DIRECTORY}" )



# #   Copy the required targets' runtime file to the target directory
# include( CopyRuntimeForRelease )

# #   Copy the runtime which is needed when using distribution.
# include( CopyRuntimeForDist )




######################################################################################
#	Ubuntu deployment
######################################################################################
#   Enable the Ubuntu system to find out the library
if( CMAKE_SYSTEM_NAME MATCHES "Linux" )
    #   move the rs2022.conf to /etc/ld.so.conf.d
    #   sudo mv rs2022.conf /etc/ld.so.conf.d/rs2022.conf
    #   To make the setting valid using
    #   sudo ldconfig
    message( STATUS "Use \"sudo mv rs2022.conf /etc/ld.so.conf.d/rs2022.conf\" to enable Ubuntu to find out the library" )
    configure_file( LD_LIBRARY_PATH_setting.sh.in rs2022.conf @ONLY )
endif()




message( STATUS "==================== End of RobotSimulator ${PROJECT_VERSION} CMake ====================\n" )
