#################################################################################
#   Find dependant packages for user tangtang 4090 Win
#   1. Head only pakcages
#   --  glm
#   --  Eigen
#
#   2. Third-Party Packages:
#   --  glfw
#   --  freeglut
#   --  glew
#
#   3. Prebuild Packages:
#   Packages for deployment
#   --  Boost   1.78.0
#   --  Qt      5.15.5
#
#   Packages for development
#   --  Assimp  static
#   --  FCL
#   --  OpenSSL 

#   --  
#   Customized Packages (Using prebuilt):
#   --  
#   Written by Tang Qing in March. 2025.
#################################################################################


#   Information
#   Current file output
get_filename_component( this_cmake_file ${CMAKE_CURRENT_LIST_FILE} ABSOLUTE )
message( STATUS "\n------------------ Find packages in ${this_cmake_file} ------------------" )


set( SAVE_MODULE_PATH ${CMAKE_MODULE_PATH} )
include( CMakePrintHelpers )

# #   Set ForDevelopment option
# if( DEFINED CACHE{ForDevelopment} )
#     unset( ForDevelopment CACHE )
# endif()
# cmake_dependent_option( ForDevelopment "Using Prebuild Package - ${CurrentPackage}" ON "UsingPrebuild_ALL" OFF )
# option( ForDevelopment "Options for selecting Development!" OFF )

#   Define the local Windows dependency root for this machine profile. A command-line
#   cache value takes precedence, while the environment variable remains an optional
#   override for release/CI jobs running with the same profile.
set(_rs4090_default_prebuild_dir "D:/PreBuild")
if(IS_DIRECTORY "J:/project/ThirdPartyLib")
    set(_rs4090_default_prebuild_dir "J:/project/ThirdPartyLib")
endif()
if(DEFINED ENV{RS2026_EXTERNAL_PREBUILD_ROOT}
   AND NOT "$ENV{RS2026_EXTERNAL_PREBUILD_ROOT}" STREQUAL "")
    set(_rs4090_default_prebuild_dir "$ENV{RS2026_EXTERNAL_PREBUILD_ROOT}")
endif()
set(PREBUILD_DIR "${_rs4090_default_prebuild_dir}" CACHE PATH
    "External Windows Qt/OMPL/prebuilt dependency root")
if(NOT IS_DIRECTORY "${PREBUILD_DIR}" AND
   IS_DIRECTORY "J:/project/ThirdPartyLib")
    set(PREBUILD_DIR "J:/project/ThirdPartyLib" CACHE PATH
        "External Windows Qt/OMPL/prebuilt dependency root" FORCE)
endif()
unset(_rs4090_default_prebuild_dir)
if(NOT PREBUILD_DIR)
    message(FATAL_ERROR
        "PREBUILD_DIR is required for the Windows build profile. "
        "Pass -DPREBUILD_DIR=<windows-dependency-root>.")
endif()

get_filename_component( BUILDINLIB_DIR ${SMROBOT_THIRDPARTY_ROOT} ABSOLUTE )

# Support both the historical vs_indep/vs2019 dependency bundle and the
# flat ThirdPartyLib layout used by the ChenYu workstation.
set(_rs4090_prebuild_flat OFF)
if(IS_DIRECTORY "${PREBUILD_DIR}/vs_indep/cmake" AND
   IS_DIRECTORY "${PREBUILD_DIR}/vs2019/cmake")
    get_filename_component( WIN_VSIndep_DIR ${PREBUILD_DIR}/vs_indep/cmake ABSOLUTE )
    get_filename_component( WIN_VS2019_DIR ${PREBUILD_DIR}/vs2019/cmake ABSOLUTE )
else()
    set(_rs4090_prebuild_flat ON)
    set(WIN_VSIndep_DIR
        "${PREBUILD_DIR}/Qt5155_x64"
        CACHE PATH "Flat Qt 5.15.5 package directory" FORCE)
    set(WIN_VS2019_DIR
        "${PREBUILD_DIR}/Boost-1.78.0;${PREBUILD_DIR}/assimp-6.0.2;${PREBUILD_DIR}/fcl-0.7.0-static;${PREBUILD_DIR}/ompl-2.0.1;${PREBUILD_DIR}/ktx"
        CACHE STRING "Flat third-party CMake module directories" FORCE)
    set(Thirdparty_Package_DIR "${PREBUILD_DIR}")
    set(PackDIR_Qt5
        "${PREBUILD_DIR}/Qt5155_x64/install/lib/cmake/Qt5"
        CACHE PATH "Qt 5.15.5 package configuration directory" FORCE)
endif()

# Pinocchio is shipped with the business-source package, while the external
# ThirdPartyLib root is used for Boost/Qt/OMPL/FCL/Assimp.
set(_rs4090_pinocchio_dir
    "${SMROBOT_THIRDPARTY_ROOT}/Windows/pinocchio_install/lib/cmake/pinocchio")
if(EXISTS "${_rs4090_pinocchio_dir}/pinocchioConfig.cmake")
    set(pinocchio_DIR "${_rs4090_pinocchio_dir}" CACHE PATH
        "SMRobot bundled Pinocchio package directory" FORCE)
endif()
unset(_rs4090_pinocchio_dir)
message( STATUS "   --  Build in library Directory: ${BUILDINLIB_DIR}" )
message( STATUS "   --  Prebuild VSIndep Directory: ${WIN_VSIndep_DIR}" )
message( STATUS "   --  Prebuild VS2019 Directory: ${WIN_VS2019_DIR}" )

option(RS2026_4090_ENABLE_DEMAND_LOADING
    "Load only dependency providers required by the current 4090 build graph" OFF)
option(RS2026_4090_DEPENDENCY_REPORT
    "Report 4090 dependency-provider decisions without changing default behavior" ON)

function(_rs4090_decide_provider out_load provider_name)
    set(_provider_packages ${ARGN})
    set(_matched_packages)
    foreach(_package IN LISTS _provider_packages)
        rs_project_requires_package("${_package}" _package_required)
        if(_package_required)
            list(APPEND _matched_packages "${_package}")
        endif()
    endforeach()

    if(_matched_packages)
        set(_demand_action LOAD)
        set(_reason "required packages: ${_matched_packages}")
    elseif(NOT DEFINED RS2026_DEPENDENCY_SNAPSHOT_COMPLETE OR
           NOT RS2026_DEPENDENCY_SNAPSHOT_COMPLETE)
        set(_demand_action LOAD)
        set(_reason "dependency snapshot is incomplete; conservative fallback")
    else()
        set(_demand_action SKIP)
        set(_reason "not required by the current build graph")
    endif()

    if(RS2026_4090_ENABLE_DEMAND_LOADING)
        set(_actual_action "${_demand_action}")
    else()
        set(_actual_action LOAD)
    endif()

    if(RS2026_4090_DEPENDENCY_REPORT)
        message(STATUS
            "[4090 Dependency] ${provider_name}: demand=${_demand_action}, "
            "actual=${_actual_action}, reason=${_reason}")
    endif()

    if(_actual_action STREQUAL "LOAD")
        set(${out_load} ON PARENT_SCOPE)
    else()
        set(${out_load} OFF PARENT_SCOPE)
    endif()
endfunction()

function(_rs4090_assert_provider_targets provider_name)
    if(RS2026_4090_ENABLE_DEMAND_LOADING)
        rs_assert_required_targets("${provider_name}" ${ARGN})
    endif()
endfunction()

function(_rs4090_prebuilt_component_required out_required package component)
    set(_required OFF)
    if(DEFINED UsingPrebuilt_${package} AND UsingPrebuilt_${package})
        rs_project_requires_component("${package}" "${component}" _required)
    endif()
    set(${out_required} "${_required}" PARENT_SCOPE)
endfunction()

function(_rs4090_force_provider out_load provider_name reason)
    set(${out_load} ON PARENT_SCOPE)
    if(RS2026_4090_DEPENDENCY_REPORT)
        message(STATUS
            "[4090 Dependency] ${provider_name}: force actual=LOAD, reason=${reason}")
    endif()
endfunction()

if(RS2026_4090_DEPENDENCY_REPORT)
    message(STATUS
        "[4090 Dependency] demand loading enabled: "
        "${RS2026_4090_ENABLE_DEMAND_LOADING}")
    message(STATUS
        "[4090 Dependency] snapshot complete: "
        "${RS2026_DEPENDENCY_SNAPSHOT_COMPLETE}")
    message(STATUS
        "[4090 Dependency] required packages: ${RS2026_REQUIRED_PACKAGES}")
endif()

_rs4090_prebuilt_component_required(
    _rs4090_prebuilt_core_collision SMRobotCore Collision)
_rs4090_prebuilt_component_required(
    _rs4090_prebuilt_core_robot_io SMRobotCore RobotIO)
_rs4090_prebuilt_component_required(
    _rs4090_prebuilt_platform_asset_core SMRobotPlatform AssetCore)
_rs4090_prebuilt_component_required(
    _rs4090_prebuilt_platform_render_core SMRobotPlatform RenderCore)
_rs4090_prebuilt_component_required(
    _rs4090_prebuilt_platform_scene_core SMRobotPlatform SceneCore)

set(_rs4090_prebuilt_platform_asset_stack OFF)
if(_rs4090_prebuilt_platform_asset_core OR
   _rs4090_prebuilt_platform_render_core OR
   _rs4090_prebuilt_platform_scene_core)
    set(_rs4090_prebuilt_platform_asset_stack ON)
endif()



#   1. Head only pakcages
#   --  glm
#   --  Eigen
set( CMAKE_MODULE_PATH ${BUILDINLIB_DIR} )

#   1.1 Find glm
_rs4090_decide_provider(_rs4090_load_glm glm glm)
if(_rs4090_prebuilt_platform_asset_stack)
    _rs4090_force_provider(_rs4090_load_glm glm
        "requested prebuilt SMRobotPlatform asset/render/scene component")
endif()
if(_rs4090_load_glm)
    include( Findglm_3rdParty )
    _rs4090_assert_provider_targets(glm glm::glm)
endif()
#   1.2. Find Eigen
_rs4090_decide_provider(_rs4090_load_eigen Eigen3
    Eigen3 ompl fcl ccd octomap pinocchio urdfdom urdfdom_headers console_bridge)
if((DEFINED UsingPrebuilt_SMRobotCore AND UsingPrebuilt_SMRobotCore) OR
   (DEFINED UsingPrebuilt_SMRobotPlatform AND UsingPrebuilt_SMRobotPlatform))
    _rs4090_force_provider(_rs4090_load_eigen Eigen3
        "requested prebuilt Core/Platform components expose Eigen3 interfaces")
endif()
if(_rs4090_load_eigen)
    include( FindEigen_3rdParty )
    _rs4090_assert_provider_targets(Eigen3 Eigen3::Eigen)
endif()
#   1.3. Find json
_rs4090_decide_provider(_rs4090_load_nlohmann_json nlohmann_json nlohmann_json)
if(_rs4090_load_nlohmann_json)
    include( Findnlohmann_json_3rdParty )
    _rs4090_assert_provider_targets(nlohmann_json nlohmann_json::nlohmann_json)
endif()


#   2. Third-Party Packages
set( CMAKE_MODULE_PATH ${BUILDINLIB_DIR}/Windows )

set(_rs4090_optional_glfw_required OFF)
foreach(_category Diagnostics FeatureProbes Regression ExternalValidation Tutorials)
    if(DEFINED Build${_category} AND Build${_category})
        set(_rs4090_optional_glfw_required ON)
    endif()
endforeach()
foreach(_package IN LISTS AllBuildPackages)
    if(DEFINED BuildExample_${_package} AND BuildExample_${_package})
        set(_rs4090_optional_glfw_required ON)
    endif()
    foreach(_category Diagnostics FeatureProbes Regression ExternalValidation Tutorials)
        if(DEFINED Build${_category}_${_package} AND Build${_category}_${_package})
            set(_rs4090_optional_glfw_required ON)
        endif()
    endforeach()
endforeach()

_rs4090_decide_provider(_rs4090_load_glfw glfw glfw)
if(_rs4090_optional_glfw_required)
    set(_rs4090_load_glfw ON)
    if(RS2026_4090_DEPENDENCY_REPORT)
        message(STATUS
            "[4090 Dependency] glfw: force actual=LOAD because an optional example "
            "category is enabled")
    endif()
endif()
if(_rs4090_load_glfw)
    include( Findglfw_3rdParty )
    _rs4090_assert_provider_targets(glfw glfw::glfw)
endif()

_rs4090_decide_provider(_rs4090_load_freeglut FreeGLUT FreeGLUT)
if(_rs4090_load_freeglut)
    include( Findfreeglut_3rdParty )
    _rs4090_assert_provider_targets(FreeGLUT FreeGLUT::freeglut)
endif()

_rs4090_decide_provider(_rs4090_load_pinocchio pinocchio/urdfdom
    pinocchio urdfdom urdfdom_headers console_bridge)
if(_rs4090_prebuilt_core_robot_io)
    _rs4090_force_provider(_rs4090_load_pinocchio pinocchio/urdfdom
        "prebuilt SMRobotCore::RobotIO runtime dependencies")
endif()
if(_rs4090_load_pinocchio)
    include( Findpinocchio_3rdParty )
endif()

# CoACD is still an optional Collision capability and is not yet represented
# completely in TargetConfigSetting metadata. Keep it loaded conservatively.
include( FindCoACD_3rdParty )
if(RS2026_4090_DEPENDENCY_REPORT)
    message(STATUS
        "[4090 Dependency] CoACD: demand=DEFERRED, actual=LOAD, "
        "reason=optional Collision capability is not fully represented in metadata")
endif()

#   3. Prebuild Packages:
set( CMAKE_MODULE_PATH ${WIN_VS2019_DIR} ) 

#   Find boost
set( Boost_USE_STATIC_LIBS           	OFF )    # only find static libs
# If you need to build a debug program, the following options should be checked (ON).
set( Boost_USE_DEBUG_LIBS            	ON )	# ignore debug libs and
set( Boost_USE_RELEASE_LIBS          	ON )    # only find release libs
set( Boost_USE_MULTITHREADED        	ON )
set( Boost_USE_STATIC_RUNTIME        	OFF )       # VS default to MD
rs_project_required_components(Boost _rs4090_boost_direct_components)
set(_rs4090_boost_provider_components)
rs_project_requires_package(ompl _rs4090_ompl_required)
if(_rs4090_ompl_required)
    list(APPEND _rs4090_boost_provider_components serialization)
endif()
if(_rs4090_load_pinocchio)
    list(APPEND _rs4090_boost_provider_components filesystem system serialization)
endif()
set(_rs4090_boost_effective_components
    ${_rs4090_boost_direct_components}
    ${_rs4090_boost_provider_components})
if(_rs4090_boost_effective_components)
    # The current Boost finder queries Boost::headers unconditionally.
    list(APPEND _rs4090_boost_effective_components headers)
    list(REMOVE_DUPLICATES _rs4090_boost_effective_components)
endif()

_rs4090_decide_provider(_rs4090_load_boost Boost
    Boost ompl pinocchio urdfdom urdfdom_headers console_bridge)
if(RS2026_4090_DEPENDENCY_REPORT)
    message(STATUS
        "[4090 Dependency] Boost direct components: "
        "${_rs4090_boost_direct_components}")
    message(STATUS
        "[4090 Dependency] Boost provider components: "
        "${_rs4090_boost_provider_components}")
    message(STATUS
        "[4090 Dependency] Boost effective components: "
        "${_rs4090_boost_effective_components}")
endif()
if(_rs4090_load_boost)
    if(RS2026_4090_ENABLE_DEMAND_LOADING)
        set(Boost_Components ${_rs4090_boost_effective_components})
    endif()
    include( CMake_FindBoost1.78.0 )
    if(RS2026_4090_ENABLE_DEMAND_LOADING)
        foreach(_component IN LISTS _rs4090_boost_effective_components)
            rs_assert_required_targets(Boost "Boost::${_component}")
        endforeach()
    endif()
endif()


_rs4090_decide_provider(_rs4090_load_assimp assimp assimp)
if(_rs4090_prebuilt_platform_asset_stack)
    _rs4090_force_provider(_rs4090_load_assimp assimp
        "prebuilt SMRobotPlatform asset/render/scene runtime dependency")
endif()
if(_rs4090_load_assimp)
    include( CMake_FindAssimp6.0.2 )
    _rs4090_assert_provider_targets(assimp assimp::assimp)
endif()

_rs4090_decide_provider(_rs4090_load_fcl FCL fcl ccd octomap)
if(_rs4090_prebuilt_core_collision)
    _rs4090_force_provider(_rs4090_load_fcl FCL
        "prebuilt SMRobotCore::Collision public dependency")
endif()
if(_rs4090_load_fcl)
    include( CMake_FindFCL )
    _rs4090_assert_provider_targets(FCL fcl::fcl)
endif()

_rs4090_decide_provider(_rs4090_load_ompl OMPL ompl)
if(_rs4090_load_ompl)
    include( CMake_FindOMPL )
    _rs4090_assert_provider_targets(OMPL ompl::ompl)
endif()

_rs4090_decide_provider(_rs4090_load_ktx KTX KTX)
if(_rs4090_load_ktx)
    include( CMake_FindKtx )
    _rs4090_assert_provider_targets(KTX KTX::ktx)
endif()

# if( NOT ${UsingPrebuild_ALL} )
#     #   Assimp static - needed by SMRobotGen1::ModelTangQing
#     include( CMake_FindAssimp6.0.2 )

#     #   FCL static - needed by SMRobotGen1::ModelTangQing
#     include( CMake_FindFCL )
# endif()


set( CMAKE_MODULE_PATH ${WIN_VSIndep_DIR} ) 
_rs4090_decide_provider(_rs4090_load_qt5 Qt5 Qt5)
if(_rs4090_load_qt5)
    include( CMake_FindQt5155 )
    if(RS2026_4090_ENABLE_DEMAND_LOADING)
        rs_project_required_components(Qt5 _rs4090_qt5_components)
        foreach(_component IN LISTS _rs4090_qt5_components)
            rs_assert_required_targets(Qt5 "Qt5::${_component}")
        endforeach()
    endif()
endif()


_rs4090_decide_provider(_rs4090_load_openssl OpenSSL OpenSSL)
if(_rs4090_load_openssl)
    include( CMake_FindOpenSSL3.6.1 )
endif()
# if( NOT ${UsingPrebuild_ALL} )
#     if( ${BuildPackage_Verification} )
#         include( CMake_FindOpenSSL )
#     endif()
# endif()


_rs4090_decide_provider(_rs4090_load_opengl OpenGL OpenGL)
if(_rs4090_load_opengl)
    find_package( OpenGL REQUIRED )
endif()
#    find_package( OpenGL REQUIRED COMPONENTS OpenGL EGL )
if(_rs4090_load_opengl AND OpenGL_FOUND )
    message( STATUS "OpenGL is FOUND!" )
    if( TARGET OpenGL::GL )
        message( STATUS "Target OpenGL::GL is FOUND!" )
    else()
        message( STATUS "Target OpenGL::GL is NOT found!" )
    endif()
elseif(_rs4090_load_opengl)
    message( WARNING "OpenGL is NOT found!" )
endif()





set( CMAKE_MODULE_PATH ${SAVE_MODULE_PATH} )
message( STATUS "\n------------------ End of finding packages in ${this_cmake_file} ------------------" )
