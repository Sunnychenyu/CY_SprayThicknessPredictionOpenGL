############################################################
#   Logical package compatibility helpers
############################################################

include(CMakePackageConfigHelpers)

function(rs_define_interface_alias NEW_TARGET LEGACY_TARGET)
    if(TARGET ${NEW_TARGET} OR NOT TARGET ${LEGACY_TARGET})
        return()
    endif()

    add_library(${NEW_TARGET} INTERFACE IMPORTED)
    set_target_properties(${NEW_TARGET}
        PROPERTIES
            INTERFACE_LINK_LIBRARIES ${LEGACY_TARGET}
    )
endfunction()

function(rs_create_logical_package_alias_targets)
    set(SMRobotCore_LogicalComponents
        RobotCore
        RobotRuntime
        RobotInstance
        RobotIO
        Kinematics
        Collision
        RobotSDK
        RobotTrajectoryCore
    )

    set(SMRobotPlatform_LogicalComponents
        AssetCore
        CameraCore
        SceneCore
        RenderCore
        RobotRenderBridge
        SimulationProject
        SimulationRuntime
        SensorCore
        SensorSimulation
    )

    set(SMRobotApps_LogicalComponents
        RobotViewerCore
        RobotQtViewer
        RobotSDKVisualQuickStart
    )

    foreach(component_ IN LISTS SMRobotCore_LogicalComponents)
        rs_define_interface_alias(SMRobotCore::${component_} SMRobotGen2::${component_})
        rs_define_interface_alias(SMRobotGen2::${component_} SMRobotCore::${component_})
    endforeach()

    foreach(component_ IN LISTS SMRobotPlatform_LogicalComponents)
        rs_define_interface_alias(SMRobotPlatform::${component_} SMRobotGen2::${component_})
        rs_define_interface_alias(SMRobotGen2::${component_} SMRobotPlatform::${component_})
    endforeach()

    foreach(component_ IN LISTS SMRobotApps_LogicalComponents)
        rs_define_interface_alias(SMRobotApps::${component_} Applications::${component_})
        rs_define_interface_alias(Applications::${component_} SMRobotApps::${component_})
    endforeach()
endfunction()

function(rs_apply_logical_prebuilt_alias_options)
    option(UsingPrebuilt_SMRobotCore "Use logical prebuilt SMRobotCore package" OFF)
    option(UsingPrebuilt_SMRobotPlatform "Use logical prebuilt SMRobotPlatform package" OFF)
    option(UsingPrebuilt_SMRobotApps "Use logical prebuilt SMRobotApps package" OFF)

    if(UsingPrebuilt_SMRobotPlatform OR UsingPrebuilt_SMRobotGen2)
        set(SMRobotGen2_EXPORT_PROFILE "Platform" CACHE STRING
            "SMRobotGen2 export profile: Core or Platform"
            FORCE
        )
    elseif(UsingPrebuilt_SMRobotCore)
        set(SMRobotGen2_EXPORT_PROFILE "Core" CACHE STRING
            "SMRobotGen2 export profile: Core or Platform"
            FORCE
        )
    endif()
endfunction()

function(rs_install_compat_package NEW_PACKAGE LEGACY_PACKAGE)
    set(options)
    set(oneValueArgs)
    set(multiValueArgs COMPONENTS)
    cmake_parse_arguments(ARG "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    if(NOT ARG_COMPONENTS)
        message(FATAL_ERROR "rs_install_compat_package requires COMPONENTS")
    endif()

    set(SUPPORTED_COMPONENTS ${ARG_COMPONENTS})
    set(NEW_PACKAGE "${NEW_PACKAGE}")
    set(LEGACY_PACKAGE "${LEGACY_PACKAGE}")
    set(_compat_config "${CMAKE_CURRENT_BINARY_DIR}/${NEW_PACKAGE}Config.cmake")
    set(_compat_version "${CMAKE_CURRENT_BINARY_DIR}/${NEW_PACKAGE}ConfigVersion.cmake")

    configure_package_config_file(
        "${PROJECT_SOURCE_DIR}/cmake/CompatibilityPackageConfig.cmake.in"
        "${_compat_config}"
        INSTALL_DESTINATION "${NEW_PACKAGE}/lib/cmake"
    )

    write_basic_package_version_file(
        "${_compat_version}"
        VERSION ${PROJECT_VERSION}
        COMPATIBILITY AnyNewerVersion
    )

    install(FILES
        "${_compat_config}"
        "${_compat_version}"
        DESTINATION "${NEW_PACKAGE}/lib/cmake"
    )

    install(FILES
        "${_compat_config}"
        "${_compat_version}"
        DESTINATION "${NEW_PACKAGE}/lib/cmake/${NEW_PACKAGE}"
    )
endfunction()
