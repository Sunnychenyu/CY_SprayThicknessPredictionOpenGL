##########################################################################
# CMake requirement
##########################################################################
cmake_minimum_required( VERSION 3.20 )

##########################################################################
# Package ${PACKAGE_NAME}
##########################################################################
get_filename_component( PACKAGE_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME )
set( PKG_MSG_HEADER "[PKG-${PACKAGE_NAME}]:" )

message( STATUS "${PKG_MSG_HEADER} Package ${PACKAGE_NAME}'s Components include: ${${PACKAGE_NAME}_Components}" )
foreach( comp_ ${${PACKAGE_NAME}_Components} )
    message( STATUS "  ----  ${comp_}" )
endforeach()
message( STATUS " " )

message( STATUS "${PKG_MSG_HEADER} ${PACKAGE_NAME}_RequiredLibsPublic: ${${PACKAGE_NAME}_RequiredLibsPublic}" )
message( STATUS "${PKG_MSG_HEADER} ${PACKAGE_NAME}_RequiredLibsPrivate: ${${PACKAGE_NAME}_RequiredLibsPrivate}" )

foreach( comp_ ${${PACKAGE_NAME}_Components} )
    message( STATUS "${PKG_MSG_HEADER} ${PACKAGE_NAME}${comp_}_RequiredLibsPublic: ${${PACKAGE_NAME}${comp_}_RequiredLibsPublic}" )
    message( STATUS "${PKG_MSG_HEADER} ${PACKAGE_NAME}${comp_}_RequiredLibsPrivate: ${${PACKAGE_NAME}${comp_}_RequiredLibsPrivate}" )
endforeach()

##########################################################################
# Set export options
##########################################################################
option( ExportPacakge${PACKAGE_NAME} "Export package -- ${PACKAGE_NAME}" ON )
foreach( export_comp_ ${${PACKAGE_NAME}_ExportComponents} )
    cmake_dependent_option( ExportPacakge${PACKAGE_NAME}_${export_comp_} "${PACKAGE_NAME} : export ${export_comp_}" ON "ExportPacakge${PACKAGE_NAME}" OFF )
endforeach()

##########################################################################
# Build ${PACKAGE_NAME}
##########################################################################
message( STATUS "BuildPackage_${PACKAGE_NAME} is ${BuildPackage_${PACKAGE_NAME}} in ${PACKAGE_NAME} directory." )

if( UsingPrebuilt_${PACKAGE_NAME} )
    message( STATUS "Use prebuilt package ${PACKAGE_NAME}; source components are not added." )
else()
    message( STATUS "Build package ${PACKAGE_NAME} on site" )
    set( PACKAGE_INC_DIR ${CMAKE_CURRENT_LIST_DIR}/include )

    foreach( each_comp_ ${${PACKAGE_NAME}_Components} )
        cmake_dependent_option( Build${PACKAGE_NAME}_${each_comp_} "Build ${PACKAGE_NAME}::${each_comp_}" ON "BuildPackage_${PACKAGE_NAME}" OFF )
        if( Build${PACKAGE_NAME}_${each_comp_} )
            add_subdirectory( ${each_comp_} )
        else()
            message( STATUS "Build${PACKAGE_NAME}_${each_comp_} is ${Build${PACKAGE_NAME}_${each_comp_}} in ${PACKAGE_NAME} directory." )
        endif()
    endforeach()
endif()

##########################################################################
# Generate package
##########################################################################
if( ${ExportPacakge${PACKAGE_NAME}} AND NOT UsingPrebuilt_${PACKAGE_NAME} )
    include(CMakePackageConfigHelpers)
    set(SUPPORTED_COMPONENTS ${${PACKAGE_NAME}_PublicComponents} ${${PACKAGE_NAME}_CompatibilityComponents})

    configure_package_config_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/Config.cmake.in
        ${CMAKE_CURRENT_BINARY_DIR}/${PACKAGE_NAME}Config.cmake
        INSTALL_DESTINATION ${PACKAGE_NAME}/lib/cmake
    )

    write_basic_package_version_file(
        ${PACKAGE_NAME}ConfigVersion.cmake
        VERSION ${PROJECT_VERSION}
        COMPATIBILITY AnyNewerVersion
    )

    install(FILES
        ${CMAKE_CURRENT_BINARY_DIR}/${PACKAGE_NAME}Config.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/${PACKAGE_NAME}ConfigVersion.cmake
        DESTINATION ${PACKAGE_NAME}/lib/cmake
    )

    install(FILES
        ${CMAKE_CURRENT_BINARY_DIR}/${PACKAGE_NAME}Config.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/${PACKAGE_NAME}ConfigVersion.cmake
        DESTINATION ${PACKAGE_NAME}/lib/cmake/${PACKAGE_NAME}
    )
endif()
