#############################################################
# Find prebuilt packages in dependency order.
#############################################################

function(rs_find_requested_prebuilt_package PACKAGE_NAME)
    if(NOT UsingPrebuilt_${PACKAGE_NAME})
        message(STATUS "Skip prebuilt package ${PACKAGE_NAME}; UsingPrebuilt_${PACKAGE_NAME}=OFF")
        return()
    endif()

    if(DEFINED ${PACKAGE_NAME}_DepLibs)
        set(_prebuilt_components_to_find ${${PACKAGE_NAME}_DepLibs})
    elseif(DEFINED ${PACKAGE_NAME}_ExportComponents)
        set(_prebuilt_components_to_find ${${PACKAGE_NAME}_ExportComponents})
    else()
        set(_prebuilt_components_to_find ${${PACKAGE_NAME}_PrebuiltComponents})
    endif()

    message(STATUS "Finding Prebuilt Package ${PACKAGE_NAME} with components ${_prebuilt_components_to_find}")
    find_prebuilt_package(
        PACKAGE_NAME ${PACKAGE_NAME}
        COMPONENTS ${_prebuilt_components_to_find}
    )
endfunction()
