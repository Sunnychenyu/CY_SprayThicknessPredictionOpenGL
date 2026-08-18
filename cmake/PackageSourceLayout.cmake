############################################################
#   Package Source Layout
############################################################

set(AdditionalBuildPackages)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/SimWorkbench/SimWorkbenchPackages.cmake")
    include("${CMAKE_CURRENT_SOURCE_DIR}/SimWorkbench/SimWorkbenchPackages.cmake")
    list(APPEND AdditionalBuildPackages ${SimWorkbenchPackages})
endif()

function(rs_get_package_source_dir PACKAGE_NAME OUT_VAR)
    if(DEFINED PackageSourceDir_${PACKAGE_NAME})
        set(${OUT_VAR} "${PackageSourceDir_${PACKAGE_NAME}}" PARENT_SCOPE)
    else()
        set(${OUT_VAR} "${PACKAGE_NAME}" PARENT_SCOPE)
    endif()
endfunction()
