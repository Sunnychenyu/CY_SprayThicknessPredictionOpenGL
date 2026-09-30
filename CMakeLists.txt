cmake_minimum_required(VERSION 3.20)

get_filename_component(TARGET_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)

set(SOURCES
    src/OpenGLComputeProgram.cpp
    src/OpenGLComputeProgram.h
    src/OpenGLThicknessPredictionBackend.cpp
    src/AxisymmetricProfileReduction.cpp
    src/PaperGaussianHistoryShader.h
    src/PeriodicSectorMappingShader.h
    src/PeriodicSectorReduction.cpp
    src/PeriodicSectorReduction.h
    src/SpatialInfluenceFiltering.cpp
    src/SpatialFilteredGaussianHistoryShader.h
    src/ThicknessBvh.cpp
)

set(INTERFACE_HEADERS
    include/${TARGET_NAME}/OpenGLThicknessPredictionBackend.h
    include/${TARGET_NAME}/AxisymmetricProfileReduction.h
    include/${TARGET_NAME}/PeriodicSectorReduction.h
    include/${TARGET_NAME}/SpatialInfluenceFiltering.h
    include/${TARGET_NAME}/ThicknessBvh.h
)

add_library(${TARGET_NAME} STATIC ${SOURCES} ${INTERFACE_HEADERS})
add_library(${PACKAGE_NAME}::${TARGET_NAME} ALIAS ${TARGET_NAME})

target_include_directories(${TARGET_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${PACKAGE_NAME}/include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/include>
)

target_link_libraries(${TARGET_NAME}
    PUBLIC ${${PACKAGE_NAME}${TARGET_NAME}_RequiredLibsPublic}
    PRIVATE ${${PACKAGE_NAME}${TARGET_NAME}_RequiredLibsPrivate}
)

target_compile_features(${TARGET_NAME} PUBLIC cxx_std_17)

if(BuildTests)
    add_subdirectory(
        regression/PeriodicSectorReductionTest
        ${PROJECT_BINARY_DIR}/tests/periodic_sector_reduction)
    add_subdirectory(
        regression/SpatialInfluenceFilteringTest
        ${PROJECT_BINARY_DIR}/tests/spatial_influence_filtering)
    add_subdirectory(
        regression/ThicknessBvhOcclusionTest
        ${PROJECT_BINARY_DIR}/tests/thickness_bvh_occlusion)
endif()

set_target_properties(${TARGET_NAME} PROPERTIES
    FOLDER ${PACKAGE_NAME}
    DEBUG_POSTFIX _shared_${CMAKE_DEBUG_POSTFIX}
    RELEASE_POSTFIX _shared_${CMAKE_RELEASE_POSTFIX}
)

if(ExportPacakge${PACKAGE_NAME}_${TARGET_NAME})
    function_InstallTarget(
        TARGET_NAME ${TARGET_NAME}
        PACKAGE_NAME ${PACKAGE_NAME}
        PACKAGE_INTERFACE_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
endif()
