############################################################
#   Project Setting
############################################################
message( STATUS "Operation System is : ${CMAKE_SYSTEM} ${CMAKE_SIZEOF_VOID_P}" )


############################################################
#   Set the C++ standard
#   C++ standard 17 is required for Win10 Visual Studio 2019
############################################################
if ( CMAKE_SYSTEM_NAME MATCHES "Linux" )
    message(STATUS "current platform: Linux " )
    set( CMAKE_C_COMPILER "/usr/bin/gcc" )
    set( CMAKE_CXX_COMPILER "/usr/bin/g++" )
    set( CMAKE_CXX_STANDARD 17 )

    # Keep Linux runtime resolution tied to the build/package tree instead of
    # accidentally preferring another Qt installation from /usr or conda.
    set(CMAKE_SKIP_RPATH FALSE)
    set(CMAKE_BUILD_RPATH "$ORIGIN/../lib;$ORIGIN")
    set(CMAKE_BUILD_RPATH_USE_ORIGIN TRUE)
    set(CMAKE_INSTALL_RPATH "$ORIGIN/../lib;$ORIGIN")
    set(CMAKE_INSTALL_RPATH_USE_LINK_PATH FALSE)

    include(ProcessorCount)
    ProcessorCount(RS2026_DETECTED_PROCESSOR_COUNT)
    if(RS2026_DETECTED_PROCESSOR_COUNT EQUAL 0)
        set(RS2026_DETECTED_PROCESSOR_COUNT 1)
    endif()

    set(RS2026_BUILD_PARALLEL_LEVEL "AUTO"
        CACHE STRING "Parallel job count used by RS2026 build helpers. Use AUTO or a positive integer."
    )
    if(RS2026_BUILD_PARALLEL_LEVEL STREQUAL "AUTO")
        set(RS2026_EFFECTIVE_BUILD_PARALLEL_LEVEL "${RS2026_DETECTED_PROCESSOR_COUNT}")
    elseif(RS2026_BUILD_PARALLEL_LEVEL MATCHES "^[1-9][0-9]*$")
        set(RS2026_EFFECTIVE_BUILD_PARALLEL_LEVEL "${RS2026_BUILD_PARALLEL_LEVEL}")
    else()
        message(FATAL_ERROR "RS2026_BUILD_PARALLEL_LEVEL must be AUTO or a positive integer.")
    endif()

    set(ENV{CMAKE_BUILD_PARALLEL_LEVEL} "${RS2026_EFFECTIVE_BUILD_PARALLEL_LEVEL}")
    message(STATUS "Building with ${RS2026_EFFECTIVE_BUILD_PARALLEL_LEVEL} parallel jobs")

    if(CMAKE_GENERATOR STREQUAL "Unix Makefiles")
        set(RS2026_MAKE_PARALLEL_WRAPPER "${CMAKE_BINARY_DIR}/rs2026_make_parallel.sh")

        if(NOT DEFINED RS2026_REAL_MAKE_PROGRAM)
            set(RS2026_REAL_MAKE_PROGRAM "${CMAKE_MAKE_PROGRAM}"
                CACHE FILEPATH "Original make program used by RS2026 parallel wrapper."
            )
        endif()

        file(WRITE "${RS2026_MAKE_PARALLEL_WRAPPER}"
            "#!/usr/bin/env sh\n"
            "exec \"${RS2026_REAL_MAKE_PROGRAM}\" -j${RS2026_EFFECTIVE_BUILD_PARALLEL_LEVEL} \"$@\"\n"
        )
        file(CHMOD "${RS2026_MAKE_PARALLEL_WRAPPER}"
            PERMISSIONS
                OWNER_READ OWNER_WRITE OWNER_EXECUTE
                GROUP_READ GROUP_EXECUTE
                WORLD_READ WORLD_EXECUTE
        )

        set(CMAKE_MAKE_PROGRAM "${RS2026_MAKE_PARALLEL_WRAPPER}"
            CACHE FILEPATH "Make program wrapped with RS2026 automatic parallel jobs." FORCE
        )
        message(STATUS "Unix Makefiles use ${RS2026_MAKE_PARALLEL_WRAPPER} for parallel builds")
    endif()


elseif( CMAKE_SYSTEM_NAME MATCHES "Windows" )
    message( STATUS "current platform: Windows" )
    set( CMAKE_CXX_STANDARD 17 )
else ()
    message( STATUS "other platform: ${CMAKE_SYSTEM_NAME}" )
endif ()

############################################################
#   Check the system bits (Correct Way)
############################################################
message( STATUS "CMAKE_SIZEOF_VOID_P is ${CMAKE_SIZEOF_VOID_P}." )
if( CMAKE_SIZEOF_VOID_P EQUAL 8 )
    message( STATUS "System is 64 bits\n" )
    set( CMAKE_DEBUG_POSTFIX        "dx64" )
    set( CMAKE_RELEASE_POSTFIX      "rx64" )
else()
    message( STATUS "System is 32 bits\n" )
    set( CMAKE_DEBUG_POSTFIX        "dx86" )
    set( CMAKE_RELEASE_POSTFIX      "rx86" )
endif()

############################################################
#   Set flags for c++ and c
############################################################
if( CMAKE_COMPILER_IS_GNUCXX )
    set( CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS}" )
    set( CMAKE_CXX_FLAGS "-fPIC ${CMAKE_CXX_FLAGS}" )
    message(STATUS "$GNUCXX optional:-std=c++14 -fPIC" )   
endif()

if( CMAKE_COMPILER_IS_GNUCC ) 
    set( CMAKE_C_FLAGS "-fPIC ${CMAKE_C_FLAGS}" )
    message(STATUS "GNUCC optional:-std=c++14 -fPIC" ) 
endif()


############################################################
#   Build options setting
############################################################
#	Set configuration : Debug, Release
set( CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE STRING "Configurations" FORCE )
set( CMAKE_BUILD_TYPE "Debug" CACHE STRING "Current Configuration" )

#   Use folders for different target in Visual Studio
set_property( GLOBAL PROPERTY USE_FOLDERS ON )
set( CMAKE_CXX_STANDARD_REQUIRED ON )
set( CMAKE_INCLUDE_CURRENT_DIR_IN_INTERFACE ON )
set( CMAKE_POSITION_INDEPENDENT_CODE ON )

if( CMAKE_VERSION VERSION_LESS "3.7.0")
    set( CMAKE_INCLUDE_CURRENT_DIR ON )
endif()

# SET( CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /bigobj" )




############################################################
#   Project Output Setting
############################################################
#   Output directory setting
set( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/$<CONFIG>/bin )
set( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/$<CONFIG>/lib ) 
#	Windows library output
set( CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/$<CONFIG>/lib )

#   Install prefix setting
if( CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT )
    set( CMAKE_INSTALL_PREFIX "${PROJECT_SOURCE_DIR}/PrebuiltPackages" CACHE PATH
        "Install directory for project packages" FORCE
    )
endif()

