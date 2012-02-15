project( CIMAGE )
cmake_minimum_required( VERSION 2.8 )

################################################################################
# Guard against in-source builds
if( ${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR} )
  message( FATAL_ERROR "In-source builds not allowed. Please make a new directory (called a build directory) and run CMake from there. You may need to remove CMakeCache.txt. ")
endif()

################################################################################
set( CIMAGE_LIBRARIES "" )
set( CIMAGE_INCLUDE_DIR ${PROJECT_SOURCE_DIR} )
set( CIMAGE_DEFINITIONS "" )

################################################################################
# Set the local version first
set( CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake_modules" )
list( APPEND CMAKE_MODULE_PATH "${CMAKE_ROOT}/Modules" )
#MESSAGE( STATUS "" ${CMAKE_ROOT} )
#MESSAGE( STATUS "" ${CMAKE_MODULE_PATH} )

################################################################################
# Simplify adding dependencies
include( "../cmake_modules/FindPackageAndAdd.cmake" ) 

################################################################################
find_package_and_add( OPENCV2 CIMAGE )

################################################################################
add_subdirectory( cimage )
add_subdirectory( Tests )

################################################################################
# Optionally build demos
OPTION( BUILD_CIMAGE_DEMOS "Build demos for using CIMAGE." ON )

if( BUILD_CIMAGE_DEMOS )
  subdirs( Demos )
endif( BUILD_CIMAGE_DEMOS )

################################################################################
MESSAGE( STATUS "BUILD_TYPE: " ${CMAKE_BUILD_TYPE} )

if( NOT CMAKE_BUILD_TYPE )
  set( CMAKE_BUILD_TYPE Release CACHE STRING "CMAKE_BUILD_TYPE" FORCE )
endif( NOT CMAKE_BUILD_TYPE )

if( "${CMAKE_BUILD_TYPE}" STREQUAL "Debug" )
  set( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -Wall -Wextra -Wuninitialized " )
  set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall  -Wextra -Wuninitialized " )
endif( "${CMAKE_BUILD_TYPE}" STREQUAL "Debug" )
 
#set( CMAKE_OSX_ARCHITECTURES "i686" )
#set( CMAKE_OSX_ARCHITECTURES "x86_64" )

if( "${CMAKE_BUILD_TYPE}" STREQUAL "Release" )
    set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -funroll-loops -finline-functions  -msse -msse2 -msse3 -mssse3 " )
    set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -funroll-loops -finline-functions  -msse -msse2 -msse3 -mssse3 " )
endif( "${CMAKE_BUILD_TYPE}" STREQUAL "Release" )

if( "${CMAKE_BUILD_TYPE}" STREQUAL "Fast" )
    set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra" )
    set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra" )
endif( "${CMAKE_BUILD_TYPE}" STREQUAL "Fast" )

################################################################################
# Create the CIMAGEConfig.cmake file for other cmake projects.
CONFIGURE_FILE( ${CMAKE_CURRENT_SOURCE_DIR}/CIMAGEConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/CIMAGEConfig.cmake @ONLY IMMEDIATE )

################################################################################
export( PACKAGE CIMAGE )
