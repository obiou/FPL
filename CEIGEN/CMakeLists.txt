project( CEIGEN )
cmake_minimum_required( VERSION 2.8 )

################################################################################
set( CEIGEN_LIBRARIES "" )
set( CEIGEN_INCLUDE_DIR ${PROJECT_SOURCE_DIR} )

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
find_package( Eigen3 REQUIRED )
SET( CEIGEN_INCLUDE_DIR ${CEIGEN_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR} )

################################################################################
FIND_PACKAGE( OPENCV QUIET )
if( OPENCV_FOUND )
    SET( CEIGEN_INCLUDE_DIR ${CEIGEN_INCLUDE_DIR} ${OPENCV_INCLUDE_DIR} )
    SET( CEIGEN_LIBRARIES   ${CEIGEN_LIBRARIES}   ${OPENCV_LIBRARIES} )
    SET( CEIGEN_HAS_OPENCV "#define CEIGEN_HAS_OPENCV 1" )
endif()

################################################################################
find_package_and_add( COPENCV CEIGEN )

################################################################################
# Create the CEIGENIncludes.h
CONFIGURE_FILE( ${CMAKE_CURRENT_SOURCE_DIR}/CEIGENIncludes.cmake.in
    ${CMAKE_CURRENT_SOURCE_DIR}/CEIGENIncludes.h @ONLY IMMEDIATE )

################################################################################
add_subdirectory( ceigen )

################################################################################
# Optionally build demos
OPTION( BUILD_CEIGEN_DEMOS "Build demos for using CEIGEN." ON )

if( BUILD_CEIGEN_DEMOS )
  subdirs( Demos )
endif( BUILD_CEIGEN_DEMOS )

################################################################################
MESSAGE( STATUS "BUILD_TYPE: " ${CMAKE_BUILD_TYPE} )

if( NOT CMAKE_BUILD_TYPE )
  set( CMAKE_BUILD_TYPE Release CACHE STRING "CMAKE_BUILD_TYPE" FORCE )
endif( NOT CMAKE_BUILD_TYPE )

if( "${CMAKE_BUILD_TYPE}" STREQUAL "Debug" )
  set( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -Wall -Wextra " )
  set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall  -Wextra " )
endif( "${CMAKE_BUILD_TYPE}" STREQUAL "Debug" )
 
#set( CMAKE_OSX_ARCHITECTURES "i686" )
#set( CMAKE_OSX_ARCHITECTURES "x86_64" )

if( "${CMAKE_BUILD_TYPE}" STREQUAL "Release" )
    set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall -O3 -funroll-loops -finline-functions  -msse -msse2 -msse3 -mssse3 " )
    set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall -O3 -funroll-loops -finline-functions  -msse -msse2 -msse3 -mssse3 " )
endif( "${CMAKE_BUILD_TYPE}" STREQUAL "Release" )

IF( "${CMAKE_BUILD_TYPE}" STREQUAL "Fast" )
  SET( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra " )
  SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra " )
ENDIF( "${CMAKE_BUILD_TYPE}" STREQUAL "Fast" )

################################################################################
# Create the CEIGENConfig.cmake file for other cmake projects.
CONFIGURE_FILE( ${CMAKE_CURRENT_SOURCE_DIR}/CEIGENConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/CEIGENConfig.cmake @ONLY IMMEDIATE )

export( PACKAGE CEIGEN )
