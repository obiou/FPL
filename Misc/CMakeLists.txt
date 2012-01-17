PROJECT( MISC )
CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )

################################################################################
SET( MISC_LIBRARIES "" )
SET( MISC_INCLUDE_DIR ${PROJECT_SOURCE_DIR} )
SET( MISC_DEFINITIONS "" )

################################################################################
# Set the local version first
SET( CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake_modules" )
LIST( APPEND CMAKE_MODULE_PATH "${CMAKE_ROOT}/Modules" )
#MESSAGE( STATUS "" ${CMAKE_ROOT} )
#MESSAGE( STATUS "" ${CMAKE_MODULE_PATH} )

################################################################################
# Optionally build demos
OPTION( BUILD_MISC_DEMOS "Build demos for using COPENCV." ON )

IF( BUILD_MISC_DEMOS )
  SUBDIRS( Demos )
ENDIF( BUILD_MISC_DEMOS )

################################################################################
IF( NOT CMAKE_BUILD_TYPE )
  SET( CMAKE_BUILD_TYPE Release CACHE STRING "CMAKE_BUILD_TYPE" FORCE )
ENDIF( NOT CMAKE_BUILD_TYPE )

IF( "${CMAKE_BUILD_TYPE}" STREQUAL "Debug" )
  SET( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -Wall -Wextra -Wconversion -Wuninitialized " )
  SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall  -Wextra -Wconversion -Wuninitialized " )
ENDIF( "${CMAKE_BUILD_TYPE}" STREQUAL "Debug" )
 
#SET( CMAKE_OSX_ARCHITECTURES "i686" )
#SET( CMAKE_OSX_ARCHITECTURES "x86_64" )

IF( "${CMAKE_BUILD_TYPE}" STREQUAL "Release" )
    SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall -O3 -funroll-loops -finline-functions  -msse -msse2 -msse3 -mssse3 " )
    SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall -O3 -funroll-loops -finline-functions  -msse -msse2 -msse3 -mssse3 " )
ENDIF( "${CMAKE_BUILD_TYPE}" STREQUAL "Release" )

IF( "${CMAKE_BUILD_TYPE}" STREQUAL "Fast" )
    SET( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra " )
    SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra " )
ENDIF( "${CMAKE_BUILD_TYPE}" STREQUAL "Fast" )

################################################################################
# Create the MiscConfig.cmake file for other cmake projects.
CONFIGURE_FILE( ${CMAKE_CURRENT_SOURCE_DIR}/MISCConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/MISCConfig.cmake @ONLY IMMEDIATE )

export( PACKAGE MISC )