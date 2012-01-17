PROJECT( CTRACK )
CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )

################################################################################
SET( CTRACK_LIBRARIES "" )
SET( CTRACK_INCLUDE_DIR ${PROJECT_SOURCE_DIR} )
SET( CTRACK_DEFINITIONS "" )

################################################################################
# Set the local version first
SET( CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake_modules" )
LIST( APPEND CMAKE_MODULE_PATH "${CMAKE_ROOT}/Modules" )

include( "../cmake_modules/FindPackageAndAdd.cmake" )

################################################################################
find_package( Eigen3 REQUIRED )
SET( CTRACK_INCLUDE_DIR ${CTRACK_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR} )

################################################################################
find_package( OPENCV QUIET )
if( OPENCV_FOUND )
    find_package_and_add( OPENCV CTRACK )
    message( STATUS "CTRACK: Adding support for OpenCV" )
    SET( CTRACK_HAS_OPENCV "#define CTRACK_HAS_OPENCV 1" )
else()
    message( STATUS "CTRACK: OpenCV NOT found" )
endif()

################################################################################
IF( NOT CMAKE_BUILD_TYPE )
  SET( CMAKE_BUILD_TYPE Debug CACHE STRING "CMAKE_BUILD_TYPE" FORCE )
ENDIF( NOT CMAKE_BUILD_TYPE )

if( "${CMAKE_BUILD_TYPE}" STREQUAL "Debug" )
  set( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -Wall -Wextra -Wuninitialized " )
  set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall  -Wextra -Wuninitialized " )
endif( "${CMAKE_BUILD_TYPE}" STREQUAL "Debug" )

IF( "${CMAKE_BUILD_TYPE}" STREQUAL "Release" )
  SET( CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -O3 -funroll-loops -finline-functions " )
  SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -funroll-loops -finline-functions " )
ENDIF( "${CMAKE_BUILD_TYPE}" STREQUAL "Release" )

IF( "${CMAKE_BUILD_TYPE}" STREQUAL "Fast" )
  SET( CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -Wall -Wextra " )
  SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra " )
ENDIF( "${CMAKE_BUILD_TYPE}" STREQUAL "Fast" )

####################################### DOXYGEN ################################
add_custom_target( doc doxygen docs/doxygen.cfg\; 
  WORKING_DIRECTORY ${PROJ_SOURCE_DIR} 
  COMMENT "Generating documentation." 
)

###############################################################################
CONFIGURE_FILE( ${CMAKE_CURRENT_SOURCE_DIR}/ctrack_includes.cmake.in
  ${CMAKE_CURRENT_SOURCE_DIR}/ctrack_includes.h @ONLY IMMEDIATE )

################################################################################
# Build CTrack
add_subdirectory( CTrack )

################################################################################
OPTION( BUILD_CTRACK_DEMOS "Build demos for using CTrack." ON )

IF( BUILD_CTRACK_DEMOS )
  add_subdirectory( Demos )
ENDIF( BUILD_CTRACK_DEMOS )

################################################################################
# target etags/tags
#ADD_CUSTOM_TARGET( tags etags `find . -name *.cc -or -name *.hh -or -name *.cpp -or -name *.h -or -name *.c -or -name *.f` )
#ADD_CUSTOM_TARGET( etags DEPENDS tags )

###############################################################################
CONFIGURE_FILE( ${CMAKE_CURRENT_SOURCE_DIR}/CTRACKConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/CTRACKConfig.cmake @ONLY IMMEDIATE )

################################################################################
export( PACKAGE CTRACK )