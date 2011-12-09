PROJECT( CCAMERAMODEL )
CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )

################################################################################
SET( CCAMERAMODEL_LIBRARIES "" )
SET( CCAMERAMODEL_INCLUDE_DIR ${PROJECT_SOURCE_DIR} )
SET( CCAMERAMODEL_DEFINITIONS "" )

################################################################################
# Set the local version first
SET( CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake_modules" )
LIST( APPEND CMAKE_MODULE_PATH "${CMAKE_ROOT}/Modules" )
#MESSAGE( STATUS "" ${CMAKE_ROOT} )
#MESSAGE( STATUS "" ${CMAKE_MODULE_PATH} )

################################################################################
include( "../cmake_modules/FindPackageAndAdd.cmake" )

################################################################################
find_package_and_add( OPENCV CCAMERAMODEL )
SET( CCAMERAMODEL_HAS_OPENCV "#define CCAMERAMODEL_HAS_OPENCV 1" )

find_package_and_add( Eigen3 CCAMERAMODEL )
SET( CCAMERAMODEL_HAS_EIGEN3 "#define CCAMERAMODEL_HAS_EIGEN3 1" )

find_package_and_add( CGEOM CCAMERAMODEL )

################################################################################
IF( NOT CMAKE_BUILD_TYPE )
  SET( CMAKE_BUILD_TYPE Release CACHE STRING "CMAKE_BUILD_TYPE" FORCE )
ENDIF( NOT CMAKE_BUILD_TYPE )

IF( "${CMAKE_BUILD_TYPE}" STREQUAL "Release" )
    # FIX ME...
    SET( CMAKE_C_FLAGS   " -g -Wall -Wextra ")
    SET( CMAKE_CXX_FLAGS " -g -Wall -Wextra ")
ENDIF( "${CMAKE_BUILD_TYPE}" STREQUAL "Release" )

IF( "${CMAKE_BUILD_TYPE}" STREQUAL "Debug" )
    SET( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -g " )
    SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -g " )
ENDIF( "${CMAKE_BUILD_TYPE}" STREQUAL "Debug" )

IF( "${CMAKE_BUILD_TYPE}" STREQUAL "Fast" )
    SET( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra " )
    SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra " )
ENDIF( "${CMAKE_BUILD_TYPE}" STREQUAL "Fast" )

################################################################################
FIND_PACKAGE( CEIGEN REQUIRED )
SET( CCAMERAMODEL_INCLUDE_DIR ${CCAMERAMODEL_INCLUDE_DIR} ${CEIGEN_INCLUDE_DIR} )
SET( CCAMERAMODEL_LIBRARIES ${CCAMERAMODEL_LIBRARIES} ${CEIGEN_LIBRARIES} )
SET( CCAMERAMODEL_DEFINITIONS ${CCAMERAMODEL_DEFINITIONS} ${CEIGEN_DEFINITIONS} )

################################################################################
# Create the CBriefIncludes.h file
CONFIGURE_FILE( ${CMAKE_CURRENT_SOURCE_DIR}/CCameraModelIncludes.cmake.in
    ${CMAKE_CURRENT_SOURCE_DIR}/CCameraModelIncludes.h @ONLY IMMEDIATE )

################################################################################
# Build CCameraModel
add_subdirectory( CCameraModel )
message( STATUS "CCAMERAMODEL_LIBRARIES: " ${CCAMERAMODEL_LIBRARIES} )

################################################################################
# Optionally build demos
OPTION( BUILD_CAMERA_MODEL_DEMOS "Build demos for using CFigs." ON )

IF( BUILD_CAMERA_MODEL_DEMOS )
  SUBDIRS( Demos )
ENDIF( BUILD_CAMERA_MODEL_DEMOS )

################################################################################
# Target etags/tags
#ADD_CUSTOM_TARGET( tags etags `find . -name *.cc -or -name *.hh -or -name *.cpp -or -name *.h -or -name *.c -or -name *.f` )
#ADD_CUSTOM_TARGET( etags DEPENDS tags )

################################################################################
# Create the CCAMERAMODELConfig.cmake file for other cmake projects.
CONFIGURE_FILE( ${CMAKE_CURRENT_SOURCE_DIR}/CCAMERAMODELConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/CCAMERAMODELConfig.cmake @ONLY IMMEDIATE )

################################################################################
export( PACKAGE CCAMERAMODEL )