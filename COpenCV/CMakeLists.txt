PROJECT( COPENCV )
CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )

################################################################################
SET( COPENCV_LIBRARIES "" )
SET( COPENCV_INCLUDE_DIR "${PROJECT_SOURCE_DIR}" )

################################################################################
# Set the local version first
SET( CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake_modules" )
LIST( APPEND CMAKE_MODULE_PATH "${CMAKE_ROOT}/Modules" )
#MESSAGE( STATUS "" ${CMAKE_ROOT} )
#MESSAGE( STATUS "" ${CMAKE_MODULE_PATH} )

################################################################################
# Get OpenCV
FIND_PACKAGE( OPENCV REQUIRED )
SET( COPENCV_LIBRARIES ${COPENCV_LIBRARIES} ${OpenCV_LIBRARIES} )
SET( COPENCV_INCLUDE_DIR ${COPENCV_INCLUDE_DIR} ${OPENCV_INCLUDE_DIR} )

################################################################################
FIND_PACKAGE( CVARS )

if( CVARS_FOUND )
    SET( COPENCV_LIBRARIES ${COPENCV_LIBRARIES} ${CVARS_LIBRARIES} )
    SET( COPENCV_INCLUDE_DIR ${COPENCV_INCLUDE_DIR} ${CVARS_INCLUDE_DIR} )
    SET( COPENCV_HAS_CVARS "#define COPENCV_HAS_CVARS 1" )
endif( CVARS_FOUND )

################################################################################
find_package( CIMAGE )
if( CIMAGE_FOUND )
    #message( STATUS "Found cimage" )
    SET( COPENCV_LIBRARIES ${COPENCV_LIBRARIES} ${CIMAGE_LIBRARIES} )
    SET( COPENCV_INCLUDE_DIR ${COPENCV_INCLUDE_DIR} ${CIMAGE_INCLUDE_DIR} )
    set( COPENCV_HAS_CIMAGE "#define COPENCV_HAS_CIMAGE 1" )
endif()

################################################################################
IF( NOT CMAKE_BUILD_TYPE )
  SET( CMAKE_BUILD_TYPE Release CACHE STRING "CMAKE_BUILD_TYPE" FORCE )
ENDIF( NOT CMAKE_BUILD_TYPE )

IF( "${CMAKE_BUILD_TYPE}" STREQUAL "Release" )
  SET( CMAKE_C_FLAGS "${CMAKE_C_FLAGS}     -g -Wall -O3 -funroll-loops -finline-functions ")
  SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall -O3 -funroll-loops -finline-functions ")
ENDIF( "${CMAKE_BUILD_TYPE}" STREQUAL "Release" )

IF( "${CMAKE_BUILD_TYPE}" STREQUAL "Fast" )
    SET( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra " )
    SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra " )
ENDIF( "${CMAKE_BUILD_TYPE}" STREQUAL "Fast" )

################################################################################
# Create the copencv_includes.h
configure_file( ${CMAKE_CURRENT_SOURCE_DIR}/copencv_includes.cmake.in
    ${CMAKE_CURRENT_SOURCE_DIR}/copencv_includes.h @ONLY IMMEDIATE )

################################################################################
# Build COPENCV
INCLUDE_DIRECTORIES( ${COPENCV_INCLUDE_DIR} )
ADD_DEFINITIONS( ${COPENCV_DEFINITIONS} )
set( LIB_SRCS
  COpenCV/copencv_basics.cpp
)
set( LIB_HDRS
  COpenCV/copencv_basics.h
)
add_library( copencv SHARED ${LIB_SRCS} ${LIB_HDRS} )
target_link_libraries( copencv  ${COPENCV_LIBRARIES}  )
get_target_property( COPENCV_LIBRARY copencv LOCATION )
set( COPENCV_LIBRARIES ${COPENCV_LIBRARIES} ${COPENCV_LIBRARY} )

################################################################################
# Optionally build demos
OPTION( BUILD_COPENCV_DEMOS "Build demos for using COpenCV." ON )

IF( BUILD_COPENCV_DEMOS )
  SUBDIRS( Demos )
ENDIF( BUILD_COPENCV_DEMOS )

################################################################################
# Target etags/tags
#ADD_CUSTOM_TARGET( tags etags `find . -name *.cc -or -name *.hh -or -name *.cpp -or -name *.h -or -name *.c -or -name *.f` )
#ADD_CUSTOM_TARGET( etags DEPENDS tags )

################################################################################
# Create the COpenCVConfig.cmake file for other cmake projects.
# requires COPENCV_LIBRARIES, COPENCV_INCLUDE_DIR, COPENCV_DEFINITIONS
CONFIGURE_FILE( ${CMAKE_CURRENT_SOURCE_DIR}/COPENCVConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/COPENCVConfig.cmake @ONLY IMMEDIATE )

################################################################################
export( PACKAGE COPENCV )