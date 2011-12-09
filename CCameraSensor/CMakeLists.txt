PROJECT( CCAMERASENSOR )
CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )

################################################################################
SET( CCAMERASENSOR_LIBRARIES "" )
SET( CCAMERASENSOR_INCLUDE_DIR ${PROJECT_SOURCE_DIR} )
SET( CCAMERASENSOR_DEFINITIONS "" )

################################################################################
# Set the local version first
SET( CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake_modules" )
LIST( APPEND CMAKE_MODULE_PATH "${CMAKE_ROOT}/Modules" )
#MESSAGE( STATUS "" ${CMAKE_ROOT} )
#MESSAGE( STATUS "" ${CMAKE_MODULE_PATH} )

include( "../cmake_modules/FindPackageAndAdd.cmake" )

################################################################################
# Get OpenCV
find_package_and_add( OPENCV CCAMERASENSOR )
SET( CCAMERASENSOR_HAS_OPENCV "#define CCAMERASENSOR_HAS_OPENCV 1" )

################################################################################
# Get Misc
find_package_and_add( MISC CCAMERASENSOR )

################################################################################
# Get UEye
FIND_PACKAGE( UEYE  QUIET)
IF( UEYE_FOUND )
  MESSAGE( STATUS "Found UEye, adding support." )
  SET( CCAMERASENSOR_LIBRARIES ${CCAMERASENSOR_LIBRARIES} ${UEYE_LIBRARIES} )
  SET( CCAMERASENSOR_INCLUDE_DIR ${CCAMERASENSOR_INCLUDE_DIR} ${UEYE_INCLUDE_DIR} )
  SET( CCAMERASENSOR_HAS_UEYE "#define CCAMERASENSOR_HAS_UEYE 1" )
  if( UNIX )
      SET( CCAMERASENSOR_DEFINITIONS ${CCAMERASENSOR_DEFINITIONS} "-D__LINUX__" )
  endif()
ELSE( UEYE_FOUND )
  MESSAGE( STATUS "Did not find UEye, not adding support." )
ENDIF( UEYE_FOUND )

################################################################################
IF( NOT CMAKE_BUILD_TYPE )
  SET( CMAKE_BUILD_TYPE Release CACHE STRING "CMAKE_BUILD_TYPE" FORCE )
ENDIF( NOT CMAKE_BUILD_TYPE )

IF( "${CMAKE_BUILD_TYPE}" STREQUAL "Release" )
  SET( CMAKE_C_FLAGS "${CMAKE_C_FLAGS}     -O3 -funroll-loops -finline-functions ")
  SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -funroll-loops -finline-functions ")
ENDIF( "${CMAKE_BUILD_TYPE}" STREQUAL "Release" )

IF( "${CMAKE_BUILD_TYPE}" STREQUAL "Fast" )
    SET( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra " )
    SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra " )
ENDIF( "${CMAKE_BUILD_TYPE}" STREQUAL "Fast" )

################################################################################
# Create the CCameraSensorIncludes.h
CONFIGURE_FILE( ${CMAKE_CURRENT_SOURCE_DIR}/CCameraSensorIncludes.cmake.in
    ${CMAKE_CURRENT_SOURCE_DIR}/CCameraSensorIncludes.h @ONLY IMMEDIATE )

################################################################################
include_directories( ${CCAMERASENSOR_INCLUDE_DIR} )
add_definitions( ${CCAMERASENSOR_DEFINITIONS} )

################################################################################
# Generate ccamerasensor library
set( LIB_SRCS
  CCameraSensor/CameraSensor.cpp
  CCameraSensor/FileReaderFromList.cpp
  CCameraSensor/Image.cpp
  CCameraSensor/OpenCVCamera.cpp
)
set( LIB_HDRS
  CCameraSensor/CameraSensor.h
  CCameraSensor/CameraSensorInterface.h
  CCameraSensor/FileReaderFromList.h
  CCameraSensor/Image.h
  CCameraSensor/OpenCVCamera.h
)
if( UEYE_FOUND )
    set( LIB_SRCS ${LIB_SRCS} 
        CCameraSensor/UEyeBlockingCamera.cpp
        CCameraSensor/UEyeCamera.cpp
        )
    set( LIB_HDRS ${LIB_HDRS}
        CCameraSensor/UEyeBlockingCamera.h
        CCameraSensor/UEyeCamera.h
        )
endif()
add_library( ccamerasensor SHARED ${LIB_SRCS} ${LIB_HDRS} )
target_link_libraries( ccamerasensor ${CCAMERASENSOR_LIBRARIES} )
get_target_property( CCAMERASENSOR_LIBRARY ccamerasensor LOCATION )
set( CCAMERASENSOR_LIBRARIES ${CCAMERASENSOR_LIBRARIES} ${CCAMERASENSOR_LIBRARY} )

################################################################################
# Optionally build demos
OPTION( BUILD_CAMERA_SENSOR_DEMOS "Build demos for using CCameraSensor." ON )

if( BUILD_CAMERA_SENSOR_DEMOS )
  add_subdirectory( Demos )
endif( BUILD_CAMERA_SENSOR_DEMOS )

################################################################################
# Target etags/tags
#ADD_CUSTOM_TARGET( tags etags `find . -name *.cc -or -name *.hh -or -name *.cpp -or -name *.h -or -name *.c -or -name *.f` )
#ADD_CUSTOM_TARGET( etags DEPENDS tags )

################################################################################
# Create the CCAMERASENSORConfig.cmake file for other cmake projects.
CONFIGURE_FILE( ${CMAKE_CURRENT_SOURCE_DIR}/CCAMERASENSORConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/CCAMERASENSORConfig.cmake @ONLY IMMEDIATE )

################################################################################
export( PACKAGE CCAMERASENSOR )
