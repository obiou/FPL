project( CAPPS )
cmake_minimum_required( VERSION 2.8 )

################################################################################
# Executable targets
set( EXECUTABLES
    ccamera_calibration
    ctrack_basic
)

################################################################################
# Guard against in-source builds
if( ${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR} )
  message( FATAL_ERROR "In-source builds not allowed. Please make a new directory (called a build directory) and run CMake from there. You may need to remove CMakeCache.txt. ")
endif()

################################################################################
set( CAPPS_LIBRARIES "" )
set( CAPPS_INCLUDE_DIR ${PROJECT_SOURCE_DIR} )

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
find_package_and_add( CEIGEN CAPPS )
find_package_and_add( OPENCV CAPPS )
find_package_and_add( CGEOM CAPPS )
find_package_and_add( CCAMERAMODEL CAPPS )
find_package_and_add( CCAMERASENSOR CAPPS )
find_package_and_add( CTRACK CAPPS )

################################################################################
if( NOT CMAKE_BUILD_TYPE )
  set( CMAKE_BUILD_TYPE Release CACHE STRING "CMAKE_BUILD_TYPE" FORCE )
endif( NOT CMAKE_BUILD_TYPE )

if( "${CMAKE_BUILD_TYPE}" STREQUAL "Debug" )
  set( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -Wall -Wextra -Wuninitialized " )
  set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall  -Wextra -Wuninitialized " )
endif( "${CMAKE_BUILD_TYPE}" STREQUAL "Debug" )
 
if( "${CMAKE_BUILD_TYPE}" STREQUAL "Release" )
    set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -funroll-loops -finline-functions  -msse -msse2 -msse3 -mssse3 " )
    set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -funroll-loops -finline-functions  -msse -msse2 -msse3 -mssse3 " )
endif( "${CMAKE_BUILD_TYPE}" STREQUAL "Release" )

if( "${CMAKE_BUILD_TYPE}" STREQUAL "Fast" )
    set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra" )
    set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra" )
endif( "${CMAKE_BUILD_TYPE}" STREQUAL "Fast" )

################################################################################
include_directories( ${CAPPS_INCLUDE_DIR} )

OPTION( BUILD_CAPPS "Build app demos using the FPL library." ON )

IF( BUILD_CAPPS )
    foreach( EXECUTABLE ${EXECUTABLES} )
        add_executable( ${EXECUTABLE} ${EXECUTABLE}.cpp )
        add_dependencies( ${EXECUTABLE} ${CAPPS_LIBRARIES} )
        target_link_libraries( ${EXECUTABLE} ${CAPPS_LIBRARIES} )
    endforeach( EXECUTABLE )
ENDIF( BUILD_CAPPS )
