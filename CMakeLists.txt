project( FPL )
cmake_minimum_required( VERSION 2.8 )

enable_testing()

################################################################################
# Guard against in-source builds
if( ${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR} )
  message( FATAL_ERROR "In-source builds not allowed. Please make a new directory (called a build directory) and run CMake from there. You may need to remove CMakeCache.txt." )
endif()

################################################################################
# Set the local version first
set( CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake_modules" )
list( APPEND CMAKE_MODULE_PATH "${CMAKE_ROOT}/Modules" )
#MESSAGE( STATUS "" ${CMAKE_ROOT} )
#MESSAGE( STATUS "" ${CMAKE_MODULE_PATH} )

################################################################################
# Optionally build demos
#OPTION( BUILD_ALL_DEMOS "Build demos for using CEIGEN." OFF )
OPTION( BUILD_ALL_DEMOS "Build demos for using CEIGEN." OFF )

if( BUILD_ALL_DEMOS )
    set( BUILD_CIMAGE_DEMOS ON CACHE BOOL "" FORCE )
    set( BUILD_CTRACK_DEMOS ON CACHE BOOL "" FORCE )
    set( BUILD_CEIGEN_DEMOS ON CACHE BOOL "" FORCE )
    set( BUILD_CGEOM_DEMOS ON CACHE BOOL "" FORCE )
    set( BUILD_CAMERA_MODEL_DEMOS ON CACHE BOOL "" FORCE )
    set( BUILD_CAMERA_SENSOR_DEMOS ON CACHE BOOL "" FORCE )
    set( BUILD_COPENCV_DEMOS ON CACHE BOOL "" FORCE )
    set( BUILD_MISC_DEMOS ON CACHE BOOL "" FORCE )
    set( BUILD_CAPPS ON CACHE BOOL "" FORCE )
else()
    set( BUILD_CIMAGE_DEMOS OFF CACHE BOOL "" FORCE )
    set( BUILD_CTRACK_DEMOS OFF CACHE BOOL "" FORCE )
    set( BUILD_CEIGEN_DEMOS OFF CACHE BOOL "" FORCE )
    set( BUILD_CGEOM_DEMOS OFF CACHE BOOL "" FORCE )
    set( BUILD_CAMERA_MODEL_DEMOS OFF CACHE BOOL "" FORCE )
    set( BUILD_CAMERA_SENSOR_DEMOS OFF CACHE BOOL "" FORCE )
    set( BUILD_COPENCV_DEMOS OFF CACHE BOOL "" FORCE )
    set( BUILD_MISC_DEMOS OFF CACHE BOOL "" FORCE )
    set( BUILD_CAPPS OFF CACHE BOOL "" FORCE )
endif()

################################################################################
if( NOT CMAKE_BUILD_TYPE )
  set( CMAKE_BUILD_TYPE Debug CACHE STRING "CMAKE_BUILD_TYPE" FORCE )
endif( NOT CMAKE_BUILD_TYPE )

################################################################################
# Order is important...
add_subdirectory( Misc )
add_subdirectory( CIMAGE )

add_subdirectory( COpenCV )
add_subdirectory( CTrack )
add_subdirectory( CEIGEN )
add_subdirectory( CGEOM )
add_subdirectory( CCameraModel )
add_subdirectory( CCameraSensor )

add_subdirectory( CAPPS )
