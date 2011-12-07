# Install script for directory: /homes/13/cmei/progs/CTrack/Eigen

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "/usr/local")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "")
  ENDIF(BUILD_TYPE)
  MESSAGE(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
ENDIF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)

# Set the component getting installed.
IF(NOT CMAKE_INSTALL_COMPONENT)
  IF(COMPONENT)
    MESSAGE(STATUS "Install component: \"${COMPONENT}\"")
    SET(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  ELSE(COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT)
  ENDIF(COMPONENT)
ENDIF(NOT CMAKE_INSTALL_COMPONENT)

# Install shared libraries without execute permission?
IF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  SET(CMAKE_INSTALL_SO_NO_EXE "1")
ENDIF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "/Eigen" TYPE FILE FILES
    "/homes/13/cmei/progs/CTrack/Eigen/Core"
    "/homes/13/cmei/progs/CTrack/Eigen/LU"
    "/homes/13/cmei/progs/CTrack/Eigen/Cholesky"
    "/homes/13/cmei/progs/CTrack/Eigen/QR"
    "/homes/13/cmei/progs/CTrack/Eigen/Geometry"
    "/homes/13/cmei/progs/CTrack/Eigen/Sparse"
    "/homes/13/cmei/progs/CTrack/Eigen/Array"
    "/homes/13/cmei/progs/CTrack/Eigen/SVD"
    "/homes/13/cmei/progs/CTrack/Eigen/LeastSquares"
    "/homes/13/cmei/progs/CTrack/Eigen/QtAlignedMalloc"
    "/homes/13/cmei/progs/CTrack/Eigen/StdVector"
    "/homes/13/cmei/progs/CTrack/Eigen/NewStdVector"
    "/homes/13/cmei/progs/CTrack/Eigen/Eigen"
    "/homes/13/cmei/progs/CTrack/Eigen/Dense"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  INCLUDE("/homes/13/cmei/progs/CTrack/Eigen/src/cmake_install.cmake")

ENDIF(NOT CMAKE_INSTALL_LOCAL_ONLY)

IF(CMAKE_INSTALL_COMPONENT)
  SET(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
ELSE(CMAKE_INSTALL_COMPONENT)
  SET(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
ENDIF(CMAKE_INSTALL_COMPONENT)

FILE(WRITE "/homes/13/cmei/progs/CTrack/Eigen/${CMAKE_INSTALL_MANIFEST}" "")
FOREACH(file ${CMAKE_INSTALL_MANIFEST_FILES})
  FILE(APPEND "/homes/13/cmei/progs/CTrack/Eigen/${CMAKE_INSTALL_MANIFEST}" "${file}\n")
ENDFOREACH(file)
