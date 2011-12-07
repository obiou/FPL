# Install script for directory: /homes/13/cmei/progs/CTrack/Eigen/src/Geometry

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
  FILE(INSTALL DESTINATION "/Eigen/src/Geometry" TYPE FILE FILES
    "/homes/13/cmei/progs/CTrack/Eigen/src/Geometry/Quaternion.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Geometry/Transform.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Geometry/Scaling.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Geometry/Translation.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Geometry/ParametrizedLine.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Geometry/AngleAxis.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Geometry/EulerAngles.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Geometry/Hyperplane.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Geometry/OrthoMethods.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Geometry/Rotation2D.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Geometry/AlignedBox.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Geometry/RotationBase.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

