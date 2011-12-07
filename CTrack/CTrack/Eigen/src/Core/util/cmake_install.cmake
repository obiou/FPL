# Install script for directory: /homes/13/cmei/progs/CTrack/Eigen/src/Core/util

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
  FILE(INSTALL DESTINATION "/Eigen/src/Core/util" TYPE FILE FILES
    "/homes/13/cmei/progs/CTrack/Eigen/src/Core/util/Constants.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Core/util/EnableMSVCWarnings.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Core/util/ForwardDeclarations.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Core/util/DisableMSVCWarnings.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Core/util/Meta.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Core/util/Macros.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Core/util/XprHelper.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Core/util/StaticAssert.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Core/util/Memory.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

