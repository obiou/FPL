# Install script for directory: /homes/13/cmei/progs/CTrack/Eigen/src/Core

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
  FILE(INSTALL DESTINATION "/Eigen/src/Core" TYPE FILE FILES
    "/homes/13/cmei/progs/CTrack/Eigen/src/Core/CommaInitializer.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Core/GenericPacketMath.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Core/MatrixStorage.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Core/Flagged.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Core/Functors.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Core/DiagonalMatrix.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Core/NumTraits.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Core/IO.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Core/DiagonalCoeffs.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Core/MathFunctions.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Core/Dot.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Core/Matrix.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Core/CwiseNullaryOp.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Core/Transpose.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Core/Visitor.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Core/Sum.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Core/CwiseBinaryOp.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Core/Redux.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Core/CwiseUnaryOp.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Core/Block.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Core/MatrixBase.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Core/Coeffs.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Core/Product.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Core/Assign.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Core/CacheFriendlyProduct.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Core/Minor.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Core/MapBase.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Core/NestByValue.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Core/Fuzzy.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Core/Map.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Core/Part.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Core/Swap.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Core/DiagonalProduct.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Core/Cwise.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Core/SolveTriangular.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  INCLUDE("/homes/13/cmei/progs/CTrack/Eigen/src/Core/util/cmake_install.cmake")
  INCLUDE("/homes/13/cmei/progs/CTrack/Eigen/src/Core/arch/cmake_install.cmake")

ENDIF(NOT CMAKE_INSTALL_LOCAL_ONLY)

