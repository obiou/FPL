# Install script for directory: /homes/13/cmei/progs/CTrack/Eigen/src/Sparse

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

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Devel")
  FILE(INSTALL DESTINATION "/Eigen/src/Sparse" TYPE FILE FILES
    "/homes/13/cmei/progs/CTrack/Eigen/src/Sparse/UmfPackSupport.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Sparse/SuperLUSupport.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Sparse/SparseVector.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Sparse/SparseProduct.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Sparse/SparseLLT.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Sparse/AmbiVector.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Sparse/MappedSparseMatrix.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Sparse/CompressedStorage.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Sparse/SparseDot.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Sparse/SparseMatrix.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Sparse/RandomSetter.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Sparse/TriangularSolver.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Sparse/SparseCwiseBinaryOp.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Sparse/SparseLDLT.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Sparse/DynamicSparseMatrix.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Sparse/SparseAssign.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Sparse/SparseCwiseUnaryOp.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Sparse/SparseTranspose.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Sparse/CoreIterators.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Sparse/SparseRedux.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Sparse/SparseUtil.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Sparse/TaucsSupport.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Sparse/SparseCwise.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Sparse/SparseDiagonalProduct.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Sparse/CholmodSupport.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Sparse/SparseBlock.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Sparse/SparseFlagged.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Sparse/SparseLU.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Sparse/SparseMatrixBase.h"
    "/homes/13/cmei/progs/CTrack/Eigen/src/Sparse/SparseFuzzy.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Devel")

