#ifndef CTRACK_MATRIX_ROOT_H
#define CTRACK_MATRIX_ROOT_H

////////////////////////////////////////////////////////////////////////////////
/**
 * @file   MatrixRoot.h
 * @author Christopher Mei
 * @date   03/09/2007
 * @brief Computes the roots (square root and pth roots) of a square matrix
 *
 * Computes the pth root of a square matrix based on the work by 
 * Iannazzo and Higham (stable Newton iterations).
 * This is a simplified version...
 *
 * Many improvements are possible such as better initialisation,
 * Schur decompositions, testing for a possible stable eigendecomposition...
 * 
 */

#include <eigen3/Eigen/Core>

#define MAX_ITER_ROOT 1000

////////////////////////////////////////////////////////////////////////////////
namespace CTrack {
    ////////////////////////////////////////////////////////////////////////////
    /// Compute the square root (default) or pth root of a square matrix
    bool rootm( 
               const Eigen::MatrixXd& M,
               Eigen::MatrixXd& rootM,
               double dMaxErr,
               int nP = 2
                );

    ////////////////////////////////////////////////////////////////////////////
    /// Compute the square root (default) or pth root of a square matrix - easy form.
    Eigen::MatrixXd rootm( 
                          const Eigen::MatrixXd& M,
                          int nP = 2 
                           );

    ////////////////////////////////////////////////////////////////////////////
    /// Compute the square root of a square matrix 
    /// (currently only works with powers of 2 because of the products...)
    bool sqrtm(
               const Eigen::MatrixXd& M, 
               Eigen::MatrixXd& sqrtM, 
               double dMaxErr
               );

    ////////////////////////////////////////////////////////////////////////////
    /// Compute the square root of a square matrix - easy form.
    Eigen::MatrixXd sqrtm(
                          const Eigen::MatrixXd& M 
                          );
}

#endif
