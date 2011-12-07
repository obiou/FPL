#ifndef CTRACK_MATRIX_LOGARITHM_H
#define CTRACK_MATRIX_LOGARITHM_H

/**
 * @file   matrixlog.h
 * @author Christopher Mei
 * @date   04/09/2007
 * @brief Computes the matrix logarithm of a square matrix
 *
 * Computes the logarithm by scaling with the pth root and calculating the series.
 *
 * Many improvements are possible such as better initialisation,
 * Schur decompositions, testing for possible stable eigendecomposition...
 * 
 */

#include <eigen3/Eigen/Core>

////////////////////////////////////////////////////////////////////////////////
namespace CTrack {
    ////////////////////////////////////////////////////////////////////////////
    /// Compute logarithm of a square matrix
    inline bool logm( 
              const Eigen::MatrixXd& M, 
              Eigen::MatrixXd& logM,
              double dMaxErr
               );
    
    ////////////////////////////////////////////////////////////////////////////
    /// Compute the logarithm of a square matrix - easy form.
    inline Eigen::MatrixXd logm( 
                         const Eigen::MatrixXd& M, ///< Input
                         bool* pSuccess=NULL ///< Output
                          );
}

#if 1
#include <assert.h>
#include <cmath>
#include <iostream>

#include <CTrack/MatrixRoot.h>

#define MAX_ITER_LOG 10000

////////////////////////////////////////////////////////////////////////////////
/**
 * -- Matlab code --
 *% Evaluation of k scaling value (based on average of sum of eigenvalues)
 * k = ceil(log2(log(2*trace(H)/size(H,2))))+1
 *
 * K = H^(1/2^k);
 * dH = K-eye(size(H,2));
 * K = dH
 *
 * for i=2:100
 *   K = K+(-1)^(i+1)*dH^i/i;
 * end
 * res = norm(dH^(i)/(i))
 * logm_H = 2^k*K
 **/
bool CTrack::logm( 
                  const Eigen::MatrixXd& M, 
                  Eigen::MatrixXd& logM,
                  double dMaxErr
                   )
{
    typedef double T;
    int nN = M.rows();
    assert( nN == M.rows() && nN == M.cols() );
    assert( nN == logM.rows() && nN == logM.cols() );

    Eigen::MatrixXd dM = Eigen::MatrixXd::Identity( nN, nN );
    Eigen::MatrixXd K;

    T k = (T)ceil(log(log((double) 2.*M.trace()/nN))/log(2.0));
    T two_k = 1;
    if( k < 0 ) {
        two_k = 1;        
        K = M;
    }
    else {
        T two_k = (T) (1 << (int)k);
        
        // Calculate pth root of X
        K = CTrack::rootm( M, two_k );
    }

    int nIter = 0;

    // Check the norm
    while( ( nIter++ < MAX_ITER_LOG ) && ( K.lpNorm<Eigen::Infinity>()>nN ) ) {
        K = CTrack::rootm( K );
        k++;
        two_k *= 2.;
    }

    if( K.lpNorm<Eigen::Infinity>() > nN ) {
        std::cerr << "Problem scaling matrix before logm, norm stays big!!!" << std::endl;
        assert( K.lpNorm<Eigen::Infinity>() <= nN );
    }

    dM.setIdentity();
    dM = K-dM;
    K = dM;

    logM = K;

    nIter = 0;
    T norm_Res = 1e10;

    T val = 1;
    T denom = 1;

    // log(X)=log(I+K) = K-K^2/2+K^3/3 ...
    while( ( nIter++ < MAX_ITER_LOG ) && ( norm_Res>dMaxErr ) ) {
        dM *= K;
        denom++;
        val = -val;

        logM += val*dM/denom;

        norm_Res = dM.lpNorm<Eigen::Infinity>()/denom;
    }

    if( norm_Res > dMaxErr ) {
        std::cout << "WARNING: matrix log unsuccessful residual error: " << norm_Res << std::endl;
        std::cout << "Iter: " << nIter << std::endl;
    } 

    if( norm_Res > dMaxErr )
        return false;

    logM = two_k*logM;

    return true;
}

////////////////////////////////////////////////////////////////////////////////
Eigen::MatrixXd CTrack::logm( 
                             const Eigen::MatrixXd& M, ///< Input
                             bool* pSuccess
                              )
{
    bool bRetVal = true;

    Eigen::MatrixXd logM( M.rows(), M.cols() );
  
    bRetVal = CTrack::logm( M, logM, 1e-10 );
 
    if( pSuccess != NULL ) {
        *pSuccess = bRetVal;
    }

    return logM;
}
#endif

#endif
