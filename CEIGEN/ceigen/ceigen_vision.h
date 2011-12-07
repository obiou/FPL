#ifndef CEIGEN_VISION_H
#define CEIGEN_VISION_H

#include <eigen3/Eigen/Core>

#include <iostream>

namespace CEIGEN {
    ////////////////////////////////////////////////////////////////////////////
    /// Compute SE3 transformation from a homography matrix H using
    /// direct Gram-Schmidt orthogonalisation
    void HToSE3( const Eigen::Matrix3d& mHIn,
                 Eigen::Matrix3d& mR,
                 Eigen::Vector3d& mt,
                 bool bFix = true );

    ////////////////////////////////////////////////////////////////////////////
    Eigen::MatrixXd make_grid( const unsigned int nWidth, 
                               const unsigned int nHeight,
                               const bool b3Rows = true,
                               const bool bColMajor = true  );

    ////////////////////////////////////////////////////////////////////////////
    Eigen::MatrixXf make_grid_f( const unsigned int nWidth, 
                                 const unsigned int nHeight,
                                 const bool b3Rows = true,
                                 const bool bColMajor = true );

    ////////////////////////////////////////////////////////////////////////////
    Eigen::MatrixXd make_border( const unsigned int nWidth, 
                                 const unsigned int nHeight );

    ////////////////////////////////////////////////////////////////////////////
    /// Currently only valid for 3x3 matrices
    template<class M, class T>
        M skew_rot( const T x, const T y, const T z ) {
        M mM( 3, 3 ); 
        mM << 0, -z, y,
            z, 0, -x,
            -y, x, 0;
        return mM;        
    }

    ////////////////////////////////////////////////////////////////////////////
    /// Currently only valid for 3x3 matrices
    template<class M>
        M skew_rot( const typename Eigen::internal::traits<M>::Scalar x, 
                    const typename Eigen::internal::traits<M>::Scalar y,
                    const typename Eigen::internal::traits<M>::Scalar z ) {
        M mM( 3, 3 ); 
        mM << 0, -z, y,
            z, 0, -x,
            -y, x, 0;
        return mM;        
    }

    ////////////////////////////////////////////////////////////////////////////
    /// Currently only valid for 3x3 matrices
    template<class M, class T>
        M skew_rot( const T& v ) {
        return skew_rot<M,typename Eigen::internal::traits<T>::Scalar>( v(0), v(1), v(2) );
    }
}

#endif
