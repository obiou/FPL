#ifndef CEIGEN_IMAGE_H
#define CEIGEN_IMAGE_H

#include<Eigen/Core>

#include <CEIGENIncludes.h>

#if CEIGEN_HAS_OPENCV
#  include <opencv2/highgui/highgui.hpp>
#endif

namespace CEIGEN {
    typedef Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> MatrixImage;

    ////////////////////////////////////////////////////////////////////////////
    Eigen::MatrixXd make_warp_grid( const int nPatchWidth,
                                    const int nPatchHeight );

    ////////////////////////////////////////////////////////////////////////////
    void Warp( const unsigned char* pImageIn,
               const int nImageWidth,
               const int nImageHeight,
               const int nImageWidthStep,
               const Eigen::Matrix3d& H,
               unsigned char* pImageOut,
               const int nPatchWidth,
               const int nPatchHeight,
               const int nPatchWidthStep
               );

    ////////////////////////////////////////////////////////////////////////////
    /// Faster version, coordinate grids are precomputed
    /// Usage:
    /// Eigen::MatrixXd grid_tmp = make_warp_grid( nPatchWidth, nPatchHeight );
    /// Eigen::MatrixXd warped_grid_tmp = make_warp_grid( nPatchWidth, nPatchHeight );
    /// Warp( ..., grid_tmp, warped_grid_tmp );
    void Warp( const unsigned char* pImageIn,
               const int nImageWidth,
               const int nImageHeight,
               const int nImageWidthStep,
               const Eigen::Matrix3d& H,
               unsigned char* pImageOut,
               const int nPatchWidth,
               const int nPatchHeight,
               const int nPatchWidthStep,              
               Eigen::MatrixXd grid_tmp,
               Eigen::MatrixXd warped_grid_tmp
               );
    
#if CEIGEN_HAS_OPENCV
    ////////////////////////////////////////////////////////////////////////////////
    void Warp( IplImage* pImageIn,
               const Eigen::Matrix3d& H,
               IplImage* pImageOut );

    ////////////////////////////////////////////////////////////////////////////////
    /// Faster version, coordinate grids are precomputed
    /// Usage:
    /// Eigen::MatrixXd grid_tmp = make_warp_grid( nPatchWidth, nPatchHeight );
    /// Eigen::MatrixXd warped_grid_tmp = make_warp_grid( nPatchWidth, nPatchHeight );
    /// Warp( ..., grid_tmp, warped_grid_tmp );
    void Warp( IplImage* pImageIn,
               const Eigen::Matrix3d& H,
               IplImage* pImageOut,
               Eigen::MatrixXd grid_tmp,
               Eigen::MatrixXd warped_grid_tmp );
#endif
}

#endif
