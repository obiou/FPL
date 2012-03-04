#ifndef CEIGEN_OPENCV_H
#define CEIGEN_OPENCV_H

#include <eigen3/Eigen/Core>

#include <cv.h>

#include <iostream>

namespace CEIGEN {
    ////////////////////////////////////////////////////////////////////////////
    template<typename Derived>
        inline void copy( const CvMat& mMIn, Eigen::MatrixBase<Derived>& mMOut ) {
        int nType = CV_MAT_TYPE( mMIn.type );
        unsigned int nFact = nType == CV_32FC1 || nType == CV_64FC1 ? 1 :
            nType == CV_32FC2 || nType == CV_64FC2 ? 2 :
            nType == CV_32FC3 || nType == CV_64FC3 ? 3 : 0;
        bool bTranspose = mMOut.rows() != nFact*mMIn.rows;
        bool bFloat = nType == CV_32FC1 || nType == CV_32FC2 || nType == CV_32FC3;
        bool bDouble = nType == CV_64FC1 || nType == CV_64FC2 || nType == CV_64FC3;
#if EIGEN_DEFAULT_TO_ROW_MAJOR
        //std::cout << "Row major" << std::endl;
        const CvMat* pMIn = &mMIn;
#else
        //std::cout << "Col major" << std::endl;
        CvMat* pMIn = cvCreateMat( mMIn.cols, mMIn.rows, mMIn.type );
        cvTranspose( &mMIn, pMIn );
#endif     
        if( bFloat ) {
            if( bTranspose ) {
                mMOut = Eigen::Map<Eigen::MatrixXf>
                    ( (float*)pMIn->data.ptr, mMIn.cols, nFact*mMIn.rows ).cast<typename Eigen::internal::traits<Derived>::Scalar>();
            }
            else {
                mMOut = Eigen::Map<Eigen::MatrixXf>
                    ( (float*)pMIn->data.ptr, nFact*mMIn.rows, mMIn.cols ).cast<typename Eigen::internal::traits<Derived>::Scalar>();

            }
        }
        else if( bDouble ) {
            if( bTranspose ) {
                mMOut = Eigen::Map<Eigen::MatrixXd>
                    ( (double*)pMIn->data.ptr, mMIn.cols, nFact*mMIn.rows ).cast<typename Eigen::internal::traits<Derived>::Scalar>();
            }
            else {
                mMOut = Eigen::Map<Eigen::MatrixXd>
                    ( (double*)pMIn->data.ptr, nFact*mMIn.rows, mMIn.cols ).cast<typename Eigen::internal::traits<Derived>::Scalar>();
            }
        }
#ifndef EIGEN_DEFAULT_TO_ROW_MAJOR
        cvReleaseMat( &pMIn );
#endif
    }

    ////////////////////////////////////////////////////////////////////////////
    /// Find image from chessboard with optional subpixel accuracy
    Eigen::MatrixXd FindChessboardCorners( 
                                          IplImage* pImage,
                                          const CvSize& sBoardSize,
                                          bool bSubPix = true,
                                          IplImage* pImageSubPix = NULL
                                           );
}
#endif
