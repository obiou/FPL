#include <COpenCV/copencv_proc.h>
#include <opencv2/features2d/features2d.hpp>

#include <algorithm>

using namespace std;

////////////////////////////////////////////////////////////////////////////////
double COPENCV::HarrisScore( const cv::Mat& mImageGradX,
                             const cv::Mat& mImageGradY ) {
    double dHessian[4];
    fill( &dHessian[0], &dHessian[4], 0 );

    for( int nRow=0; nRow<mImageGradX.rows; nRow++ ) {
        for( int nCol=0; nCol<mImageGradX.rows; nCol++ ) {
            dHessian[0] += mImageGradX.at<uchar>( nRow, nCol ) * mImageGradX.at<uchar>( nRow, nCol );
            dHessian[1] += mImageGradX.at<uchar>( nRow, nCol ) * mImageGradY.at<uchar>( nRow, nCol );
            dHessian[3] += mImageGradY.at<uchar>( nRow, nCol ) * mImageGradY.at<uchar>( nRow, nCol );
        }
    }
    dHessian[2] = dHessian[1];
    const double dDet   = dHessian[0]*dHessian[3] - dHessian[1]*dHessian[1];
    const double dTrace = dHessian[0] + dHessian[3];
    const double dK = 0.04;
    return dDet - dK*dTrace*dTrace;
}

////////////////////////////////////////////////////////////////////////////////
double COPENCV::HarrisScore( const cv::Mat& mImageGradX,
                             const cv::Mat& mImageGradY,
                             const int nRange,
                             const cv::KeyPoint& p ) {
    const int nHalfRange = (nRange-1)/2;
    cv::Rect pointRect( p.pt.x - nHalfRange, p.pt.y - nHalfRange,
                        nRange, nRange );
            
    cv::Mat mGradXRect = mImageGradX( pointRect );
    cv::Mat mGradYRect = mImageGradY( pointRect );

    return HarrisScore( mGradXRect, mGradYRect );
}
