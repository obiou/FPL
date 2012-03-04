#include <ceigen/ceigen_opencv.h>

#include <COpenCV.h>

#include <opencv2/calib3d/calib3d.hpp>

////////////////////////////////////////////////////////////////////////////////
Eigen::MatrixXd CEIGEN::FindChessboardCorners( IplImage* pImage,
                                               const CvSize& sBoardSize,
                                               bool bSubPix,
                                               IplImage* pImageSubPix ) {
    CvPoint2D32f* pImagePoints;
    pImagePoints = (CvPoint2D32f*)cvAlloc( sBoardSize.width*sBoardSize.height*sizeof( pImagePoints[0] ) );
    int nCount = 0;
#if 0
    bool bFound = cvFindChessboardCorners
        ( pImage, sBoardSize, 
          pImagePoints, &nCount, CV_CALIB_CB_NORMALIZE_IMAGE );
#else
    bool bFound = cvFindChessboardCorners
        ( pImage, sBoardSize, 
          pImagePoints, &nCount );
#endif
    if( bFound && bSubPix ) {
        IplImage* pImageGray = NULL;
        if( pImageSubPix != NULL ) {
            COPENCV::GetMonoImage( pImageSubPix, &pImageGray );
        }
        else {           
            COPENCV::GetMonoImage( pImage, &pImageGray );
        }
        cvFindCornerSubPix( pImageGray, pImagePoints, nCount, cvSize(11,11),
                            cvSize(-1,-1), cvTermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ) );
        cvReleaseImage( &pImageGray );
    }
    Eigen::MatrixXd mPoints;
    if( bFound ) {
        mPoints = Eigen::Map<Eigen::MatrixXf>( (float*) pImagePoints, 2, nCount ).cast<double>();
    }
    cvFree( &pImagePoints );
    return mPoints;
}
