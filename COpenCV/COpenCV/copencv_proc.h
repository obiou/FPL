#ifndef OPENCV_PROC_H
#define OPENCV_PROC_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc_c.h>

namespace cv {
    class KeyPoint;
}

namespace COPENCV {
    ////////////////////////////////////////////////////////////////////////////
    double HarrisScore( const cv::Mat& mImageGradX,
                        const cv::Mat& mImageGradY );

    ////////////////////////////////////////////////////////////////////////////////
    double HarrisScore( const cv::Mat& mImageGradX,
                        const cv::Mat& mImageGradY,
                        const int nRange,
                        const cv::KeyPoint& p ); 
}

#endif
