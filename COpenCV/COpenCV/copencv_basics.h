#ifndef OPENCV_BASIC_H
#define OPENCV_BASIC_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc_c.h>

#include <iomanip>
#include <iostream>

namespace COPENCV {
    ////////////////////////////////////////////////////////////////////////////////
    inline void print( const CvMat* pM ) {
        //std::cout << std::setprecision( 3 ) << std::right << std::fixed;
        for( int row = 0; row < pM->rows; ++row ) {
            for( int col = 0; col < pM->cols; ++col ) {
                //std::cout << std::setw( 5 ) << cvmGet( pM, row, col ) << " ";
                std::cout << cvmGet( pM, row, col ) << " ";
            }
            std::cout << std::endl;
        }
    }

    ////////////////////////////////////////////////////////////////////////////////
    inline void info( IplImage* pImage ) {
        std::cout << "Num channels: " << pImage->nChannels << std::endl;
        std::cout << "Depth: " << pImage->depth << std::endl;
        if( pImage->dataOrder ) {
            std::cout << "DataOrder: separate." << std::endl;
        }
        else {
            std::cout << "DataOrder: interleaved." << std::endl;
        }    
    }

    ////////////////////////////////////////////////////////////////////////////////
    inline bool GetMonoImage( IplImage* pCapturedImage,
                              IplImage** pImage ) {
        if( *pImage == NULL ) {
            *pImage = cvCreateImage( cvSize( pCapturedImage->width, pCapturedImage->height ), 
                                     IPL_DEPTH_8U, 1 );
        }

        if( pCapturedImage->nChannels == 3 ) {
            cvCvtColor( pCapturedImage, *pImage, CV_BGR2GRAY );
        }
        else if( pCapturedImage->nChannels == 1 ) {
            memcpy( (*pImage)->imageData, pCapturedImage->imageData, pCapturedImage->widthStep*pCapturedImage->height );
            //*pImage = pCapturedImage;
        }
        else {
            std::cout << "Wrong number of channels" << std::endl;
            return false;
        }
        return true;
    }

    ////////////////////////////////////////////////////////////////////////////////
    inline bool copy( IplImage* pImage, IplImage* pImageCopy ) {
        if( pImageCopy == NULL ) {
            return false;
        }
        if( pImage->nChannels == 1 ) {
            if( pImageCopy->nChannels == 1 ) {
                memcpy( pImageCopy->imageData, pImage->imageData, 
                        pImage->widthStep*pImage->height );
            }
            else if( pImageCopy->nChannels == 3 ) {
                cvCvtColor( pImage, pImageCopy, CV_GRAY2RGB );
            }
            else {
                std::cout << "Image copy has wrong number of channels" << std::endl;
                return false;
            }
        }
        else if( pImage->nChannels == 3 ) {
            if( pImageCopy->nChannels == 1 ) {
                cvCvtColor( pImage, pImageCopy, CV_RGB2GRAY );
            }
            else if( pImageCopy->nChannels == 3 ) {
                memcpy( pImageCopy->imageData, pImage->imageData, 
                        pImage->widthStep*pImage->height );
            }
            else {
                std::cout << "Image copy has wrong number of channels" << std::endl;
                return false;
            }
        }
        else {
            std::cout << "Image has wrong number of channels" << std::endl;
            return false;
        }
        return true;
    }

    ////////////////////////////////////////////////////////////////////////////////
    inline bool ColourCopy( IplImage* pImage, IplImage** pImageCopy ) {
        if( *pImageCopy == NULL ) {
            *pImageCopy = cvCreateImage( cvSize( pImage->width, pImage->height ), 
                                         IPL_DEPTH_8U, 3 );
        } else {
            std::cout << "Input image should have 3 channels" << std::endl;
            return false;
        }
        if( pImage->nChannels == 3 ) {
            memcpy( (*pImageCopy)->imageData, pImage->imageData, 
                    pImage->widthStep*pImage->height );
        }
        else if( pImage->nChannels == 1 ) {
            cvCvtColor( pImage, *pImageCopy, CV_GRAY2RGB );
        }
        else {
            std::cout << "Wrong number of channels" << std::endl;
            return false;
        }
        return true;
    }

    ////////////////////////////////////////////////////////////////////////////
    /// Extract and create template from an image
    /// The user should take care to free the image
    IplImage* ExtractPatch( IplImage* pImage,
                            const int nBegX,
                            const int nBegY,
                            const int nPatchWidth,
                            const int nPatchHeight,
                            IplImage* pPatchIn = NULL
                            );

    ////////////////////////////////////////////////////////////////////////////
    template<class T>
    IplImage* ExtractPatch( IplImage* pImage,
                            const std::vector<std::pair<T,T> > vBox,
                            IplImage* pPatchIn
                            );

    ////////////////////////////////////////////////////////////////////////////
    double RMS( const IplImage* pI1,
                const IplImage* pI2
                );
}

#endif
