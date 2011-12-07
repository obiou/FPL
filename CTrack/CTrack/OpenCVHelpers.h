// Copyright (C) 2010 Christopher Mei (christopher.m.mei@gmail.com)
//
// This file is part of the CTrack Library. This library is free
// software; you can redistribute it and/or modify it under the
// terms of the GNU Lesser General Public License as published by the
// Free Software Foundation; either version 2, or (at your option)
// any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU Lesser General Public License for more details.

// You should have received a copy of the GNU Lesser General Public
// License along with this library; see the file COPYING.LESSER. If
// not, write to the Free Software Foundation, 59 Temple Place - Suite
// 330, Boston, MA 02111-1307, USA.

// As a special exception, you may use this file as part of a free software
// library without restriction.  Specifically, if other files instantiate
// templates or use macros or inline functions from this file, or you compile
// this file and link it with other files to produce an executable, this
// file does not by itself cause the resulting executable to be covered by
// the GNU Lesser General Public License.  This exception does not however
// invalidate any other reasons why the executable file might be covered by
// the GNU Lesser General Public License.

#ifndef _OPENCV_HELPERS_H_

#define _OPENCV_HELPERS_H_

#include <string>
#include <cv.h>

#include <CTrack/Homography.h>
#include <CTrack/Misc.h>
#include <CTrack/PlaneTracking.h>
#include <CTrack/TestingHelpers.h>
#include <CTrack/Types.h>

namespace CTrack {
    ////////////////////////////////////////////////////////////////////////////
    /// Creates an OpenCV using the data from an image holder
    /// User cvReleaseImageHeader to release.
    IplImage* ImageHolderToOpenCV( ImageHolder* pImageHolder 
                                  //bool bCopy = false 
                                   ) 
    {
        IplImage* pOpenCVImage = cvCreateImageHeader( cvSize( pImageHolder->nWidth,
                                                              pImageHolder->nHeight ),
                                                      IPL_DEPTH_8U, 1 );
        pOpenCVImage->widthStep = pImageHolder->nWidthStep;
        pOpenCVImage->imageData = (char*)pImageHolder->pImageData;
        return pOpenCVImage;
    }

    ////////////////////////////////////////////////////////////////////////////
    IplImage* LoadToGray( const std::string& sImageName ) 
    {
        IplImage* pImage = NULL;
        // Load and convert to gray
        IplImage* pImageInit = cvLoadImage( sImageName.c_str() );
        if( pImageInit == NULL ) {
            printf( "Could not load image file: %s\n", sImageName.c_str() );
            return NULL;
        }
    
        pImage = cvCreateImage( cvGetSize( pImageInit ), IPL_DEPTH_8U, 1 );
        cvCvtColor( pImageInit, pImage, CV_RGB2GRAY );
        cvReleaseImage( &pImageInit );

        return pImage;
    }

    ////////////////////////////////////////////////////////////////////////////
    IplImage* ExtractPatch( IplImage* pImage,
                            const int nPatchBegX, const int nPatchBegY, 
                            const int nPatchWidth, const int nPatchHeight )
    {
        CvRect rect = cvRect( nPatchBegX, nPatchBegY, nPatchWidth, nPatchHeight );
        cvSetImageROI( pImage, rect ); // Sets the region of interest
        
        IplImage* pPatch = cvCreateImage( cvGetSize( pImage ),
                                          pImage->depth,
                                          pImage->nChannels );
        
        cvCopy( pImage, pPatch, NULL );
        cvResetImageROI( pImage );
        
        return pPatch;
    }

    ////////////////////////////////////////////////////////////////////////////
    /// Generate images for testing.
    /// The memory is allocated withing the function, the user should deallocate
    /// it calling cvReleaseImage on pRefPatch, pCurImage and pCurPatch
    ///
    /// The following equality should hold:
    /// I_c( HRef*HEst p ) = I_r( p ) or 
    /// I_c( HRef*HEst p ) = int_0^1 I_r( HPrev^{-1} exp(lambda t log(Hincr) ) HPrev p ) dt for the blur case
    bool GenerateTestImage( const std::string& sImageName, ///< Input
                            const int nPatchBegX,   ///< Input
                            const int nPatchBegY,   ///< Input
                            const int nPatchWidth,  ///< Input
                            const int nPatchHeight, ///< Input
                            const TrackingMotionModel eMotionModel, ///< Input
                            const TrackingIlluminationModel eIlluminationModel, ///< Input
                            IplImage** pRefImage, ///< Output
                            IplImage** pRefPatch, ///< Output
                            IplImage** pCurImage, ///< Output
                            Homography& HPrev, ///< Output: homography transform computed up to an assumed previous step (only important for blur)
                            Homography& HEst,  ///< Output: inter-frame homography transform without HRef (Htot = HRef*HEst = HRef*Hi*HPrev)
                            bool bBlur = false ///< Input: should the current image be blurred?
                            ) {
        bool bDraw = false;
        HPrev.id();
        switch( eMotionModel ) {
        case TRANSLATION: 
            HEst = CTrack::GenerateTranslation( 5, 8 );
            break;
        case AFFINE:
            HEst = CTrack::GenerateCenteredRotation( nPatchWidth, nPatchHeight, 15 );
            break;
        case HOMOGRAPHY:
        case HOMOGRAPHY_BLUR_MAGN: {
            Homography HScale = CTrack::GenerateScaled( 1.05 );
            HEst = CTrack::GenerateCenteredRotation( nPatchWidth, nPatchHeight, 10 );
            HEst.mult( CTrack::GenerateTranslation( 5, 8 ) );
            HEst.mult( HScale );
        }
            break;
        default:
            std::cout << "Not supported. " << std::endl;
            return false;
            break;
        }

        float fAlpha = 1; float fBeta = 0.;
        switch( eIlluminationModel ) {
        case NONE:
            fAlpha = 1; fBeta = 0;
            break;
        case AFFINE_ILLUM:
        case AFFINE_ILLUM_REGULARISED:
            fAlpha = 1.2; fBeta = 10.;
            break;
        default:
            std::cout << "OpenCVHelpers: Not supported. " << std::endl;
            return false;
            break;
        }

        *pRefImage = NULL;
        {   // Load and convert to gray
            IplImage* pRefImageInit = cvLoadImage( sImageName.c_str() );
            if( pRefImageInit == NULL ) {
                printf( "Could not load image file: %s\n", sImageName.c_str() );
                return false;
            }
            
            *pRefImage = cvCreateImage( cvGetSize( pRefImageInit ), IPL_DEPTH_8U, 1 );
            cvCvtColor( pRefImageInit, *pRefImage, CV_RGB2GRAY );
            cvReleaseImage( &pRefImageInit );
        }
        int nImageWidth  = (*pRefImage)->width;
        int nImageHeight = (*pRefImage)->height;  

        *pCurImage = cvCreateImage( cvGetSize( *pRefImage ), IPL_DEPTH_8U, 1 );

        CvRect rect = cvRect( nPatchBegX, nPatchBegY, nPatchWidth, nPatchHeight );
        cvSetImageROI( (*pRefImage), rect ); // Sets the region of interest

        *pRefPatch = cvCreateImage( cvGetSize( (*pRefImage) ),
                                    (*pRefImage)->depth,
                                    (*pRefImage)->nChannels );

        cvCopy( (*pRefImage), *pRefPatch, NULL );
        cvResetImageROI( (*pRefImage) );
 
        const int nImageWidthStep = (*pRefImage)->widthStep;
        const int nPatchWidthStep = (*pRefPatch)->widthStep;

        ////////////////////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////// 
        Homography HRef = GenerateTranslation( nPatchBegX, nPatchBegY );
        
        // Build current image
        if( !bBlur ) {
            // Compute HRef*inv(H)*inv(HRef) to produce the current image
            Homography HInv = HEst; HInv.inv();
            Homography HRefInv = HRef; HRefInv.inv();
            Homography HFull = HRef; HFull.mult( HInv ); HFull.mult( HRefInv );
            
            CTrack::Warp( (const unsigned char*)(*pRefImage)->imageData,
                          nImageWidth, nImageHeight, nImageWidthStep,
                          HFull.GetRowMajorPtr(),
                          (unsigned char*)(*pCurImage)->imageData,
                          nImageWidth, nImageHeight, nImageWidthStep );

            HFull.id();
            HFull.mult( HRef );
            HFull.mult( HEst );

            // Compute reference patch from the current image
            CTrack::Warp( (const unsigned char*)(*pCurImage)->imageData,
                          nImageWidth, nImageHeight, nImageWidthStep,
                          HFull.GetRowMajorPtr(),
                          (unsigned char*)(*pRefPatch)->imageData,
                          nPatchWidth, nPatchHeight, nPatchWidthStep );

            ////////////////////////////////////////////////////////////////////
            ////////////////////////////////////////////////////////////////////
            ////////////////////////////////////////////////////////////////////
            // Test sub-sampled images
            CTrack::ImagePyramid refPatchPyr;
            refPatchPyr.BuildPyramid( 3, 
                                      (unsigned char*)(*pRefPatch)->imageData,
                                      nPatchWidth, nPatchHeight, nPatchWidthStep
                                      );
            CTrack::ImagePyramid curPyr;
            curPyr.BuildPyramid( 3, 
                                 (unsigned char*)(*pCurImage)->imageData,
                                 nImageWidth, nImageHeight, nImageWidthStep
                                 );

            const int nLevel = 2;

            IplImage* pRefPatchScaled = ImageHolderToOpenCV( refPatchPyr.GetImage( nLevel ) );
            IplImage* pCurImageScaled  = ImageHolderToOpenCV( curPyr.GetImage( nLevel ) );

            cvNamedWindow( "Cur", 1 );
            cvShowImage(   "Cur", pCurImageScaled );

            cvNamedWindow( "RefP2", 1 );
            cvShowImage(   "RefP2", pRefPatchScaled );

            const int nImageWidthScaled     = pCurImageScaled->width;
            const int nImageHeightScaled    = pCurImageScaled->height;
            const int nImageWidthStepScaled = pCurImageScaled->widthStep;

            IplImage* pCurPatchScaled = cvCreateImage( cvSize( nPatchWidth  >> nLevel,
                                                               nPatchHeight >> nLevel ),
                                                       (*pRefImage)->depth,
                                                       (*pRefImage)->nChannels );
            const int nPatchWidthScaled     = pCurPatchScaled->width;
            const int nPatchHeightScaled    = pCurPatchScaled->height;
            const int nPatchWidthStepScaled = pCurPatchScaled->widthStep;

            CTrack::Homography HFullScaled = HFull;
            HFullScaled.scale( 1 << nLevel );
            HFullScaled.print( "HFullScaled" );
            CTrack::Warp( (const unsigned char*)pCurImageScaled->imageData, ///< Input: image input
                          nImageWidthScaled,    ///< Input: image width
                          nImageHeightScaled,   ///< Input: image height
                          nImageWidthStepScaled,
                          HFullScaled.GetRowMajorPtr(),
                          (unsigned char*)pCurPatchScaled->imageData,  ///< Input/Output: memory allocated by the user that will be used to store the result of the warping
                          nPatchWidthScaled,    ///< Input: warped patch width
                          nPatchHeightScaled,   ///< Input: warped patch height
                          nPatchWidthStepScaled
                          );

            cvNamedWindow( "CurP", 1 );
            cvShowImage(   "CurP", pCurPatchScaled );

            // Now generate the blurred version
            // Create the reference patch
            double dError = RMS( (unsigned char*)pRefPatchScaled->imageData,
                                 (unsigned char*)pCurPatchScaled->imageData,
                                 nPatchWidthScaled, nPatchHeightScaled,
                                 nPatchWidthScaled, nPatchWidthStepScaled );
            std::cout << "Avg error when scaling: " << dError << std::endl; 

            ////////////////////////////////////////////////////////////////////
            ////////////////////////////////////////////////////////////////////
            ////////////////////////////////////////////////////////////////////
            if( bDraw ) {  
                cvNamedWindow( "Reference image", 1 );
                cvShowImage( "Reference image", *pRefImage );

                cvNamedWindow( "Current image", 1 );
                cvShowImage( "Current image", *pCurImage );
                
                cvNamedWindow( "Reference patch", 1 );
                cvShowImage( "Reference patch", *pRefPatch );

                cvWaitKey();
            }
        }
        else {
            // See appendix of TRO about generating synthetic motion blur.

            // dLambda: between 0 and 1, indicates the pourcentage of time
            // the shutter was open between two time steps
            double dLambda = 0.5;            

            // HPrev can be used to make sure that the blur is only using
            // the inter-frame motion and *not* the entire homography from
            // the beginning
            HPrev = CTrack::GenerateTranslation( -20, -10 );
            //HPrev.id();
            CTrack::Homography HPrevInv( HPrev ); HPrevInv.inv();

            // Compute lambda*log(HEst*inv(HPrev))
            CTrack::Homography M;
            M.mult( HEst );
            M.mult( HPrevInv );
            M.logm();
            M.mult( dLambda );
            
            CTrack::Homography Hl = HRef;
            Hl.mult( HPrevInv );

            CTrack::Homography Hr;
            Hr.mult( HRef );
            Hr.mult( HEst );
            Hr.mult( HPrevInv );
            Hr.inv();

            CTrack::NoMask noMask;
            const double dStep   = 1;
            const bool bJacobian = false;

            // Create the new current image 
            CTrack::WarpBlur<>( (const unsigned char*)(*pRefImage)->imageData,
                                nImageWidth, nImageHeight, nImageWidthStep,
                                &Hl, &M, &Hr,
                                (unsigned char*)(*pCurImage)->imageData,
                                nImageWidth, nImageHeight, nImageWidthStep,
                                noMask, dStep, bJacobian
                                );

            // Create the reference patch
            CTrack::WarpBlur<>( (const unsigned char*)(*pRefImage)->imageData, ///< Input: image input
                                nImageWidth, nImageHeight, nImageWidthStep,
                                &Hl, &M, &HPrev,
                                (unsigned char*)(*pRefPatch)->imageData,     ///< Input/Output: memory allocated by the user that will be used to store the result of the warping
                                nPatchWidth, nPatchHeight, nPatchWidthStep,
                                noMask, dStep, bJacobian
                                );

            // Test error when applying correct transform (i.e. the solution)
            CTrack::Homography HSol = HRef;
            HSol.mult( HEst );
            IplImage* pCurPatch = cvCreateImage( cvSize( nPatchWidth, nPatchHeight ),
                                                 (*pRefImage)->depth,
                                                 (*pRefImage)->nChannels );

            CTrack::Warp( (const unsigned char*)(*pCurImage)->imageData, ///< Input: image input
                          nImageWidth,    ///< Input: image width
                          nImageHeight,   ///< Input: image height
                          nImageWidthStep,
                          HSol.GetRowMajorPtr(),
                          (unsigned char*)pCurPatch->imageData,     ///< Input/Output: memory allocated by the user that will be used to store the result of the warping
                          nPatchWidth,    ///< Input: warped patch width
                          nPatchHeight,    ///< Input: warped patch height
                          nPatchWidthStep
                          );

            cvNamedWindow( "Current sol", 1 );
            cvShowImage( "Current sol", pCurPatch );

            double dError = RMS( (unsigned char*)(*pRefPatch)->imageData,
                                 (unsigned char*)pCurPatch->imageData,
                                 nPatchWidth, nPatchHeight,
                                 nPatchWidthStep, nPatchWidthStep );

            std::cout << "dLambda: " << dLambda << std::endl; 
            std::cout << "Avg error: " << dError << std::endl; 

            ////////////////////////////////////////////////////////////////////
            ////////////////////////////////////////////////////////////////////
            ////////////////////////////////////////////////////////////////////
            // Test for sub-sampled images
            CTrack::ImagePyramid refPyr;
            refPyr.BuildPyramid( 3, 
                                 (unsigned char*)(*pRefImage)->imageData,
                                 nImageWidth, nImageHeight, nImageWidthStep
                                 );
            CTrack::ImagePyramid curPyr;
            curPyr.BuildPyramid( 3, 
                                 (unsigned char*)(*pCurImage)->imageData,
                                 nImageWidth, nImageHeight, nImageWidthStep
                                 );

            const int nLevel = 2;

            IplImage* pRefImageScaled = ImageHolderToOpenCV( refPyr.GetImage( nLevel ) );
            IplImage* pCurImageScaled = ImageHolderToOpenCV( curPyr.GetImage( nLevel ) );

            cvNamedWindow( "Ref", 1 );
            cvShowImage(   "Ref", pRefImageScaled );

            cvNamedWindow( "Cur", 1 );
            cvShowImage(   "Cur", pCurImageScaled );

            const int nImageWidthScaled     = pCurImageScaled->width;
            const int nImageHeightScaled    = pCurImageScaled->height;
            const int nImageWidthStepScaled = pCurImageScaled->widthStep;

            IplImage* pCurPatchScaled = cvCreateImage( cvSize( nPatchWidth  >> nLevel,
                                                               nPatchHeight >> nLevel ),
                                                       (*pRefImage)->depth,
                                                       (*pRefImage)->nChannels );
            const int nPatchWidthScaled     = pCurPatchScaled->width;
            const int nPatchHeightScaled    = pCurPatchScaled->height;
            const int nPatchWidthStepScaled = pCurPatchScaled->widthStep;

            CTrack::Homography HSolScaled = HSol;
            HSolScaled.scale( 1 << nLevel );

            //HSolScaled.print( "HSolScaled" );

            CTrack::Warp( (const unsigned char*)pCurImageScaled->imageData, ///< Input: image input
                          nImageWidthScaled,    ///< Input: image width
                          nImageHeightScaled,   ///< Input: image height
                          nImageWidthStepScaled,
                          HSolScaled.GetRowMajorPtr(),
                          (unsigned char*)pCurPatchScaled->imageData,     ///< Input/Output: memory allocated by the user that will be used to store the result of the warping
                          nPatchWidthScaled,    ///< Input: warped patch width
                          nPatchHeightScaled,    ///< Input: warped patch height
                          nPatchWidthStepScaled
                          );

            cvNamedWindow( "CurP", 1 );
            cvShowImage(   "CurP", pCurPatchScaled );

            // Now generate the blurred version
            // Create the reference patch
            IplImage* pRefPatchScaled = cvCreateImage( cvSize( nPatchWidthScaled,
                                                               nPatchHeightScaled ),
                                                       (*pRefImage)->depth,
                                                       (*pRefImage)->nChannels );

            CTrack::Homography HlScaled = Hl;
            HlScaled.scale( 1 << nLevel );
            CTrack::Homography HPrevScaled = HPrev;
            HPrevScaled.scale( 1 << nLevel );

            CTrack::Homography MScaled = M;
            MScaled.scale( 1 << nLevel );

            //HlScaled.print( "HlScaled" );
            //MScaled.print( "MScaled" );
            //HPrevScaled.print( "HPrevScaled" );

            CTrack::WarpBlur<>( (const unsigned char*)pRefImageScaled->imageData, ///< Input: image input
                                nImageWidthScaled,    ///< Input: image width
                                nImageHeightScaled,   ///< Input: image height
                                nImageWidthStepScaled,
                                &HlScaled, &MScaled, &HPrevScaled,
                                (unsigned char*)pRefPatchScaled->imageData,     ///< Input/Output: memory allocated by the user that will be used to store the result of the warping
                                nPatchWidthScaled,    ///< Input: warped patch width
                                nPatchHeightScaled,   ///< Input: warped patch height
                                nPatchWidthStepScaled,
                                noMask, dStep, bJacobian
                                );  
            
            cvNamedWindow( "RefP", 1 );
            cvShowImage(   "RefP", pRefPatchScaled );

            dError = RMS( (unsigned char*)pRefPatchScaled->imageData,
                          (unsigned char*)pCurPatchScaled->imageData,
                          nPatchWidthScaled, nPatchHeightScaled,
                          nPatchWidthStepScaled, nPatchWidthStepScaled );
            std::cout << "Avg error scaling: " << dError << std::endl; 
            ////////////////////////////////////////////////////////////////////
            ////////////////////////////////////////////////////////////////////
            ////////////////////////////////////////////////////////////////////
            if( bDraw ) {  
                cvNamedWindow( "Reference image", 1 );
                cvShowImage( "Reference image", *pRefImage );

                cvNamedWindow( "Current image", 1 );
                cvShowImage( "Current image", *pCurImage );
                
                cvNamedWindow( "Reference patch", 1 );
                cvShowImage( "Reference patch", *pRefPatch );

                cvNamedWindow( "Current sol", 1 );
                cvShowImage( "Current sol", pCurPatch );

                cvReleaseImage( &pCurPatch );

                cvWaitKey();
            }
            // Compute the reference patch assuming unknown motion (for drawing)
            M.zero(); CTrack::Homography Id;
            CTrack::WarpBlur<>( (const unsigned char*)(*pRefImage)->imageData, ///< Input: image input
                                nImageWidth,    ///< Input: image width
                                nImageHeight,   ///< Input: image height
                                nImageWidthStep,
                                &HRef, &M, &Id,
                                (unsigned char*)(*pRefPatch)->imageData,     ///< Input/Output: memory allocated by the user that will be used to store the result of the warping
                                nPatchWidth,    ///< Input: warped patch width
                                nPatchHeight,    ///< Input: warped patch height
                                nPatchWidthStep,
                                noMask, dStep, bJacobian
                                );
        }

        if( fAlpha != 1 || fBeta != 0 ) {
            // Add change in illumination
            for( int ii=0; ii < (*pCurImage)->width; ii++ ) {
                for( int jj=0; jj < (*pCurImage)->height; jj++ ) {
                    int nCoord = ii + jj*(*pCurImage)->widthStep;
                    unsigned char* pRefVal = &( (unsigned char*) (*pCurImage)->imageData)[nCoord];
                    float fVal = 1/fAlpha * ( (*pRefVal) - fBeta );
                    *pRefVal = fVal> 255 ? 255 : ( fVal < 0 ? 0 : char(fVal) );
                }
            }
            std::cout << "Simulation with alpha=" << fAlpha << " and beta=" << fBeta << std::endl;
        }

        return true;
    }
}

#endif
