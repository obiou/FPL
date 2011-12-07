#include <algorithm>
#include <ctype.h>
#include <iomanip>
#include <iostream>
#include <limits.h>
#include <math.h>
#include <memory>
#include <stdio.h>
#include <time.h>

#include <cv.h>
#include <highgui.h>

#include <CTrack/PlaneTracking.h>
#include <CTrack/Homography.h>
#include <CTrack/Misc.h>
#include <CTrack/TestingHelpers.h>
#include <CTrack/TrackingUpdates.h>
#include <CTrack/Transforms.h>

#include <CTrack/OpenCVHelpers.h>

using namespace std;
using namespace Eigen;

//////////////////////////////////////////////////////////////////////////////
int main( int argc, char** argv )
{
    string sImageName;
    if( argc == 1 ) {
        sImageName = "lena.jpg";
    }
    else if( argc == 2 ) {
        sImageName = argv[1];
    }
    else {
        cout << "Expecting one argument: image name" << endl;
        return -1;
    }

    //TrackingMotionModel eType = TRANSLATION;
    //TrackingMotionModel eType = AFFINE;
    //TrackingMotionModel eTrackingMotionModel = HOMOGRAPHY;
    TrackingMotionModel eTrackingMotionModel = HOMOGRAPHY_BLUR_MAGN;

    TrackingIlluminationModel eTrackingIlluminationModel = NONE;
    //TrackingIlluminationModel eTrackingIlluminationModel = AFFINE_ILLUM;

    //const int nPatchWidth  = 99;
    //const int nPatchHeight = 98;
    const int nPatchWidth  = 100;
    const int nPatchHeight = 100;
    const int nPatchBegX   = 140;
    const int nPatchBegY   = 200;

    IplImage* pRefImage = NULL;
    IplImage* pRefPatch = NULL;
    IplImage* pCurImage = NULL;
    IplImage* pCurPatch = NULL;
    CTrack::Homography HPrev, HEst;
    if( !GenerateTestImage( sImageName,
                            nPatchBegX, nPatchBegY, nPatchWidth, nPatchHeight,
                            eTrackingMotionModel,
                            eTrackingIlluminationModel,
                            &pRefImage,
                            &pRefPatch,
                            &pCurImage,
                            HPrev,
                            HEst, true
                            ) ) {
        return -1;
    }

    ////////////////////////////////////////////////////////////////////////////
    const int nImageWidth     = pCurImage->width;
    const int nImageHeight    = pCurImage->height;  
    const int nImageWidthStep = pCurImage->widthStep;
    const int nPatchWidthStep = pRefPatch->widthStep;

    cvNamedWindow( "Reference Template", 1 );
    cvShowImage( "Reference Template", pRefPatch );
    cvMoveWindow( "Current Template", 0, 100 );

    // Allocate buffers
    pCurPatch = cvCreateImage( cvGetSize( pRefPatch ),
                               pRefPatch->depth,
                               pRefPatch->nChannels );

    float pRefPatchGradX[ nPatchWidth*nPatchHeight ];
    float pRefPatchGradY[ nPatchWidth*nPatchHeight ];

    float pRefPatchWTGradX[ nPatchWidth*nPatchHeight ];
    float pRefPatchWTGradY[ nPatchWidth*nPatchHeight ];

    float pCurPatchGradX[ nPatchWidth*nPatchHeight ];
    float pCurPatchGradY[ nPatchWidth*nPatchHeight ];

    // Compute gradient of reference patch
    CTrack::Gradient( (unsigned char*)pRefPatch->imageData,
                      nPatchWidth, nPatchHeight, nPatchWidthStep,
                      pRefPatchGradX, pRefPatchGradY );

    double dPrevRMS = 0.;
    double dBestRMS = 0.;
    double dNormUpdate = 0.;

    int nNumIter = 0;
    int nMaxIter = 40;
    double dMinSumSqErrorsGain = 0.1;
    bool bSuccess = true;

    CTrack::TrackingResults trackingResultsPrev, trackingResultsCur, trackingResultsBest;

    CTrack::NoMask maskFctr( nPatchWidth, nPatchHeight );

    //eTrackingMotionModel = HOMOGRAPHY; // for debugging

    bSuccess = CTrack::SSDUpdateSL3Motion( (unsigned char*)pRefPatch->imageData, 
                                   pRefPatchGradX, pRefPatchGradY,
                                   pRefPatchWTGradX, pRefPatchWTGradY,
                                   nPatchWidth, nPatchHeight, nPatchWidthStep,
                                   (unsigned char*) pRefImage->imageData,
                                   (unsigned char*) pCurImage->imageData, 
                                   nImageWidth, nImageHeight, nImageWidthStep,
                                   nPatchBegX, nPatchBegY,
                                   (unsigned char*) pCurPatch->imageData,
                                   pCurPatchGradX, pCurPatchGradY,
                                   &trackingResultsCur,
                                   &dPrevRMS, &dNormUpdate, 
                                   eTrackingMotionModel, eTrackingIlluminationModel,
                                   maskFctr
                                   );
    if( !bSuccess ) {
        cout << "Problem when computing SL3 motion." << endl;

        cvNamedWindow( "CurT", 1 );
        cvShowImage( "CurT", pCurPatch );

        cvNamedWindow( "RefT2", 1 );
        cvShowImage( "RefT2", pRefPatch );

        cvNamedWindow( "Reference Image2", 1 );
        cvShowImage( "Reference Image2", pRefImage );

        cvWaitKey();
        return -1;
    }

    cout << "Initial RMS: " << std::setprecision(14) << dPrevRMS;
    if( CTrack::BlurModel( eTrackingMotionModel ) ) {
        std::cout << ", lambda: " << trackingResultsCur.GetBlurMagn();
    }
    std::cout << std::endl;

    dBestRMS = dPrevRMS;
    trackingResultsBest = trackingResultsPrev;
    trackingResultsPrev = trackingResultsCur;
    
    cvNamedWindow( "Current Template", 1 );
    cvMoveWindow( "Current Template", 100, 100);

    for( nNumIter=0; nNumIter < nMaxIter; nNumIter++ ) {
        cvShowImage( "Current Template", pCurPatch );
        cvShowImage( "Reference Template", pRefPatch );

        double t0 = Tic();
        bSuccess = CTrack::SSDUpdateSL3Motion<>( (unsigned char*)pRefPatch->imageData, 
                                       pRefPatchGradX, pRefPatchGradY,
                                       pRefPatchWTGradX, pRefPatchWTGradY,
                                       nPatchWidth, nPatchHeight, nPatchWidthStep,
                                       (unsigned char*) pRefImage->imageData,
                                       (unsigned char*) pCurImage->imageData, 
                                       nImageWidth, nImageHeight, nImageWidthStep,
                                       nPatchBegX, nPatchBegY,
                                       (unsigned char*) pCurPatch->imageData, 
                                       pCurPatchGradX, pCurPatchGradY,
                                       &trackingResultsCur,
                                       &dPrevRMS, &dNormUpdate, 
                                       eTrackingMotionModel, eTrackingIlluminationModel,
                                       maskFctr ); 

        if( !bSuccess ) {
            cout << "Problem when computing SL3 motion." << endl;
            break;
        }

        if( dPrevRMS > dBestRMS ) {
            cout << "RMS increasing from " << dBestRMS << " to " << dPrevRMS << endl;
            break;
        }
        if( dBestRMS - dPrevRMS < dMinSumSqErrorsGain ) {
            cout << "RMS decreased too slowly by: " << dBestRMS - dPrevRMS << " to: " << dPrevRMS << endl;
            dBestRMS = dPrevRMS;
            trackingResultsBest = trackingResultsPrev;
            break;
        }
        std::cout << "dRMS: " << dPrevRMS << "/";// << std::endl;
        std::cout << "Time: " << TocMS( t0 ) << "ms";
        if( CTrack::BlurModel( eTrackingMotionModel ) ) {
            std::cout << "/lambda: " << trackingResultsCur.GetBlurMagn();
        }
        std::cout << std::endl;

        dBestRMS = dPrevRMS;
        trackingResultsBest = trackingResultsPrev;
        trackingResultsPrev = trackingResultsCur;

        int c = cvWaitKey( 500 );
        //int c = cvWaitKey( 10 );
        if( (char) c == 27 ) {
            break;
        } 
    }

    cout << "Final RMS: " << dBestRMS << endl;
    cout << "Number of iterations: " << nNumIter << endl;

    // Check
    bSuccess = CTrack::SSDUpdateSL3Motion<>( (unsigned char*)pRefPatch->imageData, 
                                   pRefPatchGradX, pRefPatchGradY,
                                   pRefPatchWTGradX, pRefPatchWTGradY,
                                   nPatchWidth, nPatchHeight, nPatchWidthStep,
                                   (unsigned char*) pRefImage->imageData,
                                   (unsigned char*) pCurImage->imageData, 
                                   nImageWidth, nImageHeight, nImageWidthStep,
                                   nPatchBegX, nPatchBegY,
                                   (unsigned char*) pCurPatch->imageData,
                                   pCurPatchGradX, pCurPatchGradY,
                                   &trackingResultsBest,
                                   &dBestRMS, &dNormUpdate, 
                                   eTrackingMotionModel, eTrackingIlluminationModel,
                                   maskFctr
                                   );
    cout << "Final RMS: " << dBestRMS << endl;

    cvReleaseImage( &pCurImage );
    cvReleaseImage( &pRefPatch );
    cvReleaseImage( &pCurPatch );
    
    cvDestroyWindow( "Reference Template" );
    cvDestroyWindow( "Current Template" );

    return 0;
}
