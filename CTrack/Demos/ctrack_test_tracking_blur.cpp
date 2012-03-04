#include <algorithm>
#include <ctype.h>
#include <iostream>
#include <limits.h>
#include <math.h>
#include <memory>
#include <stdio.h>
#include <time.h>

#include <opencv2/highgui/highgui.hpp>

#include <CTrack/PlaneTracking.h>
#include <CTrack/PlaneTrackingOpenCV.h>
#include <CTrack/Homography.h>
#include <CTrack/MatrixLogarithm.h>
#include <CTrack/Misc.h>
#include <CTrack/OpenCVHelpers.h>
#include <CTrack/TrackingFunctors.h>
#include <CTrack/Transforms.h>
#include <CTrack/TransformsHighLevel.h>

using namespace std;
using namespace CTrack;

///
/// Test code for blurring (currently only the inefficient version is shown)
///
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

    typedef HomographyBlurMagnitudeFunctor Tracker;

    const int nPatchWidth  = 100;
    const int nPatchHeight = 100;
    const int nPatchBegX   = 140;
    const int nPatchBegY   = 200;
    const bool bDraw       = true;

    IplImage* pRefImage = NULL;
    IplImage* pRefPatch = NULL;
    IplImage* pCurImage = NULL;
    CTrack::Homography HRef, HPrev, HEst;
    if( !GenerateTestImage( sImageName,
                            nPatchBegX, nPatchBegY, nPatchWidth, nPatchHeight,
                            Tracker::MOTION_MODEL,
                            Tracker::ILLUMINATION_MODEL,
                            &pRefImage,
                            &pRefPatch,
                            &pCurImage,
                            HPrev,
                            HEst, Tracker::BLUR
                            ) ) {
        return -1;
    }

    ////////////////////////////////////////////////////////////////////////////
    if( bDraw ) {
        cvNamedWindow( "Reference image", 1 );
        cvShowImage( "Reference image", pRefImage );

        cvNamedWindow( "Reference Template", 1 );
        cvShowImage( "Reference Template", pRefPatch );

        cvNamedWindow( "Current image", 1 );
        cvShowImage( "Current image", pCurImage );
    }

    ////////////////////////////////////////////////////////////////////////////
    // Track
    // Prepare structures for tracking
    const int nImageWidth     = pCurImage->width;
    const int nImageHeight    = pCurImage->height;  
    const int nImageWidthStep = pCurImage->widthStep;
    const int nPatchWidthStep = pRefPatch->widthStep;

    const ImageHolder refImageHolder( (unsigned char*)pRefImage->imageData,
                                      nImageWidth, nImageHeight, nImageWidthStep );
    const ImageHolder refPatchHolder( (unsigned char*)pRefPatch->imageData,
                                      nPatchWidth, nPatchHeight, nPatchWidthStep );
    const ImageHolder curImageHolder( (unsigned char*)pCurImage->imageData,
                                      nImageWidth, nImageHeight, nImageWidthStep );

    TrackingStats trackingStats;
    TrackingResults trackingResults;
    TrackingSettings<Tracker,NoMask> trackingSettings1
        ( nPatchBegX, nPatchBegY, nPatchWidth, nPatchHeight, nPatchWidthStep );
    TrackingSettings<Tracker,BoolMask> trackingSettings2
        ( nPatchBegX, nPatchBegY, nPatchWidth, nPatchHeight, nPatchWidthStep );

    trackingSettings1.MaxNumIterations( 40 );
    trackingSettings2.MaxNumIterations( 40 );

    // Track
    trackingResults.SetPrevHomography( HPrev );
    TrackPlaneHomography<Tracker>
        ( trackingSettings1,
          refImageHolder, refPatchHolder, curImageHolder,                 
          &trackingStats,
          &trackingResults
          ); 

    cout << "HEst:" << endl;
    cout << HEst.m_mH << endl;

    cout << "Estimated H:" << endl;
    cout << trackingResults.GetHomography().m_mH << endl;

    trackingResults.id();
    trackingResults.SetPrevHomography( HPrev );
    TrackPlaneHomography<Tracker>
        ( 
         trackingSettings2,
         refImageHolder, refPatchHolder, curImageHolder,
         &trackingStats,
         &trackingResults
          );

    ////////////////////////////////////////////////////////////////////////////
    if( bDraw ) {
        ImageHolder image( trackingSettings1.GetTrackBuffer()->CurPatch(),
                           nPatchWidth, nPatchHeight, nPatchWidthStep );
        
        cvNamedWindow( "Final image", 1 );
        cvShowImage( "Final image", ImageHolderToOpenCV( &image ) );

        cvNamedWindow( "RT2", 1 );
        cvShowImage( "RT2", pRefPatch );

        cvWaitKey();
    }

    ////////////////////////////////////////////////////////////////////////////
    // Release buffers
    cvReleaseImage( &pRefImage );
    cvReleaseImage( &pCurImage );
    cvReleaseImage( &pRefPatch );

    return 0;
}
