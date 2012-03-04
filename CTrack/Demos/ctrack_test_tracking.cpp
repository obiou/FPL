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
#include <CTrack/TestingHelpers.h>
#include <CTrack/TrackingFunctors.h>
#include <CTrack/TrackingUpdates.h>
#include <CTrack/Transforms.h>

#include <CTrack/OpenCVHelpers.h>

using namespace std;
using namespace Eigen;
using namespace CTrack;

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

    typedef AffineFunctor Tracker;

    const int nPatchWidth  = 100;
    const int nPatchHeight = 100;
    const int nPatchBegX   = 140;
    const int nPatchBegY   = 200;
    const bool bDraw       = true;

    IplImage* pRefImage = NULL;
    IplImage* pRefPatch = NULL;
    IplImage* pCurImage = NULL;
    IplImage* pCurPatch = NULL;
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
    
        cvWaitKey();
    }

    ////////////////////////////////////////////////////////////////////////////
    // Prepare structures for tracking
    const int nImageWidth  = pCurImage->width;
    const int nImageHeight = pCurImage->height;  
    const int nPatchWidthStep = pRefPatch->widthStep;
    const int nImageWidthStep = pCurImage->widthStep;

    const ImageHolder refPatchHolder( (unsigned char*)pRefPatch->imageData,
                                      nPatchWidth, nPatchHeight, nPatchWidthStep );
    const ImageHolder curImageHolder( (unsigned char*)pCurImage->imageData,
                                      nImageWidth, nImageHeight, nImageWidthStep );

    TrackingStats trackingStats;
    TrackingResults trackingResults;
    TrackingSettings<Tracker,NoMask> trackingSettings1( nPatchBegX, nPatchBegY, 
                                                nPatchWidth, nPatchHeight, nPatchWidthStep );
    TrackingSettings<Tracker,BoolMask> trackingSettings2( nPatchBegX, nPatchBegY, 
                                                  nPatchWidth, nPatchHeight, nPatchWidthStep );

    trackingSettings1.MaxNumIterations( 100 );
    trackingSettings2.MaxNumIterations( 100 );

    // Track
    TrackPlaneHomography<Tracker>
        ( trackingSettings1,
          refPatchHolder, curImageHolder,                 
          &trackingStats,
          &trackingResults
          ); 

    trackingResults.id();

    TrackPlaneHomography<Tracker>
        ( trackingSettings2,
          refPatchHolder, curImageHolder,
          &trackingStats,
          &trackingResults
          );

    std::cout << "True value:" << std::endl;
    HEst.print();
    std::cout << "Estimated value:" << std::endl;
    trackingResults.GetHomography().print();
    
    ////////////////////////////////////////////////////////////////////////////
    if( bDraw ) {
        ImageHolder image( trackingSettings1.GetTrackBuffer()->CurPatch(),
                           nPatchWidth, nPatchHeight, nPatchWidth );
        
        cvNamedWindow( "Final image", 1 );
        cvShowImage( "Final image", ImageHolderToOpenCV( &image ) );
        
        cvWaitKey();
    }

    ////////////////////////////////////////////////////////////////////////////
    // Release buffers
    cvReleaseImage( &pRefPatch );
    cvReleaseImage( &pCurImage );
    cvReleaseImage( &pCurPatch );

    return 0;
}
