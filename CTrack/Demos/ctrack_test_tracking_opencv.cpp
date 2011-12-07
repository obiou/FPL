#include <algorithm>
#include <ctype.h>
#include <iostream>
#include <limits.h>
#include <math.h>
#include <memory>
#include <stdio.h>
#include <time.h>

#include <cv.h>
#include <highgui.h>

#include <CTrack/PlaneTracking.h>
#include <CTrack/PlaneTrackingOpenCV.h>
#include <CTrack/Homography.h>
#include <CTrack/TestingHelpers.h>
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

    //TrackingMotionModel eType = TRANSLATION;
    //TrackingMotionModel eType = AFFINE;
    TrackingMotionModel eTrackingMotionModel = HOMOGRAPHY;

    TrackingIlluminationModel eTrackingIlluminationModel = NONE;
    //TrackingIlluminationModel eTrackingIlluminationModel = AFFINE_ILLUM;

    const int nPatchWidth  = 100;
    const int nPatchHeight = 100;
    const int nPatchBegX   = 140;
    const int nPatchBegY   = 200;

    IplImage* pRefImage = NULL;
    IplImage* pRefPatch = NULL;
    IplImage* pCurImage = NULL;
    CTrack::Homography HRef, HPrev, HEst;
    if( !GenerateTestImage( sImageName,
                            nPatchBegX, nPatchBegY, nPatchWidth, nPatchHeight,
                            eTrackingMotionModel, eTrackingIlluminationModel,
                            &pRefImage, &pRefPatch,
                            &pCurImage,
                            HPrev, HEst, 
                            CTrack::BlurModel( eTrackingMotionModel ) 
                            ) ) {
        return -1;
    }
    
    const int nPatchWidthStep = pRefImage->widthStep;

    ////////////////////////////////////////////////////////////////////////////
    TrackingStats trackingStats;
    TrackingResults trackingResults;
    TrackingSettings<NoMask> trackingSettings1( nPatchBegX, nPatchBegY,
                                                nPatchWidth, nPatchHeight, nPatchWidthStep );
    TrackingSettings<BoolMask> trackingSettings2( nPatchBegX, nPatchBegY,
                                                  nPatchWidth, nPatchHeight, nPatchWidthStep );
    trackingSettings1.MaxNumIterations( 40 );
    trackingSettings2.MaxNumIterations( 40 );

    // Track
    TrackPlaneHomography( trackingSettings1,
                          *pRefPatch, *pCurImage,
                          &trackingStats,
                          &trackingResults
                          );

    trackingResults.id();

    // Track
    TrackPlaneHomography( trackingSettings2,
                          *pRefPatch, *pCurImage,
                          &trackingStats,
                          &trackingResults
                          );
    ////////////////////////////////////////////////////////////////////////////

    // Release buffers
    cvReleaseImage( &pRefImage );
    cvReleaseImage( &pRefPatch );
    cvReleaseImage( &pCurImage );

    return 0;
}
