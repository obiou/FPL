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
#include <CTrack/ImagePyramid.h>
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

    //typedef TranslationFunctor Tracker;
    //typedef HomographyFunctor Tracker; 
    typedef HomographyIlluminationFunctor Tracker; 

    bool bRegularise = true;

    const int nPatchBegX   = 143;
    const int nPatchBegY   = 205;
    const int nPatchWidth  = 95;
    const int nPatchHeight = 110;

    IplImage* pRefImage = NULL;
    IplImage* pRefPatch = NULL;
    IplImage* pCurImage = NULL;
    CTrack::Homography HRef, HPrev, HEst;
    if( !GenerateTestImage( sImageName,
                            nPatchBegX, nPatchBegY, nPatchWidth, nPatchHeight,
                            Tracker::MOTION_MODEL, 
                            Tracker::ILLUMINATION_MODEL,
                            &pRefImage, &pRefPatch,
                            &pCurImage,
                            HPrev, HEst, 
                            Tracker::BLUR
                            ) ) {
        return -1;
    }

    ////////////////////////////////////////////////////////////////////////////
    // Prepare structures for tracking
    const int nImageWidth  = pCurImage->width;
    const int nImageHeight = pCurImage->height;  
    const int nPatchWidthStep = pRefPatch->widthStep;
    const int nImageWidthStep = pCurImage->widthStep;

    int nNumLevels = 3;

    ImagePyramid refImagePyramid;
    refImagePyramid.BuildPyramid( nNumLevels,
                                  (unsigned char*)pRefImage->imageData,
                                  nImageWidth, nImageHeight, nImageWidthStep ); 
    ImagePyramid refPatchPyramid;
    refPatchPyramid.BuildPyramid( nNumLevels,
                                  (unsigned char*)pRefPatch->imageData,
                                  nPatchWidth, nPatchHeight, nPatchWidthStep ); 
    ImagePyramid curImagePyramid;
    curImagePyramid.BuildPyramid( nNumLevels,
                                  (unsigned char*)pCurImage->imageData,
                                  nImageWidth, nImageHeight, nImageWidthStep );
    TrackingStatsPyr trackingStats( nNumLevels );
    TrackingSettingsPyr<Tracker,NoMask> trackingSettingsPyr1;
    trackingSettingsPyr1.InitReset( nPatchBegX, nPatchBegY, 
                                    nPatchWidth, nPatchHeight, nPatchWidthStep,
                                    bRegularise, nNumLevels );

    TrackingSettingsPyr<Tracker,BoolMask> trackingSettingsPyr2;
    trackingSettingsPyr2.InitReset( nPatchBegX, nPatchBegY, 
                                    nPatchWidth, nPatchHeight, nPatchWidthStep,
                                    bRegularise, nNumLevels
                                    );

    TrackingResults trackingResults;
    // Track
    trackingResults.SetPrevHomography( HPrev );
    //trackingResults.SetHomography( HEst );
    //trackingResults.SetBlurMagn( 0.5 );
    TrackPlaneHomographyPyr<Tracker>
        ( trackingSettingsPyr1,
          refImagePyramid, refPatchPyramid, curImagePyramid,
          //refPatchPyramid, curImagePyramid,
          &trackingStats,
          &trackingResults
          );

    std::cout << "True value:" << std::endl;
    HEst.print();
    std::cout << "Estimated value:" << std::endl;
    trackingResults.GetHomography().print();
   
#if 0
    trackingResults.SetHomography( HPrev );
    trackingResults.CopyCurToPrev();
    //trackingResults.id();
    TrackPlaneHomographyPyr<LineJacobianHomography,BoolMask>
        ( trackingSettingsPyr2,
          refImagePyramid, refPatchPyramid, curImagePyramid,
          //refPatchPyramid, curImagePyramid,
          &trackingStats,
          &trackingResults
          );

    std::cout << "FIXME: why is the RMS not better at higher pyramid levels (aliasing?)???" << std::endl;
#endif

    ////////////////////////////////////////////////////////////////////////////
    // Release buffers
    cvReleaseImage( &pRefImage );
    cvReleaseImage( &pRefPatch );
    cvReleaseImage( &pCurImage );

    return 0;
}
