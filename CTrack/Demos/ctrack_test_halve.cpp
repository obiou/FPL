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

#include <CTrack/Homography.h>
#include <CTrack/Misc.h>
#include <CTrack/OpenCVHelpers.h>
#include <CTrack/Transforms.h>

using namespace std;

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

    // Load reference image
    IplImage* pRefImage = CTrack::LoadToGray( sImageName );
    const int nImageWidth  = pRefImage->width;
    const int nImageHeight = pRefImage->height;  
    const int nImageWidthStep = pRefImage->widthStep;

    //cvNamedWindow( "Reference image", 1 );
    //cvShowImage( "Reference image", pRefImage );

    IplImage* pHalfImage = cvCreateImage( cvSize( pRefImage->width/2, pRefImage->height/2 ),
                                          pRefImage->depth,
                                          pRefImage->nChannels );

    const int nHalfImageWidth     = pHalfImage->width;
    const int nHalfImageHeight    = pHalfImage->height;  
    const int nHalfImageWidthStep = pHalfImage->widthStep;

    cout << "widthSteps: " << pRefImage->widthStep << " " << pHalfImage->widthStep << endl;
    CTrack::HalveImage( (const unsigned char*)pRefImage->imageData,
                        nImageWidth, 
                        nImageHeight,
                        nImageWidthStep,
                        (unsigned char*)pHalfImage->imageData,
                        nHalfImageWidthStep );

    //cvNamedWindow( "Half Reference image", 1 );
    //cvShowImage( "Half Reference image", pHalfImage );

    // Create new test image
    IplImage* pHalfImageTest = cvCreateImage( cvGetSize( pHalfImage ),
                                              pHalfImage->depth,
                                              pHalfImage->nChannels );
    CTrack::Homography H;
    H.Set( 0, 0, 2 );
    H.Set( 1, 1, 2 );
    H.Set( 0, 2, 0.5 );
    H.Set( 1, 2, 0.5 );

    // Warp
    CTrack::Warp( (const unsigned char*)pRefImage->imageData, ///< Input: image input
                  nImageWidth,    ///< Input: image width
                  nImageHeight,   ///< Input: image height
                  nImageWidthStep,
                  H.GetRowMajorPtr(),   ///< Input: 3x3 homography matrix in row-major
                  (unsigned char*)pHalfImageTest->imageData,     ///< Input/Output: memory allocated by the user that will be used to store the result of the warping
                  nHalfImageWidth,    ///< Input: warped patch width
                  nHalfImageHeight,    ///< Input: warped patch height
                  nHalfImageWidthStep
                  );

    // Extract sub-image 
    //cvNamedWindow( "Check Template Scaled", 1 );
    //cvShowImage( "Check Template Scaled", pHalfImageTest );

    // Should be the same for the first level of the pyramid
    cout << "Diff: " << cvNorm( pHalfImage, pHalfImageTest ) << endl;




    // Draw halved image for the reference patch
    // Extract patch
    const int nPatchBegX   = 100;
    const int nPatchBegY   = 100;
    const int nPatchWidth  = 99;
    const int nPatchHeight = 99;
    IplImage* pPatchImage = CTrack::ExtractPatch( pRefImage,
                                                  nPatchBegX, nPatchBegY, 
                                                  nPatchWidth, nPatchHeight );
    const int nPatchWidthStep = pPatchImage->widthStep;

    IplImage* pHalfPatch = cvCreateImage( cvSize( nPatchWidth/2, nPatchHeight/2 ),
                                          pPatchImage->depth,
                                          pPatchImage->nChannels );

    const int nHalfPatchWidthStep = pHalfPatch->widthStep;

    CTrack::HalveImage( (const unsigned char*)pPatchImage->imageData,
                        nPatchWidth, 
                        nPatchHeight,
                        nPatchWidthStep,
                        (unsigned char*)pHalfPatch->imageData,
                        nHalfPatchWidthStep );

    cvNamedWindow( "Patch image", 1 );
    cvShowImage( "Patch image", pPatchImage );

    cvNamedWindow( "Half patch image", 1 );
    cvShowImage( "Half patch image", pHalfPatch );

    cvWaitKey();

    ////////////////////////////////////////////////////////////////////////////
    // Release buffers
    cvReleaseImage( &pRefImage );
    cvReleaseImage( &pHalfImage );
    cvReleaseImage( &pHalfImageTest );

    cvReleaseImage( &pPatchImage );
    cvReleaseImage( &pHalfPatch );

    return 0;
}
