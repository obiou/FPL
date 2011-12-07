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
    const int nImageWidthStep = pRefImage->widthStep;

    // Extract patch
    const int nPatchBegX   = 100;
    const int nPatchBegY   = 120;
    const int nPatchWidth  = 30;
    const int nPatchHeight = 50;
    IplImage* pRefPatchImage = CTrack::ExtractPatch( pRefImage,
                                             nPatchBegX, nPatchBegY, 
                                             nPatchWidth, nPatchHeight );
    const int nPatchWidthStep = pRefPatchImage->widthStep;

    // Test by re-extracting
    CTrack::Homography H;
    H.Set( 0, 2, nPatchBegX );
    H.Set( 1, 2, nPatchBegY );

    // Allocate memory for new image
    IplImage* pRefPatchImage2 = cvCreateImage( cvGetSize( pRefPatchImage ),
                                               pRefPatchImage->depth,
                                               pRefPatchImage->nChannels );

    // Warp
    CTrack::Warp( (const unsigned char*)pRefImage->imageData,
                  pRefImage->width, 
                  pRefImage->height,
                  nImageWidthStep,
                  H.GetRowMajorPtr(),   ///< Input: 3x3 homography matrix in row-major
                  (unsigned char*) pRefPatchImage2->imageData, 
                  nPatchWidth, 
                  nPatchHeight,
                  nPatchWidthStep
                  );

    cout << "Diff: " << cvNorm( pRefPatchImage, pRefPatchImage2 ) << endl;

    // Warp
    CTrack::WarpTranslation( (const unsigned char*)pRefImage->imageData,
                             pRefImage->width,  
                             pRefImage->height, 
                             nImageWidthStep,
                             nPatchBegX,
                             nPatchBegY,
                             (unsigned char*) pRefPatchImage2->imageData,
                             nPatchWidth,    
                             nPatchHeight,   
                             nPatchWidthStep
                             );

    cout << "Diff: " << cvNorm( pRefPatchImage, pRefPatchImage2 ) << endl;

    // Draw reference image
    cvNamedWindow( "Reference patch", 1 );
    cvShowImage( "Reference patch", pRefPatchImage );

    cvNamedWindow( "Reference patch 2", 1 );
    cvShowImage( "Reference patch 2", pRefPatchImage2 );

    cvWaitKey();

    ////////////////////////////////////////////////////////////////////////////
    // Release buffers
    cvReleaseImage( &pRefPatchImage );
    cvReleaseImage( &pRefPatchImage2 );
    cvReleaseImage( &pRefImage );

    return 0;
}
