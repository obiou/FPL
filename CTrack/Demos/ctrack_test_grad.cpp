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

    // Extract patch
    const int nPatchBegX   = 100;
    const int nPatchBegY   = 120;
    const int nPatchWidth  = 3;
    const int nPatchHeight = 5;

    IplImage* pRefPatchImage = CTrack::ExtractPatch( pRefImage,
                                                     nPatchBegX, nPatchBegY, 
                                                     nPatchWidth, nPatchHeight );

    const int nPatchWidthStep = pRefPatchImage->widthStep;

    cvNamedWindow( "Reference patch", 1 );
    cvShowImage( "Reference patch", pRefPatchImage );


    IplImage* pHalfImage = cvCreateImage( cvSize( pRefImage->width/2, pRefImage->height/2 ),
                                          pRefImage->depth,
                                          pRefImage->nChannels );

    cout << "widthSteps: " << pRefImage->widthStep << " " << pHalfImage->widthStep << endl;
    CTrack::HalveImage( (const unsigned char*)pRefImage->imageData,
                        pRefImage->width, 
                        pRefImage->height,
                        pRefImage->widthStep,
                        (unsigned char*)pHalfImage->imageData,
                        pHalfImage->widthStep );

    rect = cvRect( nPatchBegX, nPatchBegY, nPatchWidth, nPatchHeight );
    cvSetImageROI( pRefImage, rect ); // Sets the region of interest

    IplImage* pNewImage = cvCreateImage( cvGetSize( pRefImage ),
                                         pRefImage->depth,
                                         pRefImage->nChannels );

    cvCopy( pRefImage, pNewImage, NULL );
    cvResetImageROI( pRefImage );


    cvNamedWindow( "Half Reference image", 1 );
    cvShowImage( "Half Reference image", pHalfImage );

    // Extract sub-image 
    cvNamedWindow( "Template", 1 );
    cvShowImage( "Template", pNewImage );


    // Extract sub-image 
    cvNamedWindow( "Check Template", 1 );
    cvShowImage( "Check Template", pTestTemplate );

    H.Set( 0, 0, 2 );
    H.Set( 1, 1, 2 );

    // Warp
    CTrack::Warp( (const unsigned char*)pRefImage->imageData, ///< Input: image input
                  pRefImage->width,    ///< Input: image width
                  pRefImage->height,   ///< Input: image height
                  nImageWidthStep,
                  H.GetRowMajorPtr(),   ///< Input: 3x3 homography matrix in row-major
                  (unsigned char*)pTestTemplate->imageData,     ///< Input/Output: memory allocated by the user that will be used to store the result of the warping
                  nPatchWidth,    ///< Input: warped patch width
                  nPatchHeight,    ///< Input: warped patch height
                  nPatchWidthStep
                  );

    // Extract sub-image 
    cvNamedWindow( "Check Template Scaled", 1 );
    cvShowImage( "Check Template Scaled", pTestTemplate );

    // 
    float* pGradX = (float*) calloc( nImageWidth*nImageHeight, sizeof( float ) );
    float* pGradY = (float*) calloc( nImageWidth*nImageHeight, sizeof( float ) );
    CTrack::Gradient( (unsigned char*)pRefImage->imageData,
                      pRefImage->width, 
                      pRefImage->height,
                      pGradX,          
                      pGradY           
                      );

    unsigned char* pGradX2 = (unsigned char*) calloc( nImageWidth*nImageHeight, sizeof( unsigned char ) );
    unsigned char* pGradY2 = (unsigned char*) calloc( nImageWidth*nImageHeight, sizeof( unsigned char ) );

    for( int ii = 0; ii< nImageWidth*nImageHeight; ii++ ) {
        pGradX2[ ii ] = (pGradX[ ii ]*2 + 125);
        pGradY2[ ii ] = (pGradY[ ii ]*2 + 125);
    }

    cv::Mat gradMatX( nImageHeight, nImageWidth, CV_8UC1, (void*)pGradX2 );
    cv::Mat gradMatY( nImageHeight, nImageWidth, CV_8UC1, (void*)pGradY2 );

    IplImage gradX = gradMatX;
    IplImage gradY = gradMatY;

    cvNamedWindow( "GradX", 1 );
    cvShowImage( "GradX", &gradX );

    cvNamedWindow( "GradY", 1 );
    cvShowImage( "GradY", &gradY );

    cvWaitKey();
#endif
    return 0;
}
