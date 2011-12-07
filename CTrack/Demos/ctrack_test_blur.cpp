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
#include <CTrack/TransformsHighLevel.h>

using namespace std;

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

    // Load reference image
    IplImage* pRefImage = CTrack::LoadToGray( sImageName );
    if( pRefImage == NULL ) {
        return -1;
    }

    const int nImageWidth  = pRefImage->width;
    const int nImageHeight = pRefImage->height;  
    const int nImageWidthStep = pRefImage->widthStep;

    cvNamedWindow( "Reference image", 1 );
    cvShowImage( "Reference image", pRefImage );

    IplImage* pBlurredImage = cvCreateImage( cvSize( pRefImage->width, pRefImage->height ),
                                             pRefImage->depth,
                                             pRefImage->nChannels );


    CTrack::Homography Hl;
    CTrack::Homography Hr;
    //CTrack::Homography M; // blur kernel (in Lie space)
    double dLie[] = {0,0,3.14/30};
    std::vector<double> vLie( dLie, dLie+3 );
    CTrack::Homography M( vLie ); // blur kernel (in Lie space)
    //M.zero();
    //M.Set( 0, 2, 20 );
    //M.Set( 1, 2, 15 );
    //M.Set( 1, 2, 15 );

    CTrack::NoMask noMask;
    double dStep = 1;
    bool bJacobian = false;

    // Warp and blur
    CTrack::WarpBlur<>( (const unsigned char*)pRefImage->imageData, ///< Input: image input
                        nImageWidth,    ///< Input: image width
                        nImageHeight,   ///< Input: image height
                        nImageWidthStep,
                        &Hl, &M, &Hr,
                        (unsigned char*)pBlurredImage->imageData,     ///< Input/Output: memory allocated by the user that will be used to store the result of the warping
                        nImageWidth,    ///< Input: warped patch width
                        nImageHeight,    ///< Input: warped patch height
                        nImageWidthStep,
                        noMask, dStep, bJacobian
                      );

    cvNamedWindow( "Blurred image", 1 );
    cvShowImage( "Blurred image", pBlurredImage );

    cvWaitKey();

    ////////////////////////////////////////////////////////////////////////////
    // Release buffers
    cvReleaseImage( &pRefImage );
    cvReleaseImage( &pBlurredImage );

    return 0;
}
