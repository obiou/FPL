#include <algorithm>
#include <ctype.h>
#include <iostream>
#include <limits.h>
#include <math.h>
#include <memory>
#include <stdio.h>
#include <sstream>
#include <time.h>

#include <cv.h>
#include <highgui.h>

#include <CTrack/ImagePyramid.h>
#include <CTrack/Misc.h>
#include <CTrack/OpenCVHelpers.h>

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

    cvNamedWindow( "Reference image", 1 );
    cvShowImage( "Reference image", pRefImage );

    const int nMinWidth  = 10;
    const int nMinHeight = 10;

    // Draw pyramid of the reference image
    bool bForcePowersOf2 = true;
    unsigned int nNumLevels = 
        CTrack::MinImageSizeToNumLevels( nImageWidth, nImageHeight,
                                         nMinWidth,   nMinHeight,
                                         bForcePowersOf2 );
    cout << "nNumLevels: " << nNumLevels << endl;
    CTrack::ImagePyramid imPyr;
    imPyr.BuildPyramid( nNumLevels, 
                        (unsigned char*)pRefImage->imageData,
                        nImageWidth, nImageHeight, nImageWidthStep
                        );

    stringstream sName;
    for( unsigned int ii=0; ii<imPyr.GetNumLevels(); ii++ ) {
        sName << "Level " << ii;
        cvNamedWindow( sName.str().c_str(), 1 );
        IplImage* pIm = ImageHolderToOpenCV( imPyr.GetImage( ii ) );
        cout << "Level " << ii << ", size: " << pIm->width << "x" << pIm->height << endl;
        cvShowImage( sName.str().c_str(), pIm );
        cvReleaseImageHeader( &pIm );
        sName.str( "" );
    }

    // Draw pyramid for the reference patch
    // Extract patch
    const int nPatchBegX   = 100;
    const int nPatchBegY   = 100;
    const int nPatchWidth  = 100;
    const int nPatchHeight = 100;
    IplImage* pRefPatchImage = CTrack::ExtractPatch( pRefImage,
                                             nPatchBegX, nPatchBegY, 
                                             nPatchWidth, nPatchHeight );
    const int nPatchWidthStep = pRefPatchImage->widthStep;

    bForcePowersOf2 = false;
    nNumLevels = 
        CTrack::MinImageSizeToNumLevels( nPatchWidth, nPatchHeight,
                                         nMinWidth,   nMinHeight,
                                         bForcePowersOf2 );
    cout << "nNumLevels: " << nNumLevels << endl;
    imPyr.BuildPyramid( nNumLevels, 
                        (unsigned char*)pRefPatchImage->imageData,
                        nPatchWidth, nPatchHeight, nPatchWidthStep
                        );

    for( unsigned int ii=0; ii<imPyr.GetNumLevels(); ii++ ) {
        sName << "Level " << ii;
        cvNamedWindow( sName.str().c_str(), 1 );
        IplImage* pIm = ImageHolderToOpenCV( imPyr.GetImage( ii ) );
        cout << "Level " << ii << ", size: " << pIm->width << "x" << pIm->height << endl;
        cvShowImage( sName.str().c_str(), pIm );
        cvReleaseImageHeader( &pIm );
        sName.str( "" );
    }

    IplImage* pIm = ImageHolderToOpenCV( imPyr.GetImage( imPyr.GetNumLevels()-1 ) );
    cvResize( pIm, pRefPatchImage );
    cvNamedWindow( "Resized smallest image", 1 );
    cvShowImage( "Resized smallest image", pRefPatchImage );
    //cvSaveImage( "test_smallest.jpg", pRefPatchImage );
    cvReleaseImageHeader( &pIm );

    cvWaitKey();

    ////////////////////////////////////////////////////////////////////////////
    // Release buffers
    cvReleaseImage( &pRefImage );
    cvReleaseImage( &pRefPatchImage );

    return 0;
}
