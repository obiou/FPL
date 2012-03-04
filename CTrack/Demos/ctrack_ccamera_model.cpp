// Copyright (C) 2010 Christopher Mei (christopher.m.mei@gmail.com)
//
// This file is part of the CTrack Library. This library is free
// software; you can redistribute it and/or modify it under the
// terms of the GNU Lesser General Public License as published by the
// Free Software Foundation; either version 2, or (at your option)
// any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU Lesser General Public License for more details.

// You should have received a copy of the GNU Lesser General Public
// License along with this library; see the file COPYING.LESSER. If
// not, write to the Free Software Foundation, 59 Temple Place - Suite
// 330, Boston, MA 02111-1307, USA.

// As a special exception, you may use this file as part of a free software
// library without restriction.  Specifically, if other files instantiate
// templates or use macros or inline functions from this file, or you compile
// this file and link it with other files to produce an executable, this
// file does not by itself cause the resulting executable to be covered by
// the GNU Lesser General Public License.  This exception does not however
// invalidate any other reasons why the executable file might be covered by
// the GNU Lesser General Public License.

#include <algorithm>
#include <ctype.h>
#include <iostream>
#include <limits.h>
#include <math.h>
#include <memory>
#include <stdio.h>
#include <sstream>
#include <time.h>

#include <CameraSensor.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc_c.h>

#include <sys/resource.h>
#include <sys/time.h>

#include <CTrack/Homography.h>
#include <CTrack/PlaneTracking.h>
#include <CTrack/PlaneTrackingOpenCV.h>
#include <CTrack/TestingHelpers.h>
#include <CTrack/Transforms.h>
#include <CTrack/TrackingFunctors.h>

#include <CTrack/OpenCVHelpers.h>

#define NO_PYR 0

using namespace std;
using namespace CTrack;
using namespace CCameraSensor;
using namespace ImageWrapper;

typedef pair<int,int> Coords;
typedef vector<Coords> VectorCoords;

CvPoint PointBeg, PointEnd;
CvPoint PointInter;
bool bFirstPoint;
bool bGotBox;

//////////////////////////////////////////////////////////////////////////////
void DrawPyramid( const char* sMsg, CTrack::ImagePyramid& rPyr )
{
    for( unsigned int ii=0; ii < rPyr.GetNumLevels(); ii++ ) {
        stringstream sName;
        sName << sMsg << ii;
        cvNamedWindow( sName.str().c_str(), 1 );
        IplImage* pIm = ImageHolderToOpenCV( rPyr.GetImage( ii ) );
        //cout << "Level " << ii << ", size: " << pIm->width << "x" << pIm->height << endl;
        cvShowImage( sName.str().c_str(), pIm );
        cvReleaseImageHeader( &pIm );
        sName.str( "" );
    }
}

//////////////////////////////////////////////////////////////////////////////
void my_box_cb( int event, int nX, int nY, int /*flags*/, void* /*param*/ )
{        
    if( event == CV_EVENT_LBUTTONUP ) {
        if( bFirstPoint ) {
            PointBeg.x  = nX;
            PointBeg.y  = nY;
            PointInter.x = nX;
            PointInter.y = nY;
            bFirstPoint = false;
        }
        else if( !bGotBox ) {
            PointEnd.x = nX;
            PointEnd.y = nY;
            bGotBox = true;
        }
    }
    else if( event == CV_EVENT_MOUSEMOVE ) {
        if( !bFirstPoint && !bGotBox ) { // Keep point for interactive line
            PointInter.x = nX;
            PointInter.y = nY;
        }
    }
}

//////////////////////////////////////////////////////////////////////////////
bool GetBoxToTrack( CameraSensor& cameraSensor,
                    IplImage** pRefImage,
                    IplImage** pRefPatch,
                    int* nBegX, int* nBegY,
                    bool /*bImagesFromFile = false*/ ) 
{
    cvNamedWindow( "Select Box", 0 );
    bGotBox = false;
    bFirstPoint  = true;
    cvSetMouseCallback( "Select Box", my_box_cb, 0 );

    IplImage* pImage = NULL;
    
    Image aImageCapture = cameraSensor.read();
    if( aImageCapture.empty() ) { return false; }

    IplImage aI = aImageCapture.mImage;
    pImage = &aI;

    while( !bGotBox ) {
        int c = cvWaitKey(10);
        if( (char) c == 27 )
            break;

        aImageCapture = cameraSensor.read();
        if( aImageCapture.empty() ) { return false; }
        aI = aImageCapture.mImage;
        pImage = &aI;

        if( !bFirstPoint && !bGotBox ) { // Draw intermediate box
            cvRectangle( pImage, PointBeg, PointInter, cvScalar( 10 ) );
        }

        cvShowImage( "Select Box", pImage );
    }

    cvDestroyWindow( "Select Box" );

    int nWidth  = pImage->width;
    int nHeight = pImage->height;
    *pRefImage = cvCreateImage( cvSize( nWidth, nHeight ), IPL_DEPTH_8U, 1 );
    if( pImage->depth != 8 ) {
        cvCvtColor( pImage, *pRefImage, CV_RGB2GRAY );
    }
    else {
        cvCopy( pImage, *pRefImage, NULL );
    }

    // Extract template to track
    const int nPatchWidth  = PointEnd.x - PointBeg.x + 1;
    const int nPatchHeight = PointEnd.y - PointBeg.y + 1;
    
    CvRect rect = cvRect( PointBeg.x, PointBeg.y, nPatchWidth, nPatchHeight );
    cvSetImageROI( *pRefImage, rect ); // Sets the region of interest
    
    *pRefPatch = cvCreateImage( cvGetSize( *pRefImage ),
                                (*pRefImage)->depth,
                                (*pRefImage)->nChannels );
    
    cvCopy( *pRefImage, *pRefPatch, NULL );
    cvResetImageROI( *pRefImage );

    *nBegX = PointBeg.x;
    *nBegY = PointBeg.y;

    return true;
}

//////////////////////////////////////////////////////////////////////////////
bool CropBoxToTrack( CameraSensor& cameraSensor,
                     IplImage** pRefImage,
                     IplImage** pRefPatch,
                     const int nBegX, const int nBegY,
                     const int nPatchWidth, const int nPatchHeight
                     ) 
{
    Image aImageCapture = cameraSensor.read();
    if( aImageCapture.empty() ) { return false; }
    IplImage aI = aImageCapture.mImage;
    IplImage* pImage = &aI;

    int nWidth  = pImage->width;
    int nHeight = pImage->height;
    *pRefImage = cvCreateImage( cvSize( nWidth, nHeight ), IPL_DEPTH_8U, 1 );
    cvCvtColor( pImage, *pRefImage, CV_RGB2GRAY );

    // Extract template to track
    CvRect rect = cvRect( nBegX, nBegY, nPatchWidth, nPatchHeight );
    cvSetImageROI( *pRefImage, rect ); // Sets the region of interest
    
    *pRefPatch = cvCreateImage( cvGetSize( *pRefImage ),
                                (*pRefImage)->depth,
                                (*pRefImage)->nChannels );
    
    cvCopy( *pRefImage, *pRefPatch, NULL );
    cvResetImageROI( *pRefImage );
    return true;
}

//////////////////////////////////////////////////////////////////////////////
int main( int argc, char** argv )
{
    bool bLive = true;
    ////////////////////////////////////////////////////////
    // Setup camera
    std::string sCameraType = "UEyeBlocking";
    std::string sListFileName;
    if( argc > 1 ) {
        sCameraType = argv[1];
    }
    CameraSensor cameraSensor( sCameraType );
    if( argc > 2 ) {
        sListFileName = argv[2];
        cameraSensor.set( "ListFileName", sListFileName );
        bLive = false;
    }  
    if( !cameraSensor.open() ) {
        cerr << "Problem opening camera sensor." << endl;
        return -1;
    }
    Image aImageCapture = cameraSensor.read();
    if( aImageCapture.empty() ) {
        cerr << "Got NULL image." << endl;
        return -1;
    }

    std::string sSensorID = aImageCapture.sSensorID;
    std::cout << "SensorID: " << sSensorID << std::endl;
    unsigned int nImageWidth = aImageCapture.mImage.cols;
    unsigned int nImageHeight = aImageCapture.mImage.rows;
    unsigned int nImageWidthStep = (unsigned int)aImageCapture.mImage.step;

    ////////////////////////////////////////////////////////
    IplImage* pRefImage = NULL;
    IplImage* pRefPatch = NULL;

    // Wait for camera to stabilise
    for( int ii=0; ii<10; ii++ ) {
        if( cameraSensor.read().empty() ) {
            return -1;
        }
    }
    
    bool bImagesFromFile = !bLive;

    // Start by acquiring the region to track
#if 1
    int nBegX = 0;
    int nBegY = 0;
    if( !GetBoxToTrack( cameraSensor,
                        &pRefImage,
                        &pRefPatch, 
                        &nBegX, &nBegY,
                        bImagesFromFile ) ) {
        cout << "ERROR: acquiring box to track." << endl;
        return -1;
    }
#else

    const int nBegX = 200;
    const int nBegY = 220;
    const int nPatchWidth = 82;
    const int nPatchHeight = 65;
    PointBeg.x = nBegX; PointBeg.y = nBegY;
    if( !CropBoxToTrack( pCvCapture,
                         &pRefImage,
                         &pRefPatch,
                         nBegX, nBegY, nPatchWidth, nPatchHeight
                         ) ) {
        cout << "ERROR: acquiring or cropping box to track." << endl;
        return -1;
    }
#endif

    std::vector<std::pair<double,double> > vBox;
    vBox.push_back( std::pair<double,double>( 0, 0 ) );
    vBox.push_back( std::pair<double,double>( pRefPatch->width, 0 ) );
    vBox.push_back( std::pair<double,double>( pRefPatch->width, pRefPatch->height) );
    vBox.push_back( std::pair<double,double>( 0, pRefPatch->height) );
        
#if 0
    // Draw reference image
    cvNamedWindow( "Reference image", 1 );
    cvShowImage( "Reference image", pRefImage );
#endif

    // Draw reference patch
    cvNamedWindow( "Template", 1 );
    cvShowImage( "Template", pRefPatch );

    // Draw current patch
    cvNamedWindow( "Current Image", 1 );
    cvNamedWindow( "Current Patch", 1 );

    IplImage* pCurImage = cvCreateImage( cvGetSize( pRefImage ), 
                                         IPL_DEPTH_8U, 1 );
    cvCopy( pRefImage, pCurImage, NULL );

    Homography HRef = CTrack::GenerateTranslation( PointBeg.x, PointBeg.y );

    Homography HFull;

    const int nPatchWidth     = pRefPatch->width;
    const int nPatchHeight    = pRefPatch->height;
    const int nPatchWidthStep = pRefPatch->widthStep;
        
    typedef HomographyIlluminationFunctor Tracker;
    //typedef AffineIlluminationFunctor Tracker;
    typedef AffineFunctor InitTracker;

    bool bRegularise = true;

    bool bDrawCurPyr = false;
    TrackingResults trackingResults;

#if NO_PYR
    TrackingStats trackingStats;
    TrackingSettings<Tracker,NoMask> trackingSettings
        ( PointBeg.x, PointBeg.y,
          pRefPatch->width, pRefPatch->height,
          pRefPatch->widthStep );
#else
    int nNumLevels = 5;
    ImagePyramid refPatchPyramid;
    refPatchPyramid.BuildPyramid( nNumLevels,
                                  (unsigned char*)pRefPatch->imageData,
                                  nPatchWidth, nPatchHeight, nPatchWidthStep ); 
    ImagePyramid* pCurImagePyramid = new ImagePyramid();
    pCurImagePyramid->BuildPyramid( nNumLevels,
                                    (unsigned char*)pCurImage->imageData,
                                    nImageWidth, nImageHeight, nImageWidthStep );

    TrackingStatsPyr trackingStatsPyr( nNumLevels );
    TrackingSettingsPyr<Tracker,NoMask> trackingSettings;
    trackingSettings.InitReset( PointBeg.x, PointBeg.y,
                                nPatchWidth, nPatchHeight, nPatchWidthStep,
                                bRegularise, nNumLevels ); 
    cout << "Using " << trackingSettings.GetNumLevels() <<  " levels." << endl;

    if( bDrawCurPyr ) { DrawPyramid( "Cur Level ", *pCurImagePyramid ); }
    //DrawPyramid( "RefP Level ", refPatchPyramid );

    // For initialisation
    ImagePyramid* pPrevImagePyramid = new ImagePyramid();
    pPrevImagePyramid->BuildPyramid( nNumLevels,
                                     (unsigned char*)pCurImage->imageData,
                                     nImageWidth, nImageHeight, nImageWidthStep );
    TrackingStats trackingStatsInit;
    TrackingSettings<InitTracker,BoolMask> trackingSettingsInit
        ( 0, 0, 
          pCurImagePyramid->GetSmallestImage()->nWidth,
          pCurImagePyramid->GetSmallestImage()->nHeight,
          pCurImagePyramid->GetSmallestImage()->nWidthStep );
    TrackingResults trackingResultsInit;
#endif
    //cvWaitKey();

    IplImage* pCapturedImage = NULL;
        
    IplImage* pCurPatch = cvCreateImageHeader( cvGetSize( pRefPatch ), IPL_DEPTH_8U, 1 );
    cvSetData( pCurPatch, trackingSettings.GetTrackBuffer()->CurPatch(), pRefPatch->widthStep );

    //IplImage* pCurPatch = cvCreateImageHeader( cvGetSize( pRefPatch ), IPL_DEPTH_8U, 1 );
    //cvSetData( pCurPatch, trackingSettings.GetTrackBuffer()->CurPatch(), pRefPatch->widthStep );

    bool bIsClosed = true;
    std::vector<std::pair<double,double> > vWarpedBox;
    CvPoint pPts[4];
    CvPoint* ppPts[1]; // draw the bounding box
    ppPts[0] = &(pPts[0]);
    int nPts = 4;

    bool bInit = true;

    // Draw reference patch
    cvNamedWindow( "WarpedMain", 1 );
    cvNamedWindow( "PrevImage", 1 );

    IplImage* pPrevImage = cvCreateImage( cvSize( nImageWidth, nImageHeight ), IPL_DEPTH_8U, 1 );

    ////////////////////////////////////////////////////////
    // Track
    while( !( aImageCapture = cameraSensor.read() ).empty() ) {
        IplImage aI = aImageCapture.mImage;
        pCapturedImage = &aI;
            
        if( pCapturedImage->depth != 8 ) {
            cvCvtColor( pCapturedImage, pCurImage, CV_RGB2GRAY );
        }
        else {
            cvCopy( pCapturedImage, pCurImage, NULL );
        }

#if !NO_PYR
        pCurImagePyramid->BuildPyramid( nNumLevels,
                                        (unsigned char*)pCurImage->imageData,
                                        nImageWidth, nImageHeight, nImageWidthStep );

        if( bDrawCurPyr ) { DrawPyramid( "Cur Level ", *pCurImagePyramid ); }
#endif

        TrackingResults trackingResultsPrev = trackingResults;

        if( bInit ) {
            std::cout << std::endl << std::endl;
            trackingResultsInit.id();
            TrackPlaneHomography<InitTracker>
            ( trackingSettingsInit,
              *pPrevImagePyramid->GetSmallestImage(), *pCurImagePyramid->GetSmallestImage(),
              &trackingStatsInit, &trackingResultsInit
              );
            std::cout << std::endl << std::endl;
            
            Homography mH = trackingResults.GetHomography();
            Homography mHInit = trackingResultsInit.GetHomography();//.scale( 1./(1 << nNumLevels) );
            mHInit.scale( 1./(1 << (nNumLevels-1) ) );

            Homography mHTot, mHRef, mHRefInv;
            mHRef.id();
            mHRef.Set( 0, 2, nBegX );
            mHRef.Set( 1, 2, nBegY );
            mHRefInv.id();
            mHRefInv.Set( 0, 2, -nBegX );
            mHRefInv.Set( 1, 2, -nBegY );

            mHTot = mHRef;
            mHTot.mult( mHInit );
            mHTot.mult( mHRefInv );

            mH.mult( mHTot );
            trackingResults.SetHomography( mH );
        }

#if 0
        IplImage* pMainImageWarped = cvCreateImage( cvSize( nImageWidth, nImageHeight ),
                                                    pCurImage->depth, pCurImage->nChannels );

        Homography mHInit = trackingResultsInit.GetHomography();
        mHInit.scale( 1./(1 << (nNumLevels-1) ) );
        mHInit.inv();
        //mHInit.scale( 1./(1 << nNumLevels) );
        CTrack::Warp( pPrevImage, mHInit.GetRowMajorPtr(),
                      pMainImageWarped );

        cvShowImage( "PrevImage", pPrevImage );
        cvShowImage( "WarpedMain", pMainImageWarped );
#endif

#if NO_PYR
        TrackPlaneHomography<Tracker>
            ( trackingSettings,
              *pRefPatch, *pCurImage,
              &trackingStats,
              &trackingResults
              ); 
#else

#  if 1
        TrackPlaneHomographyPyr<Tracker>
            ( trackingSettings,
              refPatchPyramid, *pCurImagePyramid,
              &trackingStatsPyr,
              &trackingResults
              );  
#endif

#  if 0
        std::cout << std::endl << "Trying again: " << std::endl;

        TrackPlaneHomographyPyr<Tracker>
            ( trackingSettings,
              refPatchPyramid, *pCurImagePyramid,
              &trackingStatsPyr,
              &trackingResultsPrev
              );  

        std::cout << std::endl;
#  endif

#endif

        if( trackingStatsPyr.GetTrackingStats(0)->RMS() >= 60 ) {
            return -1;
        }

        HFull.id();
        HFull.Set( 0, 2, nBegX );
        HFull.Set( 1, 2, nBegY );
        HFull.mult( trackingResults.GetHomography() );
        HFull.warp_polymult( vBox, vWarpedBox );

        convert( vWarpedBox, pPts );
        cvPolyLine( pCurImage, 
                    ppPts, &nPts, 1, 
                    bIsClosed, 
                    cvScalar( 10 ), 4 );
        cvShowImage( "Current Image", pCurImage );
        //cvSaveImage( "test.jpg", pCurImage );
                
        cvShowImage( "Current Patch", pCurPatch );
    
        int c = cvWaitKey( bImagesFromFile ? 0 : 10 );

        //int c = cvWaitKey( 0  );

        if( (char) c == 27 ) {
            break;
        } 

        // Swap
        std::swap( pPrevImagePyramid, pCurImagePyramid );
        cvCopy( pCurImage, pPrevImage );
    }

    cvDestroyWindow( "Template" );

    return 0;
}


