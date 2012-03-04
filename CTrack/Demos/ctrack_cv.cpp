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

#include <cv.h>
#include <highgui.h>

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
bool GetBoxToTrack( CvCapture* pCvCapture,
                    IplImage** pRefImage,
                    IplImage** pRefPatch,
                    int* nBegX, int* nBegY,
                    bool bImagesFromFile = false ) 
{
    cvNamedWindow( "Select Box", 0 );
    bGotBox = false;
    bFirstPoint  = true;
    cvSetMouseCallback( "Select Box", my_box_cb, 0 );

    IplImage* pImage = NULL;

    if( !cvGrabFrame( pCvCapture ) ) {
        return false;
    }

    while( !bGotBox ) {
        int c = cvWaitKey(10);
        if( (char) c == 27 )
            break;

        pImage = cvRetrieveFrame( pCvCapture );

        if( !bFirstPoint && !bGotBox ) { // Draw intermediate box
            cvRectangle( pImage, PointBeg, PointInter, cvScalar( 10 ) );
        }

        cvShowImage( "Select Box", pImage );

        if( !bImagesFromFile ) {
            if( !cvGrabFrame( pCvCapture ) ) {
                return false;
            }
        }
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
bool CropBoxToTrack( CvCapture* pCvCapture,
                     IplImage** pRefImage,
                     IplImage** pRefPatch,
                     const int nBegX, const int nBegY,
                     const int nPatchWidth, const int nPatchHeight
                     ) 
{
    IplImage* pImage = NULL;

    if( !cvGrabFrame( pCvCapture ) ) {
        return false;
    }

    pImage = cvRetrieveFrame( pCvCapture );

    if( pImage == NULL ) {
        return false;
    }

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
    CvCapture* pCvCapture = NULL;
    bool bImagesFromFile = false;

    if( argc == 1 || (argc == 2 && strlen(argv[1]) == 1 && isdigit(argv[1][0]))) {
        pCvCapture = cvCaptureFromCAM( argc == 2 ? argv[1][0] - '0' : 0 );
        bImagesFromFile = false;
    }
    else if( argc == 2 ) {
        pCvCapture = cvCaptureFromFile( argv[1] );
        bImagesFromFile = true;
    }

    if( pCvCapture ) {
        cvSetCaptureProperty( pCvCapture, CV_CAP_PROP_FPS, 30 );
        cvSetCaptureProperty( pCvCapture, CV_CAP_PROP_FRAME_WIDTH, 640 );
        cvSetCaptureProperty( pCvCapture, CV_CAP_PROP_FRAME_HEIGHT, 480 );

        IplImage* pRefImage = NULL;
        IplImage* pRefPatch = NULL;

        // Start by acquiring the region to track
#if 1
        int nBegX = 0;
        int nBegY = 0;
        if( !GetBoxToTrack( pCvCapture,
                            &pRefImage,
                            &pRefPatch, 
                            &nBegX, &nBegY,
                            bImagesFromFile ) ) {
            cout << "ERROR: acquiring box to track." << endl;
            return -1;
        }
#else
        // Wait for camera to stabilise
        for( int ii=0; ii<100; ii++ ) {
            if( !cvGrabFrame( pCvCapture ) ) {
                return false;
            }
        }

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

        const int nImageWidth     = pRefImage->width;
        const int nImageHeight    = pRefImage->height;
        const int nImageWidthStep = pRefImage->widthStep;
        const int nPatchWidth     = pRefPatch->width;
        const int nPatchHeight    = pRefPatch->height;
        const int nPatchWidthStep = pRefPatch->widthStep;
        
        typedef HomographyIlluminationFunctor Tracker;
        //typedef AffineIlluminationFunctor Tracker;
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
        int nNumLevels = 4;
        ImagePyramid refPatchPyramid;
        refPatchPyramid.BuildPyramid( nNumLevels,
                                      (unsigned char*)pRefPatch->imageData,
                                      nPatchWidth, nPatchHeight, nPatchWidthStep ); 
        ImagePyramid curImagePyramid;
        curImagePyramid.BuildPyramid( nNumLevels,
                                      (unsigned char*)pCurImage->imageData,
                                      nImageWidth, nImageHeight, nImageWidthStep );
        TrackingStatsPyr trackingStatsPyr( nNumLevels );
        TrackingSettingsPyr<Tracker,NoMask> trackingSettings;
        trackingSettings.InitReset( PointBeg.x, PointBeg.y,
                                    nPatchWidth, nPatchHeight, nPatchWidthStep,
                                    bRegularise, nNumLevels ); 
        cout << "Using " << trackingSettings.GetNumLevels() <<  " levels." << endl;

        if( bDrawCurPyr ) { DrawPyramid( "Cur Level ", curImagePyramid ); }
        //DrawPyramid( "RefP Level ", refPatchPyramid );

#endif
        //cvWaitKey();

        IplImage* pCapturedImage = NULL;
        
        IplImage* pCurPatch = cvCreateImageHeader( cvGetSize( pRefPatch ), IPL_DEPTH_8U, 1 );
        cvSetData( pCurPatch, trackingSettings.GetTrackBuffer()->CurPatch(), pRefPatch->widthStep );

        bool bIsClosed = true;
        std::vector<std::pair<double,double> > vWarpedBox;
        CvPoint pPts[4];
        CvPoint* ppPts[1]; // draw the bounding box
        ppPts[0] = &(pPts[0]);
        int nPts = 4;

        // Track
        while( cvGrabFrame( pCvCapture ) ) {
            pCapturedImage = cvRetrieveFrame( pCvCapture );
            
            if( pCapturedImage != NULL ) {
                if( pCapturedImage->depth != 8 ) {
                    cvCvtColor( pCapturedImage, pCurImage, CV_RGB2GRAY );
                }
                else {
                    cvCopy( pCapturedImage, pCurImage, NULL );
                }

#if !NO_PYR
                curImagePyramid.BuildPyramid( nNumLevels,
                                              (unsigned char*)pCurImage->imageData,
                                              nImageWidth, nImageHeight, nImageWidthStep );

                if( bDrawCurPyr ) { DrawPyramid( "Cur Level ", curImagePyramid ); }
#endif

#if NO_PYR
                TrackPlaneHomography<Tracker>
                    ( trackingSettings,
                      *pRefPatch, *pCurImage,
                      &trackingStats,
                      &trackingResults
                      ); 
#else
                TrackPlaneHomographyPyr<Tracker>
                    ( trackingSettings,
                      refPatchPyramid, curImagePyramid,
                      &trackingStatsPyr,
                      &trackingResults
                      ); 
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
            }
            else {
                cout << "ERROR capturing image." << endl;
                continue;
            }

            int c = cvWaitKey( bImagesFromFile ? 0 : 10 );

            if( (char) c == 27 ) {
                break;
            } 
        }

        cvDestroyWindow( "Template" );

        cvReleaseCapture( &pCvCapture );
    }
    else {
        printf( "Error capturing.\n" );
    }

    return 0;
}


