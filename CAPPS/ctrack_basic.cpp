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
#include <iostream>
#include <limits.h>
#include <math.h>
#include <memory>
#include <sstream>
#include <stdio.h>
#include <time.h>

#include <cv.h>
#include <highgui.h>

#include <sys/resource.h>
#include <sys/time.h>

#include <CameraSensor.h>
#include <COpenCV.h>
#include <CTrack/Homography.h>
#include <CTrack/OpenCVHelpers.h>
#include <CTrack/PlaneTracking.h>
#include <CTrack/PlaneTrackingOpenCV.h>
#include <CTrack/TestingHelpers.h>
#include <CTrack/TrackingFunctors.h>
#include <CTrack/Transforms.h>
#if HAS_CVARS
#  include <CVars/CVar.h>
#endif

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
bool GetBoxToTrack( CCameraSensor::CameraSensor& cameraSensor,
                    IplImage** pRefImage,
                    IplImage** pRefPatch,
                    int* nBegX, int* nBegY )
{
    cvNamedWindow( "Select Box", 0 );
    bGotBox = false;
    bFirstPoint  = true;
    cvSetMouseCallback( "Select Box", my_box_cb, 0 );

    CCameraImage::Image* pImageCapture;
    IplImage *pImage;
    while( ( pImageCapture = cameraSensor.read() ) != NULL && !bGotBox ) {
        IplImage aI = CCameraImage::ToIplImage( pImageCapture );
        pImage = &aI;
      
        int c = cvWaitKey( 10 );
        if( (char) c == 27 )
            break;

        if( !bFirstPoint && !bGotBox ) { // Draw intermediate box
            cvRectangle( pImage, PointBeg, PointInter, cvScalar( 10 ) );
        }

        cvShowImage( "Select Box", pImage );
    }

    if( !bGotBox ) { return false; }

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

////////////////////////////////////////////////////////////////////////////////
const std::string USAGE =
    "Usage: ctrack_basic\n"
    "    -c camera_type        camera sensor type: OpenCV, UEyeBlocking\n";

//////////////////////////////////////////////////////////////////////////////
bool Tracking( CCameraSensor::CameraSensor& cameraSensor );

//////////////////////////////////////////////////////////////////////////////
int main( int argc, char** argv )
{
    string sCameraType =  "UEyeBlocking";

    for( int ii = 1; ii < argc; ii++ ) {
        const char* s = argv[ii];
        if( strcmp( s, "-c" ) == 0 ) {
            sCameraType = argv[++ii];
        }
        else {
            std::cerr << USAGE;
            cerr << "ERROR: unknown option: " << s << endl;
            return -1;
        }
    }
    
    ////////////////////////////////////////////////////////////
    const string sWarnMessage = 
        "WARNING: this code is a basic LKT (or ESM) implementation used mainly for low-level tracking,"
        " it has a restricted convergence rate and cannot adapt to changes in shape or occlusion...";

    cout << sWarnMessage << endl;

    ////////////////////////////////////////////////////////////
    CCameraSensor::CameraSensor cameraSensor( sCameraType );
    //if( sInputFileName != "" ) {
    //    cameraSensor.set( "ListFileName", sInputFileName );
    //}
    bool bSuccess = cameraSensor.open();
    if( !bSuccess ) {
        std::cerr << "ERROR: opening camera sensor of type: " << sCameraType << "." << std::endl;
        return -1;
    }

    CCameraImage::Image* pImageCapture = cameraSensor.read();
    if( pImageCapture == NULL ) {
        cerr << "ERROR: problem capturing image from sensor, quitting..." << endl;
        return -1;
    }

    while( !Tracking( cameraSensor ) ) {
    }
}

//////////////////////////////////////////////////////////////////////////////
// Returns false if tracking failed, true if user wishes to quit
bool Tracking( CCameraSensor::CameraSensor& cameraSensor ) {
    IplImage* pRefImage = NULL;
    IplImage* pRefPatch = NULL;

    CCameraImage::Image* pImageCapture;

    // Start by acquiring the region to track
    int nBegX = 0;
    int nBegY = 0;
    if( !GetBoxToTrack( cameraSensor,
                        &pRefImage, &pRefPatch, 
                        &nBegX, &nBegY ) ) {
        cout << "ERROR: acquiring box to track." << endl;
        cvReleaseImage( &pRefImage );
        cvReleaseImage( &pRefPatch );
        return false;
    }

    std::vector<std::pair<double,double> > vBox;
    vBox.push_back( std::pair<double,double>( 0, 0 ) );
    vBox.push_back( std::pair<double,double>( pRefPatch->width, 0 ) );
    vBox.push_back( std::pair<double,double>( pRefPatch->width, pRefPatch->height) );
    vBox.push_back( std::pair<double,double>( 0, pRefPatch->height) );
        
    // Draw reference patch
    COPENCV::Figure fig_template( "Template" );
    fig_template.imshow( pRefPatch );
    fig_template.draw();

    // Draw current patch
    COPENCV::Figure fig_cur_p( "Current Image" );
    COPENCV::Figure fig_cur_i( "Current Patch" );

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
        
    //typedef HomographyIlluminationFunctor Tracker;
    typedef AffineIlluminationFunctor Tracker;
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
                                  reinterpret_cast<unsigned char*>( pRefPatch->imageData ),
                                  nPatchWidth, nPatchHeight, nPatchWidthStep ); 
    ImagePyramid curImagePyramid;
    curImagePyramid.BuildPyramid( nNumLevels,
                                  reinterpret_cast<unsigned char*>( pCurImage->imageData ),
                                  nImageWidth, nImageHeight, nImageWidthStep );
    TrackingStatsPyr trackingStatsPyr( nNumLevels );
    TrackingSettingsPyr<Tracker,NoMask> trackingSettings;
    trackingSettings.InitReset( PointBeg.x, PointBeg.y,
                                nPatchWidth, nPatchHeight, nPatchWidthStep,
                                bRegularise, nNumLevels ); 
    //cout << "Using " << trackingSettings.GetNumLevels() <<  " levels." << endl;

    if( bDrawCurPyr ) { DrawPyramid( "Cur Level ", curImagePyramid ); }
    //DrawPyramid( "RefP Level ", refPatchPyramid );

#endif
        
    IplImage* pCurPatch = cvCreateImageHeader( cvGetSize( pRefPatch ), IPL_DEPTH_8U, 1 );
    cvSetData( pCurPatch, trackingSettings.GetTrackBuffer()->CurPatch(), pRefPatch->widthStep );

    std::vector<std::pair<double,double> > vWarpedBox;

    //bool bInit = true;

#if 0
    Homography mHInit;
    mHInit.Set(0,2,PointBeg.x);
    mHInit.Set(1,2,PointBeg.y);
    trackingResults.SetHomography(mHInit);
    //cout << trackingResults.GetHomography() << endl;
#endif

    // Track
    while( ( pImageCapture = cameraSensor.read() ) != NULL ){
        IplImage aI = CCameraImage::ToIplImage( pImageCapture );
        IplImage *pImage = &aI;
            
        if( pImage->depth != 8 ) {
            cvCvtColor( pImage, pCurImage, CV_RGB2GRAY );
        }
        else {
            cvCopy( pImage, pCurImage, NULL );
        }

#if !NO_PYR
        curImagePyramid.BuildPyramid( nNumLevels,
                                      reinterpret_cast<unsigned char*>( pCurImage->imageData ),
                                      nImageWidth, nImageHeight, nImageWidthStep );

        if( bDrawCurPyr ) { DrawPyramid( "Cur Level ", curImagePyramid ); }
#endif

#if 0
        if( bInit ) {
            // Do a simple translation search at the highest level
            double dtx = 0, dty = 0; double dBestRMS = std::numeric_limits<double>::max();
            IplImage* pImCur = ImageHolderToOpenCV( curImagePyramid.GetSmallestImage() );
            IplImage* pPaRef = ImageHolderToOpenCV( refPatchPyramid.GetSmallestImage() );
            
            int nLevel = trackingSettings.GetNumLevels()-1; 
            double dScale = 1 << nLevel;

            trackingResults.SetHomography( trackingResults.GetHomography().scale( dScale ) );

            COPENCV::Figure fig_( "Big ref" );
            fig_.imshow( ImageHolderToOpenCV(refPatchPyramid.GetImage()) );
            fig_.draw();
#if 1
            COPENCV::Figure fig_cur( "Fig cur" );
            fig_cur.imshow( pImCur );
            fig_cur.draw();

            COPENCV::Figure fig_rpat( "Fig refpat" );
            fig_rpat.imshow( pPaRef );
            fig_rpat.draw();
#endif
            IplImage* pWarpPa = cvCreateImage( cvGetSize( pPaRef ), IPL_DEPTH_8U, 1 );
            Warp( pImCur, trackingResults.GetHomography().GetRowMajorPtr(), pWarpPa );

            COPENCV::Figure fig_cpat( "Fig curpat" );
            fig_cpat.imshow( pWarpPa );
            fig_cpat.draw();
            fig_cpat.wait();

            trackingResults.SetHomography( trackingResults.GetHomography().scale( 1./dScale ) );
            
            cvReleaseImage( &pWarpPa );
            //CTrack::Warp()                          
        }
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

        if( trackingStatsPyr.GetTrackingStats(0)->RMS() >= 50 ) {
            cvReleaseImage( &pRefImage ); 
            cvReleaseImage( &pRefPatch );
            cvReleaseImage( &pCurImage );
            return false;
        }
#endif     

        HFull.id();
        HFull.Set( 0, 2, nBegX );
        HFull.Set( 1, 2, nBegY );
        HFull.mult( trackingResults.GetHomography() );
        HFull.warp_polymult( vBox, vWarpedBox );
        vWarpedBox.push_back( vWarpedBox[0] );
        fig_cur_i.imshow( pCurImage );
        fig_cur_i.plot( vWarpedBox, "r-" );
        fig_cur_i.draw();

        fig_cur_p.imshow( pCurPatch );
        fig_cur_p.draw();

        int c = fig_cur_p.wait( 10 );

        if( (char) c == 27 ) {
            break;
        } 
    }

    cvReleaseImage( &pRefImage ); 
    cvReleaseImage( &pRefPatch );
    cvReleaseImage( &pCurImage );

    return true;
}


