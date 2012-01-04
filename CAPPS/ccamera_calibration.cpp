#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/SVD>
#include <eigen3/unsupported/Eigen/MatrixFunctions>

#include <CameraModel.h>
#include <CameraSensor.h>
#include <ceigen.h>
#include <COpenCV.h>
#if HAS_CVARS
#  include <CVars/CVar.h>
#endif
#include <CTrack/Homography.h>
#include <Misc.h>

#include "cv.h"
#include "highgui.h"

#include <stdio.h>
#include <string.h>
#include <time.h>

using namespace CCameraModel;
using namespace std;

////////////////////////////////////////////////////////////////////////////////
const std::string USAGE =
    "Camera calibration tool.\n"
    "Usage: ccamera_calibration\n"
    "     -w board_width           the number of width inner corners\n"
    "     -h board_height          the number of height inner corners\n"
    "     [-d delay]               for video, delay in ms between subsequent attempts to find the calibration grid\n"
    "     [-o params_filename]     output filename for the intrinsic camera parameters\n"
    "     [-v]                     flip the captured images around the horizontal axis\n"
    "     [-t camera_model]        camera model type: PinholeRadTan, Arctan\n"
    "     [-c camera_type]         camera sensor type: OpenCV, UEyeBlocking\n"
    "     [text_views]             text file with a list of the images of the calibration grid, one per line\n"
    "                              if you do not provide a text file, a live view from the camera is used\n"
    "\n"
    "Example:\n"
    "  ccamera_calibration -w 8 -h 5 list_views\n"
    "\n";

////////////////////////////////////////////////////////////////////////////////
#define MAX_FILENAME 1024

////////////////////////////////////////////////////////////////////////////////
int main( int argc, char** argv )
{
    string sCameraParamsFile;
    string sInputFileName;
    char sImageName[MAX_FILENAME], sImageNameExtra[MAX_FILENAME];
    bool bFlipVertically = false;
    int nDelay = 0;
    clock_t prev_timestamp = 0;
    CvFont font = cvFont( 1, 1 );
    CvSize board_size = {6,8};
    string sCameraModelType = "PinholeRadTan"; 
    string sCameraType =  "UEyeBlocking";
    string sLiveHelp =
        "Input keys:\n"
            "  <ESC>, 'q' - quit the program\n"
            "  'a' - add frame to calibration list\n"
            "  'u' - switch undistortion on/off\n";

    for( int ii = 1; ii < argc; ii++ ) {
        const char* s = argv[ii];
        if( strcmp( s, "-w" ) == 0 ) {
            if( sscanf( argv[++ii], "%u", &board_size.width ) != 1 || board_size.width <= 0 ) {
                std::cerr << USAGE;
                return fprintf( stderr, "Invalid board width\n" ), -1;
            }
        }
        else if( strcmp( s, "-h" ) == 0 ) {
            if( sscanf( argv[++ii], "%u", &board_size.height ) != 1 || board_size.height <= 0 ) {
                std::cerr << USAGE;
                return fprintf( stderr, "Invalid board height\n" ), -1;
            }
        }
        else if( strcmp( s, "-d" ) == 0 ) {
            if( sscanf( argv[++ii], "%u", &nDelay ) != 1 || nDelay < 0 ) {
                std::cerr << USAGE;
                return printf("Invalid delay\n" ), -1;
            }
        }
        else if( strcmp( s, "-v" ) == 0 ) {
            bFlipVertically = true;
        }
        else if( strcmp( s, "-o" ) == 0 ) {
            sCameraParamsFile = argv[++ii];
        }
        else if( strcmp( s, "-t" ) == 0 ) {
            sCameraModelType = argv[++ii];
        }
        else if( strcmp( s, "-c" ) == 0 ) {
            sCameraType = argv[++ii];
        }
        else if( s[0] != '-' ) {
            sInputFileName = s;
        } 
        else {
            std::cerr << USAGE;
            cerr << "ERROR: unknown option: " << s << endl;
            return -1;
        }
    }

    bool bLive = true;
    if( sInputFileName != "" ) {
        sCameraType = "FileReaderFromList";
        bLive = false;
    }

    CCameraSensor::CameraSensor cameraSensor( sCameraType );
    if( sInputFileName != "" ) {
        cameraSensor.set( "ListFileName", sInputFileName );
    }

    bool bSuccess = cameraSensor.open();

    if( !bSuccess ) {
        std::cerr << "ERROR: opening camera sensor of type: " << sCameraType << "." << std::endl;
        return -1;
    }

    if( bLive ) { cout << sLiveHelp << endl; }
    
    COPENCV::Figure fig( "Image" );

    std::cout << "Board size: " << board_size.width << "x" << board_size.height << std::endl;

    // Create a vector of image points
    std::vector<Eigen::MatrixXd> vImagePoints;

    ImageWrapper::Image aImageCapture = cameraSensor.read();

    if( aImageCapture.empty() ) {
        cerr << "ERROR: problem capturing image from sensor, quitting..." << endl;
        return -1;
    }
    std::string sSensorID = aImageCapture.sSensorID;
    unsigned int nImageWidth = aImageCapture.mImage.cols;
    unsigned int nImageHeight = aImageCapture.mImage.rows;
    unsigned int nImageWidthStep = aImageCapture.mImage.step;

    IplImage* pImUndist = cvCreateImage( cvSize( nImageWidth, nImageHeight), IPL_DEPTH_8U, 1 );

    if( sCameraParamsFile == "" ) {
        stringstream oss;
        oss << "calibrated";
        if( sSensorID != "" ) {
            oss << "_" << sSensorID;
        }
        oss << ".txt";
        sCameraParamsFile = oss.str();
    }
    GridCalibrator gridCalibrator( sCameraModelType, 
                                   aImageCapture.width(), aImageCapture.height(),
                                   board_size.height, board_size.width );

    int nNumCapturedImages = 0;
    bool bAdd = false;
    prev_timestamp = clock();
    double dRMS = 0;
    while( !( aImageCapture = cameraSensor.read() ).empty() ) {
        //std::cout << "LastReadFile: " << cameraSensor.get( "LastReadFile" ) << std::endl;
        IplImage aI = aImageCapture.mImage;
        IplImage *pImage = &aI;
        bool bFound = false;
        CvPoint text_origin;
        CvSize text_size = {0,0};
        int base_line = 0;
        char sText[MAX_FILENAME];
        int key;
        
        Eigen::MatrixXd mGridPoints;

        if( bFlipVertically ) {
            cvFlip( pImage, pImage, 0 );
        }
         
        if( !bLive || clock() - prev_timestamp > nDelay*1e-3*CLOCKS_PER_SEC ) {
            mGridPoints = CEIGEN::FindChessboardCorners( pImage, board_size, true );
            bFound = mGridPoints.size() > 0;
            if( bFound && (!bLive || ( bLive && bAdd ) ) ) {
                bAdd = false;

                cout << "Adding image: " << nNumCapturedImages << endl;
                gridCalibrator.add_view( mGridPoints );
                if( gridCalibrator.get_num_views() > 1 ) {
                    gridCalibrator.iterate( dRMS );
                    cout << "RMS local: " << dRMS << endl;
                    gridCalibrator.print();
                }
                vImagePoints.push_back( mGridPoints );
                if( bLive ) {
                    snprintf( sImageName, MAX_FILENAME, "view_%03d.png", nNumCapturedImages );
                    snprintf( sImageNameExtra, MAX_FILENAME, "view_%03d.txt", nNumCapturedImages );
                    ImageWrapper::imwrite( sImageName, sImageNameExtra, aImageCapture );
                }
                nNumCapturedImages++;
            }
            if( bFound ) {
                // Comes after saving to avoid having grid drawn on the image
                Eigen::MatrixXf mGridPointsF = mGridPoints.cast<float>();
                cvDrawChessboardCorners( pImage, board_size, 
                                         reinterpret_cast<CvPoint2D32f*>( mGridPointsF.data() ),
                                         mGridPointsF.cols(), bFound );
            }
            prev_timestamp = clock();
        }
        else {
            if( bLive && gridCalibrator.get_num_views() > 1 ) {
                gridCalibrator.iterate( dRMS );
                cout << "RMS local: " << dRMS << endl;
            }
        }

        cvGetTextSize( "00000 100", &font, &text_size, &base_line );
        text_origin.x = pImage->width - text_size.width - 10;
        text_origin.y = pImage->height - base_line - 10;

        snprintf( sText, MAX_FILENAME, "%2.3f %d", dRMS, nNumCapturedImages );

        cvPutText( pImage, sText, text_origin, &font, CV_RGB(255,0,0) );

        fig.imshow( pImage );
        fig.draw();
        key = fig.wait( bLive ? 50 : 100);
        
        if( (char)key == 27 )
            break;
        
        if( bLive && (char)key == 'a' ) {
            bAdd = true;
        }
  
        if( (char)key == 'u' ) {    
            //cvNamedWindow( "Undistorted Image", 1 );
            COPENCV::Figure fig_un( "Undistorted Image" );
            CameraModel* pCam = gridCalibrator.get_camera_copy();
            pCam->undistort_image( nImageWidth, nImageHeight, nImageWidthStep,
                                   reinterpret_cast<const unsigned char*>( pImage->imageData ),
                                   reinterpret_cast<unsigned char*>( pImUndist->imageData ), true );
            fig_un.draw();
            fig_un.wait( 0 );
            delete pCam;
        }      
    }

    gridCalibrator.minimise();
    
    cout << "Saving to: " << sCameraParamsFile << endl;
    gridCalibrator.save( sCameraParamsFile );

    cvReleaseImage( &pImUndist );
    return 0;
}
