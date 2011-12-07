// CCameraSensor - framework for camera drivers
// Copyright (C) 2011 C. Mei
// 
// CCameraSensor is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// CCameraSensor is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
#include <CCameraSensorIncludes.h>
#include <CCameraSensor/OpenCVCamera.h>
#include <CCameraSensor/Image.h>

#include <iostream>

////////////////////////////////////////////////////////////////////////////////    
CCameraSensor::OpenCVCamera::OpenCVCamera() 
    : m_bOpened( false ), m_pCvCapture( NULL ),
      m_pCapturedImage( NULL ), m_pReadImageHolder( NULL ) {
    srand ( time(NULL) );
    std::ostringstream os;
    os << "OpenCV_" ;
    os << rand() % 10000000 + 1;
    m_sSensorID = os.str();
    std::cout << m_sSensorID << std::endl;
}

////////////////////////////////////////////////////////////////////////////////    
CCameraSensor::OpenCVCamera::~OpenCVCamera() {
    close();
}

////////////////////////////////////////////////////////////////////////////////    
bool CCameraSensor::OpenCVCamera::open() {
    if( m_bOpened ) {
        std::cerr << "ERROR: Open already called" << std::endl; 
        return false;
    }
    m_bOpened = true;

    std::cout << "Opening OpenCVCamera." << std::endl; 

    m_pCvCapture = cvCaptureFromCAM( 0 );

    if( m_pCvCapture == NULL ) {
        std::cerr << "ERROR: in OpenCVCamera::open(), OpenCVCamera, calling capture returned NULL" << std::endl;
        return false;
    }
    m_pReadImageHolder = new CCameraImage::Image();
    return true;
}

////////////////////////////////////////////////////////////////////////////////    
bool CCameraSensor::OpenCVCamera::close() {
    std::cout << "Closing OpenCVCamera." << std::endl; 
#if 0
    if( m_pCapturedImage != NULL ) {
        cvReleaseImage( &m_pCapturedImage );
    }
    m_pCapturedImage = NULL;
#endif
    if( m_pReadImageHolder != NULL ) {
        delete( m_pReadImageHolder );
    }
    m_pReadImageHolder = NULL; 
    if( m_pCvCapture != NULL ) {
        cvReleaseCapture( &m_pCvCapture );
    }
    m_pCvCapture = NULL;
    m_pCapturedImage = NULL;
    return true;
} 

////////////////////////////////////////////////////////////////////////////////    
bool CCameraSensor::OpenCVCamera::read( std::vector<CCameraImage::Image*>& vImages ) {
    if( !m_bOpened ) { 
        std::cerr << "ERROR: in OpenCVCamera::read(), has open() been called?" << std::endl;
        return false;
    }
    if( m_pCvCapture == NULL ) {
        std::cerr << "ERROR: in OpenCVCamera::read(), m_pCvCapture == NULL." << std::endl;
        return false;
    }
    if( !cvGrabFrame( m_pCvCapture ) ) {
        std::cerr << "ERROR: in OpenCVCamera::read(), error capturing frame." << std::endl;
        return false;
    }
    m_pCapturedImage = cvRetrieveFrame( m_pCvCapture );
    if( m_pCapturedImage == NULL ) {
        std::cerr << "ERROR: in OpenCVCamera::read(), m_pCapturedImage == NULL." << std::endl;
        return false;
    }
    *m_pReadImageHolder = CCameraImage::FromIplImage( m_pCapturedImage );
    m_pReadImageHolder->cameraTime( 0 );
#if 0
    m_pReadImageHolder->cameraTime
        ( cvGetCaptureProperty( m_pCvCapture, CV_CAP_PROP_POS_MSEC )*1e3 );
#endif
    m_pReadImageHolder->time( 0 ); // Unclear how to get the correct time
    m_pReadImageHolder->sensorID( m_sSensorID );

    vImages.push_back( m_pReadImageHolder );
#if 0
    std::cout << cvGetCaptureProperty( m_pCvCapture, CV_CAP_PROP_POS_MSEC ) << std::endl;
    std::cout << m_pReadImageHolder->cameraTime() << std::endl;
#endif
    return true;
}

////////////////////////////////////////////////////////////////////////////////
std::string CCameraSensor::OpenCVCamera::get( const std::string& sKey ) {
    std::cout << "Calling get with key: " << sKey << std::endl;
    return "";
}

////////////////////////////////////////////////////////////////////////////////
bool CCameraSensor::OpenCVCamera::set( const std::string& sKey, const std::string& /*sParameter*/ ) {
    if( sKey == "ExternalTrigger" ) {
    }
    else  {
        std::cerr << "ERROR in set, unknown option" << std::endl;
    }
    return false;
}

////////////////////////////////////////////////////////////////////////////////
bool CCameraSensor::OpenCVCamera::has( const std::string& /*sKey*/ ) {
    return false;
}

////////////////////////////////////////////////////////////////////////////////
void CCameraSensor::OpenCVCamera::info() { 
    std::cout << "OpenCVCamera camera sensor." << std::endl; 
}
