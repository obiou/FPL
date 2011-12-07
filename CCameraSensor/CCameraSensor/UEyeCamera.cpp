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
#include <CCameraSensor/UEyeCamera.h>

#include <sys/time.h>

////////////////////////////////////////////////////////////////////////////////    
const int CCameraSensor::UEyeCamera::NUM_RING_BUFFER_IMAGES = 5;

////////////////////////////////////////////////////////////////////////////////    
CCameraSensor::UEyeCamera::UEyeCamera() : m_bOpened( false ), m_bTriggered( false ), m_bFirstRead( true ), 
                                          m_pPrevLast( NULL ), m_nPrevNum( 0 ), m_nNumConnectedCams( 0 ), 
                                          //m_nCameraTimeOffset( 0 ),
                                          m_pCamList( NULL ), m_hCamera( (HIDS) 0 ),
                                          m_nImageWidth( 0 ), m_nImageHeight( 0 ),
                                          m_pReadImageHolder( NULL ) {
    //m_rEventThreadToComputeTimeOffset( this ) { 
    m_mParameters[ "ExternalTrigger" ] = "0";
}

////////////////////////////////////////////////////////////////////////////////    
CCameraSensor::UEyeCamera::~UEyeCamera() {
    close();
    if( m_pCamList != NULL ) {
        delete m_pCamList;
    }
    m_pCamList = NULL;
    if( m_pReadImageHolder != NULL ) {
        delete m_pReadImageHolder;
    }
    m_pReadImageHolder = NULL;
}

////////////////////////////////////////////////////////////////////////////////    
bool CCameraSensor::UEyeCamera::open() {
    if( m_bOpened ) {
        std::cerr << "ERROR: Open already called" << std::endl; 
        return false;
    }
    m_bOpened = true;

    std::cout << "Opening UEye." << std::endl; 

    // Query number of available camera
    if( is_GetNumberOfCameras( &m_nNumConnectedCams ) == IS_SUCCESS ) {
        std::cout << "Successful call to num. of camera: " << m_nNumConnectedCams << std::endl; 
    }
    else {
        std::cerr << "ERROR: in UEyeCamera::open(), call to num. of cameras failed." << std::endl;
        return false;
    }

    if( m_nNumConnectedCams <= 0 ) {
        std::cerr << "ERROR: in UEyeCamera::open(), got " << m_nNumConnectedCams << " cameras." << std::endl;
        return false;
    }

    // Allocate the required camera list size
    m_pCamList = (PUEYE_CAMERA_LIST) new char[ sizeof( DWORD ) + 
                                               m_nNumConnectedCams * sizeof( UEYE_CAMERA_INFO ) ];
    m_pCamList->dwCount = m_nNumConnectedCams;

    // Get camera list
    if( is_GetCameraList( m_pCamList ) != IS_SUCCESS ) {
        std::cerr << "ERROR: in UEyeCamera::open(), problem when calling is_GetCameraList." << std::endl;
        return false;
    }

    // For now, use first camera
    m_hCamera = 0;
    if( is_InitCamera( &m_hCamera, NULL ) != IS_SUCCESS ) {            
        std::cerr << "ERROR: in UEyeCamera::open(), problem starting the driver (is_InitCamera)." << std::endl;
        return false;
    }

    // Get sensor info
    if( is_GetSensorInfo( m_hCamera, &m_SensorInfo ) != IS_SUCCESS ) {
        std::cerr << "ERROR: in UEyeCamera::open(), problem when calling is_GetSensorInfo." << std::endl;
        return false;
    }
        
    std::ostringstream ss;
    // Documentation says this number should not be used for
    // identification, not sure why, using it anyway...
    ss << m_pCamList->uci[0].SerNo;
    m_sSensorID = ss.str();
    std::cout << "Sensor name: " << m_SensorInfo.strSensorName << std::endl;
    std::cout << "SensorID: " << m_sSensorID << std::endl;

    // Image width and height will be assumed to be the max (not supporting AOI)
    m_nImageWidth  = m_SensorInfo.nMaxWidth;
    m_nImageHeight = m_SensorInfo.nMaxHeight;

    // For now impose max image size
    if( is_SetImageSize( m_hCamera, m_nImageWidth, m_nImageHeight ) != IS_SUCCESS ) {
        std::cerr << "ERROR: in UEyeCamera::open(), problem when calling is_SetImageSize." << std::endl;
        return false;
    }

    std::cout << "Image size: " << m_nImageWidth << ", " << m_nImageHeight << std::endl;      

    // For now use only 8 bit monochrome
    if( is_SetColorMode( m_hCamera, IS_CM_MONO8 ) != IS_SUCCESS ) {
        std::cerr << "ERROR: in UEyeCamera::open(), problem when calling is_SetColorMode." << std::endl;
        return false;
    }

    const int nNumBits = 8;

    // Create a ring buffer and keep association between pointer values and IDs
    for( int ii=0; ii<NUM_RING_BUFFER_IMAGES; ii++ ) {
        char* pImageMem;
        int   pid;

        if( is_AllocImageMem
            ( m_hCamera, m_nImageWidth, m_nImageHeight, nNumBits, 
              &pImageMem, &pid ) != IS_SUCCESS) {
            std::cerr << "ERROR: in UEyeCamera::open(), problem when calling is_AllocImageMem." << std::endl;
            return false;
        }

        if( is_AddToSequence 
            ( m_hCamera, pImageMem, pid ) != IS_SUCCESS ) {
            std::cerr << "ERROR: in UEyeCamera::open(), problem when calling is_AddToSequence." << std::endl;
            return false;
        }

        m_vRingBuffer.push_back( std::pair<char*,int>( pImageMem, pid ) );
        m_mMemoryToImageID[ pImageMem ] = pid;
        m_mMemoryToSeqID[ pImageMem ]   = ii+1;
    }

    // Seems as though this has to be a called after memory allocation!
    if( is_GetImageMemPitch( m_hCamera, &m_nImageWidthStep ) != IS_SUCCESS ) {
        std::cerr << "ERROR: in UEyeCamera::open(), problem when calling is_GetImageMemPitch." << std::endl;
        return false;
    }
    std::cout << "widthStep: " << m_nImageWidthStep << std::endl;

    // Set automatic exposure
    double dRealExp;
    if( is_SetExposureTime( m_hCamera, IS_SET_ENABLE_AUTO_SHUTTER, &dRealExp ) != IS_SUCCESS ) {
        std::cerr << "ERROR: in UEyeCamera::open(), problem when calling is_SetExposureTime." << std::endl;
        return false;
    }

    // Start trigger
    if( is_SetExternalTrigger( m_hCamera, IS_SET_TRIGGER_HI_LO ) != IS_SUCCESS ) {
        std::cerr << "ERROR: in UEyeCamera::open(), problem when calling is_SetExternalTrigger." << std::endl;
        return false;
    }

#if 0
    // Start event threads
    // Start thread for computing time offset
    if( !m_rEventThreadToComputeTimeOffset.start( m_hCamera, IS_SET_EVENT_FRAME ) ) {
        std::cerr << "ERROR: in UEyeCamera::open(), problem starting time offset thread." << std::endl;
        return false;
    }    
#endif

    // Start capturing
    if( is_CaptureVideo( m_hCamera, IS_DONT_WAIT ) != IS_SUCCESS ) {
        std::cerr << "ERROR: in UEyeCamera::open(), problem when calling is_CaptureVideo." << std::endl;
        return false;
    }

    // Allocate image holder
    m_pReadImageHolder = new CCameraImage::Image();
    m_pReadImageHolder->width( m_nImageWidth );
    m_pReadImageHolder->height( m_nImageHeight );
    m_pReadImageHolder->widthStep( m_nImageWidthStep );
    m_pReadImageHolder->type( GL_UNSIGNED_BYTE );
    m_pReadImageHolder->format( GL_LUMINANCE );
    m_pReadImageHolder->time( 0 );
    m_pReadImageHolder->sensorID( m_sSensorID );

    return true;
}

////////////////////////////////////////////////////////////////////////////////    
bool CCameraSensor::UEyeCamera::close() {
    std::cout << "Closing UEyeCamera." << std::endl; 
    //m_rEventThreadToComputeTimeOffset.stop();
    is_StopLiveVideo( m_hCamera, IS_WAIT );
    is_ClearSequence( m_hCamera );
    // Free ring memory
    for( unsigned int ii=0; ii<m_vRingBuffer.size(); ii++ ) {
        is_FreeImageMem( m_hCamera, m_vRingBuffer[ii].first, m_vRingBuffer[ii].second );
    }
    m_vRingBuffer.clear();

    if( m_hCamera != 0 ) {
        is_ExitCamera( m_hCamera );
    }
    m_hCamera = (HIDS) 0;
    return true;
} 

////////////////////////////////////////////////////////////////////////////////    
bool CCameraSensor::UEyeCamera::read( std::vector<CCameraImage::Image*>& vImages ) {
    if( !m_bOpened ) { 
        std::cerr << "ERROR: in UEyeCamera::read(), has open() been called?" << std::endl;
        return false;
    }
    if( m_hCamera == 0 ) {
        std::cerr << "ERROR: in UEyeCamera::read() incorrect camera identifier, has open() been called?" << std::endl;
        return false;
    }

    // This is the current system, time at which we make the
    // the call to the camera
    struct timeval tv;
    gettimeofday( &tv, NULL );
    unsigned long long nAcquisitionTime = (unsigned long long)1e6*tv.tv_sec + tv.tv_usec; 

    // Force trigger
    if( m_mParameters[ "ExternalTrigger"] != "1" || !m_bTriggered ) {
        is_ForceTrigger( m_hCamera );
    }
    else {
        std::cout << "Skipping trigger" << std::endl;
    }
    m_bTriggered = false;

    // Get 
    int nDummy = 0;
    char *pLast = NULL, *pDummy = NULL;
    if( is_GetActSeqBuf( m_hCamera, &nDummy, &pDummy, &pLast ) != IS_SUCCESS ) {
        std::cerr << "ERROR: in UEyeCamera::read(), problem when calling is_GetActSeqBuf." << std::endl;
        return false;
    }

    if( pLast == NULL ) {
        std::cerr << "ERROR: in UEyeCamera::read(), last camera buffer is NULL (call too early?)." << std::endl;
        return false;
    }

    int nNum = m_mMemoryToSeqID[ pLast ];

    // Lock the buffer and give it to the caller
    // Previous locked buffer should be unlocked when
    // caller calls 'read' again.
    if( m_bFirstRead ) {
        is_LockSeqBuf( m_hCamera, nNum, pLast );
        m_bFirstRead = false;
    }
    else {
        is_UnlockSeqBuf( m_hCamera, m_nPrevNum, m_pPrevLast ); 
        is_LockSeqBuf( m_hCamera, nNum, pLast );
    }
    m_pPrevLast = pLast;
    m_nPrevNum  = nNum;

    // Extract timing
    UEYEIMAGEINFO ImageInfo;
    if( is_GetImageInfo( m_hCamera, nNum,
                         &ImageInfo, sizeof( ImageInfo ) ) != IS_SUCCESS ) {
        std::cerr << "ERROR: in UEyeCamera::read(), problem when calling is_GetImageInfo." << std::endl;
        return false;
    }

    // Associate last
    // Best time but this is camera time... 
    m_pReadImageHolder->cameraTime( ImageInfo.u64TimestampDevice/10 );
    // Not very good time ~1ms precision
    m_pReadImageHolder->time( nAcquisitionTime );
    m_pReadImageHolder->data( (unsigned char*)pLast );
    m_pReadImageHolder->sensorID( m_sSensorID );

    vImages.clear();
    vImages.push_back( m_pReadImageHolder );

    //std::cout << "Acquisition time: " << m_pReadImageHolder->time() << std::endl;

#if 0
    double dFPS;
    if( is_GetFramesPerSecond( m_hCamera, &dFPS ) != IS_SUCCESS ) {
        std::cerr << "ERROR: in UEyeCamera::open(), problem when calling is_GetFramesPerSecond." << std::endl;
        return false;
    }
    std::cout << "FPS: " << dFPS << std::endl;
#endif

    return true;
}
