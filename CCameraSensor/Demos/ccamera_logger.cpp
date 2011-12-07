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
#include <deque>
#include <iomanip>
#include <iostream>
#include <thread>
#include <tuple>
#include <string>

#include <chrono>

#if 0
typedef std::chrono::high_resolution_clock Clock;
typedef std::chrono::duration<double> sec;
int n = 256*1024*1024;
Clock::time_point start = Clock::now();
Clock::time_point end = Clock::now();
std::cout << sec(end-start).count() << " seconds\n";
#endif

#include <CameraSensor.h>

namespace CCameraSensor {
    ////////////////////////////////////////////////////////////////////////////////
    class CameraLogger {
    public:
        CameraLogger( const std::string& sCameraType,
                      const std::string& sPath,
                      const std::string& sPrefix,
                      const std::string& sSuffix,
                      unsigned int nMaxBufferSize = 100
                      );
        ~CameraLogger();
        bool Start( double dDelayMS = 0 );
        void Block();
        void Stop();
    private:
        void _DequeueAndSaveImage();
        void _GetImageAndQueue();

    private:
        // Shared value
        typedef std::tuple<CCameraImage::Image*,std::string> ImageAndName;
        std::deque<ImageAndName> m_qImages;
        volatile bool m_bRun;
        // Private to writer
        unsigned int m_nCount;
    private: // Locking mechanism
        std::condition_variable m_Condition;
        std::mutex m_Mutex, m_ConditionMutex;
        std::thread m_ThreadSave, m_ThreadGet;
    private:
        std::string m_sPath, m_sPrefix, m_sSuffix, m_sCameraType;
        unsigned int m_nMaxBufferSize, m_nDroppedFrames;
        std::unique_ptr<CameraSensor> m_pCamera;
    public:
        static const unsigned int g_nPaddingImageNumber;
        static const unsigned int g_nPaddingCameraNumber;
    };
}

////////////////////////////////////////////////////////////////////////////////
const unsigned int CCameraSensor::CameraLogger::g_nPaddingImageNumber = 6;
const unsigned int CCameraSensor::CameraLogger::g_nPaddingCameraNumber = 2;

////////////////////////////////////////////////////////////////////////////////
CCameraSensor::CameraLogger::CameraLogger
( const std::string& sCameraType,
  const std::string& sPath,
  const std::string& sPrefix,
  const std::string& sSuffix,
  const unsigned int nMaxBufferSize
  ) :
    m_bRun( false ), m_nCount( 0 ),
    m_sPath( sPath ), m_sPrefix( sPrefix ), m_sSuffix( sSuffix ),
    m_sCameraType( sCameraType ),
    m_nMaxBufferSize( nMaxBufferSize ), m_nDroppedFrames(0)
{}

////////////////////////////////////////////////////////////////////////////////
CCameraSensor::CameraLogger::~CameraLogger() {
    Stop();
}

////////////////////////////////////////////////////////////////////////////////
bool CCameraSensor::CameraLogger::Start( double dDelayMS ) {
    if( m_bRun ) { Stop(); }
    if( m_pCamera.get() == 0 ) {
        m_pCamera = std::unique_ptr<CameraSensor>( new CCameraSensor::CameraSensor( m_sCameraType ) );
    }
    if( m_pCamera.get() == 0 || !m_pCamera->open() ) { return false; }

    if( dDelayMS != 0 ) {
        usleep( dDelayMS*1000 );
    }
    m_nDroppedFrames = 0;

    m_bRun = true;
    m_ThreadSave = std::thread( &CameraLogger::_DequeueAndSaveImage, this );
    m_ThreadGet  = std::thread( &CameraLogger::_GetImageAndQueue, this );

    return true;
}

////////////////////////////////////////////////////////////////////////////////
void CCameraSensor::CameraLogger::Block() {
    if( m_ThreadGet.joinable() ) {
        m_ThreadGet.join();
    }
    if( m_ThreadSave.joinable() ) {
        m_ThreadSave.join();
    }
}

////////////////////////////////////////////////////////////////////////////////
void CCameraSensor::CameraLogger::Stop() {
    m_bRun = false;
    if( m_ThreadGet.joinable() ) {
        m_ThreadGet.join();
    }
    if( m_ThreadSave.joinable() ) {
        m_ThreadSave.join();
    }
}

////////////////////////////////////////////////////////////////////////////////
void CCameraSensor::CameraLogger::_GetImageAndQueue() {
    std::cout << "Queue Thread started" << std::endl;
    while( m_bRun ) {
        std::stringstream ssFileName;
        ssFileName << m_sPath << "/" << m_sPrefix << "_"
                   << std::setw( g_nPaddingImageNumber ) << std::setfill( '0' ) << m_nCount;
        if( m_qImages.size() > m_nMaxBufferSize ) {
            m_nDroppedFrames++;
            std::cout << "Dropping frame (num. dropped: " << m_nDroppedFrames << ")." << std::endl;
            continue;
        }
        else {
            std::lock_guard<std::mutex> mutex( m_Mutex );
            std::vector<CCameraImage::Image*> vImages;
            if( !m_pCamera->read( vImages ) ) { continue; };
            if( vImages.empty() ) {
                continue;
            }
            m_nCount++;
            if( vImages.size() == 1 ) {
                ssFileName << m_sSuffix;
                m_qImages.push_back( ImageAndName( vImages[0]->copy(), ssFileName.str() ) );
            }
            else {
                std::string sBeg = ssFileName.str();
                for( size_t ii=0; ii<vImages.size(); ii++ ) {
                    ssFileName.clear();
                    ssFileName << sBeg << "_" << std::setw( g_nPaddingCameraNumber ) << std::setfill( '0' ) << m_sSuffix;
                    m_qImages.push_back( ImageAndName( vImages[ii]->copy(), ssFileName.str() ) );
                }
            }

            m_Condition.notify_one();
#if 0
            if( m_nCount > 200 ) { // FOR DEBUGGING
                m_bRun = false;
            }
#endif
        }
    }
    m_Condition.notify_one();
}

////////////////////////////////////////////////////////////////////////////////
void CCameraSensor::CameraLogger::_DequeueAndSaveImage() {
    std::cout << "Dequeue Thread started" << std::endl;

    std::unique_lock<std::mutex> cond( m_ConditionMutex );
    while( m_bRun ) {
        while( m_qImages.empty() ) {
            //std::cout << "Empty" << std::endl;
            // Wait for signal
            m_Condition.wait( cond );
        }
        
        //std::cout << "Not empty" << std::endl;
        while( !m_qImages.empty() ) {
            std::lock_guard<std::mutex> mutex( m_Mutex );
            ImageAndName imageAndName = m_qImages.front();
            m_qImages.pop_front();
            std::cout << m_qImages.size() << " " << std::get<1>( imageAndName ) << std::endl;   
            std::string sImageName = std::get<1>( imageAndName );
            std::string sExtraImageInforName = sImageName;
            sExtraImageInforName.erase( sExtraImageInforName.rfind( '.' ) );
            sExtraImageInforName += ".txt";
            // Save
            std::get<0>( imageAndName )->save( sImageName, sExtraImageInforName);
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
using namespace CCameraSensor;
using namespace std;

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
int main()
{
    //std::string sCameraType = "FileReader";
    std::string sCameraType = "UEyeBlocking";
    std::string sPath = "./tmp/";
    std::string sPrefix = "image";
    std::string sSuffix = ".ppm";
    CameraLogger logger( sCameraType, sPath, sPrefix, sSuffix );

    cout << "\a";
    double dDelayMS = 1000;
    if( !logger.Start( dDelayMS ) ) {
        cerr << "Problem starting logger..." << endl;
        return -1;
    }

    cout << "Done" << endl;

    logger.Block();
    //    cout << "No more..." << endl;

    return 0;
}
