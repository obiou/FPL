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
#ifndef UEYE_CAMERA_H
#define UEYE_CAMERA_H

#include <CCameraSensor/CameraSensor.h>

#include <iostream>
#include <map>
#include <string>
#include <vector>

#include <ueye.h>

///
/// This driver wraps the UEye driver.
/// It works on the 'live' video flow in external triggered mode. By
/// default, the trigger is forced when 'read' is called 
/// ( set( "ExternalTrigger", "1" ) will require an true external trigger signal).
///
/// The returned image is the buffer used by the camera for copying to RAM, it should not be freed or modified.
/// A subsequent call to read could modify this buffer so the client should make a copy if required.
///
/// Currently the driver only works with monochrome 8 bit images. (TODO: any image type)
/// 
/// The returned image contains the sensor serial number and two timestamps: one containing the camera time
/// and the other an estimated system time.
namespace CCameraSensor {
    ////////////////////////////////////////////////////////////////////////////
    /// Wrapper for UEye driver.
    FACTORY_OBJECT( UEyeCamera, CameraSensorInterface, "UEye" ) {
    public:
        UEyeCamera();
        ~UEyeCamera();

        /// open() should only be called once
        bool open();
        bool close();

        /// Each call will invalidate the previous image (make a copy if you wish to use it later)
        bool read( std::vector<ImageWrapper::Image>& vImages );

        std::string get( const std::string& sKey ) {
            std::cout << "Calling get with key: " << sKey << std::endl;
            return "";
        }

        bool set( const std::string& sKey, const std::string& sParameter ) {
            if( sKey == "ExternalTrigger" ) {
                m_mParameters[ "ExternalTrigger" ] = sParameter;
            }
            else if( sKey == "Trigger" ) {
                std::cout << "Triggering" << std::endl;
                is_ForceTrigger( m_hCamera );
                m_bTriggered = true;
            }
            else  {
                std::cerr << "ERROR in set, unknown option" << std::endl;
            }
            return false;
        }

        bool has( const std::string& /*sKey*/ ) {
            return false;
        }

        static void info() { 
            std::cout << "UEyeCamera camera sensor." << std::endl; 
        }

    private:
        bool m_bOpened;
        bool m_bTriggered;
        bool m_bFirstRead;
        char* m_pPrevLast;
        int m_nPrevNum;
        int m_nNumConnectedCams;
        PUEYE_CAMERA_LIST m_pCamList;
        HIDS m_hCamera;
        SENSORINFO m_SensorInfo;
        int m_nImageWidth;
        int m_nImageHeight;
        int m_nImageWidthStep;
        std::vector<std::pair<char*,int> > m_vRingBuffer;
        std::map<char*,int> m_mMemoryToImageID;
        std::map<char*,int> m_mMemoryToSeqID;
        std::string m_sSensorID;
        ImageWrapper::Image m_ReadImageHolder;
        //UEyeEventThread<OffsetFunctor> m_rEventThreadToComputeTimeOffset;
        std::map<std::string,std::string> m_mParameters;
        static const int NUM_RING_BUFFER_IMAGES;
    };    
}

#endif
