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
#include <CCameraSensor/CameraSensor.h>

////////////////////////////////////////////////////////////////////////////////
CCameraSensor::CameraSensor::~CameraSensor() {
    if( m_pCameraSensor != NULL ) {
        delete m_pCameraSensor;
        m_pCameraSensor = NULL;
    }
}

////////////////////////////////////////////////////////////////////////////////
bool CCameraSensor::CameraSensor::init_reset( const std::string& sCameraType ) {
    m_sCameraType = sCameraType;
    if( m_pCameraSensor != NULL ) {
        delete m_pCameraSensor;
        m_pCameraSensor = NULL;
    }
    m_pCameraSensor = CameraSensorInterfaceFactory::Create( sCameraType );
    if( m_pCameraSensor == NULL ) {
        return false;
    }
    return true;
}

////////////////////////////////////////////////////////////////////////////////
bool CCameraSensor::CameraSensor::open() {
    if( m_pCameraSensor == NULL ) {
        return false;
    }
    return m_pCameraSensor->open();
}

////////////////////////////////////////////////////////////////////////////////
CCameraImage::Image* CCameraSensor::CameraSensor::read() {
    if( m_pCameraSensor == NULL ) {
        return NULL;
    }
    std::vector<CCameraImage::Image*> vImages;
    bool bRead = m_pCameraSensor->read( vImages );
    if( bRead && !vImages.empty() ) {
        return vImages[0];
    }
    return NULL;
}

////////////////////////////////////////////////////////////////////////////////
bool CCameraSensor::CameraSensor::read( std::vector<CCameraImage::Image*>& vImages ) {
    if( m_pCameraSensor == NULL ) {
        return false;
    }
    return m_pCameraSensor->read( vImages );
}

////////////////////////////////////////////////////////////////////////////////
bool CCameraSensor::CameraSensor::close() {
    if( m_pCameraSensor == NULL ) {
        return false;
    }
    return m_pCameraSensor->close();
}

////////////////////////////////////////////////////////////////////////////////
std::string CCameraSensor::CameraSensor::get( const std::string& sKey ) {
    if( m_pCameraSensor == NULL ) {
        return "";
    }
    return m_pCameraSensor->get( sKey );
}

////////////////////////////////////////////////////////////////////////////////
bool CCameraSensor::CameraSensor::set( const std::string& sKey, const std::string& sParameter ) {
    if( m_pCameraSensor == NULL ) {
        return false;
    }
    return m_pCameraSensor->set( sKey, sParameter );
}

////////////////////////////////////////////////////////////////////////////////
bool CCameraSensor::CameraSensor::has( const std::string& sKey ) {
    if( m_pCameraSensor == NULL ) {
        return false;
    }
    return m_pCameraSensor->has( sKey );
}
