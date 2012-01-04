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
#ifndef CCAMERA_SENSOR_H
#define CCAMERA_SENSOR_H

#include <CCameraSensor/CameraSensorInterface.h>

#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

///
/// This defines the basic class used by the users.
///
/// It is composed of an interface that should be implemented by each
/// specific camera model class and a 'CameraSensor' class, used by the
/// users that will generate any specific class declared by using the
/// underlying factory (@see PinholeCamera for an example).
///
/// Macros are used to make the code easier to read.
namespace CCameraSensor {
    ////////////////////////////////////////////////////////////////////////////
    /// Conversion function: converts to a string
    template<class T> inline std::string ToString( const T& param ) {
        std::ostringstream os;
        os << std::setprecision( 9 ) << std::setiosflags( std::ios::fixed ) << param;
        return os.str();
    }

    /// Conversion function: converts from a string
    template<class T> inline T FromString( const std::string& s ) {
        std::stringstream ss;
        ss << s; T t; ss >> t;
        return t;
    }

    ////////////////////////////////////////////////////////////////////////////
    /// Main class used by the final user
    class CameraSensor : public CameraSensorInterface<> {
    public:
    CameraSensor( std::string sCameraType ) : 
        m_pCameraSensor( CameraSensorInterfaceFactory::Create( sCameraType ) ),
            m_sCameraType( sCameraType ){}

        ////////////////////////////////////////////////////////////////////////
    CameraSensor() :
        m_pCameraSensor( NULL ) {}

        ////////////////////////////////////////////////////////////////////////
        ~CameraSensor();

        ////////////////////////////////////////////////////////////////////////
        bool init_reset( const std::string& sCameraType );

        ////////////////////////////////////////////////////////////////////////
        bool open();

        ////////////////////////////////////////////////////////////////////////
        /// Simplified call to only retrieve one image.
        /// (does not appear in the interface)
        ImageWrapper::Image read();

        ////////////////////////////////////////////////////////////////////////
        bool read( std::vector<ImageWrapper::Image>& vImages );

        ////////////////////////////////////////////////////////////////////////
        bool close();

        ////////////////////////////////////////////////////////////////////////
        /// Generic get (does not appear in the interface)
        template <class T>
            T get( const std::string& sKey ) {
            if( m_pCameraSensor == NULL ) {
                return false;
            }
            return FromString<T>( m_pCameraSensor->get( sKey ) ); 
        }

        ////////////////////////////////////////////////////////////////////////
        std::string get( const std::string& sKey );

        ////////////////////////////////////////////////////////////////////////
        /// Generic set (does not appear in the interface)
        template <class T>
            bool set( const std::string& sKey, const T& parameter ) {
            if( m_pCameraSensor == NULL ) {
                return false;
            }
            const std::string sParameter = ToString<T>( parameter );
            return m_pCameraSensor->set( sKey, sParameter );
        }

        ////////////////////////////////////////////////////////////////////////
        bool set( const std::string& sKey, const std::string& sParameter );

        ////////////////////////////////////////////////////////////////////////
        bool has( const std::string& sKey );

    private:
        CameraSensorInterface<>* m_pCameraSensor;
        std::string m_sCameraType;
    }; 
}

#endif
