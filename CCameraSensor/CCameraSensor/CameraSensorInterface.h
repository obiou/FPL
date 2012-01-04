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
#ifndef CCAMERA_SENSOR_INTERFACE_H
#define CCAMERA_SENSOR_INTERFACE_H

#include <cimage.h>
#include <CCameraSensor/factory.h>

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
    FACTORY_INTERFACE_WITH_STATIC( CameraSensorInterface ) {
    public:
        virtual ~CameraSensorInterface() {}
        /// Initialises a particular driver, returns false if it fails
        virtual bool open() = 0;
        /// Read several images
        virtual bool read( std::vector<ImageWrapper::Image>& vImages ) = 0;
        virtual bool close() = 0;

        virtual std::string get( const std::string& sKey ) = 0;
        virtual bool set( const std::string& sKey, const std::string& sParameter ) = 0;
        virtual bool has( const std::string& sKey ) = 0;

        static void info();
    };

    FACTORY_INTERFACE_ONLY_STATIC( CameraSensorInterface ) {
    public:
        virtual void info() = 0;
    };

    FACTORY_INTERFACE_ONLY_STATIC_IMPL( CameraSensorInterface ) {
    public:
        void info() { Derived::info(); }
    };
}

#endif
