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
#ifndef FILE_READER_H
#define FILE_READER_H

#include <iostream>
#include <string>

#include <CCameraSensor/CameraSensor.h>

///
/// This defines the basic call made by users.
/// The class is in fact a factory, it standardises the calls by implementing the CameraSensorInterface.h
///
/// To avoid complicating the code, the users 
namespace CCameraSensor {

    /// Specific camera: Pinhole sensor
    FACTORY_OBJECT( FileReader, CameraSensorInterface, "FileReader" ) {
    public:
        FileReader() { 
            std::cout << "FileReader constructor" << std::endl; 
        }

        bool open() {
            std::cout << "Opening FileReader." << std::endl; 
            return true;
        }

        bool close() {
            std::cout << "Closing FileReader." << std::endl; 
            return true;
        }

        bool read( std::vector<ImageWrapper::Image>& /*vImages*/ ) {
            std::cout << "Reading images." << std::endl; 
            return true;
        }

        std::string get( const std::string& sKey ) {
            std::cout << "Calling get with key: " << sKey << std::endl;
            return "";
        }

        bool set( const std::string& sKey, const std::string& /*sParameter*/ ) {
            std::cout << "Calling set with key: " << sKey << std::endl;
            return false;
        }

        bool has( const std::string& /*sKey*/ ) {
            return false;
        }

        static void info() { 
            std::cout << "FileReader camera sensor." << std::endl; 
        }
    };
}

#endif
