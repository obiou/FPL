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
#ifndef FILE_READER_FROM_LIST_H
#define FILE_READER_FROM_LIST_H

#include <fstream>
#include <string>

#include <CCameraSensor/CameraSensorInterface.h>

///
/// This defines the basic call made by users.
/// The class is in fact a factory, it standardises the calls by implementing the CameraSensorInterface.h
///
/// To avoid complicating the code, the users 
namespace CCameraSensor {
   ////////////////////////////////////////////////////////////////////////////
    FACTORY_OBJECT( FileReaderFromList, CameraSensorInterface, "FileReaderFromList" ) {
    public:
        ////////////////////////////////////////////////////////////////////////
    FileReaderFromList() : 
        m_bOpened(false), m_pCapturedImage( NULL ) 
            {}

        ////////////////////////////////////////////////////////////////////////
        ~FileReaderFromList();

        ////////////////////////////////////////////////////////////////////////
        bool open();

        ////////////////////////////////////////////////////////////////////////
        bool close();

        ////////////////////////////////////////////////////////////////////////
        bool read( std::vector<ImageWrapper::Image>& vImages );

        ////////////////////////////////////////////////////////////////////////
        std::string get( const std::string& sKey );

        ////////////////////////////////////////////////////////////////////////
        bool set( const std::string& sKey, const std::string& sParameter );

        ////////////////////////////////////////////////////////////////////////
        bool has( const std::string& sKey );

        ////////////////////////////////////////////////////////////////////////
        static void info();

        ////////////////////////////////////////////////////////////////////////
    private:
        std::ifstream m_File;
        std::string m_sListFileName;
        std::string m_sLastReadFile;
        ImageWrapper::Image m_ReadImageHolder; 
        bool m_bOpened;
        IplImage* m_pCapturedImage;
    };
}

#endif
