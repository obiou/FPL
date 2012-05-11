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
#include <fstream>
#include <iostream>
#include <string>

#include <CCameraSensorIncludes.h>
#include <CCameraSensor/FileReaderFromList.h>

#if CCAMERASENSOR_HAS_OPENCV

#include <opencv2/highgui/highgui.hpp>

///
/// This defines the basic call made by users.
/// The class is in fact a factory, it standardises the calls by implementing the CameraSensorInterface.h
///
/// To avoid complicating the code, the users 
namespace CCameraSensor {
    ////////////////////////////////////////////////////////////////////////////
    inline std::string remove_spaces( std::string str ) {
        str.erase( str.find_last_not_of( ' ' ) + 1 );
        str.erase( 0, str.find_first_not_of( ' ' ) );
        return str;
    }

    ////////////////////////////////////////////////////////////////////////////
    inline bool get_not_comment_line( std::istream& iStream, std::string& sLineNoComment ) {
        std::string sLine;
        while( !iStream.eof() ) {
            getline( iStream, sLine );
            sLine.erase( 0, sLine.find_first_not_of( ' ' ) );
            if( sLine.empty() ) {
                continue;
            }
            else if( sLine[0] != '#' ) {
                sLineNoComment = sLine;
                return true;
            }
        }
        return false;
    }

    ////////////////////////////////////////////////////////////////////////////
    inline bool get_parameter( const std::string& sName, const std::string& sDel,
                               const std::string& sLine, std::string& sPar ) {
        sPar = sLine;
        if( sPar.find( sName ) == std::string::npos ) { return false; }
        sPar.erase( 0, sPar.find( sName )+sName.length() );
        if( sPar.find( sDel ) == std::string::npos ) { return false; }
        sPar.erase( 0, sPar.find( sDel )+sDel.length() );
        sPar = remove_spaces( sPar );
        return true;
    }

    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    FileReaderFromList::~FileReaderFromList() {
        close();
    }
    
    ////////////////////////////////////////////////////////////////////////////
    bool FileReaderFromList::open() {
        if( m_bOpened ) {
            std::cerr << "ERROR: successful open already called" << std::endl; 
            return false;
        }
        if( m_sListFileName == "" ) {
            std::cerr << "ERROR in FileReaderFromList, no file name provided, ";
            std::cerr << "you should call set( \"ListFileName\" )." << std::endl;
            return false;
        }
        m_File.open( m_sListFileName.c_str() );
        m_bOpened = m_File.is_open();
        if( !m_bOpened ) {
            std::cerr << "ERROR in FileReaderFromList, could not open: " << m_sListFileName << "." << std::endl;
        }
        return m_bOpened;
    }

    ////////////////////////////////////////////////////////////////////////////
    bool FileReaderFromList::close() {
        m_File.close();
        if( m_pCapturedImage != NULL ) {
            cvReleaseImage( &m_pCapturedImage );
            m_pCapturedImage = NULL;
        }
        return true;
    }

    ////////////////////////////////////////////////////////////////////////////
    bool FileReaderFromList::read( std::vector<ImageWrapper::Image>& vImages ) {
        std::string sLine;
        if( get_not_comment_line( m_File, sLine ) ) {
            if( m_pCapturedImage != NULL ) {
                cvReleaseImage( &m_pCapturedImage );
                m_pCapturedImage = NULL;
            }

            m_ReadImageHolder.read( sLine.c_str(), true, CV_LOAD_IMAGE_UNCHANGED );

#if 0
            m_pCapturedImage = cvLoadImage( sLine.c_str(), CV_LOAD_IMAGE_UNCHANGED );
            if( m_pCapturedImage == NULL ) {
                return false;
            }
            m_sLastReadFile = sLine;
            m_ReadImageHolder.mImage = m_pCapturedImage;
            m_ReadImageHolder.dCameraTime = 0; 
            m_ReadImageHolder.dSystemTime = 0;
            m_ReadImageHolder.sSensorID = "";
            {
                // Look for extra file
                std::string sExtraInfo = sLine;
                if( sExtraInfo.rfind( "." ) != std::string::npos ) {
                    sExtraInfo.erase( sExtraInfo.rfind( "." ) );
                    sExtraInfo += ".txt";
                    std::ifstream oFile( sExtraInfo.c_str() );
                    std::string sExtraLine;
                    while( oFile.is_open() && !oFile.eof() ) {
                        get_not_comment_line( oFile, sExtraLine ) ;
                        std::string sPar;
                        if( get_parameter( "image.sensorID", "=",
                                           sExtraLine, sPar ) ||
                            get_parameter( "SensorID", ":",
                                           sExtraLine, sPar ) ) {
                            size_t nBeg = sPar.find( "\"" );
                            if( nBeg != std::string::npos ) {
                                sPar.erase( 0, nBeg+1 );
                            }
                            size_t nEnd = sPar.rfind( "\"" );
                            if( nEnd != std::string::npos ) {
                                sPar.erase( nEnd );
                            }
                            m_ReadImageHolder.sSensorID = sPar;
                        }
                        else if( get_parameter( "image.cameraTime", "=",
                                                sExtraLine, sPar ) ||
                                 get_parameter( "CameraTime", ":",
                                                sExtraLine, sPar ) ) {
                            size_t nEnd = sPar.rfind( "." );
                            if( nEnd != std::string::npos ) {
                                sPar.erase( nEnd );
                            }
                            std::stringstream oss; oss << sPar;
                            double dCameraTime;
                            if( oss >> dCameraTime ) {
                                m_ReadImageHolder.dCameraTime = dCameraTime;
                            }
                        }
                        else if( get_parameter( "image.time", "=",
                                                sExtraLine, sPar ) ||
                                 get_parameter( "SystemTime", ":",
                                                sExtraLine, sPar ) ) {
                            size_t nEnd = sPar.rfind( "." );
                            if( nEnd != std::string::npos ) {
                                sPar.erase( nEnd );
                            }
                            std::stringstream oss; oss << sPar;
                            double dTime;
                            if( oss >> dTime ) {
                                m_ReadImageHolder.dSystemTime = dTime;
                            }
                        }
                    }
                    oFile.close();
                }
            }
#endif
            vImages.push_back( m_ReadImageHolder );
            return true;
        }
        return false;
    }

    ////////////////////////////////////////////////////////////////////////////
    std::string FileReaderFromList::get( const std::string& sKey ) { 
        if( sKey == "LastReadFile" ) {
            return m_sLastReadFile;
        }
        return "";
    }

    ////////////////////////////////////////////////////////////////////////////
    bool FileReaderFromList::set( const std::string& sKey, const std::string& sParameter ) {
        if( sKey == "ListFileName" ) {
            m_sListFileName = sParameter;
            return true;
        }
        return false;
    }

    ////////////////////////////////////////////////////////////////////////////
    bool FileReaderFromList::has( const std::string& sKey ) {
        if( sKey == "ListFileName" ) {
            return true;
        }
        return false;
    }

    ////////////////////////////////////////////////////////////////////////////
    void FileReaderFromList::info() { 
        std::cout << "FileReaderFromList, reads list of images contained in a file." << std::endl; 
    }
}

#endif
