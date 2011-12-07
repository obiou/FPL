#include <CCameraModel/CameraStringParsing.h>

#include <iostream>
#include <sstream>

namespace CCameraModel {
    ////////////////////////////////////////////////////////////////////////////
    std::string remove_spaces( std::string str ) {
        str.erase( str.find_last_not_of( ' ' ) + 1 );
        str.erase( 0, str.find_first_not_of( ' ' ) );
        return str;
    }

    ////////////////////////////////////////////////////////////////////////////
    bool get_not_comment_line( std::istream& iStream, std::string& sLineNoComment ) {
        std::string sLine;
        if( !iStream.good() ) { return false; }
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
    bool get_parameter( const std::string& sName, const std::string& sDel,
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
    bool parse_camera_type( const std::string& sLine, std::string& sCameraType ) {
        std::string sPar;
        if( get_parameter( "cameraModel.CameraType", "=", sLine, sPar ) ) {
            sCameraType = sPar;
            return true;
        }
        return false;
    }

    ////////////////////////////////////////////////////////////////////////
    bool FindCameraType( std::iostream& ioStream, 
                         const std::string& sCameraTypePrefix, 
                         std::string& sCameraType ) {
        bool bFoundCameraType = false;
        std::stringstream sUnusedLines;
        std::string sLine;
        while( !bFoundCameraType && get_not_comment_line( ioStream, sLine ) ) {
            if( !bFoundCameraType && sLine.find( sCameraTypePrefix ) != std::string::npos ) {
                if( parse_camera_type( sLine, sCameraType ) ) {
                    bFoundCameraType = true;
                }
            }
            else {
                sUnusedLines << sLine;
            }
        }
        ioStream << sUnusedLines.str();
        return bFoundCameraType;
    }

    ////////////////////////////////////////////////////////////////////////
    bool FindCameraType( std::istream& iStream, 
                         const std::string& sCameraTypePrefix, 
                         std::string& sCameraType ) {
        bool bFoundCameraType = false;
        std::string sLine;
        while( !bFoundCameraType && get_not_comment_line( iStream, sLine ) ) {
            if( !bFoundCameraType && sLine.find( sCameraTypePrefix ) != std::string::npos ) {
                if( parse_camera_type( sLine, sCameraType ) ) {
                    bFoundCameraType = true;
                }
            }
        }
        return bFoundCameraType;
    }
}
