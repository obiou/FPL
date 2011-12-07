#ifndef CAMERA_STRING_PARSING
#define CAMERA_STRING_PARSING

#include <string>
#include <istream>
#include <iostream>

/// Bits of code to parse streams describing camera models
namespace CCameraModel {
    ////////////////////////////////////////////////////////////////////////////
    std::string remove_spaces( std::string );

    ////////////////////////////////////////////////////////////////////////////
    bool get_not_comment_line( std::istream& iStream, std::string& sLineNoComment );

    ////////////////////////////////////////////////////////////////////////////
    bool get_parameter( const std::string& sName, const std::string& sDel,
                        const std::string& sLine, std::string& sPar );

    
    ////////////////////////////////////////////////////////////////////////////
    bool parse_camera_type( const std::string& sLine, std::string& sCameraType );

    ////////////////////////////////////////////////////////////////////////////
    bool FindCameraType( std::iostream& ioStream, 
                         const std::string& sCameraTypePrefix, 
                         std::string& sCameraType );

    ////////////////////////////////////////////////////////////////////////////
    bool FindCameraType( std::istream& iStream, 
                         const std::string& sCameraTypePrefix, 
                         std::string& sCameraType );
}

#endif
