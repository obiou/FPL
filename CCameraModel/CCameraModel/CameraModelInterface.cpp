#include <CCameraModel/CameraModelInterface.h>
#include <CCameraModel/CameraStringParsing.h>

#include <iostream>
#include <sstream>

namespace CCameraModel {
    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    void CameraModelInterface<>::ToStream( std::ostream& oStream ) const { 
        oStream << "cameraModel.CameraType = " << get_camera_type() << std::endl;
        oStream << "# ";
        for( int ii=0; ii<get_num_parameters(); ++ii ) {
            std::string sVarName = parameter_index_to_name( ii );
            oStream << sVarName << " "; 
        }
        oStream << std::endl;
        oStream << "cameraModel.Parameters = [ ";
        for( int ii=0; ii<get_num_parameters(); ++ii ) {
            std::string sVarName = parameter_index_to_name( ii );
            oStream << get( sVarName ) << " "; 
        }
        oStream << "]" << std::endl;
    }

    ////////////////////////////////////////////////////////////////////////////
    void CameraModelInterface<>::FromStream( std::istream& iStream ) {
        std::string sLine;
        bool bFoundParameters = false;
        while( get_not_comment_line( iStream, sLine ) ) {
            if( sLine.find( "cameraModel.Parameters" ) != std::string::npos ) {
                std::string sPar;
                if( get_parameter( "cameraModel.Parameters", "=", sLine, sPar ) ) {
                    if( sPar.find( "[" ) != std::string::npos &&
                        sPar.find( "]" ) != std::string::npos ) {
                        sPar.erase( 0, sPar.find( "[" )+1 );
                        sPar.erase( sPar.find( "]" ) );
                        sPar = remove_spaces( sPar );
                        std::stringstream myStream( sPar );
                        bFoundParameters = true;
                        for( int ii=0; ii<get_num_parameters(); ++ii ) {
                            std::string sVarName = parameter_index_to_name( ii );
                            std::string sVal;
                            if( !(myStream >> sVal) ) {
                                std::cout << "ERROR: in  CameraModelInterface<>::FromStream, parsing " << sVal << std::endl;
                                bFoundParameters = false;
                                break;
                            }
                            set( sVarName, sVal );
                        }
                    }
                }
            }
            else if( sLine.find( "cameraModel.CameraType" ) == std::string::npos ) {
                std::cout << "WARNING: (CameraModelInterface<>::FromStream) spurious line: " << sLine << std::endl;
            }
        }
        
        if( !bFoundParameters ) {
            std::cerr << "ERROR in CameraModelInterface::FromStream, ";
            std::cerr << "did not find parameters when parsing." << std::endl;
            return;
        }
    }

    ////////////////////////////////////////////////////////////////////////////
    template<class T>
    void transpose( T* pData, const int nWidth, const int nHeight ) {
        for( int nCol=0; nCol<nWidth; nCol++ ) {
            for( int nRow=nCol; nRow<nHeight; nRow++ ) {
                std::swap( pData[ nRow + nCol*nHeight ], pData[ nCol + nRow*nWidth ] );
            }
        }
    }
    
    ////////////////////////////////////////////////////////////////////////////
    // Instantiate
    #define INSTANTIATE_TRANSPOSE( T )               \
        template void transpose( T*, const int, const int );

    INSTANTIATE_TRANSPOSE( char );
    INSTANTIATE_TRANSPOSE( int );
    INSTANTIATE_TRANSPOSE( float );
    INSTANTIATE_TRANSPOSE( double );
}
