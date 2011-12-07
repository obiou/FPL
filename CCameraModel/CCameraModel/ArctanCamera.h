#ifndef ARCTAN_CAMERA_H
#define ARCTAN_CAMERA_H

#include <iostream>
#include <string>

///
/// This defines the basic call made by users.
/// The class is in fact a factory, it standardises the calls by implementing the CameraModelInterface.h
///
/// To avoid complicating the code, the users 
namespace CCameraModel {
    const int ARCTAN_NUM_PARAMS = 5;

    ////////////////////////////////////////////////////////////////////////////
    /// Specific camera: Pinhole model
    FACTORY_OBJECT( ArctanCamera, CameraModelInterface, "Arctan" ) {
    public:
        ////////////////////////////////////////////////////////////////////////
    ArctanCamera() : m_dFx(0), m_dFy(0), m_dCx(0), m_dCy(0), m_dO(0) {
            std::cout << "ARCTANCAMERA constructor" << std::endl; }

        ////////////////////////////////////////////////////////////////////////
        template<class T>
            bool project( const unsigned int nNumPoints,
                          const T* pPoints3D,
                          T* pPoints2D,
                          bool bColMajor ) const {
            if( bColMajor ) { 
                return project_col_major( nNumPoints, pPoints3D, pPoints2D );
            }
            else {
                return project_row_major( nNumPoints, pPoints3D, pPoints2D );
            }
        }

        ////////////////////////////////////////////////////////////////////////
        template<class T>
            bool project_row_major( const unsigned int nNumPoints,
                                    const T* pPoints3D,
                                    T* pPoints2D ) const {
            for( size_t ii=0; ii<nNumPoints; ii++ ) {
                const T divZ = 1/pPoints3D[ ii + 2*nNumPoints ];
                double dx = pPoints3D[ ii ]*divZ;
                double dy = pPoints3D[ ii + nNumPoints ]*divZ;
                double dNorm  = sqrt( dx*dx + dy*dy );
                //double dNormD = 1/m_dO * atan( dNorm * m_dO );
                double dNormD = 1/m_dO * atan( dNorm * 2*tan( m_dO/2 ) );
                double dRatio = dNormD/dNorm;
                pPoints2D[ ii     ] = 
                    ( m_dFx*dRatio*pPoints3D[ ii ] + m_dCx*pPoints3D[ ii + 2*nNumPoints ] )*divZ;
                pPoints2D[ ii + nNumPoints ] = 
                    ( m_dFy*dRatio*pPoints3D[ ii + nNumPoints ] + m_dCy*pPoints3D[ ii + 2*nNumPoints ])*divZ;
            }
            return true;
        }

        ////////////////////////////////////////////////////////////////////////
        template<class T>
            bool project_col_major( const unsigned int nNumPoints,
                                    const T* pPoints3D,
                                    T* pPoints2D ) const {
            for( size_t ii=0; ii<nNumPoints; ii++ ) {
                const T divZ = 1/pPoints3D[ 3*ii + 2 ];
                double dx = pPoints3D[ ii ]*divZ;
                double dy = pPoints3D[ ii + 1 ]*divZ;
                double dNorm  = sqrt( dx*dx + dy*dy );
                double dNormD = 1/m_dO * atan( dNorm * m_dO );
                //double dNormD = 1/m_dO * atan( dNorm * 2*tan( m_dO/2 ) );
                double dRatio = dNormD/dNorm;
                pPoints2D[ 2*ii     ] = 
                    ( m_dFx*dRatio*pPoints3D[ 3*ii ] + m_dCx*pPoints3D[ 3*ii + 2 ] )*divZ;
                pPoints2D[ 2*ii + 1 ] = 
                    ( m_dFy*dRatio*pPoints3D[ 3*ii + 1 ] + m_dCy*pPoints3D[ 3*ii + 2 ])*divZ;
            }
            return true;
        }

        ////////////////////////////////////////////////////////////////////////
        template<class T>
            bool project( const unsigned int, const T*, T*, T*, bool ) const {
            return false;
        }

        ////////////////////////////////////////////////////////////////////////
        template<class T>
            bool project( const unsigned int, const T*, T*, T*, T*, bool ) const {
            return false;
        }

        ////////////////////////////////////////////////////////////////////////
        bool project( const unsigned int nNumPoints,
                      const double* pPoints3D,
                      double* pPoints2D,
                      bool bColMajor
                      ) const {
            return project<double>( nNumPoints, pPoints3D, pPoints2D, bColMajor );
        }

        ////////////////////////////////////////////////////////////////////////
        bool project( const unsigned int nNumPoints,
                      const float* pPoints3D,
                      float* pPoints2D,
                      bool bColMajor
                      ) const {
            return project<float>( nNumPoints, pPoints3D, pPoints2D, bColMajor );
        }

        ////////////////////////////////////////////////////////////////////////
        bool project( const unsigned int nNumPoints,
                      const double* pPoints3D,
                      double* pPoints2D,
                      double* pd2d3d,
                      bool bColMajor
                      ) const {
            return project<double>( nNumPoints, pPoints3D, pPoints2D,
                                    pd2d3d, bColMajor );
        }

        ////////////////////////////////////////////////////////////////////////
        bool project( const unsigned int nNumPoints,
                      const float* pPoints3D,
                      float* pPoints2D,
                      float* pd2d3d,
                      bool bColMajor
                      ) const {
            return project<float>( nNumPoints, pPoints3D, pPoints2D,
                                   pd2d3d, bColMajor );
        }

        ////////////////////////////////////////////////////////////////////////
        bool project( const unsigned int nNumPoints,
                      const double* pPoints3D,
                      double* pPoints2D,
                      double* pd2d3d,
                      double* pd2dParam,
                      bool bColMajor
                      ) const {
            return project<double>( nNumPoints, pPoints3D, pPoints2D,
                                    pd2d3d, pd2dParam, bColMajor );
        }

        ////////////////////////////////////////////////////////////////////////
        bool project( const unsigned int nNumPoints,
                      const float* pPoints3D,
                      float* pPoints2D,
                      float* pd2d3d,
                      float* pd2dParam,
                      bool bColMajor
                      ) const {
            return project<float>( nNumPoints, pPoints3D, pPoints2D,
                                   pd2d3d, pd2dParam, bColMajor );
        }

        ////////////////////////////////////////////////////////////////////////
        bool lift( const unsigned int /*nNumPoints*/, 
                   const double* /*pPoints2D*/,
                   double* /*pPoints3D*/,
                   bool ) const {
            return false;
        }

        ////////////////////////////////////////////////////////////////////////
        bool initialise_parameters( const int nImageWidth, const int nImageHeight ) {
            m_dFx = nImageWidth; m_dFy = nImageWidth;
            m_dCx = nImageWidth/2; m_dCy = nImageHeight/2;
            m_dO = 0.01;
            return true;
        }

        ////////////////////////////////////////////////////////////////////////
        int get_num_parameters() const { return ARCTAN_NUM_PARAMS; }

        ////////////////////////////////////////////////////////////////////////
        int name_to_parameter_index( const std::string& sName ) const {
            if( sName == "fx" ) { return 0; }
            if( sName == "fy" ) { return 1; }
            if( sName == "cx" ) { return 2; }
            if( sName == "cy" ) { return 3; }
            if( sName == "o" ) { return 4; }
            return -1;
        }

        ////////////////////////////////////////////////////////////////////////
        std::string parameter_index_to_name( const int nIndex ) const {
            if( nIndex == 0 ) { return "fx"; }
            if( nIndex == 1 ) { return "fy"; }
            if( nIndex == 2 ) { return "cx"; }
            if( nIndex == 3 ) { return "cy"; }
            if( nIndex == 4 ) { return "o"; }
            return "";
        }

        ////////////////////////////////////////////////////////////////////////
        std::string get( const std::string& sKey ) const { 
            if( sKey == "fx" ) { return ToString<double>( m_dFx ); }
            if( sKey == "fy" ) { return ToString<double>( m_dFy ); }
            if( sKey == "cx" ) { return ToString<double>( m_dCx ); }
            if( sKey == "cy" ) { return ToString<double>( m_dCy ); }
            if( sKey == "o" ) { return ToString<double>( m_dO ); }
            return "";
        }

        ////////////////////////////////////////////////////////////////////////
        bool set( const std::string& sKey, const std::string& sParameter ) {  
            if( sKey == "fx" ) {
                m_dFx = FromString<double>( sParameter );
                return true;
            }
            if( sKey == "fy" ) {
                m_dFy = FromString<double>( sParameter );
                return true;
            }
            if( sKey == "cx" ) {
                m_dCx = FromString<double>( sParameter );
                return true;
            }
            if( sKey == "cy" ) {
                m_dCy = FromString<double>( sParameter );
                return true;
            }
            if( sKey == "o" ) {
                m_dO = FromString<double>( sParameter );
                return true;
            }
            return false; 
        }

        ////////////////////////////////////////////////////////////////////////
         void info() const { 
            std::cout << "Pinhole camera, contains four parameters: fx, fy, cx, cy, o." << std::endl; 
        }

    private:
         double m_dFx, m_dFy, m_dCx, m_dCy, m_dO;
    };
}

#endif
