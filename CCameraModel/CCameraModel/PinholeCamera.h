#ifndef PINHOLE_CAMERA_H
#define PINHOLE_CAMERA_H

#include <iostream>
#include <string>

///
/// This defines the basic call made by users.
/// The class is in fact a factory, it standardises the calls by implementing the CameraModelInterface.h
///
/// To avoid complicating the code, the users 
namespace CCameraModel {
    const int PINHOLE_NUM_PARAMS = 4;

    ////////////////////////////////////////////////////////////////////////////
    /// Specific camera: Pinhole model
    FACTORY_OBJECT( PinholeCamera, CameraModelInterface, "Pinhole" ) {
    public:
        ////////////////////////////////////////////////////////////////////////
    PinholeCamera() : m_dFx(0), m_dFy(0), m_dCx(0), m_dCy(0) {}

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
        template<class T>
            bool project( const unsigned int nNumPoints,
                          const T* pPoints3D,
                          T* pPoints2D,
                          bool bColMajor ) const {
            if( bColMajor ) { 
                return project_col_major<T,false,false>
                    ( nNumPoints, pPoints3D, pPoints2D, NULL, NULL );
            }
            else {
                return project_row_major<T,false,false>
                    ( nNumPoints, pPoints3D, pPoints2D, NULL, NULL );
            }
        }
        
        ////////////////////////////////////////////////////////////////////////
        template<class T>
            bool project( const unsigned int nNumPoints,
                          const T* pPoints3D,
                          T* pPoints2D,
                          T* pd2d3d,
                          bool bColMajor ) const {
            if( bColMajor ) { 
                return project_col_major<T,true,false>
                    ( nNumPoints, pPoints3D, pPoints2D,
                      pd2d3d, NULL );
            }
            else {
                return project_row_major<T,true,false>
                    ( nNumPoints, pPoints3D, pPoints2D,
                      pd2d3d, NULL );
            }
        }

        ////////////////////////////////////////////////////////////////////////
        template<class T>
            bool project( const unsigned int nNumPoints,
                          const T* pPoints3D,
                          T* pPoints2D,
                          T* pd2d3d,
                          T* pd2dParam,
                          bool bColMajor ) const {
            if( bColMajor ) { 
                return project_col_major<T,true,true>
                    ( nNumPoints, pPoints3D, pPoints2D,
                      pd2d3d, pd2dParam );
            }
            else {
                return project_row_major<T,true,true>
                    ( nNumPoints, pPoints3D, pPoints2D,
                      pd2d3d, pd2dParam );
            }
        }

        ////////////////////////////////////////////////////////////////////////
        template<class T, bool COMPUTE_JAC_E, bool COMPUTE_JAC_I>
            bool project_col_major( const unsigned int nNumPoints,
                                    const T* pPoints3D,
                                    T* pPoints2D,
                                    T* pd2d3d,
                                    T* pd2dParam ) const {
            const int nNumPoseParams = 6;
            const int nNumCamParams = 2*PINHOLE_NUM_PARAMS;
            for( size_t ii=0; ii<nNumPoints; ii++ ) {
                const T divZ = 1/pPoints3D[ 3*ii + 2 ];
                pPoints2D[ 2*ii     ] = 
                    ( m_dFx*pPoints3D[ 3*ii ] + m_dCx*pPoints3D[ 3*ii + 2 ] )*divZ;
                pPoints2D[ 2*ii + 1 ] = 
                    ( m_dFy*pPoints3D[ 3*ii + 1 ] + m_dCy*pPoints3D[ 3*ii + 2 ])*divZ;
                if( COMPUTE_JAC_E ) {
                    // Compute 2x3=6 values dudxyz and then dvdxyz
                    pd2d3d[ nNumPoseParams*ii     ] = m_dFx*divZ;                
                    pd2d3d[ nNumPoseParams*ii + 1 ] = 0;
                    pd2d3d[ nNumPoseParams*ii + 2 ] = 0;
                    pd2d3d[ nNumPoseParams*ii + 3 ] = m_dFy*divZ;                
                    pd2d3d[ nNumPoseParams*ii + 4 ] = -m_dFx*pPoints3D[ 3*ii ]*divZ*divZ;
                    pd2d3d[ nNumPoseParams*ii + 5 ] = -m_dFy*pPoints3D[ 3*ii + 1 ]*divZ*divZ;
                }
                if( COMPUTE_JAC_I ) {
                    // Compute 2xN=2*N (N=4) value dudP1P2...PN and dvdP1P2...PN
                    // P1=fx, P2=fy, P3=cx, P4=cy
                    pd2dParam[ nNumCamParams*ii     ] = pPoints3D[ 3*ii ]*divZ;
                    pd2dParam[ nNumCamParams*ii + 1 ] = 0;
                    pd2dParam[ nNumCamParams*ii + 2 ] = 0;
                    pd2dParam[ nNumCamParams*ii + 3 ] = pPoints3D[ 3*ii + 1 ]*divZ;
                    pd2dParam[ nNumCamParams*ii + 4 ] = 1;
                    pd2dParam[ nNumCamParams*ii + 5 ] = 0;
                    pd2dParam[ nNumCamParams*ii + 6 ] = 0;
                    pd2dParam[ nNumCamParams*ii + 7 ] = 1;
                }
            }
            return true;
        }

        ////////////////////////////////////////////////////////////////////////
        template<class T, bool COMPUTE_JAC_E, bool COMPUTE_JAC_I>
            bool project_row_major( const unsigned int nNumPoints,
                                    const T* pPoints3D,
                                    T* pPoints2D,
                                    T* pd2d3d,
                                    T* pd2dParam ) const {
            for( size_t ii=0; ii<nNumPoints; ii++ ) {
                const T divZ = 1/pPoints3D[ ii + 2*nNumPoints ];
                pPoints2D[ ii     ] = 
                    ( m_dFx*pPoints3D[ ii ] + m_dCx*pPoints3D[ ii + 2*nNumPoints ] )*divZ;
                pPoints2D[ ii + nNumPoints ] = 
                    ( m_dFy*pPoints3D[ ii + nNumPoints ] + m_dCy*pPoints3D[ ii + 2*nNumPoints ])*divZ;
                if( COMPUTE_JAC_E ) {
                    // Compute 2x3=6 values dudxyz and then dvdxyz
                    pd2d3d[ ii                ] = m_dFx*divZ;                
                    pd2d3d[ ii + nNumPoints   ] = 0;
                    pd2d3d[ ii + 2*nNumPoints ] = 0;
                    pd2d3d[ ii + 3*nNumPoints ] = m_dFy*divZ;                
                    pd2d3d[ ii + 4*nNumPoints ] = -m_dFx*pPoints3D[ 3*ii ]*divZ*divZ;
                    pd2d3d[ ii + 5*nNumPoints ] = -m_dFy*pPoints3D[ 3*ii + 1 ]*divZ*divZ;
                }
                if( COMPUTE_JAC_I ) {
                    // Compute 2xN=2*N (N=4) value dudP1P2...PN and dvdP1P2...PN
                    // P1=fx, P2=fy, P3=cx, P4=cy
                    pd2dParam[ ii     ]            = pPoints3D[ ii ]*divZ;
                    pd2dParam[ ii +   nNumPoints ] = 0;
                    pd2dParam[ ii + 2*nNumPoints ] = 0;
                    pd2dParam[ ii + 3*nNumPoints ] = pPoints3D[ ii + nNumPoints ]*divZ;;
                    pd2dParam[ ii + 4*nNumPoints ] = 1;
                    pd2dParam[ ii + 5*nNumPoints ] = 0;
                    pd2dParam[ ii + 6*nNumPoints ] = 0;
                    pd2dParam[ ii + 7*nNumPoints ] = 1;
                }
            }
            return true;
        }

        ////////////////////////////////////////////////////////////////////////
        bool lift( const unsigned int nNumPoints,
                   const double* pPoints2D,
                   double* pPoints3D,
                   bool bColMajor ) const {
            return lift<double>( nNumPoints, pPoints2D, pPoints3D, bColMajor );
        }

        ////////////////////////////////////////////////////////////////////////
        bool lift( const unsigned int nNumPoints,
                   const float* pPoints2D,
                   float* pPoints3D,
                   bool bColMajor ) const {
            return lift<float>( nNumPoints, pPoints2D, pPoints3D, bColMajor );
        }

        ////////////////////////////////////////////////////////////////////////
        template<class T>
            bool lift( const unsigned int nNumPoints,
                       const T* pPoints2D,
                       T* pPoints3D,
                       bool bColMajor ) const {
            if( bColMajor ) { 
                return lift_col_major<T>( nNumPoints, pPoints2D, pPoints3D );
            }
            else {
                return lift_row_major<T>( nNumPoints, pPoints2D, pPoints3D );
            }
        }

        ////////////////////////////////////////////////////////////////////////
        template<class T>
            bool lift_col_major( const unsigned int nNumPoints,
                                 const T* pPoints2D,
                                 T* pPoints3D ) const {
            const T divFx = 1/m_dFx;
            const T divFy = 1/m_dFy;
            for( size_t ii=0; ii<nNumPoints; ii++ ) {
                const T liftX = ( pPoints2D[ 2*ii ]     - m_dCx )*divFx;
                const T liftY = ( pPoints2D[ 2*ii + 1 ] - m_dCy )*divFy;
                const T divNorm  = 1/sqrt( liftX*liftX + liftY*liftY + 1 );
                pPoints3D[ 3*ii     ] = liftX * divNorm;
                pPoints3D[ 3*ii + 1 ] = liftY * divNorm;;
                pPoints3D[ 3*ii + 2 ] =         divNorm;
            }
            return true;
        }

        ////////////////////////////////////////////////////////////////////////
        template<class T>
            bool lift_row_major( const unsigned int nNumPoints,
                                 const T* pPoints2D,
                                 T* pPoints3D ) const {
            const T divFx = 1/m_dFx;
            const T divFy = 1/m_dFy;
            for( size_t ii=0; ii<nNumPoints; ii++ ) {
                const T liftX = ( pPoints2D[ ii ]              - m_dCx )*divFx;
                const T liftY = ( pPoints2D[ ii + nNumPoints ] - m_dCy )*divFy;
                const T divNorm  = 1/sqrt( liftX*liftX + liftY*liftY + 1 );
                pPoints3D[ ii                ] = liftX * divNorm;
                pPoints3D[ ii + nNumPoints   ] = liftY * divNorm;;
                pPoints3D[ ii + 2*nNumPoints ] =         divNorm;
            }
            return true;
        }

        ////////////////////////////////////////////////////////////////////////
        bool initialise_parameters( const int nImageWidth, const int nImageHeight ) {
            m_dFx = nImageWidth; m_dFy = nImageWidth;
            m_dCx = nImageWidth/2; m_dCy = nImageHeight/2;
            return true;
        }

        ////////////////////////////////////////////////////////////////////////
        int get_num_parameters() const { return PINHOLE_NUM_PARAMS; }

        ////////////////////////////////////////////////////////////////////////
        int name_to_parameter_index( const std::string& sName ) const {
            if( sName == "fx" ) { return 0; }
            if( sName == "fy" ) { return 1; }
            if( sName == "cx" ) { return 2; }
            if( sName == "cy" ) { return 3; }
            return -1;
        }

        ////////////////////////////////////////////////////////////////////////
        std::string parameter_index_to_name( const int nIndex ) const {
            if( nIndex == 0 ) { return "fx"; }
            if( nIndex == 1 ) { return "fy"; }
            if( nIndex == 2 ) { return "cx"; }
            if( nIndex == 3 ) { return "cy"; }
            return "";
        }

        ////////////////////////////////////////////////////////////////////////
        std::string get( const std::string& sKey ) const { 
            if( sKey == "fx" ) { return ToString<double>( m_dFx ); }
            if( sKey == "fy" ) { return ToString<double>( m_dFy ); }
            if( sKey == "cx" ) { return ToString<double>( m_dCx ); }
            if( sKey == "cy" ) { return ToString<double>( m_dCy ); }
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
            return false; 
        }

        ////////////////////////////////////////////////////////////////////////
         void info() const { 
            std::cout << "Pinhole camera, contains four parameters: fx, fy, cx, cy." << std::endl; 
        }

    private:
        double m_dFx, m_dFy, m_dCx, m_dCy;
    };
}

#endif
