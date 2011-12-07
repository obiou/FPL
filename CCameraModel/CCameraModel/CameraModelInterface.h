#ifndef CCAMERA_MODEL_INTERFACE_H
#define CCAMERA_MODEL_INTERFACE_H

#define GENERIC_FACTORY_NAMESPACE CFactory
#include <CCameraModel/factory.h>

#include <string>
#include <vector>

///
/// This defines the basic interface implemented by the programmer.
/// The used uses CameraModel directly that implements this interface
/// and adds extra functionality based on the interface functions.
///
/// (@see PinholeCamera for an example).
///
/// Macros are used to clean up the code.
namespace CCameraModel {
    ////////////////////////////////////////////////////////////////////////////
    FACTORY_INTERFACE( CameraModelInterface ) {
    public:
        virtual ~CameraModelInterface() {}
        virtual bool project( const unsigned int nNumPoints,
                              const double* pPoints3D,
                              double* pPoints2D,  ///< Output: 2xnNumPoints, points projected in image
                              bool bColMajor
                              ) const = 0;

        virtual bool project( const unsigned int nNumPoints,
                              const float* pPoints3D,
                              float* pPoints2D,   ///< Output: 2xnNumPoints, points projected in image
                              bool bColMajor
                              ) const = 0;

        virtual bool project( const unsigned int nNumPoints,
                              const double* pPoints3D,
                              double* pPoints2D, ///< Output: points projected in image
                              double* pd2d3d,    ///< Output: 2x(3xnNumPoints) Jacobian of image points with respect to 3D points
                              bool bColMajor
                              ) const = 0;

        virtual bool project( const unsigned int nNumPoints,
                              const double* pPoints3D,
                              double* pPoints2D, ///< Output: points projected in image
                              double* pd2d3d,    ///< Output: 2x(3xnNumPoints) Jacobian of image points with respect to 3D points
                              double* pd2dParam, ///< Output: 2x(NxnNumPoints) Jacobian of image points with respect to the N camera parameters, N = get_num_parameters();
                              bool bColMajor
                              ) const = 0;

        virtual bool project( const unsigned int nNumPoints,
                              const float* pPoints3D,
                              float* pPoints2D, ///< Output: points projected in image
                              float* pd2d3d,    ///< Output: 2x(3xnNumPoints) Jacobian of image points with respect to 3D points
                              bool bColMajor
                              ) const = 0;

        virtual bool project( const unsigned int nNumPoints,
                              const float* pPoints3D,
                              float* pPoints2D, ///< Output: points projected in image
                              float* pd2d3d,    ///< Output: 2x(3xnNumPoints) Jacobian of image points with respect to 3D points
                              float* pd2dParam, ///< Output: 2x(NxnNumPoints) Jacobian of image points with respect to the N camera parameters, N = get_num_parameters();
                              bool bColMajor
                              ) const = 0;

        virtual bool lift( const unsigned int /*nNumPoints*/, 
                           const double* /*pPoints2D*/,
                           double* /*pPoints3D*/,
                           bool /*bColMajor*/
                           ) const { return false; }

        virtual bool lift( const unsigned int /*nNumPoints*/, 
                           const float* /*pPoints2D*/,
                           float* /*pPoints3D*/,
                           bool /*bColMajor*/
                           ) const { return false; }

        /// Computes a rectified camera (i.e. simplified in some way, for a perspective
        /// camera this could mean computing a camera without distortion)
        virtual CameraModelInterface* new_rectified_camera( const int /*nImageWidth*/, const int /*nImageHeight*/,
                                                            const std::vector<double>* /*pvParams*/ = NULL ) {
            std::cerr << "ERROR: Rectification function not implemented" << std::endl;
            return NULL;
        } 

        virtual bool initialise_parameters( const int nImageWidth, const int nImageHeight ) = 0;
        virtual int get_num_parameters() const = 0;
        virtual int name_to_parameter_index( const std::string& sName ) const = 0;
        virtual std::string parameter_index_to_name( const int nIndex ) const = 0;

        virtual std::string get( const std::string& sKey ) const = 0;
        virtual bool set( const std::string& sKey, const std::string& sParameter ) = 0;

        /// Give information about the current camera
        /// (not static as it makes it difficult to change the behaviour of meta-models)
        virtual void info() const = 0;

        ////////////////////////////////////////////////////////////////////////
        /// Stream output interface
        /// A default output function is provided with the interface.
        friend std::ostream& operator<<( std::ostream& oStream, const CameraModelInterface& cameraModel ) {
            cameraModel.ToStream( oStream );
            return oStream;
        }

        ////////////////////////////////////////////////////////////////////////
        /// Stream input interface
        /// A default input function is provided with the interface.
        friend std::istream& operator>>( std::istream& iStream, CameraModelInterface& cameraModel ) {
            cameraModel.FromStream( iStream );
            return iStream;
        }

        ////////////////////////////////////////////////////////////////////////
        virtual std::string get_camera_type() const { return m_sCameraType; }

        ////////////////////////////////////////////////////////////////////////
        virtual void set_camera_type( const std::string& sCameraType ) { m_sCameraType = sCameraType; }

    private:
        virtual void ToStream( std::ostream& ) const;
        virtual void FromStream( std::istream& );    

        std::string m_sCameraType;
    };
    
    ////////////////////////////////////////////////////////////////////////////
    /// Function can be used to reorder buffers to only code col- or row-major versions
    /// of functions.
    template<class T>
        void transpose( T* pData, const int nWidth, const int nHeight );
}

#endif
