#ifndef GENERATE_SCENE_H
#define GENERATE_SCENE_H

#include <eigen3/Eigen/Core>

/// Functions used to generate a cloud of points and their
/// projections in the image for testing camera pose estimation
/// algorithms.
namespace CGEOM {
    ////////////////////////////////////////////////////////////////////////////
    class SceneGeneratorOptions {
    public:
        ////////////////////////////////////////////////////////////////////////
    SceneGeneratorOptions() :
        dNoise( 2. ),
            dMinX( -2. ), dMaxX( 2. ),
            dMinY( -2. ), dMaxY( 2. ),
            dMinZ( 3. ), dMaxZ( 3. ),
            dFx( 800. ), dFy( 800. ),
            dCx( 320. ), dCy( 240 ),
            nImageWidth( 640 ), nImageHeight( 480 ),
            nNumPoints( 100 )
                {}

        ////////////////////////////////////////////////////////////////////////
        Eigen::Matrix3d MakeCameraMatrix() const {
            Eigen::Matrix3d mK;
            mK << dFx, 0., dCx,
                0., dFy, dCy,
                0., 0., 1.;
            return mK;
        }

    public:
        double dNoise;
        double dMinX, dMaxX, dMinY, dMaxY, dMinZ, dMaxZ;
        double dFx, dFy, dCx, dCy;
        int nImageWidth, nImageHeight;
        int nNumPoints;
    };


    ////////////////////////////////////////////////////////////////////////////
    bool generate_scene( const SceneGeneratorOptions& sc_opts,
                         Eigen::MatrixXd& mP3D,
                         Eigen::MatrixXd& mMeasT,
                         Eigen::MatrixXd& mMeasN
                         );


    ////////////////////////////////////////////////////////////////////////////
    bool generate_scene_trans
        ( const SceneGeneratorOptions& sc_opts,
          Eigen::MatrixXd& mP3D,
          Eigen::MatrixXd& mMeasT,
          Eigen::MatrixXd& mMeasN,
          Eigen::Matrix3d& mR,
          Eigen::Vector3d& mt
          );
}

#endif
