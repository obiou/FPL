#include <cgeom/generate_scene.h>

#include <ceigen.h>
#include <cgeom/CGeom.h>
#include <cgeom/objpose.h>

#include <eigen3/Eigen/Core>
#include <eigen3/unsupported/Eigen/MatrixFunctions>

#include <stdlib.h>

////////////////////////////////////////////////////////////////////////////////
// Return a random integer between two integer values (comprised)
int rand_range_i( int nMin, int nMax ) {
    const double nIntRange = nMax - nMin + 1;
    int nRandVal = rand()*nIntRange / RAND_MAX;
    nRandVal += nMin;
    if( nRandVal == nMax + 1 ) {
        --nRandVal;
    }                       
    return nRandVal;
}

////////////////////////////////////////////////////////////////////////////////
// Return a random double between two double values (comprised)
double rand_range_d( double dMin, double dMax ) {
    const double dRange = dMax - dMin;
    double dRandVal = rand()*dRange / double(RAND_MAX);
    dRandVal += dMin;
    return dRandVal;
}

////////////////////////////////////////////////////////////////////////////////
static bool in_image( const Eigen::Vector2d& mM, 
                      const int nImageWidth, 
                      const int nImageHeight ) {
    return mM(0) >= 0 && mM(1) >= 0 &&
        mM(0) < nImageWidth && mM(1) < nImageHeight;
}

////////////////////////////////////////////////////////////////////////////////
static double randd() {
    return double(rand())/double(RAND_MAX);
}

////////////////////////////////////////////////////////////////////////////
static void rand_gaussd( double& y1, double& y2 ) {
    double x1, x2, w;
    do {
        x1 = 2.0 * randd() - 1.0;
        x2 = 2.0 * randd() - 1.0;
        w = x1 * x1 + x2 * x2;
    } while ( w >= 1.0 );
    
    w = sqrt( (-2.0 * log( w ) ) / w );
    y1 = x1 * w;
    y2 = x2 * w;        
}

////////////////////////////////////////////////////////////////////////////
static void rand_gaussd( double dMean, double dStd, double& y1, double& y2 ) {
    rand_gaussd( y1, y2 );
    y1 = dMean + y1*dStd;
    y2 = dMean + y2*dStd;
}

////////////////////////////////////////////////////////////////////////////////
bool CGEOM::generate_scene( const SceneGeneratorOptions& sc_opts,
                            Eigen::MatrixXd& mP3D,
                            Eigen::MatrixXd& mMeasT,
                            Eigen::MatrixXd& mMeasN
                            ) {
    const int nNumPoints = sc_opts.nNumPoints;
    Eigen::Matrix3d mK = sc_opts.MakeCameraMatrix();
    //PRINT_MATRIX( mK );

    mP3D.resize( 3, nNumPoints );
    mMeasT.resize( 2, nNumPoints );
    mMeasN.resize( 2, nNumPoints );
 
    // Generate random image points
    int nNumGenPoints = 0;
    Eigen::Vector3d mP;
    Eigen::Vector2d mMT, mMN;
    const int nMaxNumIter = 100*nNumPoints;
    int nNumIter = 0;
    while( nNumGenPoints < nNumPoints && ++nNumIter <= nMaxNumIter ) {
        mP << rand_range_d( sc_opts.dMinX, sc_opts.dMaxX ),
            rand_range_d( sc_opts.dMinY, sc_opts.dMaxY ),
            rand_range_d( sc_opts.dMinZ, sc_opts.dMaxZ );
        // Project to image
        mMT = CEIGEN::metric( mK * mP );
        // Add noise
        rand_gaussd( 0., sc_opts.dNoise, mMN(0), mMN(1) );
        mMN += mMT;
        if( in_image( mMT, sc_opts.nImageWidth, sc_opts.nImageHeight ) &&
            in_image( mMN, sc_opts.nImageWidth, sc_opts.nImageHeight ) ) {
            mP3D.col( nNumGenPoints ) = mP;
            mMeasT.col( nNumGenPoints ) = mMT;
            mMeasN.col( nNumGenPoints ) = mMN;
            nNumGenPoints++;
        }
    }
    return nNumIter != nMaxNumIter;
}

////////////////////////////////////////////////////////////////////////////////
bool CGEOM::generate_scene_trans
( const SceneGeneratorOptions& sc_opts,
  Eigen::MatrixXd& mP3D,
  Eigen::MatrixXd& mMeasT,
  Eigen::MatrixXd& mMeasN,
  Eigen::Matrix3d& mR,
  Eigen::Vector3d& mt
  ) {
    if( !generate_scene( sc_opts,
                         mP3D, mMeasT, mMeasN
                         ) ) {
        return false;
    }

    // Create random transform
    mt = mP3D.rowwise().mean();
    const double drotx = rand_range_d( -45., 45. )*3.14159/180.; 
    const double droty = rand_range_d( -45., 45. )*3.14159/180.; 
    const double drotz = rand_range_d( -45., 45. )*3.14159/180.; 
#if 1
    mR = 
        ( CEIGEN::skew_rot<Eigen::Matrix3d>( drotx, 0., 0. ) +
          CEIGEN::skew_rot<Eigen::Matrix3d>( 0., droty , 0.) +
          CEIGEN::skew_rot<Eigen::Matrix3d>( 0., 0., drotz ) ).exp();
#else
    mR = Eigen::Matrix3d::Identity();
#endif

    mP3D.colwise() -= mt;
    mP3D = mR.transpose() * mP3D;

    return true;
}
