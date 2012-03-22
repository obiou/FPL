#include <fstream>
#include <iostream>
#include <numeric>
#include <stdio.h>

#include <ceigen.h>
#include <cgeom/CGeom.h>
#include <cgeom/generate_scene.h>
#include <cgeom/objpose.h>
#include <cgeom/objpose_weighted.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>

#include <Misc.h>

using namespace std;
using namespace CEIGEN;
using namespace CGEOM;

////////////////////////////////////////////////////////////////////////////////
int main( int /*argv*/, char** /*argc*/ ) {
    int nSeed = 0;
    srand( nSeed );

    // Define simulation parameters
    SceneGeneratorOptions sc_opts;
    sc_opts.dMinZ = 3.;
    sc_opts.dMaxZ = 3.;

    // Generate random image points
    Eigen::MatrixXd mP3D( 3, sc_opts.nNumPoints );
    Eigen::MatrixXd mMeasT( 2, sc_opts.nNumPoints );
    Eigen::MatrixXd mMeasN( 2, sc_opts.nNumPoints );

    Eigen::Vector3d mt;
    Eigen::Matrix3d mR;

    if( !generate_scene_trans( sc_opts, mP3D, mMeasT, mMeasN,
                               mR, mt ) ) {
        cerr << "Problem generating scene" << endl;
        return -1;
    }

#if 0
    PRINT_MATRIX( mP3D );
    PRINT_MATRIX( mMeasT );
    PRINT_MATRIX( mMeasN );
#endif

#if 1
    PRINT_MATRIX( mR );    
    PRINT_MATRIX( mt );
    cout << endl;
#endif

    Eigen::Matrix3d mKinv = sc_opts.MakeCameraMatrix().inverse();
    Eigen::MatrixXd mMeasNP; // normalised plane
#if 1
    mMeasNP = 
         CEIGEN::metric( mKinv * CEIGEN::projective( mMeasN ) );
#else
    mMeasNP = 
        CEIGEN::metric( mKinv * CEIGEN::projective( mMeasT ) );
#endif

    Eigen::MatrixXd mM = mR * mP3D;
    mM.colwise() += mt;
    mM = metric( mM );

#if 0
    cout << endl;
    cout << endl;
    PRINT_MATRIX( mM );
    PRINT_MATRIX( mMeasNP );
    cout << endl;
    cout << endl;
#endif

    const int nMaxNumIters = 30;
    const double dTol      = 1e-5;
    const double dEpsilon  = 1e-8;
    Eigen::Matrix3d mREst = Eigen::Matrix3d::Identity();
    //Eigen::Matrix3d mREst = mR;
    Eigen::Vector3d vtEst = Eigen::Vector3d::Zero();
    int nNumIterations = 0;
    double dObjError;


    //double d0 = CMISC::Tic();
#if 0
    CGEOM::objpose( mP3D, mMeasNP,
                    nMaxNumIters, dTol, dEpsilon,
                    mREst, vtEst, nNumIterations, dObjError, true );
#else
    const double dExpectedNoiseStd = 2./800.;
    vector<bool> vInliers( mP3D.cols(), false );

    CGEOM::objpose_weighted<CGEOM::TukeyWeights>
        ( mP3D, mMeasNP,
          nMaxNumIters, dTol, dEpsilon,
          dExpectedNoiseStd,
          mREst, vtEst, nNumIterations, dObjError, 
          vInliers, true, false );
    cout << "Number of inliers: " << 
        accumulate( vInliers.begin(), vInliers.end(), int(0) ) << endl;
#endif
    //cout << "Time (ms): " << CMISC::TocMS( d0 ) << endl;

    PRINT_MATRIX( mREst );    
    PRINT_MATRIX( vtEst );

    cout << "nNumIterations: " << nNumIterations << endl;
    cout << "dObjError: " << dObjError << endl;

    ofstream f2d( "m2d_c.txt" );
    if( f2d.good() ) {
        f2d << mMeasNP << endl;
    }
    f2d.close();

    ofstream f3d( "p3d_c.txt" );
    if( f3d.good() ) {
        f3d << mP3D << endl;
    }
    f3d.close();

    return 0;
}
