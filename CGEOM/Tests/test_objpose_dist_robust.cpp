#include <cgeom/objpose_weighted.h>
#include <ceigen.h>
#include <Misc.h>

#include <fstream>
#include <iostream>
#include <numeric>
#include <queue>
#include <sstream>
#include <string>

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Cholesky>

#include <eigen3/Eigen/Cholesky>
#include <Eigen/LU>
#include <eigen3/Eigen/SVD>

#include <CGEOMIncludes.h>

using namespace Eigen;
using namespace std;

using namespace CEIGEN;

////////////////////////////////////////////////////////////////////////////////
int main() {
    MatrixXd mP3D;
    MatrixXd mM2D;
    const char* sP3D = CGEOM_SOURCE_DIR"/Tests/p3d.txt";
    const char* sM2D = CGEOM_SOURCE_DIR"/Tests/m2d.txt";
    fstream fs1( sP3D, fstream::in );
    if( fs1.good() ) {
        fs1 >> mP3D;
    }
    else {
        cerr << "ERROR reading data from: " << sP3D << endl;
        return -1;
    }
    fstream fs2( sM2D, fstream::in );
    if( fs2.good() ) {
        fs2 >> mM2D;
    }
    else {
        cerr << "ERROR reading data from: " << sM2D << endl;
        return -1;
    }
    if( mM2D.rows() == 2 ) {
        mM2D = CEIGEN::projective( mM2D );
    }

    if( mM2D.rows() != 3 ) {
        cerr << "Wrong number of rows, expecting 3, got: " << mM2D.rows() << endl;
        return -1;
    }

    //cout << "mP3D: " << endl << mP3D << endl;

    const int nMaxNumIters = 50;
    const double dTol      = 1e-5;
    const double dEpsilon  = 1e-8;
    const double dExpectedNoiseStd = 2./800.;
    Eigen::Matrix3d mR = Eigen::Matrix3d::Identity();
    Eigen::Vector3d vt = Eigen::Vector3d::Zero();
    int nNumIterations = 0;
    double dObjError;

    // Add outliers
    const int nNumOuliers = 5;
    Eigen::Vector3d mAdd = 10 * Eigen::VectorXd::Ones(3);
    mAdd(2) = 0;
    for( int ii=0; ii<nNumOuliers; ii++ ) {
        mM2D.col(ii) = mM2D.col(ii) + mAdd;
    }
    //cout << "mM2D: " << endl << mM2D << endl;

    vector<bool> vInliers( mP3D.cols(), false );
    cout << "Num points: " << mP3D.cols() << endl;
    double d0 = CMISC::Tic();
    CGEOM::objpose_weighted<CGEOM::TukeyInvSqDistWeights>
        ( mP3D, mM2D,
          nMaxNumIters, dTol, dEpsilon,
          dExpectedNoiseStd,
          mR, vt, nNumIterations, dObjError, 
          vInliers, true, false );
    cout << "Time (ms): " << CMISC::TocMS( d0 ) << endl;
    cout << "Number of inliers: " << 
        accumulate( vInliers.begin(), vInliers.end(), int(0) ) << endl;
    cout << mR << endl;
    cout << vt << endl;
    cout << "nNumIterations: " << nNumIterations << endl;
    cout << "dObjError: " << dObjError << endl;

    return (dObjError < 0.1) ? 0 : -1;
}
