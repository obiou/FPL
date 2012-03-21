#include <cgeom/CGeom.h>
#include <ceigen.h>

#include <fstream>
#include <iostream>
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
    //cout << "mM2D: " << endl << mM2D << endl;

    const int nMaxNumIters = 20;
    const double dTol      = 1e-5;
    const double dEpsilon  = 1e-8;
    Eigen::Matrix3d mR = Eigen::Matrix3d::Identity();
    Eigen::Vector3d vt = Eigen::Vector3d::Zero();
    int nNumIterations = 0;
#if 0
    double dObjError;
    CGEOM::objpose( mP3D, mM2D,
                    nMaxNumIters, dTol, dEpsilon,
                    mR, vt, nNumIterations, dObjError, false );
    cout << "dObjError: " << dObjError << endl;
    cout << "From objpose, imgpose: " << endl;
#endif
    double dImgError;
    vt(2) = 2.5;
    CGEOM::imgpose( mP3D, mM2D,
                    nMaxNumIters, dTol, dEpsilon,
                    mR, vt, nNumIterations, dImgError, true );
    cout << mR << endl;
    cout << vt << endl;
    cout << "nNumIterations: " << nNumIterations << endl;
    cout << "dImgError: " << dImgError << endl;

    return (dImgError < 0.1) ? 0 : -1;
}
