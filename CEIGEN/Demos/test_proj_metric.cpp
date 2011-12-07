#include <iostream>
#include <Eigen/Core>

#include <ceigen/ceigen_basics.h>

using namespace std;
using namespace Eigen;
using namespace CEIGEN;

////////////////////////////////////////////////////////////////////////////////
int main() {
    MatrixXf mC1f = MatrixXf::Random(2,4);
    MatrixXd mC1d = MatrixXd::Random(2,4);
    Matrix<float,2,4> mC2f = Matrix<float,2,4>::Random();
    Matrix<double,2,4> mC2d = Matrix<double,2,4>::Random();

    Matrix<double,Dynamic,Dynamic,RowMajor> mC3d = Matrix<double,Dynamic,Dynamic,RowMajor>::Random(2,4);
    Matrix<double,2,4,RowMajor> mC4d = Matrix<double,2,4,RowMajor>::Random();

    Matrix3f mHf = Matrix3f::Identity();
    MatrixXf mHf_dyn = MatrixXf::Identity(3,3);
    Matrix3d mHd = Matrix3d::Identity();
    MatrixXd mHd_dyn = MatrixXd::Identity(3,3);

    cout << "mC1f" << endl << mC1f << endl;
    cout << "mp mC1f" << endl << metric( projective( mC1f ) ) << endl;
    cout << "Id mp mC1f" << endl << metric( mHf*projective( mC1f ) ) << endl;
    cout << "Id_dyn mp mC1f" << endl << metric( mHf_dyn*projective( mC1f ) ) << endl;
    cout << endl;
    cout << "mC1d" << endl << mC1d << endl;
    cout << "mp mC1d" << endl << metric( projective( mC1d ) ) << endl;
    cout << "Id mpmC1d" << endl << metric( mHd*projective( mC1d ) ) << endl;
    cout << "Id_dyn mpmC1d" << endl << metric( mHd_dyn*projective( mC1d ) ) << endl;
    cout << endl;
    cout << "mC2f" << endl << mC2f << endl;
    cout << "mp mC2f" << endl << metric( projective( mC2f ) ) << endl;
    cout << "Id mpmC2f" << endl << metric( mHf*projective( mC2f ) ) << endl;
    cout << endl;
    cout << "mC2d" << endl << mC2d << endl;
    cout << "mp mC2d" << endl << metric( projective( mC2d ) ) << endl;
    cout << "Id mpmC2d" << endl << metric( mHd*projective( mC2d ) ) << endl;
    cout << endl;
    cout << "mC3d" << endl << mC3d << endl;
    cout << "mp mC3d" << endl << metric( projective( mC3d ) ) << endl;
    cout << "Id mpmC3d" << endl << metric( mHd*projective( mC3d ) ) << endl;
    cout << "Id_dyn mp mC3d" << endl << metric( mHd_dyn*projective( mC3d ) ) << endl;
    cout << endl;
    cout << "mC4d" << endl << mC4d << endl;
    cout << "mp mC4d" << endl << metric( projective( mC4d ) ) << endl;
    cout << "Id mp mC4d" << endl << metric( mHd*projective( mC4d ) ) << endl;
    cout << "Id_dyn mp mC4d" << endl << metric( mHd_dyn*projective( mC4d ) ) << endl;
    cout << endl;
 
    return 0;
}
