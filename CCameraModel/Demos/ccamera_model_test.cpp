#include <stdio.h>

#include <algorithm>
#include <iostream>
#include <iterator>
#include <map>
#include <sstream>
#include <string>

#include <CameraModel.h>

using namespace CCameraModel;
using namespace std;

////////////////////////////////////////////////////////////////////////////////
int main()
{
    //CameraModel cameraModel( "Pinhole" );
    CameraModel cameraModel( "PinholeRadTan" );
    cameraModel.set( "fx", 10 );
    cameraModel.set( "fy", 8 );
    cameraModel.set( "cx", 120 );
    cameraModel.set( "cy", 100 );
    cameraModel.set( "k1", .5 );

    const unsigned int nNumPoints = 1;
    double P[3]; P[0] = 1; P[1] = 2; P[2] = 3;
    double M[2];

    cameraModel.project( nNumPoints, P, M );
    cameraModel.lift( nNumPoints, M, P );
    cameraModel.info();

    cout << "Camera model: " << endl;
    cout << cameraModel << endl;
    std::stringstream sCamera;
    sCamera << cameraModel;

    cout << "Change, print, re-load and print: " << endl;
    cameraModel.set( "fx", 0 );
    cout << cameraModel << endl;
    sCamera >> cameraModel;
    cout << cameraModel << endl;

    cameraModel.save( "test_save.txt" );

    cout << "Testing generic loading." << endl;

    CameraModel cameraModel2;
    cameraModel2.load( "test_save.txt" );
    cout << cameraModel2 << endl;
    cameraModel2.save( "test_save2.txt" );

    return 0;
}
