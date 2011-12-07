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
    CameraModel cameraModel;
    cout << "Info: " << endl;
    cameraModel.info();
    cameraModel.load( "test_save.txt" );
    cout << cameraModel << endl;
    cout << "Info: " << endl;
    cameraModel.info();

    return 0;
}
