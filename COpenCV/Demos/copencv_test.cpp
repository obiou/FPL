#include <CameraSensor.h>
#include <COpenCV.h>
#if COPENCV_HAS_CVARS
#  include <CVars/CVar.h>
#endif
#include <Misc.h>

#include <stdio.h>

#include <algorithm>
#include <iostream>
#include <iterator>
#include <map>
#include <string>

using namespace CCameraSensor;
using namespace ImageWrapper;
using namespace std;

////////////////////////////////////////////////////////////////////////////////
int main( int argc, char** argv )
{
    string sCameraType = "OpenCV";
    if( argc > 1 ) {
        sCameraType = argv[1];
    }
    CameraSensor cameraSensor( sCameraType );

#if COPENCV_HAS_CVARS
    //CVars have a name and a default value
    CVarUtils::CreateCVar( "vars.myvar1", 1, "some useful help..." );
    CVarUtils::CreateCVar( "vars.myvar2", 10. );
    CVarUtils::CreateCVar( "vars.var1.myvar1", 1);
    CVarUtils::CreateCVar( "vars.var1.myvar2", 10. );
#endif

    if( !cameraSensor.open() ) {
        cerr << "Problem opening camera sensor." << endl;
        return -1;
    }

    COPENCV::Figure fig1( "UEyeImage1" );
    COPENCV::Figure fig2( "UEyeImage2" );

    char nKey = 0;
    Image aImage = cameraSensor.read();
    if( !aImage.empty() ) {
        std::cout << "SensorID: " << aImage.sSensorID << std::endl;
    }
    while( !aImage.empty() && nKey != 32 ) {
        fig1.imshow( aImage );
        fig1.draw();

        fig2.imshow( aImage );
        fig2.draw();

        nKey = fig1.wait( 10 );
        fig2.wait( 10 );
        //double d0 = CMISC::Tic();
        aImage = cameraSensor.read();
        //cout << "Time taken to acquire: " << CMISC::TocMS( d0 ) << endl;
        if( (char)nKey == 27 ) {
            return 0;
        }
    }

    cameraSensor.close();
    
    return 0;
}
