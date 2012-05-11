// CCameraSensor - framework for camera drivers
// Copyright (C) 2011 C. Mei
// 
// CCameraSensor is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// CCameraSensor is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
#include <stdio.h>

#include <algorithm>
#include <iostream>
#include <iterator>
#include <map>
#include <string>

#include <CameraSensor.h>
#include <COpenCV.h>
#include <Misc.h>

using namespace CCameraSensor;
using namespace ImageWrapper;
using namespace std;

////////////////////////////////////////////////////////////////////////////////
int main( int argc, char** argv )
{
    std::string sCameraType = "OpenCV";
    std::string sListFileName;
    if( argc > 1 ) {
        sCameraType = argv[1];
    }

    CameraSensor cameraSensor( sCameraType );

    if( argc > 2 ) {
        sListFileName = argv[2];
        cameraSensor.set( "ListFileName", sListFileName );
    }
    
    if( !cameraSensor.open() ) {
        cerr << "Problem opening camera sensor." << endl;
        return -1;
    }

    COPENCV::Figure fig( "Camera Image", false );

    int nKey = 0;
    Image aImage = cameraSensor.read();
    if( aImage.empty() ) {
        cerr << "Got empty image." << endl;
        return -1;
    }
    std::cout << "SensorID: " << aImage.Map.GetProperty( "SensorID", "" ) << std::endl;

    while( !aImage.empty() && nKey != 27 ) {
        fig.imshow( aImage );
        fig.draw();

        //std::cout << "Time: " << pImage->time() << std::endl;

        if( sCameraType.find( "FileReaderFromList" ) != std::string::npos ) {
            nKey = fig.wait();
        }
        else {
            nKey = fig.wait( 5 );
        }
        double d0 = CMISC::Tic();
        aImage = cameraSensor.read();
        cout << "Time taken to acquire: " << CMISC::TocMS( d0 ) << endl;
    }

    //double d = cameraSensor.get<double>( "ad" );
    cameraSensor.close();
    
    return 0;
}
