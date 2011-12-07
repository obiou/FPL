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
using namespace CCameraImage;
using namespace std;

////////////////////////////////////////////////////////////////////////////////
int main()
{
    CameraSensor cameraSensor( "UEye" );

    if( !cameraSensor.open() ) {
        cerr << "Problem opening camera sensor." << endl;
        return -1;
    }

    COPENCV::Figure fig( "UEyeImage", false );

    char nKey = 0;
    Image* pImage = cameraSensor.read();
    std::cout << "SensorID: " << pImage->sensorID() << std::endl;

    while( pImage != NULL && nKey != 32 ) {
        IplImage aI = ToIplImage( pImage );
        fig.imshow( &aI );
        fig.draw();

        //std::cout << "Time: " << pImage->time() << std::endl;

        nKey = fig.wait( 5 );
        double d0 = CMISC::Tic();
        pImage = cameraSensor.read();
        cout << "Time taken to acquire: " << CMISC::TocMS( d0 ) << endl;
    }

    //double d = cameraSensor.get<double>( "ad" );
    cameraSensor.close();
    
    return 0;
}
