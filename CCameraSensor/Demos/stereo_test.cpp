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
int main()
{
    CameraSensor cameraSensor1( "UEye" );
    CameraSensor cameraSensor2( "UEye" );

    if( !cameraSensor1.open() ) {
        cerr << "Problem opening camera sensor 1." << endl;
        return -1;
    }

    if( !cameraSensor2.open() ) {
        cerr << "Problem opening camera sensor 2." << endl;
        return -1;
    }

    COPENCV::Figure fig( "UEyeImage", false );

    int nKey = 0;
    Image aImage1 = cameraSensor1.read();
    Image aImage2 = cameraSensor2.read();
    std::cout << "SensorID: " << 
        aImage1.Map.GetProperty( "SensorID", "" ) << std::endl;
    std::cout << "SensorID: " << 
        aImage2.Map.GetProperty( "SensorID", "" ) << std::endl;

    bool bBlock = false;

    while( !aImage1.empty() && !aImage2.empty() && nKey != 32 ) {
        fig.imshow2( aImage1, aImage2 );
        fig.draw();

        //std::cout << "Time: " << pImage->time() << std::endl;

        nKey = fig.wait( bBlock ? 0 : 10 );

        if( nKey == 'b' ) {
            bBlock = !bBlock;
        }

        double d0 = CMISC::Tic();
        cameraSensor1.set( "Trigger", true );
        cameraSensor2.set( "Trigger", true );
        aImage1 = cameraSensor1.read();
        aImage2 = cameraSensor2.read();
        cout << "Time taken to acquire: " << CMISC::TocMS( d0 ) << endl;
    }

    //double d = cameraSensor.get<double>( "ad" );
    cameraSensor1.close();
    cameraSensor2.close();
    
    return 0;
}
