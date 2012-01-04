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
#include <COpenCV.h>
#include <CameraSensor.h>

using namespace CCameraSensor;
using namespace ImageWrapper;

////////////////////////////////////////////////////////////////////////////////
int main()
{
    std::string sCameraType = "FileReaderFromList";
    CameraSensor cameraSensor( sCameraType );

    std::string sListFile = "list_views";
    cameraSensor.set( "ListFileName", sListFile );
    cameraSensor.open();
    Image* pImage = cameraSensor.read();
    COPENCV::Figure fig( "UEyeImage", false );
    int nKey = 0;
    while( pImage != NULL && (char)nKey != 'q' ) {
        IplImage aI = pImage->mImage;
        fig.imshow( &aI );
        fig.draw();

        nKey = fig.wait();
        pImage = cameraSensor.read();
    }
    cameraSensor.close();   

    return 0;
}
