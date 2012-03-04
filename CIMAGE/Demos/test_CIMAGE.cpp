#include <stdio.h>

#include <cv.h>
#include <highgui.h>

using namespace cv;

int main( int argc, char** argv )
{
    Mat src;
    if( argc != 2 || !(src=imread(argv[1], 1)).data )
        return -1;

    namedWindow( "Image", 1 );
    imshow( "Image", src );

    waitKey();

    return 0;
}
