#include <iostream>
#include <vector>

#include <cimage.h>

using namespace std;

int main() {
    // Create a buffer of chars
    std::vector<unsigned char> v( 100, 255 );

    for( size_t ii=0; ii<v.size(); ++ii ) {
        if( ( ii / 10 )  == ii%10 ) {
            v[ii] = 0;
        }
    }

    ImageWrapper::Image write_image;
    cv::Mat I( 10, 10, CV_8U, &v[0] );
    write_image.mImage = I;
    write_image.sSensorID = "ACamera";
    write_image.dCameraTime = 123456;
    write_image.dSystemTime = 654321;

    ImageWrapper::imwrite( "test_read_write_im.png", "test_read_write_im.txt", write_image );

    ImageWrapper::Image read_image;

    read_image = ImageWrapper::imread( "test_read_write_im.png", "test_read_write_im.txt", -1 );

    if( read_image.sSensorID == write_image.sSensorID &&
        read_image.dCameraTime == write_image.dCameraTime &&
        read_image.dSystemTime == write_image.dSystemTime &&
        cv::sum(cv::abs( write_image.mImage - read_image.mImage))[0] == 0) {
        return 0;
    }
    else {
        
        return -1;
    }
}
