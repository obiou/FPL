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
    write_image.Image = I;
    const string sSensorID = "ACamera";
    const double dCameraTime = 123456;
    const double dSystemTime = 654321;
    write_image.Map.SetProperty( "SensorID", "ACamera" );
    write_image.Map.SetProperty( "CameraTime", 123456 );
    write_image.Map.SetProperty( "SystemTime", 654321 );

    ImageWrapper::imwrite( "test_read_write_im.png", "test_read_write_im.txt", write_image );

    ImageWrapper::Image read_image;

    read_image = ImageWrapper::imread( "test_read_write_im.png", "test_read_write_im.txt", -1 );

    if( read_image.Map.GetProperty( "SensorID", "" ) == sSensorID &&
        read_image.Map.GetProperty( "CameraTime", 0. ) == dCameraTime &&
        read_image.Map.GetProperty( "SystemTime", 0. ) == dSystemTime &&
        cv::sum(cv::abs( write_image.Image - read_image.Image))[0] == 0) {
        return 0;
    }
    else {
        
        return -1;
    }
}
