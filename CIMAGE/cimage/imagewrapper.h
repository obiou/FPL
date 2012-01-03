// ImageWrapper - wrapper image class
// Copyright (C) 2012 C. Mei
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
#ifndef CIMAGE_H
#define CIMAGE_H

#include <cv.h>
#include <highgui.h>

#include <iostream>
#include <string>

namespace ImageWrapper {
    typedef struct _Image {
        cv::Mat mImage; // OpenCV image
        double dSystemTime; /// System time at which the image was taken (often estimated) in micro-seconds.
        double dCameraTime; /// Time on the camera device at which the image was taken (often more precise) in micro-seconds.
        std::string sSensorID;    /// Unique camera identifier. 
    } Image;

    ////////////////////////////////////////////////////////////////////////////
    bool imwrite( const std::string& sImageFileName,     ///<Input: image file name
                  const std::string& sExtraInfoFileName, ///<Input: text containing extra info (e.g. camera time, sensor ID,...)
                  const Image& image );


    ////////////////////////////////////////////////////////////////////////////
    bool imwrite( const std::string& sImageFileName,    ///<Input: image file name
                  const Image& image );

    ////////////////////////////////////////////////////////////////////////////
    Image imread( const std::string& sImageFileName, 
                  const std::string& sExtraInfoFileName = "",
                  int nFlags = 1 //<Input: same flag convention as in OpenCV (>0 colour, 0 greyscale, <0 as is)
                  );
}

////////////////////////////////////////////////////////////////////////////////
bool ImageWrapper::imwrite( const std::string& sImageFileName,    
                            const std::string& sExtraInfoFileName,
                            const Image& image ) {
    bool bSuccess = cv::imwrite( sImageFileName.c_str(), image.mImage );
    //std::cerr << "Saving " << sImageFileName << " and " << sExtraInfoFileName << std::endl;
    if( sExtraInfoFileName != "" ) {
        cv::FileStorage oFile( sExtraInfoFileName.c_str(), cv::FileStorage::WRITE );
        oFile << "SystemTime" << image.dSystemTime;
        oFile << "CameraTime" << image.dCameraTime;
        oFile << "SensorID"   << image.sSensorID;
    }
    return bSuccess;
}

////////////////////////////////////////////////////////////////////////////////
bool ImageWrapper::imwrite( const std::string& sImageFileName,  
                            const Image& image ) {
    return imwrite( sImageFileName, "", image );
}

////////////////////////////////////////////////////////////////////////////////
ImageWrapper::Image ImageWrapper::imread( const std::string& sImageFileName, 
                                          const std::string& sExtraInfoFileName,
                                          int nFlags
                                          ) {
    Image retImage;
    retImage.mImage = cv::imread( sImageFileName.c_str(), nFlags );
    if( sExtraInfoFileName != "" ) {
        cv::FileStorage oFile( sExtraInfoFileName.c_str(), cv::FileStorage::READ );
        retImage.dSystemTime = (double)oFile[ "SystemTime" ];
        retImage.dCameraTime = (double)oFile[ "CameraTime" ];
        retImage.sSensorID   = (std::string)oFile[ "SensorID" ];
    }
    return retImage;
}

#endif
