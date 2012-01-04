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
#ifndef OPENCV_CAMERA_H
#define OPENCV_CAMERA_H

#include <CCameraSensor/CameraSensor.h>

#include <string>
#include <vector>

#include <cv.h>
#include <highgui.h>

///
/// This driver wraps the OpenCV capture mode.
///
/// The returned image is the buffer used by the camera for copying to RAM, it should not be freed or modified.
/// A subsequent call to read could modify this buffer so the client should make a copy if required.
///
namespace CCameraSensor {
    ////////////////////////////////////////////////////////////////////////////
    /// Wrapper for OpenCV capture mode.
    FACTORY_OBJECT( OpenCVCamera, CameraSensorInterface, "OpenCV" ) {
    public:
        OpenCVCamera();
        ~OpenCVCamera();

        /// open() should only be called once
        bool open();
        bool close();

        /// Each call will invalidate the previous image (make a copy if you wish to use it later)
        bool read( std::vector<ImageWrapper::Image>& vImages );

        std::string get( const std::string& sKey );
        bool set( const std::string& sKey, const std::string& sParameter );
        bool has( const std::string& sKey );
        static void info();

    private:
        bool m_bOpened;
        CvCapture* m_pCvCapture;
        IplImage* m_pCapturedImage;
        ImageWrapper::Image m_ReadImageHolder;
        std::string m_sSensorID;
    };
}

#endif
