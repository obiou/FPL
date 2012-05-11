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
#ifndef IMAGE_WRAPPER_H
#define IMAGE_WRAPPER_H

#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <string>

namespace ImageWrapper {
    class PropertyMap {
    public:
        ////////////////////////////////////////////////////////////////////////
        std::string GetProperty( const std::string& sPropertyName,
                                 const std::string& sDefaultValue = ""
                                 ) {
            std::map<std::string,std::string>::iterator it;
            it = m_mPropertyMap.find( sPropertyName );
            if( it == m_mPropertyMap.end() ){
                return sDefaultValue; 
            }
            return it->second;
        }

        ////////////////////////////////////////////////////////////////////////
        template <class T> T GetProperty( const std::string& sPropertyName,
                                          const T& tDefaultValue = T()
                                          ) {
            std::map<std::string,std::string>::iterator it;
            it = m_mPropertyMap.find( sPropertyName );
            if( it == m_mPropertyMap.end() ){
                return tDefaultValue; 
            }
            T t; StrToVal( t, it->second );
            return t;
        }

        ////////////////////////////////////////////////////////////////////////
        template <class T>
            void SetProperty( const std::string& sPropertyName,
                              T tDesiredValue ) {
            m_mPropertyMap[ sPropertyName ] = ValToStr( tDesiredValue );
        }
        
        ////////////////////////////////////////////////////////////////////////
        template<class T>
        friend void output( T& oStream, const PropertyMap& prop ) {
            std::map<std::string,std::string>::const_iterator it;
            for( it = prop.m_mPropertyMap.begin(); it != prop.m_mPropertyMap.end(); it++ ){
                oStream << it->first << it->second;
            }            
        }
        
        ////////////////////////////////////////////////////////////////////////
        friend std::ostream& operator<<( std::ostream& oStream, const PropertyMap& prop ) {
            std::map<std::string,std::string>::const_iterator it;
            for( it = prop.m_mPropertyMap.begin(); it != prop.m_mPropertyMap.end(); it++ ){
                oStream << it->first << it->second;
            }            
            return oStream;
        }

        ////////////////////////////////////////////////////////////////////////
        void PrintPropertyMap() { output( std::cout, *this ); }

    private:
        ////////////////////////////////////////////////////////////////////////
        template <class T>
            void StrToVal( T& t, const std::string& sValue ) {
            std::istringstream iss( sValue );
            iss >> t;
        }

        ////////////////////////////////////////////////////////////////////////
        template <class T>
            std::string ValToStr( const T& t ) {
            std::ostringstream oss;
            oss << t;
            return oss.str();
        }

        ////////////////////////////////////////////////////////////////////////
        // Specialization for double for maximum precision 
        std::string ValToStr( const double& val ) {
            std::ostringstream oss;
            oss.precision( std::numeric_limits<double>::digits10 );
            oss << val;
            return oss.str();
        }

        ////////////////////////////////////////////////////////////////////////
        // Specialization for float for maximum precision
        std::string ValToStr( const float& val ) {
            std::ostringstream oss;
            oss.precision( std::numeric_limits<float>::digits10 );
            oss << val;
            return oss.str();
        }

        std::map<std::string,std::string>   m_mPropertyMap;
    };

    ////////////////////////////////////////////////////////////////////////////
    typedef struct _Image Image;

    ////////////////////////////////////////////////////////////////////////////
    inline bool imwrite( const std::string& sImageFileName,     ///<Input: image file name
                         const std::string& sExtraInfoFileName, ///<Input: text containing extra info (e.g. camera time, sensor ID,...)
                         const Image& image );

    ////////////////////////////////////////////////////////////////////////////
    inline bool imwrite( const std::string& sImageFileName,    ///<Input: image file name
                         const Image& image,
                         bool bWriteExtraInfo = true );
    
    ////////////////////////////////////////////////////////////////////////////
    inline Image imread( const std::string& sImageFileName, 
                         const std::string& sExtraInfoFileName,
                         int nFlags = -1 //<Input: same flag convention as in OpenCV (>0 colour, 0 greyscale, <0 as is)
                         );

    ////////////////////////////////////////////////////////////////////////////
    inline Image imread( const std::string& sImageFileName, 
                         bool bReadExtraInfo = true,
                         int nFlags = -1 //<Input: same flag convention as in OpenCV (>0 colour, 0 greyscale, <0 as is)
                         );

    ////////////////////////////////////////////////////////////////////////////
    inline Image FromIplImage( const IplImage* pImage, bool bClone = true );

    ////////////////////////////////////////////////////////////////////////////
    typedef struct _Image {
        cv::Mat Image; // OpenCV image
        PropertyMap Map;

        int width() { return Image.cols; }
        int height() { return Image.rows; }
        int widthStep() { return static_cast<int>( Image.step ); }
        struct _Image clone() { struct _Image ret = *this; this->Image = ret.Image.clone(); return *this; }
        bool empty() { return Image.data == NULL; }

        /// Write an image to sImageName and the property map to sExtraInfoName.
        /// No checks are made for overwriting.
        inline bool write( const std::string& sImageName, 
                           const std::string& sExtraInfoName ) {
            return imwrite( sImageName, sExtraInfoName, *this );
        }

        /// If bWriteExtraInfo is set to true, this call will automatically deduce the 
        /// sExtraInfoName from the sImageName by replacing the
        /// extension by ".txt" (if not extension is found, ".txt" will be
        /// appended). No checks are made for overwriting.
        inline bool write( const std::string& sImageName, bool bWriteExtraInfo = true ) {
            return imwrite( sImageName, bWriteExtraInfo, Image );
        }

        /// Read an image from sImageName and the property map from sExtraInfoName.
        /// nFlags can be used to automatically load in color or image (or keep as such - default).
        inline void read( const std::string& sImageFileName, 
                          const std::string& sExtraInfoFileName,
                          int nFlags = -1 ) {
            *this = imread( sImageFileName, sExtraInfoFileName, nFlags );
        }

        /// Read an image from sImageName and the property map from sExtraInfoName.
        /// If bReadExtraInfo is set to true, this call will automatically deduce the 
        /// sExtraInfoName from the sImageName by replacing the
        /// extension by ".txt" (if not extension is found, ".txt" will be
        /// appended).
        /// nFlags can be used to automatically load in color or image (or keep as such - default).
        inline void read( const std::string& sImageFileName, 
                          bool bReadExtraInfo = true,
                          int nFlags = -1 //<Input: same flag convention as in OpenCV (>0 colour, 0 greyscale, <0 as is)
                          ) {
            *this = imread( sImageFileName, bReadExtraInfo, nFlags );
        }
    } Image;
}

////////////////////////////////////////////////////////////////////////////////
inline bool ImageWrapper::imwrite( const std::string& sImageFileName,    
                                   const std::string& sExtraInfoFileName,
                                   const Image& image ) {
    bool bSuccess = cv::imwrite( sImageFileName.c_str(), image.Image );
    //std::cerr << "Saving " << sImageFileName << " and " << sExtraInfoFileName << std::endl;
    if( sExtraInfoFileName != "" ) {
        cv::FileStorage oFile( sExtraInfoFileName.c_str(), cv::FileStorage::WRITE );
        if( !oFile.isOpened() ) { return false; }
        output( oFile, image.Map );
    }
    return bSuccess;
}

////////////////////////////////////////////////////////////////////////////////
inline bool ImageWrapper::imwrite( const std::string& sImageFileName,  
                                   const Image& image,
                                   bool bWriteExtraInfo ) {
    std::string sExtraInfoName = "";
    if( bWriteExtraInfo ) {
        sExtraInfoName = sImageFileName;
        sExtraInfoName.erase( sExtraInfoName.rfind( '.' ) );
        sExtraInfoName += ".txt";
    }
    return imwrite( sImageFileName, sExtraInfoName, image );
}

////////////////////////////////////////////////////////////////////////////////
inline ImageWrapper::Image ImageWrapper::imread( const std::string& sImageFileName, 
                                                 const std::string& sExtraInfoFileName,
                                                 int nFlags
                                                 ) {
    Image retImage;
    retImage.Image = cv::imread( sImageFileName.c_str(), nFlags );
    if( sExtraInfoFileName != "" ) {
        cv::FileStorage oFile( sExtraInfoFileName.c_str(), cv::FileStorage::READ );

        cv::FileNode r = oFile.root();
        cv::FileNodeIterator it_r_b = r.begin();
        cv::FileNodeIterator it_r_e = r.end();

        for( ; it_r_b != it_r_e; it_r_b++ ) {
            cv::FileNode fnode = *it_r_b;
            for( cv::FileNodeIterator it = fnode.begin(); it != fnode.end(); it++ ) {
                //std::cout << (*it).name() << std::endl;
                //std::cout << (std::string)oFile[ (*it).name() ] << std::endl;
                retImage.Map.SetProperty( (*it).name(), 
                                          (std::string)oFile[ (*it).name() ] );
            }
        }
    }
    return retImage;
}

////////////////////////////////////////////////////////////////////////////////
inline ImageWrapper::Image ImageWrapper::imread
( const std::string& sImageFileName, bool bReadExtraInfo, int nFlags
  ) {
    std::string sExtraInfoName = "";
    if( bReadExtraInfo ) {
        sExtraInfoName = sImageFileName;
        sExtraInfoName.erase( sExtraInfoName.rfind( '.' ) );
        sExtraInfoName += ".txt";
    }
    return imread( sImageFileName, sExtraInfoName, nFlags );
}

////////////////////////////////////////////////////////////////////////////////
inline ImageWrapper::Image ImageWrapper::FromIplImage( const IplImage* pImage, bool bClone ) {
    Image retImage;
    if( bClone ) {
        retImage.Image = cv::cvarrToMat( pImage ).clone();
    }
    else {
        retImage.Image = cv::cvarrToMat( pImage );
    }
#if 0
    retImage.dSystemTime = 0;
    retImage.dCameraTime = 0;
    retImage.sSensorID   = "";
#endif
    return retImage;
}

#endif
