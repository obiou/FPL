// Copyright (C) 2010 Christopher Mei (christopher.m.mei@gmail.com)
//
// This file is part of the CTrack Library. This library is free
// software; you can redistribute it and/or modify it under the
// terms of the GNU Lesser General Public License as published by the
// Free Software Foundation; either version 2, or (at your option)
// any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU Lesser General Public License for more details.

// You should have received a copy of the GNU Lesser General Public
// License along with this library; see the file COPYING.LESSER. If
// not, write to the Free Software Foundation, 59 Temple Place - Suite
// 330, Boston, MA 02111-1307, USA.

// As a special exception, you may use this file as part of a free software
// library without restriction.  Specifically, if other files instantiate
// templates or use macros or inline functions from this file, or you compile
// this file and link it with other files to produce an executable, this
// file does not by itself cause the resulting executable to be covered by
// the GNU Lesser General Public License.  This exception does not however
// invalidate any other reasons why the executable file might be covered by
// the GNU Lesser General Public License.

#ifndef CTRACK_PLANE_TRACKING_OPENCV_H

#define CTRACK_PLANE_TRACKING_OPENCV_H

#include <utility>
#include <vector>

#include <CTrack/PlaneTracking.h>
#include <cv.h>
#include <highgui.h>

/// This file contains wrappers for OpenCV
namespace CTrack {
    ////////////////////////////////////////////////////////////////////////////
    template< class TrackingFunctor,class MaskFunctorT >
    void TrackPlaneHomography( TrackingSettings<TrackingFunctor,MaskFunctorT>& trackingSettings, ///< Input
                               const IplImage& refPatchImage,     ///< Input
                               const IplImage& curImage,          ///< Input
                               TrackingStats* pTrackingStats,     ///< Output:
                               TrackingResults* pTrackingResults  ///< Output:
                               ) {    
        const ImageHolder refPatchHolder( (unsigned char*)refPatchImage.imageData,
                                          refPatchImage.width, refPatchImage.height, 
                                          refPatchImage.widthStep );
        const ImageHolder curImageHolder( (unsigned char*)curImage.imageData,
                                          curImage.width, curImage.height, 
                                          curImage.widthStep );
        
        TrackPlaneHomography( trackingSettings, refPatchHolder, curImageHolder,
                              pTrackingStats, pTrackingResults );
    }

    ////////////////////////////////////////////////////////////////////////////
    void convert( const std::vector<std::pair<double,double> >& vPolyIn,
                  CvPoint* pPts ) {
        for( size_t ii=0; ii<vPolyIn.size(); ii++ ) {
            pPts[ii].x = vPolyIn[ii].first;
            pPts[ii].y = vPolyIn[ii].second;
        }
    }
}

#endif
