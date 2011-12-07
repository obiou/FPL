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

#ifndef TESTING_HELPERS_H

#define TESTING_HELPERS_H

#include <CTrack/Homography.h>

namespace CTrack {
    ////////////////////////////////////////////////////////////////////////////
    CTrack::Homography GenerateTranslation( double dX, double dY ) {
        CTrack::Homography HTrans;
        HTrans.Set( 0, 2, dX );
        HTrans.Set( 1, 2, dY );
        return HTrans;
    }
 
    ////////////////////////////////////////////////////////////////////////////
    CTrack::Homography GenerateCenteredRotation( const int nPatchWidth, 
                                                 const int nPatchHeight, 
                                                 const double dAngleDeg ) {
        CTrack::Homography HRot;
        const double dAngleRad = dAngleDeg*3.14159/180.;
        double dRot[] = {0,0,dAngleRad,-dAngleRad};
        HRot.SL3Update( 4, dRot );
        
        CTrack::Homography HCenter;
        HCenter.Set( 0, 2, nPatchWidth/2 );
        HCenter.Set( 1, 2, nPatchHeight/2 );

        CTrack::Homography HCenterM;
        HCenterM.Set( 0, 2, -nPatchWidth/2 );
        HCenterM.Set( 1, 2, -nPatchHeight/2 );
        
        CTrack::Homography HFull;
        HFull.mult( HCenter );
        HFull.mult( HRot );
        HFull.mult( HCenterM );

        return HFull;
    }
}

#endif
