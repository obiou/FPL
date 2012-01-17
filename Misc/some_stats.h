#ifndef SOME_STATS_H
#define SOME_STATS_H

#include <stdlib.h>

#if 0
#include <tr1/random>
std::tr1::mt19937 rand_gen( 0 );
std::tr1::normal_distribution<float> noise(0,dStd);
std::tr1::variate_generator<std::tr1::mt19937, 
                                std::tr1::normal_distribution<float> > gen_val( rand_gen, noise );
#endif

namespace CMISC {
    ////////////////////////////////////////////////////////////////////////////
    float randf();
    
    ////////////////////////////////////////////////////////////////////////////
    /// Compute random number from Gausssian distribution, returns two
    /// values at once (Box-Muller transform)
    void rand_gaussf( float fMean, float fStd, float& y1, float& y2 );

    ////////////////////////////////////////////////////////////////////////////
    /// Compute random number from Gausssian distribution, returns two
    /// values at once (Box-Muller transform)
    void rand_gaussf( float& y1, float& y2 );
    
    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    float randf() {
        return static_cast<float>(rand())/static_cast<float>(RAND_MAX);
    }
    
    ////////////////////////////////////////////////////////////////////////////
    void rand_gaussf( float fMean, float fStd, float& y1, float& y2 ) {
        rand_gaussf( y1, y2 );
        y1 = fMean + y1*fStd;
        y2 = fMean + y2*fStd;
    }

    ////////////////////////////////////////////////////////////////////////////
    void rand_gaussf( float& y1, float& y2 ) {
        float x1, x2, w;
        do {
            x1 = 2.0f * randf() - 1.0f;
            x2 = 2.0f * randf() - 1.0f;
            w = x1 * x1 + x2 * x2;
        } while ( w >= 1.0 );
        
        w = sqrtf( (-2.0f * logf( w ) ) / w );
        y1 = x1 * w;
        y2 = x2 * w;        
    }
}

#endif
