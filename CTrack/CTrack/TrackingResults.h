#ifndef CTRACK_TRACKING_RESULTS_H

#define CTRACK_TRACKING_RESULTS_H

#include <vector>

namespace CTrack {    
     ////////////////////////////////////////////////////////////////////////////
    /// This class contains the tracking results
    class TrackingResults {
    public:
    TrackingResults() :
        m_dAlpha( 1 ), m_dAlphaPrev( -1 ), m_dBeta( 0 ), m_dBetaPrev( 1000 ), m_dLambda( 0 ), m_dLambdaPrev( -1 ) {}

        // The tracking code uses the default copy constructor (make explicit if required).
        Homography GetHomography() { return m_H; }
        Homography GetPrevHomography() { return m_HPrev; }
        double GetAlpha() { return m_dAlpha; }
        double GetAlphaPrev() { return m_dAlphaPrev; }
        double GetBeta() { return m_dBeta; }
        double GetBetaPrev() { return m_dBetaPrev; }
        double GetBlurMagn() { return m_dLambda; }
        double GetBlurMagnPrev() { return m_dLambdaPrev; }

        void SetHomography( const Homography& mH ) { m_H = mH; }
        void SetPrevHomography( const Homography& mHPrev ) { m_HPrev = mHPrev; }
        void SetAlpha( const double& dAlpha ) { m_dAlpha = dAlpha; }
        void SetBeta( const double& dBeta ) { m_dBeta = dBeta; } 
        void SetBlurMagn( const double& dLambda ) { m_dLambda = dLambda; } 

        /// Copies all current values to previous values (homography, alpha, ...)
        /// AND sets the current homography to the identity
        void CopyCurToPrevIgnoringH() {
            m_dAlphaPrev  = m_dAlpha;
            m_dBetaPrev   = m_dBeta;
            m_dLambdaPrev = m_dLambda;
        }

        void id() { m_dAlpha = 1; m_dBeta = 0; m_dLambda = 0; m_H.id(); m_HPrev.id();}
        friend std::ostream& operator<< (std::ostream& o, TrackingResults const& trackingResults) {
            o << "Alpha: " << trackingResults.m_dAlpha << ", Beta: " << trackingResults.m_dBeta << ", Lambda: " << trackingResults.m_dLambda;
            return o;
        }
    private:
        double m_dAlpha, m_dAlphaPrev;
        double m_dBeta, m_dBetaPrev;
        double m_dLambda, m_dLambdaPrev;
        CTrack::Homography m_H;
        CTrack::Homography m_HPrev;
    };
}

#endif
