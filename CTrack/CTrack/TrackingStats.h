#ifndef CTRACK_TRACKING_STATS_H

#define CTRACK_TRACKING_STATS_H

#include <vector>

namespace CTrack {    
    ////////////////////////////////////////////////////////////////////////////
    /// This class holds statistics about the tracking
    class TrackingStats {
    public:
        TrackingStats() {
            m_dFinalRMS = 0.;
            m_nNumIter  = 0;
        }
        void RMS( double dRMS ) {
            m_dFinalRMS = dRMS;
        }
        double RMS() {
            return m_dFinalRMS;
        }
        void NumIter( int nNumIter ) {
            m_nNumIter = nNumIter;
        }
    private:
        double m_dFinalRMS;
        int    m_nNumIter;
    };
}

#endif
