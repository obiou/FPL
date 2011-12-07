#ifndef MASKS_H
#define MASKS_H

namespace CTrack {
    /////////////////////////////////////////////////////////////////////////////
    /// This class is used to set and save out-of-image or outliers values.
    class MaskFunctor {
    public:
        virtual ~MaskFunctor() {}
        virtual void fwd() = 0;
        virtual void set( bool bVal ) = 0;
        virtual void end() = 0;
        virtual void set_to_nan( float* pBuffer, int nWidth, int nHeight ) = 0;
    };

    /////////////////////////////////////////////////////////////////////////////
    /// This functor class does not save any information about out-of-image
    /// values and can be used for faster processing when this information is not
    /// required (e.g. subpixel refinement)
    class NoMask { //: public MaskFunctor { // do not inherit explicitly: too expensive (virtual function)
    public: 
        NoMask() {}
        NoMask( int, int ) {}
        ~NoMask() {}
        void fwd() {}
        void set( bool ) {}
        void end() {}
        void set_to_nan( float*, int, int ) {}
    };

    /////////////////////////////////////////////////////////////////////////////
    /// This functor class saves out-of-image values in a boolean image the size
    /// of the image.
    /// 1: ok pixel
    /// 0: not ok (out-of-image)
    class BoolMask { //: public MaskFunctor { // do not inherit explicitly: too expensive (virtual function)
    public:
    BoolMask( int nWidth, int nHeight ) : 
        m_nWidth( nWidth ), m_nHeight( nHeight ), m_pMask( new bool[ nWidth*nHeight ] ) {
            m_pTmpPtr = m_pMask;
        }
        ~BoolMask() { delete[] m_pMask; m_pTmpPtr = NULL; }
        void fwd() { m_pTmpPtr++; }
        void set( bool bVal ) { *m_pTmpPtr = bVal; }
        void end() { m_pTmpPtr = m_pMask; }
        void set_to_nan( float* pBuffer, const int nWidth, const int nHeight ) {
            if( nWidth != m_nWidth || nHeight != m_nHeight ) {
                std::cout << "ERROR: in set_to_nan, incompatible buffer sizes." << std::endl;
                return;
            }
            m_pTmpPtr = m_pMask;
            for( size_t ii=0; ii<(unsigned int)nWidth*nHeight; ii++ ) {
                if( !*m_pTmpPtr++ ) {
                    *pBuffer = std::numeric_limits<float>::quiet_NaN();
                }
                pBuffer++;
            }
            m_pTmpPtr = m_pMask;
        }
        bool get( const int nX, const int nY ) {
            return m_pMask[ nX + nY*m_nWidth ];
        }
    private:
        BoolMask();        
    private:
        const int m_nWidth, m_nHeight;
        bool* const m_pMask;
        bool* m_pTmpPtr;
    };
}

#endif
