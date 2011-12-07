#ifndef PROJECTION_FUNCTIONS_H
#define PROJECTION_FUNCTIONS_H

#define NOT_ZERO( Q, x ) ( (Q) ? (x) : 0 )
#define NOT_ONE( Q, x )  ( (Q) ? (x) : 1 )

#define ADD_IF( x1, Q2, x2 )  ( (Q2) ? (x1+x2) : (x1) )
#define ADD2_IF( x1, Q2, x2, Q3, x3 )  ( ADD_IF( ADD_IF( x1, Q2, x2 ), Q3, x3 ) )
#define ADD3_IF( x1, Q2, x2, Q3, x3, Q4, x4 )  ( ADD_IF( ADD2_IF( x1, Q2, x2, Q3, x3 ), Q4, x4 ) )
#define ADD4_IF( x1, Q2, x2, Q3, x3, Q4, x4, Q5, x5 )  ( ADD_IF( ADD3_IF( x1, Q2, x2, Q3, x3, Q4, x4 ), Q5, x5 ) )
#define ADD5_IF( x1, Q2, x2, Q3, x3, Q4, x4, Q5, x5, Q6, x6 ) \
    ( ADD_IF( ADD4_IF( x1, Q2, x2, Q3, x3, Q4, x4, Q5, x5 ), Q6, x6 ) )

template<class T,bool condition>
    struct CTEST {
        inline static T NOT_ZEROF( T ) { return 0; }
    };

template<class T>
struct CTEST<T,true> {
        inline static T NOT_ZEROF( T x ) { return x; }
    };

namespace CPROJECTIONS {
    ////////////////////////////////////////////////////////////////////////////
    /// Radial and distortion forward model with optional Jacobian
    /// computation
    template< class T,
        bool USE_K1, bool USE_K2, bool USE_P1, bool USE_P2,
        bool COMPUTE_JAC_E, bool COMPUTE_JAC_I>
        inline void distort( const T k1, const T k2, const T p1, const T p2,
                             const T x, const T y,
                             T* xd, T* yd,
                             T* d_xydx,  ///<Output: diff [x,y] = distort(x,y) with resp. to x
                             T* d_xydy,  ///<Output: diff [x,y] = distort(x,y) with resp. to y
                             T* d_xydk1, ///<Output: diff [x,y] = distort(x,y) with resp. to k1
                             T* d_xydk2, ///<Output: diff [x,y] = distort(x,y) with resp. to k2
                             T* d_xydp1, ///<Output: diff [x,y] = distort(x,y) with resp. to p1
                             T* d_xydp2  ///<Output: diff [x,y] = distort(x,y) with resp. to p2
                             ) {
            const T r2  = NOT_ZERO( USE_K1 || USE_K2 || USE_P1 || USE_P2, x*x + y*y );
            const T r4  = NOT_ZERO( USE_K2, r2*r2 );
            const T m2xy  = NOT_ZERO( USE_P1 || USE_P2, 2*x*y );
            const T uk1 = NOT_ZERO( USE_K1, k1 ); const T uk2 = NOT_ZERO( USE_K2, k2 );
            const T up1 = NOT_ZERO( USE_P1, p1 ); const T up2 = NOT_ZERO( USE_P2, p2 );
            const T rad_dist = ADD2_IF( 1, USE_K1, uk1*r2, USE_K2, uk2*r4 );

            *xd = ADD2_IF( x * rad_dist, USE_P1, up1 * m2xy, USE_P2, up2 * ( r2 + 2*x*x ) ) ;
            *yd = ADD2_IF( y * rad_dist, USE_P2, up2 * m2xy, USE_P1, up1 * ( r2 + 2*y*y ) );

            if( COMPUTE_JAC_E ) {
                d_xydx[0] = ADD5_IF( 0, ( USE_K1 || USE_K2 ), rad_dist, 
                                     USE_K1, uk1*2*x*x, USE_K2, uk2*r2*4*x*x, 
                                     USE_P1, 2*up1*y, USE_P2, 6*up2*x );
                d_xydx[1] = ADD4_IF( 0, USE_K1, uk1*2*x*y, USE_K2, uk2*4*r2*x*y, 
                                     USE_P1, up1*2*x, USE_P2, 2*up2*y );
                d_xydy[0] = d_xydx[1];
                d_xydy[1] = ADD5_IF( 0, ( USE_K1 || USE_K2 ), rad_dist,
                                     USE_K1, uk1*2*y*y, USE_K2, uk2*r2*4*y*y,
                                     USE_P1, 6*up1*y, USE_P2, 2*up2*x );
            }
            if( COMPUTE_JAC_I ) {
                d_xydk1[0] = x*r2;
                d_xydk1[1] = y*r2;
                d_xydk2[0] = x*r4;
                d_xydk2[1] = y*r4;
                d_xydp1[0] = m2xy;
                d_xydp1[1] = r2 + 2*y*y;
                d_xydp2[0] = r2 + 2*x*x;
                d_xydp2[1] = m2xy;
            }
        }

    ////////////////////////////////////////////////////////////////////////////
    template< class T, bool COMPUTE_JAC, bool IGNORE_Z >
        inline void normalise
        ( const T x, const T y, const T z, 
          T* xn, T* yn, 
          T* d_xnx, T* d_yny,
          T* d_xnz, T* d_ynz
         ) {
        const T divZ = IGNORE_Z ? 1 : 1/z;
        *xn = IGNORE_Z ? x : x*divZ;
        *yn = IGNORE_Z ? y : y*divZ;
        if( COMPUTE_JAC ) {
            *d_xnx = divZ;
            *d_yny = divZ;
            *d_xnz = IGNORE_Z ? -x : -*xn*divZ;
            *d_ynz = IGNORE_Z ? -y : -*yn*divZ;
        }
    }

    ////////////////////////////////////////////////////////////////////////////
    template< class T>
        inline void pinhole
        ( const T fx, const T fy, const T cx, const T cy,
          const T xn, const T yn, T* u, T* v
          ) {
        *u = fx*xn + cx;
        *v = fy*yn + cy;
    }

    ////////////////////////////////////////////////////////////////////////////
    /// Projection with radial and distortion model with optional
    /// Jacobian computation.
    ///
    /// A template is used here so the compiler can optimize the function
    /// if some of the distortion parameters are not used.
    /// If COMPUTE_JAC_E or COMPUTE_JAC_I are 0, the corresponding output variable
    /// can be set to NULL.
    template< class T,
        bool USE_K1, bool USE_K2, bool USE_P1, bool USE_P2,
        bool COMPUTE_JAC_E, bool COMPUTE_JAC_I, bool IGNORE_Z
        >
        inline void project
        ( const T fx, const T fy, const T cx, const T cy,
          const T k1, const T k2, const T p1, const T p2,
          const T x, const T y, const T z, 
          T* u, T* v,
          T* d_uvdxyz, ///<Output: extrinsic Jacobian (u,v) wr (x,y,z), u first then v
          T* d_uvfxfycxcyk1k2p1p2 ///<Output: extrinsic Jacobian (u,v) wr to internals, u first then v
          ) {
        T xn, yn, d_xnx, d_yny, d_xnz, d_ynz;
        normalise<T,COMPUTE_JAC_E,IGNORE_Z>( x, y, z, &xn, &yn,
                                             &d_xnx, &d_yny,
                                             &d_xnz, &d_ynz );
        T xd, yd, d_xydxn[2], d_xydyn[2];
        T d_xydk1[2], d_xydk2[2], d_xydp1[2], d_xydp2[2];
        distort<T,USE_K1,USE_K2,USE_P1,USE_P2,COMPUTE_JAC_E,COMPUTE_JAC_I>
            ( k1, k2, p1, p2, xn, yn, &xd, &yd, 
              d_xydxn, d_xydyn, d_xydk1, d_xydk2, d_xydp1, d_xydp2 );

        pinhole<T>( fx, fy, cx, cy, xd, yd, u, v );

        if( COMPUTE_JAC_E ) {
            // duvdxyz: dudx dvdx dudy dvdy... (column major)
            d_uvdxyz[0] = fx*d_xydxn[0]*d_xnx;
            //fx*d_xydxn_0_d*d_xnx ); 
            d_uvdxyz[1] = fy*d_xydxn[1]*d_xnx;
            //fy*d_xydxn_1_d*d_xnx );
            d_uvdxyz[2] = fx*d_xydyn[0]*d_yny;
            d_uvdxyz[3] = fy*d_xydyn[1]*d_yny;
            d_uvdxyz[4] = fx*d_xydxn[0]*d_xnz + fx*d_xydyn[0]*d_ynz;
            d_uvdxyz[5] = fy*d_xydxn[1]*d_xnz + fy*d_xydyn[1]*d_ynz;
        }

        if( COMPUTE_JAC_I ) {
            T* pDiff = d_uvfxfycxcyk1k2p1p2;
            pDiff[0] = xd;
            pDiff[1] = 0;

            pDiff[2] = 0;
            pDiff[3] = yd;

            pDiff[4] = 1;
            pDiff[5] = 0;

            pDiff[6] = 0;
            pDiff[7] = 1;

            pDiff[8] = fx*d_xydk1[0];
            pDiff[9] = fy*d_xydk1[1];

            pDiff[10] = fx*d_xydk2[0];
            pDiff[11] = fy*d_xydk2[1]; 

            pDiff[12] = fx*d_xydp1[0]; 
            pDiff[13] = fy*d_xydp1[1];

            pDiff[14] = fx*d_xydp2[0]; 
            pDiff[15] = fy*d_xydp2[1];
        }
    }

    ////////////////////////////////////////////////////////////////////////////
    /// Projection with radial and distortion model with optional
    /// Jacobian computation.
    ///
    /// A template is used here so the compiler can optimize the function
    /// if some of the distortion parameters are not used.
    template< class T,
        bool USE_K1, bool USE_K2, bool USE_P1, bool USE_P2,
        bool IGNORE_Z
        >
        inline void project
        ( const T fx, const T fy, const T cx, const T cy,
          const T k1, const T k2, const T p1, const T p2,
          const T x, const T y, const T z, 
          T* u, T* v ) {
            project<T, USE_K1, USE_K2, USE_P1, USE_P2, 0, 0, IGNORE_Z>
                ( fx, fy, cx, cy, k1, k2, p1, p2, x, y, z, u, v,
                  NULL, NULL );
        }
}

#endif
