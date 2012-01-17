#include <COpenCV.h>
#if HAS_CVARS
#  include <CVars/CVar.h>
#endif
#include <Misc.h>

#include <iostream>
#include <string>
#include <vector>

using namespace std;

////////////////////////////////////////////////////////////////////////////////
void genSin( vector<pair<double,double> >& v, const int nNumPoints,
             const int nImageWidth, const int nImageHeight,
             const int nOffset ) {             
    for( double ii=0; ii<nNumPoints; ii++ ) {
        double dVal = nImageWidth*ii/nNumPoints;
        v[ii] = pair<double,double>( dVal, 
                                     sin(CVarUtils::GetCVar<double>( "wlength" )*2.*3.1459*(dVal+(double)nOffset)/(double)nImageWidth)*(double)nImageHeight/10.+(double)nImageHeight/2. );
    }
}
 
////////////////////////////////////////////////////////////////////////////////
int main( int argc, char** argv )
{
    string sFileName;
    if( argc > 1 ) {
        sFileName = argv[1];
    }
    else {
        cerr << "Expecting the name of an image file as input." << endl;
        return -1;
    }

#if COPENCV_HAS_CVARS
    //CVars have a name and a default value
    CVarUtils::CreateCVar( "vars.myvar1", 1, "some useful help..." );
    CVarUtils::CreateCVar( "vars.myvar2", 10. );
    CVarUtils::CreateCVar( "vars.var1.myvar1", 1);
    CVarUtils::CreateCVar( "vars.var1.myvar2", 10. );
    CVarUtils::CreateCVar( "wlength", 4., "Wavelength" );
#endif

    IplImage* pImage = NULL;
    pImage = cvLoadImage( sFileName.c_str() );
    if( pImage == NULL ) { cerr << "ERROR: problem reading file." << endl; return -1;}

    const int nNumPoints = 100;
    vector<pair<double,double> > v( nNumPoints );
    genSin( v, nNumPoints, pImage->width, pImage->height, 0 );

    int nOffset = 0;

    COPENCV::Figure fig( "Image" );
    int nKey = 0;
    while( (char)nKey != 0x1B ) {
        nOffset = (nOffset+1) %pImage->width;
        genSin( v, nNumPoints, pImage->width, pImage->height, nOffset++ );
        fig.imshow( pImage );
        fig.plot( v, "b-" );
        fig.draw();   
        nKey = fig.wait(20);
    }
    return 0;
}
