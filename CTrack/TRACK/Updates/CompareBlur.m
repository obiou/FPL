function CompareBlur
load mandrill;

IRef         = X;
nImageWidth  = size( IRef, 2 );
nImageHeight = size( IRef, 1 );

v            = [10;5];
M            = zeros(3);
M(1:2,3)     = v;
bJac         = 1;

[BIRef,BIRefMask] = blur_warping( IRef, eye(3), M, eye(3), nImageWidth, nImageHeight, bJac );
if bJac
  BIRefFFT = fftBlur( IRef, v, 'blur_dir_diff' );
  %BIRefFFT = fftBlur( IRef, v, 'blur_gauss_diff' );
else
  BIRefFFT = fftBlur( IRef, v, 'blur_dir' );
end

mask = BIRefMask == 1;
good = find( mask == 1 );

figure
imshow( uint8( BIRef ) )
title( 'Blur computation through warping' )

figure
imshow( uint8( BIRefFFT ) )
title( 'Blur computation through FFT' )

figure
imagesc( BIRef ), colorbar
title( 'Blur computation through warping' )

figure
imagesc( BIRefFFT ), colorbar
title( 'Blur computation through FFT' )

%figure
%imshow( mask )

B1 = zeros(size(BIRef));
B2 = zeros(size(BIRef));

B1(good) = BIRef(good);
B2(good) = BIRefFFT(good);

figure
imagesc( abs( B1 - B2 ) ), colorbar;

max( max( abs( BIRef(good) - BIRefFFT(good) ) ) )

%keyboard