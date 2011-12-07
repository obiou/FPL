% function [MEst,RMS,dNormUpdate,TIRefEst] = SSDUpdateNoMotionVelBlur( IRef, TIRef, HRef, VEst, bRobust )
%
% Compute a blur velocity update.
% The homography is not estimated, this function can be used for an
% alternation scheme or for debugging.
%
% Input:
% - IRef: unblurred image (from TIRef has been extracted)
% - TIRef: unblurred template
% - TIRefMask: binary image of the same size as TIRef with 0 for the parts
%   of TIRef that should be ignored and 1 for the rest
% - HRef: homography to obtain the unblurred template
% - VEst: current blur estimate (in Lie algebra space)
% - bRobust: should m-estimators be used?
%
% Output:
% - VEst: current estimated velocity blur
% - RMS: residual error BEFORE the current updated estimate
% - dNormUpdate: norm of the computed update (can be used to compute a stopping criteria)
% - TIRefEst: estimated template BEFORE the current updated estimate
%
function [VEst,RMS,dNormUpdate,TIRefEst] = SSDUpdateNoMotionVelBlur( IRef, TIRef, TIRefMask, HRef, VEst, bRobust )

if nargin == 0
  fprintf( 'Launching test...');
  test();
  return;
end

if nargin < 6
  bRobust = 0;
end

nWidth  = size( TIRef, 2 );
nHeight = size( TIRef, 1 );

%[TIRefEst,TIMask] = blur_warping( IRef, HRef, VEst, eye(3), nWidth, nHeight );
[TIRefEst,TIMask] = fft_blur_warping( IRef, HRef, nWidth, nHeight, VEst, 0 );

TIIn = find( TIRefMask(:) == 1 & TIMask(:) == 1 );

di = TIRef( : ) - TIRefEst( : );
di = di( TIIn );

% Compute image Jacobian
%JBlur = -blur_warping( IRef, HRef, VEst, eye(3), nWidth, nHeight, 1 );
JBlur = -fft_blur_warping( IRef, HRef, nWidth, nHeight, VEst, 1 );
[Jcol,Jrow] = gradient( JBlur );

% Combine with homography Jacobian to obtain full update
Jh = Jhomography( Jcol, Jrow, nWidth, nHeight, 2 );
Jh = Jh( TIIn, : );

RMS = sqrt(mean(di.^2));

d = -pinv(Jh)*di;
dNormUpdate = norm( d );
VEst = VEst + LieH( d );

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function test( V )

if nargin == 0
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % Make matrix
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  V = zeros(3,3);
  V(1,3) = V(1,3)+15;
  V(2,3) = V(2,3)-5;
  
  test( V );
  return
end

load mandrill;
I = X;

nWidth  = 150;
nHeight = 200;
cx      = size( I, 1 )/2 - nWidth/2;
cy      = size( I, 2 )/2 - nHeight/2;

HRef = [ 1 0 cx;
         0 1 cy;
         0 0 1 ];

%TIRef = blur_warping( I, HRef, V, eye(3), nWidth, nHeight );
TIRef = fft_blur_warping( I, HRef, nWidth, nHeight, V, 0 );

figure
subplot( 1, 2, 1 );
imshow( uint8( TIRef ) );
title( 'Blurred template' )

VEst = zeros( 3 );
%VEst = V;
nMaxIter = 10;

fprintf( '\n' );
for ii=1:nMaxIter
  [VEst,RMS,dNormUpdate,TIRefEst] = ...
      SSDUpdateNoMotionVelBlur( I, TIRef, ones(size(TIRef)), HRef, VEst );

  subplot( 1, 2, 2 );
  imshow( uint8( TIRefEst ) );  
  title( 'Estimated blur' )
  drawnow
  
  fprintf( 'RMS %i: %f\n', ii, RMS );
end
fprintf( '\n' );

function [TIRefEst,TIMask] = fft_blur_warping( IRef, HRef, nWidth, nHeight, VEst, bJac ) 

if bJac
  if 0
    [TIRefEst,TIMask] = blur_warping( IRef, HRef, VEst, eye(3), nWidth, nHeight, 1 );
  else
    sType = 'blur_dir_diff';
    %sType = 'blur_gauss_diff';
    BIRef = fftBlur( IRef, VEst(1:2,3), sType );
    [TIRefEst,TIMask] = warping( BIRef, HRef, nWidth, nHeight );
  end
else
  sType = 'blur_dir';
  %sType = 'blur_gauss';
  BIRef = fftBlur( IRef, VEst(1:2,3), sType );
  [TIRefEst,TIMask] = warping( BIRef, HRef, nWidth, nHeight );
end
  
