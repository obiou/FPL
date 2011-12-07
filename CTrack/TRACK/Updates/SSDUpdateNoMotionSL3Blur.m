% function [MEst,RMS,dNormUpdate,TIRefEst] = SSDUpdateNoMotionSL3Blur( IRef, TIRef, HRef, MEst, nNumParameters )
%
% Compute a blur matrix update.
% The homography is not estimated, this function can be used for an
% alternation scheme or for debugging.
%
% Input:
% - IRef: unblurred image (from TIRef has been extracted)
% - TIRef: unblurred template
% - TIRefMask: binary image of the same size as TIRef with 0 for the parts
%   of TIRef that should be ignored and 1 for the rest
% - HRef: homography to obtain the unblurred template
% - MEst: current blur estimate (in Lie algebra space)
% - nNumParameters: number of parameters to estimate for MEst
% (default: 8 )
%
% Output:
% - MEst: current estimated blur
% - RMS: residual error BEFORE the current updated estimate
% - dNormUpdate: norm of the computed update (can be used to compute a stopping criteria)
% - TIRefEst: estimated template BEFORE the current updated estimate
%
function [MEst,RMS,dNormUpdate,TIRefEst] = SSDUpdateNoMotionSL3Blur( IRef, TIRef, TIRefMask, HRef, MEst, nNumParameters, bRobust )

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

[TIRefEst,TIMask] = blur_warping( IRef, HRef, MEst, eye(3), nWidth, nHeight );
TIIn = find( TIRefMask(:) == 1 & TIMask(:) == 1 );

di = TIRef( : ) - TIRefEst( : );
di = di( TIIn );

% Compute image Jacobian
JBlur = -blur_warping( IRef, HRef, MEst, eye(3), nWidth, nHeight, 1 );
[Jcol,Jrow] = gradient( JBlur );

% Combine with homography Jacobian to obtain full update
Jh = Jhomography( Jcol, Jrow, nWidth, nHeight, nNumParameters );
Jh = Jh( TIIn, : );

RMS = sqrt(mean(di.^2));

d = -pinv(Jh)*di;
dNormUpdate = norm( d );
MEst = MEst + LieH( d );
% Better:
% MEst = logm( expm(MEst)*LieToH( d ) );

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function test( M, nNumParameters )

if nargin == 0
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % Make matrix
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  blur_type = 'rot';
  %blur_type = 'zoom';
  %blur_type = 'trans';

  switch blur_type
    case 'rot'
      nNumParameters = 5;
      
      rot_ax = [0;0;1];
      angle = 10*pi/180;
      M = angle*skew(rot_ax);
      N = M*[100;100;0];
      M(1,3) = M(1,3)-N(1);
      M(2,3) = M(2,3)-N(2);
    case 'zoom'
      nNumParameters = 8;
      
      M = zeros(3,3);
      M(1,1) = M(1,1)-0.3;
      M(2,2) = M(2,2)-0.3;
      N = M*[150/2;150/2;0];
      M(1,3) = M(1,3)-N(1);
      M(2,3) = M(2,3)-N(2);
    case 'trans'
      nNumParameters = 2;

      M = zeros(3,3);
      M(1,3) = M(1,3)+20;
      M(2,3) = M(2,3);
  end 

  test( M, nNumParameters );
  return
end

if nargin == 1
  nNumParameters = 8;
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

TIRef = blur_warping( I, HRef, M, eye(3), nWidth, nHeight );

figure
subplot( 1, 2, 1 );
imshow( uint8( TIRef ) );
title( 'Blurred template' )

MEst = zeros( 3 );

nMaxIter = 15;

fprintf( '\n' );
for ii=1:nMaxIter
  [MEst,RMS,dNormUpdate,TIRefEst] = ...
      SSDUpdateNoMotionSL3Blur( I, TIRef, ones(size(TIRef)), HRef, MEst, nNumParameters );

  subplot( 1, 2, 2 );
  imshow( uint8( TIRefEst ) );  
  title( 'Estimated blur' )
  drawnow
  
  fprintf( 'RMS %i: %f\n', ii, RMS );
end
fprintf( '\n' );
