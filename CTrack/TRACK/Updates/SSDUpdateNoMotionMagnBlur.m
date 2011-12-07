% function [lambdaEst,RMS,dNormUpdate,TIRefEst] = SSDUpdateNoMotionMagnBlur( IRef, TIRef, HRef, M, lambdaEst )
%
% Compute the blur magnitude given a particular motion (homography).
% The homography is not estimated, this function is mainly
% for debugging.
%
% Input:
% - IRef: unblurred image (from TIRef has been extracted)
% - TIRef: unblurred template
% - HRef: homography to obtain the unblurred template
% - M: blur estimate (in Lie algebra space)
% - lambdaEst: current blur magnitude estimate
%
% Output:
% - lambdaEst: current estimated blur magnitude
% - RMS: residual error BEFORE the current updated estimate
% - dNormUpdate: norm of the computed update (can be used to compute a stopping criteria)
% - TIRefEst: estimated template BEFORE the current updated estimate
%
function [lambdaEst,RMS,dNormUpdate,TIRefEst] = SSDUpdateNoMotionMagnBlur( IRef, TIRef, HRef, M, lambdaEst )

if nargin == 0
  fprintf( 'Launching test...' );
  test();
  return
end

nWidth  = size( TIRef, 2 );
nHeight = size( TIRef, 1 );

[TIRefEst,TIMask] = blur_warping( IRef, HRef, lambdaEst*M, eye(3), nWidth, nHeight );
TIIn = find( TIMask(:) == 1 );

di  = TIRef( : ) - TIRefEst( : );
di  = di( TIIn );
RMS = sqrt(mean(di.^2));

Jl  = JacobianBlurMagn( IRef, HRef, nWidth, nHeight, M, lambdaEst );
Jl  = Jl( TIIn, : );

d   = -pinv(-Jl)*di;
dNormUpdate = norm( d );
lambdaEst   = lambdaEst + d;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Calculates the Jacobian of the refence part of the cost function
% with respect to the magnitude
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function Jl = JacobianBlurMagn( IRef, HRef, nWidth, nHeight, M, lambda )
% Calculate gradient PSF
bJac = 1;
JBlur = blur_warping( IRef, HRef, lambda*M, eye(3), nWidth, nHeight, bJac );

[Jcol,Jrow] = gradient( JBlur );

Jcol = Jcol(:); Jrow = Jrow(:);

% Calculate Jacobian with respect to lambda
% [dBIx dBIy -(x dBIx + y dBIy)]*M*p
[col,row] = meshgrid( 1:nWidth, 1:nHeight );
col = col(:); row = row(:);

vcol = M(1,1)*col + M(1,2)*row + M(1,3);
vrow = M(2,1)*col + M(2,2)*row + M(2,3);
vz   = M(3,1)*col + M(3,2)*row + M(3,3);

Jl = Jcol.*vcol + Jrow.*vrow - (col.*Jcol+row.*Jrow).*vz;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function test( M, lambda )

if nargin == 0
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % Make matrix
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %blur_type = 'rot';
  %blur_type = 'zoom';
  blur_type = 'trans';

  switch blur_type
    case 'rot'
      rot_ax = [0;0;1];
      angle = 10*pi/180;
      M = angle*skew(rot_ax);
      N = M*[100;100;0];
      M(1,3) = M(1,3)-N(1);
      M(2,3) = M(2,3)-N(2);
    case 'zoom'
      M = zeros(3,3);
      M(1,1) = M(1,1)-0.3;
      M(2,2) = M(2,2)-0.3;
      N = M*[150/2;150/2;0];
      M(1,3) = M(1,3)-N(1);
      M(2,3) = M(2,3)-N(2);
    case 'trans'
      M = zeros(3,3);
      M(1,3) = M(1,3)+20;
      M(2,3) = M(2,3);
  end 
  lambda = 0.9;
  
  test( M, lambda );
  return
end

load mandrill;
I = X;

nWidth  = 150;
nHeight = 200;
cx      = size( I, 1 )/2 - nWidth/2;
cy      = size( I, 2 )/2 - nHeight/2 + 70;

HRef = [ 1 0 cx;
         0 1 cy;
         0 0 1 ];

TIRef = blur_warping( I, HRef, lambda*M, eye(3), nWidth, nHeight );

figure
subplot( 1, 3, 1 );
imshow( uint8( TIRef ) );
title( 'Optimal blurred template' )

lambdaEst = 0;

subplot( 1, 3, 2 );
imshow( uint8( blur_warping( I, HRef, lambdaEst*M, eye(3), nWidth, nHeight ) ) );
title( 'Template before min. (lambdaEst=0)' )

nMaxIter = 15;

fprintf( '\n' );
for ii=1:nMaxIter
  [lambdaEst,RMS,dNormUpdate,TIRefEst] = ...
      TrackingUpdateNoMotionMagnBlur( I, TIRef, HRef, M, lambdaEst );

  subplot( 1, 3, 3 );
  imshow( uint8( TIRefEst ) );  
  title( 'Estimated blur' )
  drawnow
  
  fprintf( 'RMS %i: %f\n', ii, RMS );
end
fprintf( '\n' );
lambda
lambdaEst