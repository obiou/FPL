%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function [HEst,RMS,dNormUpdate,TICur] = SSDUpdateSL3Motion( TIRef, ICur, HRef, HEst, nNumParameters, bRobust )
%
% Compute a SL(3) ESM update (translation/affine/homography depending
% on nNumParameters).
%
% This update minimises | ICur( HRef*HEst x ) - TIRef( x ) |
%
% Input:
% - TIRef: reference template (extracted from which IRef, TIRef = IRef( HRef x ) )
% - ICur: current image
% - HRef: homography generating the reference template (TIRef = IRef( HRef x ) )
% - HEst: current estimate homography to obtain TIRef
% - nNumParameters: number of parameters to estimate for HEst
% (default: 8 )
% - bRobust: should m-estimators (Huber) be applied for the update?
%
% Output:
% - HEst: current estimate for the SL(3) transform (contains HRef)
% - RMS: residual error BEFORE the current updated estimate
% - dNormUpdate: norm of the computed update (can be used to compute a stopping criteria)
% - TICur: estimated template BEFORE the current updated estimate
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [HEst,RMS,dNormUpdate,TICur] = SSDUpdateSL3Motion( TIRef, ICur, HRef, HEst, nNumParameters, bRobust )

if nargin == 0
  fprintf( 'Launching test...');
  test();
  return;
end

if nargin<5
  nNumParameters = 8;
end
if nargin<6
  bRobust = 0;
end

nWidth  = size( TIRef, 2 );
nHeight = size( TIRef, 1 );

[TICur,TIMask] = warping( ICur, HRef*HEst, nWidth, nHeight );
TIIn = find( TIMask(:) == 1 );

di = TICur( : ) - TIRef( : );
di = di( TIIn );
RMS = sqrt(mean(di.^2));

[JColRef,JRowRef] = gradient( TIRef );
[JColCur,JRowCur] = gradient( TICur );

% Combine with homography Jacobian to obtain the full update
Jh = Jhomography( (JColRef+JColCur)/2, (JRowRef+JRowCur)/2, ...
                  nWidth, nHeight, nNumParameters );
Jh = Jh( TIIn, : );

if bRobust
  d = -MesHuber( Jh, di )
else
  d = -pinv(Jh)*di;
end
dNormUpdate = norm( d );
HEst = HEst*LieToH( d );

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function test( H, nNumParameters )

if nargin == 0
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % Make matrix
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %sl3_type = 'rot';
  %sl3_type = 'zoom';
  sl3_type = 'trans';

  switch sl3_type
    case 'rot'
      nNumParameters = 5;
      
      rot_ax = [0;0;1];
      angle = 5*pi/180;
      M = angle*skew(rot_ax);
      N = M*[150;200;0];
      M(1,3) = M(1,3)-N(1);
      M(2,3) = M(2,3)-N(2);
      H = expm( M );
    case 'zoom'
      nNumParameters = 8;
      
      M = zeros(3,3);
      M(1,1) = M(1,1)-0.05;
      M(2,2) = M(2,2)-0.1;
      N = M*[150;200;0];
      M(1,3) = M(1,3)-N(1);
      M(2,3) = M(2,3)-N(2);
      H = expm( M );
    case 'trans'
      nNumParameters = 2;

      H = eye(3);
      H(1,3) = H(1,3)+5;
      H(2,3) = H(2,3);
  end 

  test( H, nNumParameters );
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

TIRef = warping( I, HRef*H, nWidth, nHeight );
ICur  = I;

fig_est = figure;
subplot( 1, 2, 1 )
imshow( uint8( TIRef ) );
title( 'Initial template' )

HEst = eye(3);

nMaxIter = 30;
bRobust  = 0;

fprintf( '\n' );
for ii=1:nMaxIter
  [HEst,RMS,dNormUpdate,TICur] = ...
      SSDUpdateSL3Motion( TIRef, ICur, HRef, HEst, nNumParameters, bRobust );

  figure( fig_est )
  subplot( 1, 2, 2 )
  imshow( uint8( TICur ) );  
  title( 'Estimated template' )
  drawnow

  fprintf( 'RMS iter %i: %f\n', ii, RMS );
end
fprintf( '\n' );
%H
%inv( HRef )*HEst