% function [HEst,MEst,RMS,dNormUpdate,TIRefEst,TICurEst] = SSDUpdateSL3MotionSL3Blur( IRef, ICur, HRef, nWidth,
% nHeight, HEst, MEst, nNumParametersH, nNumParametersM, bRobust )
%
% Estimate a motion and independent blur.
%
% Input:
% - IRef: unblurred image
% - ICur: blurred image
% - HRef: homography to obtain the unblurred template
% - nWidth/nHeight: template width and height (with HRef this
%                   defines the tracked region)
% - HEst: current homography estimate
% - MEst: current blur estimate (in Lie algebra space)
% - nNumParametersH: number of parameters to estimate for HEst
% (default: 8 )
% - nNumParametersM: number of parameters to estimate for MEst
% (default: 8 )
% - bRobust: (optional, default 'false') apply m-estimators (Huber)
%
% Output:
% - HEst: current estimated homography
% - MEst: current estimated blur
% - RMS: residual error BEFORE the current updated estimate
% - dNormUpdate: norm of the computed update (can be used to compute a stopping criteria)
% - TIRefEst: estimated template BEFORE the current updated estimate
% - TICurEst: estimated template BEFORE the current updated estimate
%
function [HEst,MEst,RMS,dNormUpdate,TIRefEst,TICurEst] = ...
    SSDUpdateSL3MotionSL3Blur( IRef, ICur, HRef, nWidth, nHeight, HEst, MEst, ...
                               nNumParametersH, nNumParametersM, bRobust )

if nargin == 0
  fprintf( 'Launching test...\n');
  test();
  return;
end

if nargin < 10
  bRobust = 0;
end

[TIRefEst,TIRefMask] = blur_warping( IRef, HRef, MEst, eye(3), nWidth, nHeight );
[TICurEst,TICurMask] = warping( ICur, HRef*HEst, nWidth, nHeight );
TIIn = find( TIRefMask( : ) == 1 & TICurMask( : ) == 1 );

di = TICurEst( : ) - TIRefEst( : );
di = di( TIIn );
RMS = sqrt( mean( di.^2 ) );

% Compute reference image Jacobian
bDiff = 1;
JBlur = blur_warping( IRef, HRef, MEst, eye(3), nWidth, nHeight, bDiff );
[JCol,JRow] = gradient( JBlur );
% Combine with homography Jacobian
JBlurFull = Jhomography( JCol, JRow, nWidth, nHeight, nNumParametersM );

% Compute current image Jacobian
[JColRef,JRowRef] = gradient( TIRefEst );
[JColCur,JRowCur] = gradient( TICurEst );

% Combine with homography Jacobian to obtain the full update
JCur = Jhomography( (JColRef+JColCur)/2, (JRowRef+JRowCur)/2, ...
                    nWidth, nHeight, nNumParametersH );

J = [ JCur -JBlurFull ];
J = J( TIIn, : );

d = -pinv( J )*di;
dNormUpdate = norm( d );
dH = d( 1:nNumParametersH );
dM = d( nNumParametersH+1:nNumParametersH+nNumParametersM );
HEst = HEst*LieToH( dH );
MEst = MEst + LieH( dM );

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function test( H, M, nNumParametersH, nNumParametersM )

if nargin == 0  
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % Make homography matrix
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %sl3_type = 'rot';
  sl3_type = 'zoom';
  %sl3_type = 'trans';

  switch sl3_type
    case 'rot'
      nNumParametersH = 5;
      
      rot_ax = [0;0;1];
      angle = 5*pi/180;
      M = angle*skew(rot_ax);
      N = M*[150;200;0];
      M(1,3) = M(1,3)-N(1);
      M(2,3) = M(2,3)-N(2);
      H = expm( M );
    case 'zoom'
      nNumParametersH = 8;
      
      M = zeros(3,3);
      M(1,1) = M(1,1)-0.05;
      M(2,2) = M(2,2)-0.1;
      N = M*[150;200;0];
      M(1,3) = M(1,3)-N(1);
      M(2,3) = M(2,3)-N(2);
      H = expm( M );
      H = H/det(H)^(1/3);
    case 'trans'
      nNumParametersH = 2;

      H = eye(3);
      H(1,3) = H(1,3)+5;
      H(2,3) = H(2,3);
  end 

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % Make blur matrix
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  blur_type = 'rot';
  %blur_type = 'zoom';
  %blur_type = 'trans';

  switch blur_type
    case 'rot'
      nNumParametersM = 5;
      
      rot_ax = [0;0;1];
      angle = 15*pi/180;
      M = angle*skew(rot_ax);
      N = M*[100;100;0];
      M(1,3) = M(1,3)-N(1);
      M(2,3) = M(2,3)-N(2);
    case 'zoom'
      nNumParametersM = 8;
      
      M = zeros(3,3);
      M(1,1) = M(1,1)-0.2;
      M(2,2) = M(2,2)-0.2;
      N = M*[150/2;150/2;0];
      M(1,3) = M(1,3)-N(1);
      M(2,3) = M(2,3)-N(2);
    case 'trans'
      nNumParametersM = 2;

      M = zeros(3,3);
      M(1,3) = M(1,3)+20;
      M(2,3) = M(2,3);
  end 

  test( H, M, nNumParametersH, nNumParametersM );
  return
end

load mandrill;
I            = X;
nImageWidth  = size( I, 2 );
nImageHeight = size( I, 1 );
nWidth       = 150;
nHeight      = 150;
cx           = nImageWidth/2 - nWidth/2;
cy           = nImageHeight/2 - nHeight/2 + 70;

HRef = [ 1 0 cx;
         0 1 cy;
         0 0 1 ];

%
% Build simulation values: see last section of TRO article
%
%M = zeros(3);
%H = eye(3);
bNew = 1; bBlur = 1;
if bNew
  % Add blur to avoid aliasing effects due to double warping
  % to build the current image and then current template
  if bBlur
    h = fspecial( 'gaussian', 5, 5 );
    I = imfilter( I, h );
  end

  IRef  = I;
  ICur  = blur_warping( I, HRef, M, inv(H)*inv(HRef), nImageWidth, nImageHeight );
  
  TIRef = blur_warping( IRef, HRef, M, eye(3), nWidth, nHeight );
  TICur = warping( ICur, HRef*H, nWidth, nHeight );
else
  % Old method, only valid if M or H are neutral values
  IRef = warping( I, HRef*H*inv(HRef), nImageWidth, nImageHeight );
  ICur = blur_warping( I, HRef, M, inv(HRef), nImageWidth, nImageHeight );

  TIRef = blur_warping( IRef, HRef, M, eye(3), nWidth, nHeight );
  TICur = warping( ICur, HRef*H, nWidth, nHeight );
end

absDiff = abs( TIRef(:) - TICur(:) );

fprintf( 'Simulation errors: max: %f, RMS: %f, median: %f.\n', ...
         max( absDiff ), sqrt( mean( absDiff.^2  ) ), median( absDiff ) );

fig_est = figure;
subplot( 3, 2, 1 );
imshow( uint8( TIRef ) );
title( 'Optimal blurred reference template (MEst=M)' )

subplot( 3, 2, 2 );
imshow( uint8( TICur ) );
title( 'Optimal warped current template (HEst=H)' )

subplot( 3, 2, 3 );
TIRef = warping( IRef, HRef, nWidth, nHeight );
imshow( uint8( TIRef ) );
title( 'Reference template (MEst=0)' )

TICur = warping( ICur, HRef, nWidth, nHeight );
subplot( 3, 2, 4 );
imshow( uint8( TICur ) );
title( 'Current template HEst=Id' )

% Initialise values
HEst = eye(3);
MEst = zeros( 3 );
%HEst = H;
%MEst = M;

nMaxIter = 15;

% We use an alternation scheme to increase the basin of convergence
% Start by only estimating H (for stability)
nMaxIterPrev = 5;
for ii=1:nMaxIterPrev
  [HEst,RMS,dNormUpdate,TICurEst] = SSDUpdateSL3Motion( TIRef, ICur, HRef, HEst, nNumParametersH );
  subplot( 3, 2, 5 )
  imshow( uint8( TIRef ) );  
  title( 'Reference template (no estimated blur)' )
  subplot( 3, 2, 6 )
  imshow( uint8( TICurEst ) );  
  title( 'Warped current template' )  
  drawnow
  
  fprintf( '\r RMS %i (only H): %f', ii, RMS );
end
fprintf( '\n' )

% Now only estimating M (for stability)
nMaxIterPrev = 5;
for ii=1:nMaxIterPrev
  [MEst,RMS,dNormUpdate,TIRefEst] = SSDUpdateNoMotionSL3Blur( IRef, TICurEst, HRef, MEst, nNumParametersM );
  subplot( 3, 2, 5 )
  imshow( uint8( TIRefEst ) );  
  title( 'Reference template (no estimated blur)' )
  subplot( 3, 2, 6 )
  imshow( uint8( TICurEst ) );  
  title( 'Warped current template' )  
  drawnow
  fprintf( '\r RMS %i (only M): %f', ii, RMS );
end
fprintf( '\n' )

% Full estimation
for ii=1:nMaxIter
  [HEst,MEst,RMS,dNormUpdate,TIRefEst,TICurEst] = ...
      SSDUpdateSL3MotionSL3Blur( IRef, ICur, HRef, nWidth, nHeight, HEst, MEst, ...
                                 nNumParametersH, nNumParametersM );

  subplot( 3, 2, 5 )
  imshow( uint8( TIRefEst ) );  
  title( 'Reference template with estimated blur' )
  subplot( 3, 2, 6 )
  imshow( uint8( TICurEst ) );  
  title( 'Warped current template' )  
  drawnow
  
  fprintf( '\r RMS %i: %f', ii, RMS );
end
fprintf( '\n' );
H
HEst
M
MEst
