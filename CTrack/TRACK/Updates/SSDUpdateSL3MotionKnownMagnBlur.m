%[HEst,RMS,dNormUpdate,TIRefEst,TICurEst] = TrackingUpdateSL3MotionKnownMagnBlur( IRef, ICur, HRef, HPrev,
%                                           nWidth, nHeight, HEst, lambda, nNumParametersH, bHPrev )
%
% Compute an inter-frame homography assuming known blur magnitude
% (ie % of time the shutter is open during the image acquisition).
%
% Input:
% - IRef: unblurred image (from which TIRef has been extracted)
% - ICur: blurrend current template
% - HRef: homography to obtain the reference template
% - HPrev: previous homography estimate
% - nWidth/nHeight: template width and height (with HRef this
%                   defines the tracked region)
% - HEst : current homography estimate
% - lambda: given blur magnitude (not estimated)
% - nNumParametersH: number of parameters to estimate for the
% homography (default: 8)
% - bHPrev: switches between minimising 'inv(HPrev)*expm( )*HPrev' and simply 'expm()'
%
% Output:
% - HEst: current homography estimate
% - RMS: residual error BEFORE the current updated estimate
% - dNormUpdate: norm of the computed update (can be used to compute a stopping criteria)
% - TIRefEst: blurred reference template based on homography estimate BEFORE the current update
% - TICurEst: estimated template BEFORE the current updated estimate
%
function [HEst,RMS,dNormUpdate,TIRefEst,TICurEst] = SSDUpdateSL3MotionKnownMagnBlur( IRef, ICur, HRef, HPrev, ...
                                                  nWidth, nHeight, HEst, lambda, nNumParametersH, bHPrev )

if nargin == 0
  fprintf( 'Launching test...' );
  test();
  return
end

if nargin < 8
  nNumParametersH = 8;
end

M = logm( inv(HPrev)*HEst );

[TIRefEst,TIRefMask] = blur_warping( IRef, HRef*inv(HPrev), lambda*M, HPrev, nWidth, nHeight );
[TICurEst,TICurMask] = warping( ICur, HRef*HEst, nWidth, nHeight );
TIIn = find( TIRefMask( : ) == 1 & TICurMask( : ) == 1 );

di  = TICurEst( : ) - TIRefEst( : );
di  = di( TIIn );
RMS = sqrt( mean( di.^2 ) );

JFull  = JacobianBlurMagn( IRef, TIRefEst, TICurEst, HRef, HPrev, nWidth, nHeight, M, lambda, nNumParametersH, bHPrev );
JFull = JFull( TIIn, : );

d = -pinv( JFull )*di;
dNormUpdate = norm( d );
HEst   = HEst*LieToH( d );

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Calculates the Jacobian of the current and reference parts of the
% cost function with respect to the magnitude
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function JFull = JacobianBlurMagn( IRef, TIRefEst, TICurEst, HRef, HPrev, nWidth, nHeight, M, lambda, nNumParametersH, bHPrev )
% Calculate gradient PSF
bJac = 1;
if bHPrev
  JBlur = blur_warping( IRef, HRef*inv(HPrev), lambda*M, HPrev, nWidth, nHeight, bJac );
else
  JBlur = blur_warping( IRef, HRef, lambda*M, eye(3), nWidth, nHeight, bJac );
end
[Jcol,Jrow] = gradient( JBlur );

Jcol = Jcol(:); Jrow = Jrow(:);

% Calculate Jacobian with respect to the homography
% parameterisation
% You can test here the two different Jacobians.
% JhomographyAd is the 'correct' one but the difference is not
% major (and 'far' from the solution the approximate value sometimes
% converges faster).
%
if bHPrev
  JBlur = lambda*JhomographyAd( Jcol, Jrow, HPrev, nWidth, nHeight, nNumParametersH );
else
  JBlur = lambda*Jhomography( Jcol, Jrow, nWidth, nHeight, nNumParametersH );
end
% Compute current image Jacobian
[JColRef,JRowRef] = gradient( TIRefEst );
[JColCur,JRowCur] = gradient( TICurEst );

% Combine with homography Jacobian to obtain the full update
JCur = Jhomography( (JColRef+JColCur)/2, (JRowRef+JRowCur)/2, ...
                    nWidth, nHeight, nNumParametersH );

JFull = JCur - JBlur;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function test( H, lambda, nNumParametersH )

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
      angle = 35*pi/180;
      M = angle*skew(rot_ax);
      N = M*[150;200;0];
      M(1,3) = M(1,3)-N(1);
      M(2,3) = M(2,3)-N(2);
      H = expm( M );
    case 'zoom'
      nNumParametersH = 8;
      
      M = zeros(3,3);
      M(1,1) = M(1,1)-0.15;
      M(2,2) = M(2,2)-0.1;
      N = M*[150;200;0];
      M(1,3) = M(1,3)-N(1);
      M(2,3) = M(2,3)-N(2);
      H = expm( M );
    case 'trans'
      nNumParametersH = 2;

      H = eye(3);
      H(1,3) = H(1,3)+15;
      H(2,3) = H(2,3);
  end 
  lambda = 1;

  test( H, lambda, nNumParametersH );
  return
end

load mandrill;
I            = X;
nImageWidth  = size( I, 2 );
nImageHeight = size( I, 1 );
%I            = crop( I, 100, 100 ); % for speed
%nImageWidth  = size( I, 2 );
%nImageHeight = size( I, 1 );
nWidth       = 150;
nHeight      = 150;
cx           = nImageWidth/2 - nWidth/2;
cy           = nImageHeight/2 - nHeight/2 + 70;

HRef = [ 1 0 cx;
         0 1 cy;
         0 0 1 ];

if 1
  if 0
    HPrev = [ 1 0 50;
              0 1 -35;
              0 0 1];
  else
    rot_ax = [0;0;1];
    angle = -10*pi/180;
    HPrev = expm( angle*skew(rot_ax) );
  end
  H = HPrev*H;
else
  HPrev = eye(3);
end

%
% Building simulation values: see last section of TRO article
%
bBlur = 1;
% Add blur to avoid aliasing effects due to double warping
% to build the current image and then current template
if bBlur
  h = fspecial( 'gaussian', 5, 5 );
  I = imfilter( I, h );
end

IRef = I;
ICur  = blur_warping( I, HRef*inv(HPrev), lambda*logm( inv(HPrev)*H ), HPrev*inv(H)*inv(HRef), nImageWidth, nImageHeight );

TIRef = blur_warping( IRef, HRef*inv(HPrev), lambda*logm( inv(HPrev)*H ), HPrev, nWidth, nHeight );
TICur = warping( ICur, HRef*H, nWidth, nHeight );

absDiff = abs( TIRef(:) - TICur(:) );

fprintf( 'Simulation errors: max: %f, RMS: %f, median: %f.\n', ...
         max( absDiff ), sqrt( mean( absDiff.^2  ) ), median( absDiff ) );

fig_est = figure;
subplot( 3, 2, 1 );
imshow( uint8( TIRef ) );
title( 'Optimal blurred reference template (M=lambda*H)' )

subplot( 3, 2, 2 );
imshow( uint8( TICur ) );
title( 'Optimal warped current template (HEst=H)' )

subplot( 3, 2, 3 );
TIRef = warping( IRef, HRef, nWidth, nHeight );
imshow( uint8( TIRef ) );
title( 'Reference template (HEst=Id)' )

TICur = warping( ICur, HRef*HPrev, nWidth, nHeight );
subplot( 3, 2, 4 );
imshow( uint8( TICur ) );
title( 'Current template HEst=Id' )

drawnow

% Initialise values
HPrev    = HPrev;
HEst     = HPrev;
nMaxIter = 10;

fprintf( '\n' );

for ii=1:nMaxIter
  [HEst,RMS,dNormUpdate,TIRefEst,TICurEst] = SSDUpdateSL3MotionKnownMagnBlur( IRef, ICur, HRef, HPrev, nWidth, nHeight,...
                                                    HEst, lambda, nNumParametersH );
  subplot( 3, 2, 5 )
  imshow( uint8( TIRefEst ) );  
  title( 'Reference template (for HEst)' )
  subplot( 3, 2, 6 )
  imshow( uint8( TICurEst ) );  
  title( 'Warped current template (using HEst)' )  
  drawnow
    
  fprintf( ' RMS %i: %f\n', ii, RMS );
end
fprintf( '\n' )

%H
%invHRef_HEst = HEst

