%[HEst,lambdaEst,RMS,dNormUpdate,TIRefEst,TICurEst] = SSDUpdateSL3MotionMagnBlur( IRef, ICur, HRef, nWidth, nHeight, HEst, lambdaEst, nNumParametersH )
%
% Compute an inter-frame homography and a blur magnitude
% (ie % of time the shutter is open during the image acquisition)
% dependent on the motion.
%
% The following equation is minimised over HEst:
% \| ICur( HRef*HEst ) - \int IRef( HRef*expm( lambdaEst*logm( inv(HPrev)*HEst ) t ) x ) dt \|
%
% Input:
% - IRef: unblurred image 
% - ICur: blurrend current template
% - HRef: homography to obtain the reference template
% - HPrev: previous homography estimate (to ensure the blur only uses the update inv(HPrev)*HEst)
% - nWidth/nHeight: template width and height (with HRef this
%                   defines the tracked region)
% - HEst : current homography estimate
% - lambdaEst: current blur magnitude estimate
% - nNumParametersH: number of parameters to estimate for the
% homography (default: 8)
%
% Output:
% - HEst: current homography estimate
% - lambdaEst: current blur magnitude estimate
% - RMS: residual error BEFORE the current updated estimate
% - dNormUpdate: norm of the computed update (can be used to compute a stopping criteria)
% - TIRefEst: blurred reference template based on homography estimate BEFORE the current update
% - TICurEst: estimated template BEFORE the current updated estimate
%
function [HEst,lambdaEst,RMS,dNormUpdate,TIRefEst,TICurEst] = ...
    SSDUpdateSL3MotionMagnBlur( IRef, ICur, HRef, HPrev, ...
                                nWidth, nHeight, HEst, lambdaEst, nNumParametersH, bRobust )

if nargin == 0
  fprintf( 'Launching test...\n' );
  test();
  return
end

if nargin < 8
  nNumParametersH = 8;
end
if nargin < 9
  bRobust = 0;
end

M = logm( inv(HPrev)*HEst );

%[TIRefEst,TIRefMask] = blur_warping( IRef, HRef, lambdaEst*M, eye(3), nWidth, nHeight );
[TIRefEst,TIRefMask] = blur_warping( IRef, HRef*inv(HPrev), lambdaEst*M, HPrev, nWidth, nHeight );
[TICurEst,TICurMask] = warping( ICur, HRef*HEst, nWidth, nHeight );
TIIn = find( TIRefMask( : ) == 1 & TICurMask( : ) == 1 );

di  = TICurEst( : ) - TIRefEst( : );
di  = di( TIIn );
RMS = sqrt(mean(di.^2));

[JFull JLambda] = JacobianBlurMagn( IRef, TIRefEst, TICurEst, HRef, HPrev, nWidth, nHeight, M, lambdaEst, nNumParametersH );
J = [JFull -JLambda];
J = J( TIIn, : );

d = -pinv( J )*di;
dNormUpdate = norm( d );
if( ( lambdaEst + d( nNumParametersH+1 ) > 1 ) || ...
    ( lambdaEst + d( nNumParametersH+1 ) < 0 ) )
  
  d = -pinv( JFull( TIIn, : ) )*di;
  dNormUpdate = norm( d );
  HEst      = HEst*LieToH( d );
else
  HEst      = HEst*LieToH( d( 1:nNumParametersH ) );
  lambdaEst = lambdaEst + d( nNumParametersH+1 );
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Calculates the Jacobian of the current and reference parts of the
% cost function with respect to the magnitude
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [JFull JLambda]= JacobianBlurMagn( IRef, TIRefEst, TICurEst, HRef, HPrev, nWidth, nHeight, M, lambdaEst, nNumParametersH )
% Calculate gradient PSF
bJac = 1;
JBlur = blur_warping( IRef, HRef*inv(HPrev), lambdaEst*M, HPrev, nWidth, nHeight, bJac );

[Jcol,Jrow] = gradient( JBlur );

Jcol = Jcol(:); Jrow = Jrow(:);

% Calculate Jacobian with respect to the homography
% parameterisation
JBlur = lambdaEst*Jhomography( Jcol, Jrow, nWidth, nHeight, nNumParametersH );

% Calculate Jacobian with respect to lambda
% [dBIx dBIy -(x dBIx + y dBIy)]*H*p
[col,row] = meshgrid( 1:nWidth, 1:nHeight );
col = col(:); row = row(:);

vcol = M(1,1)*col + M(1,2)*row + M(1,3);
vrow = M(2,1)*col + M(2,2)*row + M(2,3);
vz   = M(3,1)*col + M(3,2)*row + M(3,3);

JLambda = Jcol.*vcol + Jrow.*vrow - (col.*Jcol+row.*Jrow).*vz;

% Compute current image Jacobian
[JColRef,JRowRef] = gradient( TIRefEst );
[JColCur,JRowCur] = gradient( TICurEst );

% Combine with homography Jacobian to obtain the full update
if 1
  JCur = Jhomography( (JColRef+JColCur)/2, (JRowRef+JRowCur)/2, ...
                      nWidth, nHeight, nNumParametersH );
else
  JCur = Jhomography( JColCur, JRowCur, ...
                      nWidth, nHeight, nNumParametersH );  
end
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
      angle = 10*pi/180;
      M = angle*skew(rot_ax);
      N = M*[150;200;0];
      M(1,3) = M(1,3)-N(1);
      M(2,3) = M(2,3)-N(2);
      H = expm( M );
    case 'zoom'
      nNumParametersH = 8;
      
      M = zeros(3,3);
      M(1,1) = M(1,1)-0.2;
      M(2,2) = M(2,2)-0.2;
      N = M*[150;200;0];
      M(1,3) = M(1,3)-N(1);
      M(2,3) = M(2,3)-N(2);
      H = expm( M );
      H = H/det(H)^(1/3);
    case 'trans'
      nNumParametersH = 2;

      H = eye(3);
      H(1,3) = H(1,3)+30;
      H(2,3) = H(2,3)-5;
  end 
  lambda = 0.5;
  
  test( H, lambda, nNumParametersH );
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

if 1
  HPrev = [ 1 0 20;
           0 1 -15;
           0 0 1];
  H = HPrev*H;
else
  HPrev = eye(3);
end

%
% Building simulation values: see last section of TRO article
%
%M = zeros(3);
%H = eye(3);
bBlur = 1;
% Add blur to avoid aliasing effects due to double warping
% to build the current image and then current template
if bBlur
  h = fspecial( 'gaussian', 5, 5 );
  I = imfilter( I, h );
end

IRef  = I;
ICur  = blur_warping( I, HRef*inv(HPrev), lambda*logm( inv(HPrev)*H ), HPrev*inv( H )*inv( HRef ), nImageWidth, nImageHeight );

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
HPrev     = HPrev;
HEst      = HPrev;
lambdaEst = 1;
nMaxIter  = 10;

for ii=1:nMaxIter
  [HEst,lambdaEst,RMS,dNormUpdate,TIRefEst,TICurEst] = ...
      SSDUpdateSL3MotionMagnBlur( IRef, ICur, HRef, HPrev, nWidth, nHeight,...
                                  HEst, lambdaEst, nNumParametersH );

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

H
HEst
lambda
lambdaEst
