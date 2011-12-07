%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function [TIBlur,mask] = blur_warping( I, H, M, nWidth, nHeight, ...
%                                        bJacobian, dStep, warp_type, mask_im )
%
% Warps an image patch according to a homography H and blurs
% using a matrix M:
% TIBlur( x ) = \int_0^1 I( Hl*expm( -t*M )*Hr x ) dt. 
%
% Mask indicates the points inside and outside the image
% 1 : point is good
% 0 : point is bad
%
% If no images are provided, an example is given with a 45deg rotation
% with 10% blur of a 200x200 patch in the center of the Mandrill image.
%
% Parameters:
% * I: initial image
% * Hl: homography between the initial image and new image ie x_new = H x_old
% * M: blur matrix (Lie algebra value correponding to a constant velocity model)
% * Hr: extra right transform
% * nWidth: output width
% * nHeight: output height
% * bJacobian: compute Jacobian (default: false)
% * dStep: pixel step for computing the increment (default: 1)
% * warp_type: (optional) 'nearest', 'linear' (default: 'linear')
% * mask_im: (optional) mask in the initial image (default: entire template)
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [TIBlur,mask] = blur_warping( I, Hl, M, Hr, nWidth, nHeight, ...
                                       bJacobian, dStep, warp_type, mask_im )

if nargin == 0
  fprintf( 'Launching test...\n' );
  test();
  return;
end

if nargin<6
  help blur_warping;
  error( 'Not enough inputs!' );
end  
if nargin<7
  bJacobian = 0;
end
if nargin<8
  dStep = 1; % Step in pixels between samples
end  
if nargin<9
  warp_type = 'linear';
end
if nargin<10
  mask_im = ones( size( I ) );
end

% Compute increment magnitude based on corner points and center point
mTestPoints = [ 1 nWidth nWidth  1       nWidth/2;
                1 1      nHeight nHeight nHeight/2 ];

s = 1/dStep*max( max( abs( mTestPoints- metric(expm(M)*projective( mTestPoints )) ) ) );
if s==0 || isnan( s )
  fact = 0;
else
  fact = 1/s;
end

if 0
  mWarpedTestPoints = metric( expm( M )*projective( mTestPoints ) )
  mIncrTestPoints   = metric( expm( fact*M )*projective( mTestPoints ) )

  figure;
  imshow( uint8( I ) );
  hold on;
  plot( mTestPoints( 1, : ), mTestPoints( 2, : ), 'rx' );
  plot( mWarpedTestPoints( 1, : ), mWarpedTestPoints( 2, : ), 'gx' );
  plot( mIncrTestPoints( 1, : ), mIncrTestPoints( 2, : ), 'm+' );
  
  axis on;
  grid on;
end

% Compute all warps
allMults = 0:fact:1;
if isempty( allMults ) 
  allMults = [0 1];
elseif allMults( end ) ~= 1;
  allMults = [ allMults 1 ];
end

TIBlur   = zeros( size( nWidth, nHeight ) );
sum_mask = zeros( size( nWidth, nHeight ) );
mask     = ones( size( TIBlur ) );

%
% About the template weighting: it is useful when the blur is
% anisotropic but can also be replaced by a *lot* of samples...
%
for dMult = allMults
  %
  % WARNING: it can seem confusing that there is NO '-' sign
  % This is normal: warping takes values in the blurred image (and
  % not the reference image).
  %
  % x_current = cHb x_blurred
  %
  % \int I( bHc x_current ) dt (hence the inverse)
  [wI,cur_mask,nx,ny] = warping( I, Hl*expm( dMult*M )*Hr, nWidth, nHeight, warp_type, mask_im );
  if bJacobian
    % Trapezium rule
    if dMult == 0 || dMult == 1
      TIBlur   = TIBlur + 0.5*dMult*wI;
      sum_mask = sum_mask + 0.5*mask;
    else
       TIBlur   = TIBlur + dMult*wI;
       sum_mask = sum_mask + mask;
    end
  else
    % Trapezium rule
    if dMult == 0 || dMult == 1
      TIBlur   = TIBlur + 0.5*wI;
      sum_mask = sum_mask + 0.5*mask;
    else
      TIBlur   = TIBlur + wI;
      sum_mask = sum_mask + mask;
    end
  end
  mask = mask & cur_mask;
end

mask           = mask & sum_mask ~= 0;
good           = find( mask == 1 ); 
bad            = find( mask == 0 ); 
TIBlur( good ) = TIBlur( good )./sum_mask( good );
TIBlur( bad )  = 255;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function test()

load mandrill;

width  = 200;
height = 200;
cx     = size( X, 1 )/2 - width/2;
cy     = size( X, 2 )/2 - height/2;
perc   = 0.2;

HRef = [ 1 0 cx;
         0 1 cy;
         0 0 1 ];

initial_patch = warping( X, HRef, width, height );

figure
imshow( uint8( X ) )
title( 'Initial image' )

figure
imshow( uint8( initial_patch ) )
title( 'Initial patch' )


Hc = [ 1 0 -width/2;
       0 1 -height/2;
       0 0 1 ];

v = sqrt(2)/2;

R = [ v v 0;
      -v v 0;
      0 0 1 ];

T = [ 1 0 0;
      0 1 20;
      0 0 1 ];
%R = T;

Rc = inv(Hc)*R*Hc;
M  = perc*logm( Rc );

Hn = [ 1 0 -size(X,1)/2;
       0 1 -size(X,2)/2;
       0 0 1 ];

rotated_patch = warping( X, HRef*Rc, width, height );

figure
imshow( uint8( rotated_patch ) )
title( 'Rotated patch' )

rotated_blurred_patch = blur_warping( X, HRef*Rc, M, eye(3), width, height );

figure
imshow( uint8( rotated_blurred_patch ) )
title( 'Rotated and blurred patch' )
