% Warps an image patch according to a homography H.
%
% Mask indicates the points inside and outside the image
% 1 : point is good
% 0 : point is bad
%
% If no images are provided, an example is given with a 45deg rotation
% of a 200x200 patch in the center of the Mandrill image.
%
% Parameters:
% * im: initial image
% * H: homography between the initial image and new image ie x_new = H x_old
% * width: output width
% * height: output height
% * warp_type: (optional) 'nearest', 'linear'
% * mask_im: (optional) mask in the initial image (same size as im)
% * nx: warped x coordinates
% * ny: warped y coordinates
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [warped,mask,nx,ny] = warping( im, H, width, height, warp_type, mask_im )

if nargin == 0
  fprintf( 'Launching test...\n' );
  test();
  return;
end
  
if nargin<5
  warp_type = 'linear';
end

[imc,imr] = meshgrid( 1:width, 1:height );
c = H(1,1)*imc + H(1,2)*imr + H(1,3);
r = H(2,1)*imc + H(2,2)*imr + H(2,3);
z = H(3,1)*imc + H(3,2)*imr + H(3,3);
w = 1./z;
nx = c.*w;
ny = r.*w;

warped = interp2( im, nx, ny, warp_type );

mask = ones( size( warped ) );
mask( isnan( warped ) ) = 0;

if nargin>=6
  % Check mask has the right size
  if size(mask_im,1) ~= size(im,1) || size(mask_im,2) ~= size(im,2)
    size_mask  = size(mask_im)
    size_image = size(im)
    error( 'In warping: image should have the same size as mask' );
  end
  %warped_mask = interp2( mask_im, c.*w, r.*w, warp_type );
  warped_mask = interp2( double(mask_im), c.*w, r.*w, 'linear' );
  warped_mask( find( isnan(warped_mask) ) ) = 0;
  warped_mask( find( warped_mask~=1 ) )     = 0;
  warped_mask( find( warped_mask==1 ) )     = 1;
  
  mask = mask & warped_mask;
end

warped( find( mask ~= 1 ) ) = 255;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function test()

load mandrill;

width  = 200;
height = 200;
cx     = size( X, 1 )/2 - width/2;
cy     = size( X, 2 )/2 - height/2;

H = [ 1 0 cx;
      0 1 cy;
      0 0 1 ];

initial_patch = warping( X, H, width, height );

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

Hn = [ 1 0 -size(X,1)/2;
       0 1 -size(X,2)/2;
       0 0 1 ];

rotated_patch = warping( X, H*inv(Hc)*R*Hc, width, height );
%rotated_patch = warping( X, inv(Hn)*R*Hn, size(X,1), size(X,2) );

figure
imshow( uint8( rotated_patch ) )
title( 'Rotated patch' )
