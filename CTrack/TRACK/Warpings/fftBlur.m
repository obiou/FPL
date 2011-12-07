%
% Calculates motion blur or Jacobian by fft
%
% Input:
% - I: image
% - v:  velocity expressed in vector form (2x1) or matrix form (3x3)
% - type:  
%  * 'blur_gauss', 'blur_gauss_diff'
%  * 'blur_dir', 'blur_dir_diff'
%  * 'blur_sym', 'blur_sym_diff'
%
% Ouput:
% - IBlur: blurred image
% - G: fft transform of the blur function
% - spec: fft transform of the blurred image (before ifft)
%
function [IBlur,G,spec] = fftBlur(I,v,type)

if(size(v,2)>1)
  v = v(1:2,3);
end

[sy,sx] = size(I);
[fx,fy] = meshgrid(-1/2:1/(sx-1):1/2,-1/2:1/(sy-1):1/2);
  
v_i2piqv = i*2*pi*(v(1)*fx+v(2)*fy);
ind_nzero = find(v_i2piqv~=0);
%ind_nzero = find( abs(i*v_i2piqv) > 0.01 );

switch type
  case 'blur_gauss'
  % Motion blur Jin & al
  G = exp( 1/2*v_i2piqv.^2);
  global fft_I;
  if isempty(fft_I)||sx~=size(fft_I,2)||sy~=size(fft_I,1)
    fft_I = fftshift(fft2(I));
  end
 case 'blur_gauss_diff'
   G = v_i2piqv.*exp( 1/2*v_i2piqv.^2);
   fft_I = fftshift(fft2(I));
 case 'blur_dir'
  G = ones(size(I));
  % Motion blur for an instantaneaous shutter
  G(ind_nzero) = 1./(v_i2piqv(ind_nzero)).*(exp(v_i2piqv(ind_nzero))-1);
  global fft_I;
  if isempty(fft_I)||sx~=size(fft_I,2)||sy~=size(fft_I,1)
    fft_I = fftshift(fft2(I));
  end
 case 'blur_dir_diff'
  G = ones(size(I)); % integral is 1 for zeros...
  if 1
    %G(ind_nzero) = (exp(-v_i2piqv(ind_nzero)).*(1+v_i2piqv(ind_nzero))-1)./(v_i2piqv(ind_nzero).^2);
    G(ind_nzero) = (exp(v_i2piqv(ind_nzero)).*(v_i2piqv(ind_nzero)-1)+1)./(v_i2piqv(ind_nzero).^2);
    fft_I = fftshift(fft2(I));
  else
    G(ind_nzero) = 1./(v_i2piqv(ind_nzero));
    fft_I = fftshift(fft2(I));
  end
end

if find(isnan(G))|find(isinf(G))
  disp('Nan or Inf in the Jacobian');
  G(find(isnan(G))) = 1;
  G(find(isinf(G))) = 0;
end

spec = fft_I.*G;
IBlur = real(ifft2(ifftshift(spec)));


