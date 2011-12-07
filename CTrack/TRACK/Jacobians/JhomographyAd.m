function [J,T] = JhomographyAd( dIc, dIr, HAd, ncols, nrows, nNumParams );
%
% Calculates the following Jacobian
% [dIc dIr]*JHAdp(:,1:nNumParams)
%
% with JHAdp the Jacobian of Proj( inv(HAd)*H(x)*HAd*p ) with respect
% to x in x=0 with Gi the Lie algebra generators H(x) =
% expm(sum(xi*Gi)) and with HAd a matrix for changing basis.
%
% with:
%  * dIc the gradient of the image in the column direction
%  * dIr the gradient of the image in the row direction
%  * HAd a matrix for changing basis
%  * JHAdp the homography Jacobian with a change in basis
%
% This product appears for the homography estimation
% and the blur homography estimation 
% (in the case of the velocity, we only have the first 2 columns
% that are eye(2))
%
if nargin<6
  nNumParams = 8;
end

[imc,imr] = meshgrid(1:ncols,1:nrows);
c = reshape( imc, ncols*nrows, 1 );
r = reshape( imr, ncols*nrows, 1 );
Ones = ones( ncols*nrows, 1 );

Ic = reshape( dIc, ncols*nrows, 1 );
Ir = reshape( dIr, ncols*nrows, 1 );
if nNumParams<2
  error(['Expecting at least 2 parameters for the tracking motion ' ...
         'model.']);
end

% Start by computing the Jacobian of
% inv(HAd)*M*HAd*p with respect to M in M=Id
for ii=1:9
  Mi = [0 0 0;0 0 0;0 0 0];
  Mi = Mi';
  Mi( ii ) = 1;
  Mi = Mi';
  JAdc = inv(HAd)*Mi*HAd*[c';r';Ones'];
  JAd( :, :, ii ) = JAdc';
end

% Compute first and second line in different matrices to ensure vectorisation
JHAdp = zeros( ncols*nrows, nNumParams, 2 );

if nNumParams >= 2
  JHAdp(:,1,1) = JAd(:,1,3) - JAd(:,3,3).*c;
  JHAdp(:,1,2) = JAd(:,2,3) - JAd(:,3,3).*r;
  
  JHAdp(:,2,1) = JAd(:,1,6) - JAd(:,3,6).*c;
  JHAdp(:,2,2) = JAd(:,2,6) - JAd(:,3,6).*r;
end
if nNumParams >= 3
  JHAdp(:,3,1) = JAd(:,1,2) - JAd(:,3,2).*c;
  JHAdp(:,3,2) = JAd(:,2,2) - JAd(:,3,2).*r;
end
if nNumParams >= 4
  JHAdp(:,4,1) = JAd(:,1,4) - JAd(:,3,4).*c;
  JHAdp(:,4,2) = JAd(:,2,4) - JAd(:,3,4).*r;
end
if nNumParams >= 5
  JHAdp(:,5,1) = JAd(:,1,1)-JAd(:,1,5) - (JAd(:,3,1)-JAd(:,3,5)).*c;
  JHAdp(:,5,2) = JAd(:,2,1)-JAd(:,2,5) - (JAd(:,3,1)-JAd(:,3,5)).*r;
end
if nNumParams >= 6
  JHAdp(:,6,1) = -JAd(:,1,5)+JAd(:,1,9) + (JAd(:,3,5)-JAd(:,3,9)).*c;
  JHAdp(:,6,2) = -JAd(:,2,5)+JAd(:,2,9) + (JAd(:,3,5)-JAd(:,3,9)).*r;
end
if nNumParams >= 7
  JHAdp(:,7,1) = JAd(:,1,7) - JAd(:,3,7).*c;
  JHAdp(:,7,2) = JAd(:,2,7) - JAd(:,3,7).*r;
end
if nNumParams >= 8
  JHAdp(:,8,1) = JAd(:,1,8) - JAd(:,3,8).*c;
  JHAdp(:,8,2) = JAd(:,2,8) - JAd(:,3,8).*r;
end

for ii=1:nNumParams
  J(:,ii) = Ic .* JHAdp(:,ii,1) + Ir .* JHAdp(:,ii,2);
end
%cIc = c.*Ic; rIc = r.*Ic; cIr = c.*Ir; rIr = r.*Ir; temp = -cIc-rIr;
%J = [Ic Ir rIc cIr (-rIr+cIc) (temp-rIr) (temp.*c) (temp.*r)];
