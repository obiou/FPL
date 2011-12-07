function [J,T] = Jhomography(dIc,dIr,ncols,nrows,nb_param,normalise);
%
% Calculates the following Jacobian
% [dIc dIr]*JHp(:,1:nb_param)
%
% with:
%  * dIc the gradient of the image in the column direction
%  * dIr the gradient of the image in the row direction
%  * JHp the jacobian of Proj( H(x)*p ) with respect to x in x=0
%    with Gi the Lie algebra generators H(x) = expm(sum(xi*Gi)) 
%
% This product appears for the homography estimation
% and the blur homography estimation 
% (in the case of the velocity, we only have the first 2 columns
% that are eye(2))
%
if nargin<5
  nb_param = 8;
end
if nargin<6
  normalise = 0;
end

if ~normalise
  T = eye(3);
end

[imc,imr] = meshgrid(1:ncols,1:nrows);
c = reshape(imc,ncols*nrows,1);
r = reshape(imr,ncols*nrows,1);

if normalise
  % Following same scheme as Hartley 8 point method
  c_mean = mean(c);
  r_mean = mean(r);
  
  meandist = mean(sqrt((c-c_mean).^2 + (r-r_mean).^2));
  
  scale = sqrt(2)/meandist;
  
  T = [scale   0   -scale*c_mean
       0     scale -scale*r_mean
       0       0      1      ];

  pts = T*[c'; r'; ones(1, ncols*nrows)];
  
  c = pts(1,:)';
  r = pts(2,:)';
end
   
Ic = reshape(dIc,ncols*nrows,1);
Ir = reshape(dIr,ncols*nrows,1);
if nb_param<2
  error(['Expecting at least 2 parameters for the tracking motion ' ...
         'model.']);
end
if (nb_param>=2)
  J = [Ic Ir];
end
if (nb_param>=3)
  rIc = r.*Ic;
  cIr = c.*Ir;
  J = [J rIc-cIr];
end
if (nb_param>=4)
  rIr = r.*Ir; cIc = c.*Ic;
  temp = cIc+rIr;
  J = [J temp];
end
if (nb_param>=5)
  J = [J cIc-rIr];
end
if (nb_param>=6)
  J = [J rIc];
end
if (nb_param>=7)
  J = [J -temp.*c];
end
if (nb_param>=8)
  J = [J -temp.*r];
end
%cIc = c.*Ic; rIc = r.*Ic; cIr = c.*Ir; rIr = r.*Ir; temp = -cIc-rIr;
%J = [Ic Ir rIc cIr (-rIr+cIc) (temp-rIr) (temp.*c) (temp.*r)];

if normalise
  J = 1/scale*J;
end