%
% Calculates the Jacobian with respect to an affine illumination
% model:
% patc = alpha*patc+beta;
% 
function J = Jillumination(patc,ncols,nrows)
J = [patc ones(ncols*nrows,1)];