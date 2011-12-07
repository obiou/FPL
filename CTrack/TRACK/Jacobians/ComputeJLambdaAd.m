function J = ComputeJLambdaAd( dIc, dIr, HAd, M, nWidth, nHeight );
%
% Calculates the following Jacobian
% [dIc dIr]*JLambda
%
% with JLambda the Jacobian of Proj( inv(HAd)*expm( lambda*t*M )*HAd*p ) with respect
% to lambda in lambda=0 with M the Lie algebra representation of
% the blur and HAd the change of basis
%
% with:
%  * dIc the gradient of the image in the column direction
%  * dIr the gradient of the image in the row direction
%  * HAd a matrix for changing basis
%  * M blur matrix
%  * J the Jacobian of the blur magnitude
%
[imc,imr] = meshgrid(1:nWidth,1:nHeight);
c = reshape( imc, nWidth*nHeight, 1 );
r = reshape( imr, nWidth*nHeight, 1 );
Ones = ones( nWidth*nHeight, 1 );

Ic = reshape( dIc, nWidth*nHeight, 1 );
Ir = reshape( dIr, nWidth*nHeight, 1 );

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
JH1 = ( JAd(:,1,1) - c.*JAd(:,3,1) ) * M(1,1) + ...
      ( JAd(:,1,2) - c.*JAd(:,3,2) ) * M(1,2) + ...
      ( JAd(:,1,3) - c.*JAd(:,3,3) ) * M(1,3) + ...
      ( JAd(:,1,4) - c.*JAd(:,3,4) ) * M(2,1) + ...
      ( JAd(:,1,5) - c.*JAd(:,3,5) ) * M(2,2) + ...
      ( JAd(:,1,6) - c.*JAd(:,3,6) ) * M(2,3) + ...
      ( JAd(:,1,7) - c.*JAd(:,3,7) ) * M(3,1) + ...
      ( JAd(:,1,8) - c.*JAd(:,3,8) ) * M(3,2) + ...
      ( JAd(:,1,9) - c.*JAd(:,3,9) ) * M(3,3);
JH2 = ( JAd(:,2,1) - r.*JAd(:,3,1) ) * M(1,1) + ...
      ( JAd(:,2,2) - r.*JAd(:,3,2) ) * M(1,2) + ...
      ( JAd(:,2,3) - r.*JAd(:,3,3) ) * M(1,3) + ...
      ( JAd(:,2,4) - r.*JAd(:,3,4) ) * M(2,1) + ...
      ( JAd(:,2,5) - r.*JAd(:,3,5) ) * M(2,2) + ...
      ( JAd(:,2,6) - r.*JAd(:,3,6) ) * M(2,3) + ...
      ( JAd(:,2,7) - r.*JAd(:,3,7) ) * M(3,1) + ...
      ( JAd(:,2,8) - r.*JAd(:,3,8) ) * M(3,2) + ...
      ( JAd(:,2,9) - r.*JAd(:,3,9) ) * M(3,3);

J = Ic .* JH1 + Ir .* JH2;

