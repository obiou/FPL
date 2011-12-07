function J = ComputeJLambda( dIc, dIr, M, nWidth, nHeight );
%
% Calculates the following Jacobian
% [dIc dIr]*JLambda
%
% with JLambda the Jacobian of Proj( expm( lambda*t*M )*p ) with respect
% to lambda in lambda=0 with M the Lie algebra representation of
% the blur.
%
% This Jacobian is decomposed into [dIc dIr]*Proj'*M*p
%
% with:
%  * dIc the gradient of the image blurred weighted by time in the column direction
%  * dIr the gradient of the image blurred weighted by time in the row direction
%  * M blur matrix
%  * J the Jacobian of the blur magnitude
%
% Blurred weighted by time (see blur_warping):
%  * grad( \int_0^1 [ t I( Proj( expm( lambda*t*M )*p ) ) ] dt )
%
if nargin == 0
  disp( 'Running test...' )
  test();
  return
end

[imc,imr] = meshgrid( 1:nWidth, 1:nHeight );
c    = reshape( imc, nWidth*nHeight, 1 );
r    = reshape( imr, nWidth*nHeight, 1 );
Ones = ones( nWidth*nHeight, 1 );

Ic   = reshape( dIc, nWidth*nHeight, 1 );
Ir   = reshape( dIr, nWidth*nHeight, 1 );

% Calculate Jacobian with respect to lambda
% [dBIx dBIy -(x dBIx + y dBIy)]*H*p
[col,row] = meshgrid( 1:nWidth, 1:nHeight );
col = col(:); row = row(:);

vcol = M(1,1)*col + M(1,2)*row + M(1,3);
vrow = M(2,1)*col + M(2,2)*row + M(2,3);
vz   = M(3,1)*col + M(3,2)*row + M(3,3);

J = Ic.*vcol + Ir.*vrow - ( col.*Ic + row.*Ir ).*vz;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function test()
syms m11 m12 m13 m21 m22 m23 m31 m32 m33 real;
syms x y z real;
syms dIc dIr real;

M = [ m11 m12 m13;
      m21 m22 m23;
      m31 m32 m33 ];

Proj = [x/z;y/z];
JProj = [ jacobian( Proj, 'x' ) ...
          jacobian( Proj, 'y' ) ...
          jacobian( Proj, 'z' ) ]

JLambda = subs( M*[ x; y; z ], {'x','y','z'}, {'c','r',1} );
syms vcol vrow vz real;
JFull = subs( [ dIc dIr ]*JProj*[ vcol vrow vz ]', {'z'}, {1} )

JLambda = subs( M*[ x; y; z ],'z', 1 );
JLambda = subs( [ dIc dIr ]*JProj*JLambda,{'dIc','dIr','z'},{'dx','dy',1} )

