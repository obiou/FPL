%
% Computes the Jacobian of JHp = Proj( H(x)*p ) with respect to x in 0 symbolically
% and [dIc dIr]*JHp
function computeSymbolicJacobian()

syms x1 x2 x3 x4 x5 x6 x7 x8 real;
syms x y z real;

Hx = LieH( [x1 x2 x3 x4 x5 x6 x7 x8]' )

JH = [];

for ii=1:8
  JH= [ JH jacobian( Hx*[x;y;1], [ 'x' num2str(ii) ] )];
end

Proj = [x/z;y/z]
JProj = [ jacobian( Proj, 'x' ) ...
          jacobian( Proj, 'y' ) ...
          jacobian( Proj, 'z' ) ]
          
JH
JHp = subs( JProj, 'z', 1 )*JH

syms dIc dIr real;

JFull = [ dIc dIr ]*JHp

syms r c real;
subs( JFull, {'x','y'}, {'c','r'} )
subs( JFull, {'dIc','dIr'}, {'dx','dy'} )
