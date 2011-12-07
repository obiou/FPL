%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Passes from the Lie algebra sl(3)
% to the Lie group SL(3) 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function lH = LieH( x )

if( size(x,1) < 8 )
  x = [ x; zeros(8-size(x,1),1) ];
end

%lH =[ x(5) x(3) x(1); 
%        x(4) -x(5)-x(6) x(2); 
%        x(7) x(8) x(6) ];
%[    1,    0,    y,    0,    x,   -x, -x^2, -x*y]
%[    0,    1,    0,    x,   -y, -2*y, -x*y, -y^2]

lH = [1/3*x(4)+x(5) x(3)+x(6) x(1); 
      -x(3) 1/3*x(4)-x(5) x(2); 
      x(7) x(8) -2/3*x(4)];

%[    1,    0,    y,    x,    x,    0, -x^2, -x*y]
%[    0,    1,   -x,    y,   -y,    x, -x*y, -y^2]


