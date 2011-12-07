function x = HLie( lH )

if nargin==0
  disp( 'Launching test...' );
  test()
  return
end
x = [lH(1,3);
     lH(2,3);
     -lH(2,1);
     -3/2*lH(3,3);
     lH(1,1)+1/2*lH(3,3);
     lH(2,1)+lH(1,2);
     lH(3,1);
     lH(3,2)
    ];

function test()

x = 10*rand(8,1);

if norm( x - HLie( LieH( x ) ) ) > 0.1
  disp( 'Test failed' )
  x
  HLie( LieH( x ) )
else
  disp( 'Passed test' )
end

%for ii=1:8
%  x = zeros(8,1);
%  x(ii)=1;
%  LieH( x )
%end
  