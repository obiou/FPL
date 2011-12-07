%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Passes from the Lie algebra sl(3)
% to the Lie group SL(3)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [H,M] = LieToH(x)

if nargin==0
  disp('Launching test...')
  test;
  return
end

if(size(x,1)<8)
  x = [x;zeros(8-size(x,1),1)];
end


M = LieH( x );

H = expm( M );

function test

x = 5*rand(8,1);
[H,M] = LieToH(x);

traceM = trace( M )
detH = det( H )

xb = HToLie( H );
error = norm( x-xb )

if error>1
  disp( 'Probably due to an instable logm:' )
  M
  logExpM = logm( expm( M ) )
end
