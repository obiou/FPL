%
% Computes the explicit solution for finding the homography between
% a 0-1 box and 4 points.
%
% This version assumes H(3,3) = 1 is acceptable (ie the plane does
% not go throught the optical center)
%
function homog_simpl

syms h1 h2 h3 h4 h5 h6 h7 h8 h9 real;
syms mBox00 mBox01 mBox02 mBox03 mBox10 mBox11 mBox12 mBox13 real;

B = [mBox00 mBox01 mBox02 mBox03;
     mBox10 mBox11 mBox12 mBox13;
     1 1 1 1];

H = [h1 h2 h3;
     h4 h5 h6;
     h7 h8 h9];

C = [0 1 1 0;
     0 0 1 1;
     1 1 1 1];

P = H*C;

V = [];
for ii=1:4
  V = [ V; cross( B(:,ii), P(:,ii) ) ];
end

V

s = H(:);

J = jacobian( V, s );

Js=[];
for ii=1:4
  Js = [Js; J(3*(ii-1)+1:3*(ii-1)+2,:)];
end

A = Js(:,1:end-1)
b = Js(:,end)
if 1
  S = simplify( -inv(A)*b )
  mH = reshape( [S; 1;], 3, 3 );
else
  detA = det(A)
  S = simplify( -det(A)*inv(A)*b )
  syms invDetA real;
  mH = reshape( [invDetA*S; 1;], 3, 3 );
end

% Check
Sl = [S; 1;];
simplify(Js*Sl)

sOutput = ccode( mH );
sInvDetA = ccode( 1/det(A) );

for nRow=0:1
  for nCol=0:3
    mBoxS = sprintf( 'mBox%d%d', nRow, nCol );
    mBoxSNew = sprintf( 'mBox(%d,%d)', nRow, nCol );
    sInvDetA = strrep( sInvDetA, mBoxS, mBoxSNew );
    sOutput = strrep( sOutput, mBoxS, mBoxSNew );
  end
end
for nRow=0:2
  for nCol=0:2
    mHS = sprintf( 'mH[%d][%d]', nRow, nCol );
    mHSNew = sprintf( 'H(%d,%d)', nRow, nCol );
    sOutput = strrep( sOutput, mHS, mHSNew );
  end
end

sOutput
sInvDetA

