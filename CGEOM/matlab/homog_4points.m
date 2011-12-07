%
% Computes the matrices required for homography estimation from 4
% points.
% This version assumes H(3,3) = 1 is acceptable (ie the plane does
% not go throught the optical center)
%
function homog_4points

syms h1 h2 h3 h4 h5 h6 h7 h8 h9 real;
syms mC100 mC101 mC102 mC103 mC110 mC111 mC112 mC113 real;
syms mC200 mC201 mC202 mC203 mC210 mC211 mC212 mC213 real;

mC1 = [mC100 mC101 mC102 mC103;
       mC110 mC111 mC112 mC113;
       1 1 1 1];
mC2 = [mC200 mC201 mC202 mC203;
       mC210 mC211 mC212 mC213;
       1 1 1 1];

H = [h1 h2 h3;
     h4 h5 h6;
     h7 h8 h9];

P = H*mC1;

V = [];
for ii=1:4
  V = [ V; cross( mC2(:,ii), P(:,ii) ) ];
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

sA = ccode( A );
sb = ccode( b );

sA = strfixmat( sA, 'mC1%d%d', 'mC1(%d,%d)', 7, 7 );
sA = strfixmat( sA, 'mC2%d%d', 'mC2(%d,%d)', 7, 7 );
sA = strfixmat( sA, 'A[%d][%d]', 'mA(%d,%d)', 7, 7 );
sb = strfixmat( sb,  'mC2%d%d', 'mC2(%d,%d)', 7, 7 );
sb = strfixmat( sb, 'b[%d][%d]', 'mb(%d,%d)', 7, 7 );

sA
sb

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function sM = strfix( sM, sIn, sOut, nRow, nCol )
sIn  = sprintf( sIn, nRow, nCol );
sOut = sprintf( sOut, nRow, nCol );
sM = strrep( sM, sIn, sOut );

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function sM = strfixmat( sM, sIn, sOut, nRows, nCols )
for nRow=0:nRows
  for nCol=0:nCols
    sM = strfix( sM, sIn, sOut, nRow, nCol );
  end
end

 