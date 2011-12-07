%
% Returns 8x1 vector x from homography H such that
% H = expm(LieToH(x))
%
function x = HToLie(H)

x = HLie( logm(H) );
     
