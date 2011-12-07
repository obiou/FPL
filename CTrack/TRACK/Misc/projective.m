function P = projective(P)
%
% Adds a row of ones to P
%

P = [P; ones(1,size(P,2))];