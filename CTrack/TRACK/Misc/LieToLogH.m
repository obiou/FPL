%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Convert the Lie algebra vector representation to the matrix
% representation
function logH = LieToLogH(x)

if(size(x,1)<8)
  x = [x;zeros(8-size(x,1),1)];
end

%logH = [x(5) x(3) x(1); 
%        x(4) -x(5)-x(6) x(2); 
%        x(7) x(8) x(6)];

logH = [x(4)+x(5) x(3)+x(6) x(1); 
        -x(3)+x(6) x(4) x(2); 
        x(7) x(8) -2*x(4)-x(5)];
