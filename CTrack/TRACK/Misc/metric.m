function newP = metric(P)
%
% Divides P by the last row and removes the row.
%
newP = P(1:end-1,:).*repmat(1./P(end,:),size(P,1)-1,1);


