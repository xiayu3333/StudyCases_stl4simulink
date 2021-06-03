function [C,ia,ib] = intersectTol(A,B,tol)
% Custom intersect function to handle equality within a tolerance (for
% vectors only)

if nargin < 2 || nargin > 3
    error('Incorrect number of input arguments');
elseif nargin == 2
    tol = 10^-12;
else
end

C = [];
ia = [];
ib = [];

for i=1:length(A)
    j = find(abs(A(i)-B)<tol);
%     if length(j)>1
%         error(' ');
%     end
    if ~isempty(j)
        ia = [ia;i];
        ib = [ib;j]; % captures all matching indices, remove dupes at end
%         ib = [ib;j(1)]; % if there are multiple indices that match, only keep first
        C = [C;A(i)];
    end
end

ia = unique(ia);
ib = unique(ib);

end
