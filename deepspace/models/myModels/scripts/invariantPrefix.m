function [Reach] = invariantPrefix(Reach,loc)
% Returns the portion of the reachtube that may or must be in mode "loc"

% Simulations are no longer include transitions so the old method of
% relying on ARPOD_update to give us the correct loc on each reachset, we
% must now test the reachtube against the invariant in this function

% NOTE: the method below avoids using polybool to check true containment
% and intersection and is not sound if a guard/invariant can possibly be
% smaller than the radius of a reachset (only checking Up/Low bounds and
% trajectory itself) (should not be a problem here)

global guardPassive

% First find reachsets that are contained in each location
ind0 = find(Reach.T>=guardPassive(2));
ind2 = intersect(find(Reach.Y(:,end)==2),find(Reach.Yup(:,end)==2));
ind2 = intersect(ind2,find(Reach.Ylow(:,end)==2));
ind3 = intersect(find(Reach.Y(:,end)==3),find(Reach.Yup(:,end)==3));
ind3 = intersect(ind3,find(Reach.Ylow(:,end)==3));

switch true
    case loc == 0
        indices = find(Reach.T>=guardPassive(1));
        
    case loc == 2
        % Remove sets contained in Passive and P3
        ind = union(ind0,ind3);
        indices = setdiff((1:length(Reach.T))',ind);
        
        % Check for disjunct sub-tubes and only keep first one
        last = find(diff(indices)~=1,1);
        if ~isempty(last)
            indices = indices(1:last);
        end
%         
%         % Find reachsets that intersect (but not contained in) P2 invariant
%         ind = setdiff(indices,ind2);
        
        % Ensure that a reachset with t>120 is included (since t=120
        % unlikely) so a transition to passive can occur
        if length(Reach.T)>indices(end)+1 && Reach.T(indices(end)+1)>=guardPassive(1)
            indices = [indices;indices(end)+1];
        end
    case loc == 3
        % Remove sets contained in Passive and P2
        ind = union(ind0,ind2);
        indices = setdiff((1:length(Reach.T))',ind);
        
        % Check for disjunct sub-tubes and only keep first one
        last = find(diff(indices)~=1,1);
        if ~isempty(last)
            indices = indices(1:last);
        end

        % Ensure that a reachset with t>120 is included (since t=120
        % unlikely) so a transition to passive can occur
        if length(Reach.T)>indices(end)+1 && Reach.T(indices(end)+1)>=guardPassive(1)
            indices = [indices;indices(end)+1];
        end
        
    otherwise
        error('Location out of bounds');
end

Reach.Y = Reach.Y(indices,:);
Reach.Yup = Reach.Yup(indices,:);
Reach.Ylow = Reach.Ylow(indices,:);
Reach.T = Reach.T(indices);
