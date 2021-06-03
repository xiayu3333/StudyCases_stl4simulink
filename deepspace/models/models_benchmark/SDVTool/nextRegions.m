function next = nextRegions(Reach,source)
% Must update Guards here when changing application
% Computes reachable states in other modes
% next(1): transition from P2 to P3
% next(2): transition from P2 to Passive
% next(3): transition from P3 to Passive

next = cell(3,1); % cell array of next regions
global guardPassive dim

%% Any mode --> Passive
ind = find(Reach.T >= guardPassive(1));
if ~isempty(ind) && source~=0
    % Union of reachsets is simpler here because the guard is function of time, not state
    newCov = Reach.newCover(ind(1));
    for i=2:length(ind)
        newCov = newCov.coverUnion(Reach,ind(i));
    end
    % Update mode
    newCov.x0(end) = 0;
    newCov.Y(:,end) = 0;
    newCov.Yup(:,end) = 0;
    newCov.Ylow(:,end) = 0;
%     newCov.Y = ARPOD_update(newCov.Y,newCov.t0);
%     newCov.Yup = ARPOD_update(newCov.Yup,newCov.t0);
%     newCov.Ylow = ARPOD_update(newCov.Ylow,newCov.t0);
    next{source} = newCov;
end

%% Phase 2 --> Phase 3
if source == 2
    % Get guard
    R = 100;
    guard23=horzcat([R*(sqrt(2)-1);R*(1-sqrt(2));-R;-R;R*(1-sqrt(2));...
        R*(sqrt(2)-1);R;R;R*(sqrt(2)-1)],...
        [-R;-R;R*(1-sqrt(2));R*(sqrt(2)-1);R;R;R*(sqrt(2)-1);R*(1-sqrt(2));-R]); % guard polygon vertices
    
    % Get indices for reachsets that MAY transition to phase 3
    ind = find(Reach.Y(:,end)==3);
    ind = union(ind,find(Reach.Yup(:,end)==3));
    ind = union(ind,find(Reach.Ylow(:,end)==3));
    
    if isempty(ind)
        return
    end
    
    for i=1:length(ind)
        % Get x,y position reachset from Yup and Ylow
        reachset = horzcat([Reach.Yup(ind(i),1);Reach.Yup(ind(i),1);...
            Reach.Ylow(ind(i),1);Reach.Ylow(ind(i),1)],...
            [Reach.Yup(ind(i),2);Reach.Ylow(ind(i),2);...
            Reach.Ylow(ind(i),2);Reach.Yup(ind(i),2)]);

        % Get intersection between guard and reachset
        [x,y] = polybool('intersection',reachset(:,1),reachset(:,2),...
            guard23(:,1),guard23(:,2));
        
        % Get union with previously checked reachsets
        % NOTE: newCov is the precise intersection of the reachset with
        % guard, whereas next{1} is a hyperrectangular cover
        if isempty(next{1})
%             newCov = [x,y];
            next{1} = Reach.newCover(ind(1)); % initialize next{1} to cover R_i
            next{1}.Yup(1,1:2) = [max(x),max(y)];
            next{1}.Ylow(1,1:2) = [min(x),min(y)];
            % update x0, T, and dia outside of loop
        else
            vert = horzcat([next{1}.Yup(:,1);next{1}.Yup(:,1);next{1}.Ylow(:,1);...
                next{1}.Ylow(:,1)],[next{1}.Yup(:,2);next{1}.Ylow(:,2);...
                next{1}.Ylow(:,2);next{1}.Yup(:,2)]);
            [x,y] = polybool('union',vert(:,1),vert(:,2),x,y);
%             [x,y] = polybool('union',newCov(:,1),newCov(:,2),x,y);
%             newCov = [x,y];
            % Update x,y
            next{1}.Yup(1,1:2) = [max(x),max(y)];
            next{1}.Ylow(1,1:2) = [min(x),min(y)]; 
        end
    end
    % Update t0 to earliest start time
    next{1}.t0 = min(Reach.T(ind));
    % Update vx,vy
    next{1}.Yup(1,3) = max(Reach.Yup(ind,3));
    next{1}.Yup(1,4) = max(Reach.Yup(ind,4));
    next{1}.Ylow(1,3) = min(Reach.Ylow(ind,3));
    next{1}.Ylow(1,4) = min(Reach.Ylow(ind,4));
    % Update ux,uy
    next{1}.Yup(1,5) = max(Reach.Yup(ind,5));
    next{1}.Yup(1,6) = max(Reach.Yup(ind,6));
    next{1}.Ylow(1,5) = min(Reach.Ylow(ind,5));
    next{1}.Ylow(1,6) = min(Reach.Ylow(ind,6));
    % Update remaining cover parameters
    next{1}.T = next{1}.t0;
    next{1}.dia = (next{1}.Yup(1,1:dim)-next{1}.Ylow(1,1:dim))./2; % radius
    next{1}.x0 = next{1}.Ylow(1,1:dim)+next{1}.dia;
    next{1}.x0 = horzcat(next{1}.x0,3); % Add updated mode
    
    % Check that center point is indeed in next mode (since nextRegion is overapproximated)
    if next{1}.x0(end) == source
        error('Need to refine approximated nextRegion (ln 81)');
    end
% % Check if reach is contained in guard? (MUST if intersection = reachset)
    % Debug only
%     figure
%     vert = horzcat([next{1}.Yup(:,1);next{1}.Yup(:,1);next{1}.Ylow(:,1);...
%         next{1}.Ylow(:,1)],[next{1}.Yup(:,2);next{1}.Ylow(:,2);...
%         next{1}.Ylow(:,2);next{1}.Yup(:,2)]);
%     plot(guard23(:,1),guard23(:,2)); % Phase 3
%     grid on; grid minor; hold on
%     patch(vert(:,1),vert(:,2),'blue'); % nextRegion
%     patch('XData',[-100,-100,0],'YData',[100/sqrt(3),-100/sqrt(3),0],...
%         'FaceAlpha',0.65,'EdgeColor','none','FaceColor',[1 1 0]); % LOS 
end

