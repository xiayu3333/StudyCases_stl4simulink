function [safe,unsafe] = issafe(Y,Yup,Ylow)
% Can be called to check reachtube (Yup/Ylow) OR simulation trace (Y)

% % Linearized constraints
safe = 1;
unsafe = 0;

loc = Y(1,end); % check properties for current mode

if nargin == 1
    loc = -1; % check only simulation trace
end

switch true
    case loc == -1 
        %% Check thrust
        index = find((abs(Y(:,5:6)) > 36000.01),1); % ux, uy bound
        if ~isempty(index)
            unsafe = 1;
            safe = 0;
            disp('Thrust constraint violated at ');
            disp(index);
            return
        end
        %% Check passive collision avoidance
        R = 0.1;
        index = find(Y(:,end)==0);
        for i = 1:length(index)
            if Y(index(i),1)+Y(index(i),2) < R && ...
                    Y(index(i),1)+Y(index(i),2) > -R  && ...
                    Y(index(i),1)-Y(index(i),2) < R && ...
                    Y(index(i),1)-Y(index(i),2) > -R
                unsafe = 1;
                safe = 0;
                disp('Collision avoidance violated at '); 
                disp(index(i));
                return
            end
        end
        %% Check LOS and total velocity
        index = find(Y(:,end)==3);
        for i = 1:length(index)
            if Y(index(i),3)>3 || Y(index(i),3) < -3 || ...
                    Y(index(i),4) > 3 || Y(index(i),4) < -3 || ...
                    Y(index(i),3)+Y(index(i),4) > 3*sqrt(2) || ...
                    Y(index(i),3)+Y(index(i),4) < -3*sqrt(2)|| ...
                    Y(index(i),3)-Y(index(i),4) > 3*sqrt(2) || ...
                    Y(index(i),3)-Y(index(i),4) < -3*sqrt(2)
                unsafe = 1;
                safe = 0;
                disp('Velocity constraint in phase 3 violated at '); 
                disp(index(i));
                return
            end
            if Y(index(i),1) < -100 || ...
                    Y(index(i),2)+Y(index(i),1)/sqrt(3) > 0 || ...
                    Y(index(i),2)-Y(index(i),1)/sqrt(3) < 0
                unsafe = 1;
                safe = 0;
                disp('LOS constraint violated at ');
                disp(index(i));
                return
            end
        end
    case loc == 0 
        %% Check passive collision avoidance
        indices1 = find(Yup(:,end)==0);
        indices2 = find(Ylow(:,end)==0);

        if any(indices1~=indices2) % since guard is dep. on time, should be uniformly in passive mode
            error('Non uniform passive safety reachset');
        end

        % Get unsafe polygon
        R = 0.1;
        unsafeset = horzcat([R;0;-R;0;R],[0;-R;0;R;0]);
%         v=horzcat([R;0;-R;0],[0;-R;0;R]);
%         numPts = 50;
%         unsafeset = horzcat(linspace(v(size(v,1),1),v(1,1),numPts)',linspace(v(size(v,1),2),v(1,2),numPts)');
%         for i=2:size(v,1)
%             temp = horzcat(linspace(v(i-1,1),v(i,1),numPts)',linspace(v(i-1,2),v(i,2),numPts)');
%             unsafeset = vertcat(unsafeset,temp);
%         end

        Jc=indices1;
        for i=1:length(Jc)
            % Get reachset from Yup and Ylow
            reach = horzcat([Yup(Jc(i),1);Yup(Jc(i),1);Ylow(Jc(i),1);Ylow(Jc(i),1);Yup(Jc(i),1)],...
                [Yup(Jc(i),2);Ylow(Jc(i),2);Ylow(Jc(i),2);Yup(Jc(i),2);Yup(Jc(i),2)]);
%             v = horzcat([Yup(Jc(i),1);Yup(Jc(i),1);Ylow(Jc(i),1);Ylow(Jc(i),1)],...
%                 [Yup(Jc(i),2);Ylow(Jc(i),2);Ylow(Jc(i),2);Yup(Jc(i),2)]);
%             numPts = 500;
%             reach = horzcat(linspace(v(size(v,1),1),v(1,1),numPts)',linspace(v(size(v,1),2),v(1,2),numPts)');
%             for j=2:size(v,1)
%                 temp = horzcat(linspace(v(j-1,1),v(j,1),numPts)',linspace(v(j-1,2),v(j,2),numPts)');
%                 reach = vertcat(reach,temp);
%             end
            % Get intersection between unsafe and reachset
            [x,~] = polybool('intersection', reach(:,1), reach(:,2), unsafeset(:,1), unsafeset(:,2));

            if ~isempty(x)
                safe = 0;
                disp('Possible collision avoidance violation in Passive mode at ');
                disp(Jc(i));
                break
            end
        end
        
    case loc == 2
        %% Check thrust constraint
        index1 = find((abs(Yup(:,5:6)) > 36000.01),1);
        index1 = [index1,find((abs(Ylow(:,5:6)) > 36000.01),1)];
        if ~isempty(index1)
            safe = 0;
            disp('Possible thrust constraint violated at ');
            disp(index1);
        end
        
    case loc == 3
        %% Check thrust constraint
        index1 = find((abs(Yup(:,5:6)) > 36000.01),1);
        index1 = [index1,find((abs(Ylow(:,5:6)) > 36000.01),1)];
        if ~isempty(index1)
            safe = 0;
            disp('Possible thrust constraint violated at ');
            disp(index1);
            return
        end
        
        %% Check velocity constraints in phase 3
        indices1 = find(Yup(:,end)==3);
        indices2 = find(Ylow(:,end)==3);

        % Get indices for uniform phase3 reachsets
        [Ic,~,~] = intersectTol(indices1,indices2);

        % Check safety of phase 3 reachsets
        for i=1:length(Ic)
            if Yup(Ic(i),3)>3 || Yup(Ic(i),3) < -3 || ...
                    Yup(Ic(i),4) > 3 || Yup(Ic(i),4) < -3 || ...
                    Yup(Ic(i),3)+Yup(Ic(i),4) > 3*sqrt(2) || ...
                    Yup(Ic(i),3)+Yup(Ic(i),4) < -3*sqrt(2)|| ...
                    Yup(Ic(i),3)-Yup(Ic(i),4) > 3*sqrt(2) || ...
                    Yup(Ic(i),3)-Yup(Ic(i),4) < -3*sqrt(2)
                safe = 0;
                disp('Possible velocity constraint in phase 3 violated at '); 
                disp(Ic(i));
                return
            end
            if Yup(Ic(i),1) < -100 || ...
                    Yup(Ic(i),2)+Yup(Ic(i),1)/sqrt(3) > 0 || ...
                    Yup(Ic(i),2)-Yup(Ic(i),1)/sqrt(3) < 0
                safe = 0;
                disp('Possible LOS constraint violated at ');
                disp(Ic(i));
                return
            end
            if Ylow(Ic(i),3)>3 || Ylow(Ic(i),3) < -3 || ...
                    Ylow(Ic(i),4) > 3 || Ylow(Ic(i),4) < -3 || ...
                    Ylow(Ic(i),3)+Ylow(Ic(i),4) > 3*sqrt(2) || ...
                    Ylow(Ic(i),3)+Ylow(Ic(i),4) < -3*sqrt(2)|| ...
                    Ylow(Ic(i),3)-Ylow(Ic(i),4) > 3*sqrt(2) || ...
                    Ylow(Ic(i),3)-Ylow(Ic(i),4) < -3*sqrt(2)
                safe = 0;
                disp('Possible velocity constraint in phase 3 violated at '); 
                disp(Ic(i));
                return
            end
            if Ylow(Ic(i),1) < -100 || ...
                    Ylow(Ic(i),2)+Ylow(Ic(i),1)/sqrt(3) > 0 || ...
                    Ylow(Ic(i),2)-Ylow(Ic(i),1)/sqrt(3) < 0
                safe = 0;
                disp('Possible LOS constraint violated at ');
                disp(Ic(i));
                return
            end
        end

        % Get non-overlapping indices
        Jup = setdiff(indices1,Ic); % not uniform phase 3 Yup
        Jlow = setdiff(indices2,Ic); % not uniform phase 3 Ylow
        [Jc,~,~] = union(Jup,Jlow);

        % % Check safety of mustbe-phase3 subreachset
        % Get guard set
        R = 100;
        guard=horzcat([R*(sqrt(2)-1);R*(1-sqrt(2));-R;-R;R*(1-sqrt(2));...
            R*(sqrt(2)-1);R;R;R*(sqrt(2)-1)],...
            [-R;-R;R*(1-sqrt(2));R*(sqrt(2)-1);R;R;R*(sqrt(2)-1);...
            R*(1-sqrt(2));-R]);
%         v=horzcat([R*(sqrt(2)-1);R*(1-sqrt(2));-R;-R;R*(1-sqrt(2));R*(sqrt(2)-1);...
%             R;R],[-R;-R;R*(1-sqrt(2));R*(sqrt(2)-1);R;R;R*(sqrt(2)-1);R*(1-sqrt(2))]);
%         numPts = 50;
%         guard = horzcat(linspace(v(size(v,1),1),v(1,1),numPts)',linspace(v(size(v,1),2),v(1,2),numPts)');
%         for i=2:size(v,1)
%             temp = horzcat(linspace(v(i-1,1),v(i,1),numPts)',linspace(v(i-1,2),v(i,2),numPts)');
%             guard = vertcat(guard,temp);
%         end

        for i=1:length(Jc)
            % Check total velocity
            if Yup(Jc(i),3)>3 || Yup(Jc(i),3) < -3 || ...
                    Yup(Jc(i),4) > 3 || Yup(Jc(i),4) < -3 || ...
                    Yup(Jc(i),3)+Yup(Jc(i),4) > 3*sqrt(2) || ...
                    Yup(Jc(i),3)+Yup(Jc(i),4) < -3*sqrt(2)|| ...
                    Yup(Jc(i),3)-Yup(Jc(i),4) > 3*sqrt(2) || ...
                    Yup(Jc(i),3)-Yup(Jc(i),4) < -3*sqrt(2)
                safe = 0;
                disp('Possible velocity constraint in phase 3 violated at '); 
                disp(Jc(i));
                return
            end
            if Ylow(Jc(i),3)>3 || Ylow(Jc(i),3) < -3 || ...
                    Ylow(Jc(i),4) > 3 || Ylow(Jc(i),4) < -3 || ...
                    Ylow(Jc(i),3)+Ylow(Jc(i),4) > 3*sqrt(2) || ...
                    Ylow(Jc(i),3)+Ylow(Jc(i),4) < -3*sqrt(2)|| ...
                    Ylow(Jc(i),3)-Ylow(Jc(i),4) > 3*sqrt(2) || ...
                    Ylow(Jc(i),3)-Ylow(Jc(i),4) < -3*sqrt(2)
                safe = 0;
                disp('Possible velocity constraint in phase 3 violated at '); 
                disp(Jc(i));
                return
            end
            
            % Get reachset from Yup and Ylow
            reach = horzcat([Yup(Jc(i),1);Yup(Jc(i),1);Ylow(Jc(i),1);Ylow(Jc(i),1);Yup(Jc(i),1)],...
                [Yup(Jc(i),2);Ylow(Jc(i),2);Ylow(Jc(i),2);Yup(Jc(i),2);Yup(Jc(i),2)]);
%             v = horzcat([Yup(Jc(i),1);Yup(Jc(i),1);Ylow(Jc(i),1);Ylow(Jc(i),1)],...
%                 [Yup(Jc(i),2);Ylow(Jc(i),2);Ylow(Jc(i),2);Yup(Jc(i),2)]);
%             numPts = 50;
%             reach = horzcat(linspace(v(size(v,1),1),v(1,1),numPts)',linspace(v(size(v,1),2),v(1,2),numPts)');
%             for j=2:size(v,1)
%                 temp = horzcat(linspace(v(j-1,1),v(j,1),numPts)',linspace(v(j-1,2),v(j,2),numPts)');
%                 reach = vertcat(reach,temp);
%             end
            % Get intersection between guard and reachset
            [x,y] = polybool('intersection', reach(:,1), reach(:,2), guard(:,1), guard(:,2));

            % Check LOS safety of intersection
            for j=1:length(x)
                if x(j) < -100 || ...
                        y(j)+x(j)/sqrt(3) > 0 || ...
                        y(j)-x(j)/sqrt(3) < 0
                    safe = 0;
                    disp('Possible LOS constraint violated at ');
                    disp(Jc(i));
                    return
                end
            end
        end
    otherwise
        disp('Location out of bounds');     
end