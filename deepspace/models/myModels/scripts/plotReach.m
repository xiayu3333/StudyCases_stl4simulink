function plotReach(Reach,plotOption)

global guardPassive

%% Plot x-y reachtube
if find(plotOption==1)
    figure; grid on; grid minor; hold on
    xlabel('x-position (m)');
    ylabel('y-position (m)');

    % Get guard for P2/P3
    R = 100;
    guard=horzcat([R*(sqrt(2)-1);R*(1-sqrt(2));-R;-R;R*(1-sqrt(2));...
        R*(sqrt(2)-1);R;R;R*(sqrt(2)-1)],....
        [-R;-R;R*(1-sqrt(2));R*(sqrt(2)-1);R;R;R*(sqrt(2)-1);R*(1-sqrt(2));-R]);
    
    % Plot LOS cone
    los=patch([-100,-100,0],[100/sqrt(3),-100/sqrt(3),0],[1 1 0],...
            'FaceAlpha',0.65,'EdgeColor','none'); % LOS 
    
    % Plot unsafe collision region
    R = 0.1;
    unsafe = horzcat([R;0;-R;0;R],[0;-R;0;R;0]);
    coll=patch(unsafe(:,1),unsafe(:,2),[1 0 0],'EdgeColor','none'); 
%     title('x-y');

    % Get reachset for P2\P3
    transPassive = find(Reach.T>=guardPassive(1),1);
    if isempty(transPassive)
        transPassive = length(Reach.T)+1;
    end
    for i=1:floor((transPassive-1)/10)
        reach = horzcat([Reach.Yup(10*i,1);Reach.Yup(10*i,1);...
            Reach.Ylow(10*i,1);Reach.Ylow(10*i,1);Reach.Yup(10*i,1)],...
            [Reach.Yup(10*i,2);Reach.Ylow(10*i,2);...
            Reach.Ylow(10*i,2);Reach.Yup(10*i,2);Reach.Yup(10*i,2)]);
        
%         % Plot P2/P3 as same color
%         patch(reach(:,1),reach(:,2),[1,0,1],'EdgeColor','none');
%         
        % Plot P2 region
        [x,y] = polybool('subtraction',reach(:,1),reach(:,2),guard(:,1),guard(:,2));
        if i==1
            p2=patch(x,y,[0,0.5,1],'EdgeColor','none');
        else
            patch(x,y,[0,0.5,1],'EdgeColor','none');
        end
        % Plot P3 region
        [x,y] = polybool('intersection',guard(:,1),guard(:,2),reach(:,1),reach(:,2));
        p3=patch(x,y,[1,0,1],'EdgeColor','none');
    end
%     for i=1:transPassive-1
%         reach = horzcat([Reach.Yup(i,1);Reach.Yup(i,1);...
%             Reach.Ylow(i,1);Reach.Ylow(i,1);Reach.Yup(i,1)],...
%             [Reach.Yup(i,2);Reach.Ylow(i,2);...
%             Reach.Ylow(i,2);Reach.Yup(i,2);Reach.Yup(i,2)]);
%         % Plot P2 region
%         [x,y] = polybool('subtraction',reach(:,1),reach(:,2),guard(:,1),guard(:,2));
%         patch(x,y,[0,0.5,1],'EdgeColor','none');
%         % Plot P3 region
%         [x,y] = polybool('intersection',guard(:,1),guard(:,2),reach(:,1),reach(:,2));
%         patch(x,y,[0,1,0.5],'EdgeColor','none');
%     end

    % Plot reachset for passive mode
    for i=transPassive:length(Reach.T)
        reach = horzcat([Reach.Yup(i,1);Reach.Yup(i,1);...
            Reach.Ylow(i,1);Reach.Ylow(i,1);Reach.Yup(i,1)],...
            [Reach.Yup(i,2);Reach.Ylow(i,2);...
            Reach.Ylow(i,2);Reach.Yup(i,2);Reach.Yup(i,2)]);
        passive=patch(reach(:,1),reach(:,2),[0.5,1,0],'EdgeColor','none');
    end
    
    legend([p2,p3,passive,los,coll],'ProxA','ProxB','Passive',...
        'Safe LOS region','Unsafe Collision Region','Location','best');
end
%% Plot t-vx,t-vy
if find(plotOption==2)
    figure; grid on; grid minor; hold on
    xlabel('x-velocity (m/min)');
    ylabel('y-velocity (m/min)');
    
    % Plot safe region
    R = 3;
    safe=horzcat([R*(sqrt(2)-1);R*(1-sqrt(2));-R;-R;R*(1-sqrt(2));...
        R*(sqrt(2)-1);R;R;R*(sqrt(2)-1)],....
        [-R;-R;R*(1-sqrt(2));R*(sqrt(2)-1);R;R;R*(sqrt(2)-1);R*(1-sqrt(2));-R]);
    vel=patch(safe(:,1),safe(:,2),[1,1,0],'FaceAlpha',0.65,'EdgeColor','none');
    
    % Plot vx-vy
    % Plot P2 region (line, not tube)
    ind2 = union(find(Reach.Yup(:,end)==2),find(Reach.Ylow(:,end)==2));
    ind3 = find(Reach.Yup(:,end)==3);
%     plot(Reach.Ylow(ind2,3),Reach.Ylow(ind2,4),'Color',[0,0.5,1]);

    % Plot P2 region
    for i=1:length(ind2)
        reach = horzcat([Reach.Yup(ind2(i),3);Reach.Yup(ind2(i),3);...
            Reach.Ylow(ind2(i),3);Reach.Ylow(ind2(i),3);Reach.Yup(ind2(i),3)],...
            [Reach.Yup(ind2(i),4);Reach.Ylow(ind2(i),4);...
            Reach.Ylow(ind2(i),4);Reach.Yup(ind2(i),4);Reach.Yup(ind2(i),4)]);
        p2=patch(reach(:,1),reach(:,2),[0,0.5,1],'EdgeColor','none');
    end
    
    % Plot P3 region
    for i=1:length(ind3)
        reach = horzcat([Reach.Yup(ind3(i),3);Reach.Yup(ind3(i),3);...
            Reach.Ylow(ind3(i),3);Reach.Ylow(ind3(i),3);Reach.Yup(ind3(i),3)],...
            [Reach.Yup(ind3(i),4);Reach.Ylow(ind3(i),4);...
            Reach.Ylow(ind3(i),4);Reach.Yup(ind3(i),4);Reach.Yup(ind3(i),4)]);
        p3=patch(reach(:,1),reach(:,2),[1,0,1],'EdgeColor','none');
    end
    
    % Plot passive region
    ind0 = union(find(Reach.Yup(:,end)==0),find(Reach.Ylow(:,end)==0));
    for i=1:length(ind0)
        reach = horzcat([Reach.Yup(ind0(i),3);Reach.Yup(ind0(i),3);...
            Reach.Ylow(ind0(i),3);Reach.Ylow(ind0(i),3);Reach.Yup(ind0(i),3)],...
            [Reach.Yup(ind0(i),4);Reach.Ylow(ind0(i),4);...
            Reach.Ylow(ind0(i),4);Reach.Yup(ind0(i),4);Reach.Yup(ind0(i),4)]);
        passive=patch(reach(:,1),reach(:,2),[0.5,1,0],'EdgeColor','none');
    end
    
    legend([p2,p3,passive,vel],'ProxA','ProxB','Passive',...
        'Safe velocity region in ProxB','Location','best');
end

%% Plot thrust
if find(plotOption==3)
    figure; grid on; grid minor; hold on
    xlabel('Time (min)');
    ylabel('Thrust (N)');
    
    ind2 = union(find(Reach.Yup(:,end)==2),find(Reach.Ylow(:,end)==2));
    ind3 = find(Reach.Yup(:,end)==3);
    
    % Plot P2 region
    for i=1:10:length(ind2)-10
        reachX = horzcat([Reach.T(ind2(i));Reach.T(ind2(i));...
            Reach.T(ind2(i+10));Reach.T(ind2(i+10));Reach.T(ind2(i))],...
            [Reach.Yup(ind2(i),5);Reach.Ylow(ind2(i),5);...
            Reach.Ylow(ind2(i+10),5);Reach.Yup(ind2(i+10),5);Reach.Yup(ind2(i),5)]);
        p2x=patch(reachX(:,1),reachX(:,2)/3600,[0,0.5,1],'EdgeColor','none');
        reachY = horzcat([Reach.T(ind2(i));Reach.T(ind2(i));...
            Reach.T(ind2(i+10));Reach.T(ind2(i+10));Reach.T(ind2(i))],...
            [Reach.Yup(ind2(i),6);Reach.Ylow(ind2(i),6);...
            Reach.Ylow(ind2(i+10),6);Reach.Yup(ind2(i+10),6);Reach.Yup(ind2(i),6)]);
        p2y=patch(reachY(:,1),reachY(:,2)/3600,[0.5,0,1],'EdgeColor','none');
    end
    
    % Plot P3 region
    for i=1:10:length(ind3)-10
        reachX = horzcat([Reach.T(ind3(i));Reach.T(ind3(i));...
            Reach.T(ind3(i+10));Reach.T(ind3(i+10));Reach.T(ind3(i))],...
            [Reach.Yup(ind3(i),5);Reach.Ylow(ind3(i),5);...
            Reach.Ylow(ind3(i+10),5);Reach.Yup(ind3(i+10),5);Reach.Yup(ind3(i),5)]);
        p3x=patch(reachX(:,1),reachX(:,2)/3600,[1,0.5,0],'EdgeColor','none');
        reachY = horzcat([Reach.T(ind3(i));Reach.T(ind3(i));...
            Reach.T(ind3(i+10));Reach.T(ind3(i+10));Reach.T(ind3(i))],...
            [Reach.Yup(ind3(i),6);Reach.Ylow(ind3(i),6);...
            Reach.Ylow(ind3(i+10),6);Reach.Yup(ind3(i+10),6);Reach.Yup(ind3(i),6)]);
        p3y=patch(reachY(:,1),reachY(:,2)/3600,[0.5,1,0],'EdgeColor','none');
    end
    
    ylim([-0.02,0.06]);
    legend([p2x,p2y,p3x,p3y],'x-thrust in ProxA','y-thrust in ProxA',...
        'x-thrust in ProxB','y-thrust in ProxB','Location','best');
end