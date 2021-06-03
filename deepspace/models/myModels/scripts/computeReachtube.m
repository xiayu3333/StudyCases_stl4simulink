function [cov,safeflag,unsafeflag]=computeReachtube(cov)
% "cov" argument is the initial cover, computeReachtube updates "cov" to
% include full reachtube
global fulldim dim time_span tol

init_state = cov.x0;
timespan = cov.t0:0.1:time_span(2);
options = odeset('RelTol',1e-12,'AbsTol',1e-12);

switch true
    case init_state(fulldim) == 0 % Passive
        test_case = @ARPOD_model0;
%         Jac = ARPOD_jac0();
    case init_state(fulldim) == 2 || init_state(fulldim) == 1
        test_case = @ARPOD_model2;
%         Jac = ARPOD_jac2();
    case init_state(fulldim) == 3
        test_case = @ARPOD_model3;
%         Jac = ARPOD_jac3();
    otherwise
        error('Location out of bounds in computeReachtube.m ln.15');
end

% % USING PARSIMONIOUS METHOD, OVERAPPROXIMATED WITH HYPERRECTANGLES % %
V0 = diag(cov.dia);
V0 = cat(1,V0,zeros(1,size(V0,2))); % add zeros for extra dimension (loc)
V0 = cat(2,V0,zeros(size(V0,1),1));
V = cell(dim,1);

% Simulate from center state
[cov.T,cov.Y] = ode45(test_case,timespan,init_state,options);

% Simulate with errors
for i=1:dim
    [~,Ytemp] = ode45(test_case,timespan,init_state+V0(i,:),options);
    V{i} = Ytemp(:,1:dim)-cov.Y(:,1:dim);
end

% Get hyperrectangle reachsets
cov.Yup = zeros(size(cov.Y,1),fulldim);
cov.Ylow = zeros(size(cov.Y,1),fulldim);
for i=1:length(timespan)
    cov.Yup(i,1:dim) = cov.Y(i,1:dim)+max(-V{1}(i,:),V{1}(i,:))+max(-V{2}...
        (i,:),V{2}(i,:))+max(-V{3}(i,:),V{3}(i,:))+max(-V{4}(i,:),V{4}(i,:))...
        +max(-V{5}(i,:),V{5}(i,:))+max(-V{6}(i,:),V{6}(i,:));
    cov.Ylow(i,1:dim) = cov.Y(i,1:dim)+min(-V{1}(i,:),V{1}(i,:))+min(-V{2}...
        (i,:),V{2}(i,:))+min(-V{3}(i,:),V{3}(i,:))+min(-V{4}(i,:),V{4}(i,:))...
        +min(-V{5}(i,:),V{5}(i,:))+min(-V{6}(i,:),V{6}(i,:));
end

% % [cov.T,cov.Y] = ARPOD_sim(test_case,timespan,init_now);
% % [~,Ysim,~,cov.T,cov.Y] = ARPOD_sim(test_case,timespan,init_state);
% [cov.T,cov.Y] = ode45(test_case,timespan,init_state,options);
% 
% % % NEW METHOD OF COMPUTING YUP/YLOW "EXACTLY"
% cov.Yup = zeros(size(cov.Y,1),fulldim);
% cov.Ylow = zeros(size(cov.Y,1),fulldim);
% 
% cov.Yup(1,1:dim) = cov.x0(1:dim)+cov.dia;
% cov.Ylow(1,1:dim) = cov.x0(1:dim)-cov.dia;
% % initVertices = repmat(cov.x0(1:dim),4,1)+repmat(cov.dia,4,1).*[1,1,0,0;-1,-1,0,0;1,-1,0,0;-1,1,0,0];
% initVertices = repmat(cov.x0(1:dim),4,1)+repmat(cov.dia,4,1).*[1,0,0,0;-1,0,0,0;0,-1,0,0;0,1,0,0];
% nextVertices = zeros(4,4);
% for i=2:length(cov.T)
% %    cov.Yup(i,1:dim) = cov.Yup(1,1:dim)*expm(Jac*(cov.T(i)-cov.T(1)))';
% %    cov.Ylow(i,1:dim) = cov.Ylow(1,1:dim)*expm(Jac*(cov.T(i)-cov.T(1)))';
%    for j=1:4
%        nextVertices(j,:) = ode45(initVertices(j,:),)';
%    end
% %    a = cov.Yup(1,1:dim)*expm(Jac*(cov.T(i)-cov.T(1)))';
% %    b = cov.Ylow(1,1:dim)*expm(Jac*(cov.T(i)-cov.T(1)))';
%    cov.Yup(i,1:dim) = max(nextVertices);
%    cov.Ylow(i,1:dim) = min(nextVertices);
% end



% Update corresponding discrete modes if the mode is not Passive
if init_state(fulldim) ~= 0
    % If a mode should be updated, then make sure the mode~=0 when passed to ARPOD_update
    cov.Yup(:,end) = -1;
    cov.Ylow(:,end) = -1;
    for i = 1:size(cov.Y,1) 
        cov.Y(i,:) = ARPOD_update(cov.Y(i,:),cov.T(i));
        cov.Yup(i,:) = ARPOD_update(cov.Yup(i,:),cov.T(i));
        cov.Ylow(i,:) = ARPOD_update(cov.Ylow(i,:),cov.T(i));
    end
end

% Check for bug trace
% if cov.t0 == time_span(1)
%     [~,unsafeflag] = issafe(Ysim);
% else
    unsafeflag = 0;
% end

% Get reachtube for current mode
[cov] = invariantPrefix(cov,init_state(end)); 

% Check for safety of current mode
[safeflag,~] = issafe(cov.x0,cov.Yup,cov.Ylow);
if ~unsafeflag %safeflag == 1 && unsafeflag == 0
    % Check "nextRegions"
    nextCov = nextRegions(cov,init_state(end));
    for i=1:length(nextCov)
        if ~isempty(nextCov{i})
            [nextCov{i},nextSafe,~] = computeReachtube(nextCov{i});
            if nextSafe == 0
                % Break and partition initial set
                safeflag = 0;
                disp('partition');
                cov.addMode(nextCov{i});
%                 cov = cov.reduceCover(1); % no need to return entire reachtube
                return
            end
            cov.addMode(nextCov{i}); % if safe, add to reachtube and continue checking subsequent modes
        end
    end
    disp('This partition is safe');
elseif unsafeflag == 1
    % Return unsafe
    disp('terminate unsafe');
    % get unsafe trajectory; store it in cov.Y & cov.Yup/low=[]
    % cov = ...
    return
else % safeflag == 0
    % Break and partition initial set 
    disp('partition');
    cov = cov.reduceCover(1);
end

% Returns safe if diameters becomes less than tolerance
if any(cov.dia(find(cov.dia)) < tol) && (safeflag==0) && (unsafeflag==0)
    safeflag = 1;
    disp('cover diameter smaller than user-defined tolerance, will not partition');
end
