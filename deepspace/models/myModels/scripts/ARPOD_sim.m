function [T,Y,transitions,T_loc,Y_loc] = ARPOD_sim(test_case,time_span,init_now)
% % This version should be faster because it only simulates in the first
% % mode, then checks for transition point, simulates in the next mode,
% % repeat. Only calls "ARPOD_update" at the very end.

% Custom simulation that utilizes CONTINUOUS updates on the thrust (LTI 
% model = Jacobian = A_cl = A-BK); but discretely updates "extended"
% variables (i.e. theta, rho, mode, and explicit values of ux,uy) 
% NOTE: "ux" and "uy" are used to check safety constraints on thrust, but
% these variables are independent from simulation

transitions = 1; % include initial starting time in transition times
flag = true;

freq = 100; %60; % scaled frequency (1/min)

options = odeset('RelTol',1e-3,'MaxStep',1/freq);
[T,Y] = ode45(test_case,time_span,init_now,options); 

% Return T_loc and Y_loc for bloating and nextRegions, but use T,Y for
% checking trajectory safety
T_loc = T;
Y_loc = Y;

% Update loc for Y_loc
for i=1:length(T_loc)
    try
        Y_loc(i,:) = ARPOD_update(Y_loc(i,:),T_loc(i));
    catch
        disp('NaN/Inf caught: ARPOD_sim, ln 30');
    end
end

% Check for transitions since not function of extended vars
transPassive = find(T>=120,1); % to passive mode
transP3= find(init_now(end)==2 & Y(:,1)<100 & Y(:,1)>-100 & ...
    Y(:,2)<100 & Y(:,2) > -100 & Y(:,1)+Y(:,2) < 100*sqrt(2) & ...
    Y(:,1)+Y(:,2) > -100*sqrt(2) & Y(:,1)-Y(:,2) < 100*sqrt(2) & ...
    Y(:,1)-Y(:,2) > -100*sqrt(2),1);
trans = min(squeeze([transPassive,transP3])); % get first transition time

if isempty(trans)
   flag = false;
   trans = length(T); % add time bound to list of transitions
end

transitions = [transitions; trans+transitions(end)-1];

% Update starting parameters for next simulation
time(1) = T(trans);
time(2) = time_span(2);
init_next = Y(trans,:);

while flag
    switch logical(true)
        case trans == transPassive
            test_case=@ARPOD_model0;
            init_next(end)=0;
        case trans == transP3
            test_case=@ARPOD_model3;
            init_next(end)=3;
        otherwise
            error('Herp derp.');
    end 
    
    [nextT,nextY] = ode45(test_case,time,init_next,options);
    
    % Check for transitions since not function of extended vars
    transPassive = find(init_next(end)~=0 & nextT>=120,1); % to passive mode
    transP3 = find(init_next(end)==2 & nextY(:,1)<100 & nextY(:,1)>-100 ...
        & nextY(:,2)<100 & nextY(:,2) > -100 & nextY(:,1)+nextY(:,2) < ...
        100*sqrt(2) & nextY(:,1)+nextY(:,2) > -100*sqrt(2) & ...
        nextY(:,1)-nextY(:,2) < 100*sqrt(2) & nextY(:,1)-nextY(:,2) > ...
        -100*sqrt(2),1);
    
    % Update next transition time and next sim starting parameters
    trans = min(squeeze([transPassive,transP3])); 
    if isempty(trans)
       flag = false;
       trans = length(nextT); % add time bound to list of transitions
    end
    time(1) = nextT(trans);
    init_next = nextY(trans,:);
    
    % Update simulation trace
    Y = vertcat(Y(1:transitions(end)-1,:),nextY(1:trans,:));
    T = vertcat(T(1:transitions(end)-1),nextT(1:trans));
    
    transitions = [transitions; trans+transitions(end)-1];
end

% transitions = [transitions; length(T)]; % also include time bound

% After sim complete, update all extended vars
for i = 1:size(Y,1)
    try
        Y(i,:) = ARPOD_update(Y(i,:),T(i));
    catch
        disp('NaN/Inf caught: ARPOD_sim, ln 99');
        
    end
end