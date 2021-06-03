function y=ARPOD_update(init_state,t)
% Update with approximated rho; precise rho code is commented out

global guardPassive

% persistent Klqr1 Klqr2 Klqr3; 
% if isempty(Klqr1) 
%     [Klqr1,Klqr2,Klqr3]=getlqr();
% end

if any(isnan(init_state)) || any(isinf(init_state))
    ME = MException('ARPOD_update:init_state', ...
        'NaN or Inf detected in init_state');
    return
%     throw(ME)
end

y=init_state;

% Update loc
switch logical(true)
    % Must be Passive
    case t >= guardPassive(2)
        y(7) = 0;
    % Phase 3
    case init_state(1)<100 && init_state(1)>-100 && init_state(2)<100 ...
        && init_state(2) > -100 && init_state(1)+init_state(2) ...
        < 100*sqrt(2) && init_state(1)+init_state(2) > -100*sqrt(2) ...
        && init_state(1)-init_state(2) < 100*sqrt(2) && ...
        init_state(1)-init_state(2) > -100*sqrt(2) && init_state(7)~=0
        y(7) = 3;
    % Phase 2
    case init_state(1)<1000 && init_state(1)>-1000 && init_state(2)<1000 ...
        && init_state(2) > -1000 && init_state(1)+init_state(2) ...
        < 1000*sqrt(2) && init_state(1)+init_state(2) > -1000*sqrt(2) ...
        && init_state(1)-init_state(2) < 1000*sqrt(2) && ...
        init_state(1)-init_state(2) > -1000*sqrt(2) && init_state(7)~=0
        y(7) = 2;
    % Just in case? Fails phase 2 and phase 3
    case t >= guardPassive(1)
        y(7) = 0;
    otherwise
        error('Failed to determine which mode this state belongs to.');
end
    
