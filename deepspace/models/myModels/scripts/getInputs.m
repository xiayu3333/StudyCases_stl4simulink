function [u,uDia] = getInputs(init,dia) % u-- thrust
% assuming starting in phase 2
% [initial_state(5:6),~] = getInputs(initial_state,initial_diameter)
[~,K2,~] = getlqr();

% get center state
u = init(1:4)*K2';

init_low = repmat(init(1:4),2,1)-sign(K2).*repmat(dia(1:4),2,1);
init_up = repmat(init(1:4),2,1)+sign(K2).*repmat(dia(1:4),2,1);
ux_low = init_low(1,:)*K2';
uy_low = init_low(2,:)*K2';
ux_up = init_up(1,:)*K2';
uy_up = init_up(2,:)*K2';

uDia(1,1) = max(abs(u(1)-ux_low(1)),abs(u(1)-ux_up(1)));
uDia(1,2) = somax(abs(u(2)-uy_low(2)),abs(u(2)-uy_up(2)));