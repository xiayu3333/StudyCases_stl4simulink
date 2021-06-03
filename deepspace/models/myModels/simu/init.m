clear;
%% Set up parameters
n = 0.0044;   % [1/min], target's angular velocity
m = 500;  %[kg], chaser's mass

[Klqr,Glqr]=getlqr();    % Klqr -- Mode 1;  
                         % Glqr  -- Mode 2;
Jlqr = zeros(2,4);        % Jlqr  -- Mode Passive;

K11 = 3*n^2 -Klqr(1,1)/m;
K12 = -Klqr(1,2)/m;
K13 = -Klqr(1,3)/m;
K14 = 2*n -Klqr(1,4)/m;
K21 = -Klqr(2,1)/m;
K22 = -Klqr(2,2)/m;
K23 = -2*n -Klqr(2,3)/m;
K24 = -Klqr(2,4)/m;

G11 = 3*n^2 -Glqr(1,1)/m;
G12 = -Glqr(1,2)/m;
G13 = -Glqr(1,3)/m;
G14 = 2*n -Glqr(1,4)/m;
G21 = -Glqr(2,1)/m;
G22 = -Glqr(2,2)/m;
G23 = -2*n -Glqr(2,3)/m;
G24 = -Glqr(2,4)/m;

J11 = 3*n^2 -Jlqr(1,1)/m;
J12 = -Jlqr(1,2)/m;
J13 = -Jlqr(1,3)/m;
J14 = 2*n -Jlqr(1,4)/m;
J21 = -Jlqr(2,1)/m;
J22 = -Jlqr(2,2)/m;
J23 = -2*n -Jlqr(2,3)/m;
J24 = -Jlqr(2,4)/m;

%% initial_state
[x0, y0, vx0, vy0, ux0, uy0] = deal(-900, -400, 0, 0, 0, 0);

% At t = 76.8, mode_2 activated
% [x0_mode2, y0_mode2, vx0_mode2, vy0_mode2, ux0_mode2, uy0_mode2] = ... 
%     deal(-92, -42, 3.912, 1.585, 0, 0);

