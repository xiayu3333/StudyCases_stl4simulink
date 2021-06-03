function J = ARPOD_jac2(~)
% [x y xd yd]
n = 7.3023e-05*60; % [1/min]
m = 500;
persistent Klqr; 
if isempty(Klqr) 
    [~,Klqr,~]=getlqr();
end

J = [0,0,1,0,0,0; 0,0,0,1,0,0;... % xd,yd
    3*n^2,0,0,2*n,-1/m,0;... % xdd
    0,0,-2*n,0,0,-1/m;... % ydd
    Klqr(1,1),Klqr(1,2),Klqr(1,3),Klqr(1,4),0,0;... % ux
    Klqr(2,1),Klqr(2,2),Klqr(2,3),Klqr(2,4),0,0]; % uy