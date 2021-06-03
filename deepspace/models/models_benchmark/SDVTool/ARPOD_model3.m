function dy=ARPOD_model3(t,y)
dy=zeros(7,1);
n = 7.3023e-05;
n = n*60; % scaled
m = 500;

persistent Klqr; 
if isempty(Klqr) 
    [~,~,Klqr]=getlqr();
end

%% ux and uy independent
dy(1) = y(3);
dy(2) = y(4);
dy(3) = (3*n^2)*y(1)+2*n*y(4)-(Klqr(1,1)*y(1)+Klqr(1,2)*y(2)+Klqr(1,3)*...
    y(3)+Klqr(1,4)*y(4))/m;
dy(4) = -2*n*y(3)-(Klqr(2,1)*y(1)+Klqr(2,2)*y(2)+Klqr(2,3)*...
    y(3)+Klqr(2,4)*y(4))/m;
dy(5) = Klqr(1,1)*y(3)+Klqr(1,2)*y(4)+...
    Klqr(1,3)*((3*n^2)*y(1)+2*n*y(4)-y(5)/m)+...
    Klqr(1,4)*(-2*n*y(3)-y(6)/m);
dy(6) = Klqr(2,1)*y(3)+Klqr(2,2)*y(4)+...
    Klqr(2,3)*((3*n^2)*y(1)+2*n*y(4)-y(5)/m)+...
    Klqr(2,4)*(-2*n*y(3)-y(6)/m);
dy(7) = 0;

% %% 6 dim function
% dy(1) = y(3);
% dy(2) = y(4);
% dy(3) = (3*n^2)*y(1)+2*n*y(4)-y(5)/m;
% dy(4) = -2*n*y(3)-y(6)/m;
% dy(5) = Klqr(1,1)*y(3)+Klqr(1,2)*y(4)+...
%     Klqr(1,3)*((3*n^2)*y(1)+2*n*y(4)-y(5)/m)+...
%     Klqr(1,4)*(-2*n*y(3)-y(6)/m);
% dy(6) = Klqr(2,1)*y(3)+Klqr(2,2)*y(4)+...
%     Klqr(2,3)*((3*n^2)*y(1)+2*n*y(4)-y(5)/m)+...
%     Klqr(2,4)*(-2*n*y(3)-y(6)/m);
% dy(7) = 0;

% dy(1) = y(3);
% dy(2) = y(4);
% dy(3) = (3*n^2)*y(1)+2*n*y(4)-(Klqr(1,1)*y(1)+Klqr(1,2)*y(2)+Klqr(1,3)*...
%     y(3)+Klqr(1,4)*y(4))/m;
% dy(4) = -2*n*y(3)-(Klqr(2,1)*y(1)+Klqr(2,2)*y(2)+Klqr(2,3)*...
%     y(3)+Klqr(2,4)*y(4))/m;
% dy(5) = 0;