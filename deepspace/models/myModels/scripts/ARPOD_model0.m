function dy=ARPOD_model0(t,y)
dy=zeros(7,1);
n = 7.3023e-05;
n = n*60; % scaled
m = 500;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% closed-loop system
dy(1) = y(3);
dy(2) = y(4);
dy(3) = (3*n^2)*y(1)+2*n*y(4);
dy(4) = -2*n*y(3);
dy(5) = 0;
dy(6) = 0;
dy(7) = 0;