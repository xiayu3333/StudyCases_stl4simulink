function [K1,K2,K3]=getlqr()
% function [K1,K2,K3,K2b]=getlqr()

syms n m t tau real
%syms n m t tf tau real
% global qx2 qx3 qy2 qy3 qxd2 qxd3 qyd2 qyd3 rx ry; 

qx2=1000; 
qx3=100;
qy2=866;
qy3=100; 
qxd2=20; 
qxd3=3; 
qyd2=20;
qyd3=3;
rx=8;
ry=8; 

m1 = 500;
n1 = 7.3023e-05; % [1/s]
n1 = n1*60; % [1/min]
A = [0 0 1 0; 0 0 0 1; 3*n^2 0 0 2*n; 0 0 -2*n 0];
B = [0 0; 0 0; 1/m 0; 0 1/m];
A1=double(subs(A,n,n1));
B1=double(subs(B,m,m1));

Q = [1/qx3^2,0,0,0; 0,1/qy3^2,0,0; 0,0,1/(qxd3)^2,0; 0,0,0,1/(qyd3)^2]; % [m] [m/min] & |xd,yd|<0.05*60
R = [1/(rx*60^2)^2,0; 0,1/(ry*60^2)^2]; % |u|<10[kg*m/min^2]
N = zeros(4,2);

[K3,~,~] = lqr(A1,B1,Q,R,N); % <100m


% Q = [1/1000^2,0,0,0; 0,1/866^2,0,0; 0,0,1/100^2,0; 0,0,0,1/100^2]; % [m] and [m/s]
Q = [1/qx2^2,0,0,0; 0,1/qy2^2,0,0; 0,0,1/(qxd2)^2,0; 0,0,0,1/(qyd2)^2]; % [m] and [m/min]
[K2,~,~] = lqr(A1,B1,Q,R,N); % 500 to 1000m

K1=K2;
