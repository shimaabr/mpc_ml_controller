% System

 nx=5;nu=2;ny=4;
 A=[0 0 1 0 0;0 -0.154 -0.0042 1.54 0;0 0.249 -1 -5.2 0;
    0.0386 -0.996 -0.0003 -0.117 0;0 0.5 0 0 -0.5];
 B=[0 0;-0.744 -0.032;0.337 -1.12;0.02 0;0 0];
 C=[0 1  0 0 -1;0 0 1 0 0;0 0 0 1 0;1 0 0 0 0];

  D=zeros(ny,nu);
 plant=ss(A,B,C,D);
 % time sampling
Ts = .1;
 
% Initial state for simulation
x0 = ones(nx,1);

%% controllability of system with kalman 
ctrb=[B];
for i=1:nx-1;
    ctrb=[ctrb (A^i)*B];
end
rnk=rank(ctrb);
if rnk==nx
    disp('System is controllable')
else
    disp('System is not controllable')
end
        


