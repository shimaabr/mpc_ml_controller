
function uout= ac18controller(currentx,currentr,t,Q,R,N)
persistent Controller  % Store optimizer object across function calls

if t == 0 
%% define discrete-time parametrs for system model

 nx=5;nu=2;ny=4;
 A=[0 0 1 0 0;0 -0.154 -0.0042 1.54 0;0 0.249 -1 -5.2 0;
    0.0386 -0.996 -0.0003 -0.117 0;0 0.5 0 0 -0.5];
 B=[0 0;-0.744 -0.032;0.337 -1.12;0.02 0;0 0];
 C=[0 1  0 0 -1;0 0 1 0 0;0 0 0 1 0;1 0 0 0 0];

 D=zeros(ny,nu);
 plant=ss(A,B,C,D);% Continuous  plant

Ts=.1 ; % Sampling time 
plantd = c2d(plant,Ts);  % Discrete  plant
% Discrete  plant matrics
Ad = plantd.A;
Bd = plantd.B;
Cd = plantd.C;
Dd = plantd.D;

%% define mpc controller parameters
N=10; % Prediction horizon
Rbar=R*eye(nu); % weight of control input
Qbar=Q*eye(ny);  % weight of state
 % Avoid variable explosion in YALMIP
    yalmip('clear')
%% define u and x as semi definite program
u = sdpvar(repmat(nu,1,N),repmat(1,1,N));% U : N inputs of dimension nu
x = sdpvar(repmat(nx,1,N+1),repmat(1,1,N+1));% X: N+1 states of dimension nx

sdpvar r % Reference signal 
    
%% contraints of signal control
delta_a_max = 15*pi/180;% 0.262 rad
delta_r_max = 30*pi/180;% 0.524 rad


%% define cost funtion and objective function 
constraints = [];
objective = 0;
    for k = 1:N
         eror=r-C*x{k};
        objective = objective + (eror)'*Qbar*(eror)+u{k}'*Rbar*u{k};% Quadratic cost
        
        constraints = [constraints, x{k+1} == Ad*x{k}+Bd*u{k}]; % state dynamics
       

      % input limit
       constraints = [constraints, -delta_a_max<= u{k}(1) <= delta_a_max ];
       constraints = [constraints, -delta_r_max <= u{k}(2) <= delta_r_max];

% norm-1 for cost function of controller 
% objective = objective + norm(Q*eror,1) + norm(R*u{k},1);
%  constraints = [constraints, x{k+1} == Ad*x{k} + Bd*u{k}];
%  constraints = [constraints, -delta_a_max<= u{k}(1) <= delta_a_max ];
%  constraints = [constraints, -delta_r_max <= u{k}(2) <= delta_r_max];
 
        
    end
%% Solve!
 ops = sdpsettings('verbose',2);%slover
    Controller = optimizer(constraints,objective,[],{x{1},r},u{1}); %takes {x{1},r} as inputs and outputs u{1}
    
    [uout,problem] = Controller(currentx,currentr); % Solve at t=0 
    
    if problem
        warning('MPC failed to solve at t=0');
    end
    
else    
     % use persistent optimizer
    [uout,problem] = Controller(currentx,currentr);
    
    if problem
      warning('MPC failed to solve at t=%d',t);
    end

end

end
