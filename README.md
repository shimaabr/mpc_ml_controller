#  Aircraft MPC Controller (MATLAB & Simulink)

This project implements an MPC controller for the **L-1011 aircraft** using **YALMIP** (optimization toolbox) and **Simulink**.  
- The system results are observed using scopes.  
- The cost function, Integral of Squared Error (ISE), and Integral of Squared Input (ISU) are calculated.  
- The sensitivity to parameters, constraints, and the cost function is evaluated.  

Ultimately, the MPC controller is compared with an **LQR controller**.  

---

##  MPC Controller
Model Predictive Control (MPC) is a feedback control algorithm that uses a model to predict future outputs of the process and solves an optimization problem to select the optimal control.

**Advantages of MPC:**  
1. It is a multivariable controller that can control multiple outputs simultaneously.  
2. It can handle constraints.  
3. It has preview capability.  

**Disadvantages of MPC:**  
- It needs a powerful, fast processor with large memory because it solves an online optimization problem at each step.  

---

##  Theory of MPC
For a discrete-time dynamic system:


x(k+1) = A x(k) + B u(k) 


y(k)   = C x(k) + D u(k)


Here, \(x\) is the state vector and \(y\) is the system output.  
The goal is to implement a control signal \([u_1, u_2, \dots, u_n]\) that achieves the best performance with minimum effort.  

**Cost function:**  

![formula](https://latex.codecogs.com/svg.latex?\color{white}J_{(0-N)}=x_N^TPx_N+\sum_{k=0}^{N-1}(x_k^TQx_k+u_k^TRu_k))

- **Q** demonstrates state deviation (performance).  
- **R** deminstrates  control effort.  
- **P** is the terminal weight.  

Because the cost function is quadratic, it is convex, so it has a unique global minimum.

---

##  Find Model for MPC (L-1011)

I use the [**COMPLIB library**](http://www.complib.de/) to find the state-space matrices of my system.  

- COMPLIB has more than 120 examples of real systems, which are linearized and LTI.  
- I use **AC3**, which is defined for a lateral-axis model of the L-1011 aircraft in cruise flight conditions, presented by **C. Edwards and S.K. Spurgeon**.  

<img src="https://github.com/user-attachments/assets/fbab897c-4fad-4a29-a96a-235eaf23ea00" width="400"/>  


*Reference:* [NASA Report](https://contrails.library.iit.edu/files/original/253c069aca0f1971c8e1b708576252f2c75363ef.pdf)



State, Input, and Output defined as

**States (x):**
- \(phi\) = Bank angle  
- \(r) = Yaw rate  
- \(p) = Roll rate  
- \(beta) = Sideslip angle  
- \(w) = Washout filter state  

**Inputs (u):**
- \(delta_r) = Rudder deflection  
- \(delta_a) = Aileron deflection

- 

- <img src="https://github.com/user-attachments/assets/b14f305f-5b27-45af-b8ae-734edeedb400" width="400"/>  

- Aileron: \(\pm 15^\circ \, (0.262 \,\text{rad})\)  
- Rudder: \(\pm 30^\circ \, (0.524 \,\text{rad})\)  

*Reference:* [NASA Beginner's Guide to Aeronautics](https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/learn-about-aerodynamics/#aircraft-motion)

**Outputs (y):**
- \(r_{wo}) = Washed-out yaw rate  
- \(p) = Roll rate  
- \(beta) = Sideslip angle  
- \(phi) = Bank angle  



--

### ⚙️ MATLAB Implementation
I extract the A, B, and C matrices from COMPLIB and put them in `plantac3.m`. Then I define a plant with these parameters and evaluate whether this system is controllable or not using the Kalman test.

```matlab
% System dimensions
nx = 5; nu = 2; ny = 4;

% State-space matrices
A = [0 0 1 0 0;
     0 -0.154 -0.0042 1.54 0;
     0 0.249 -1 -5.2 0;
     0.0386 -0.996 -0.0003 -0.117 0;
     0 0.5 0 0 -0.5];

B = [0 0;
    -0.744 -0.032;
     0.337 -1.12;
     0.02 0;
     0 0];

C = [0 1 0 0 -1;
     0 0 1 0 0;
     0 0 0 1 0;
     1 0 0 0 0];

D = zeros(ny,nu);

% Define plant
plant = ss(A,B,C,D);

% Time sampling
Ts = 0.1;

% Initial state
x0 = ones(nx,1);

%% Controllability check (Kalman test)
ctrb_matrix = ctrb(A,B);
rnk = rank(ctrb_matrix);

if rnk == nx
    disp('System is controllable')
else
    disp('System is NOT controllable')
end


```
*Reference:* [NASA NTRS Report 19900007452](https://ntrs.nasa.gov/api/citations/19900007452/downloads/19900007452.pdf)

## MPC controller implementation

YALMIP is used to formulate and solve the online  MPC.  
I adapted the Simulink YALMIP-MPC example   https://yalmip.github.io/example/simulink/ and selected a fast QP solver / structure to reduce execution time.  
The implementation below is a modify YALMIP-based  MPC controller. In wich it get Q R  state at the moment (current x) refrence(current r) and time and give signal control (uout) to the plant. you can find my code in AC3controller.m 

```matlab
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

Ts=.1  % Sampling time 
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
delta_a_max = 15*pi/180;  % 0.262 rad
delta_r_max = 30*pi/180;  % 0.524 rad

%% define cost funtion and objective function 
constraints = [];
objective = 0;
    for k = 1:N
        eror=r-C*x{k};
        objective = objective + (eror)'*Qbar*(eror)+u{k}'*Rbar*u{k};% Quadratic cost
        
        constraints = [constraints, x{k+1} == Ad*x{k}+Bd*u{k}]; % state dynamics
       

      % input limit
       constraints = [constraints, -delta_r_max  <= u{k}(1) <= delta_r_max ];
       constraints = [constraints, - delta_a_max<= u{k}(2) <= delta_a_max];
   
     
         
        
    end
%% Solve!
 ops = sdpsettings('verbose',2);%slover
    Controller = optimizer(constraints,objective,[],{x{1},r},u{1});Optimizer %takes {x{1},r} as inputs and outputs u{1}
    
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
```
At the next step, I used a YALMIP example for Simulink; however, I modified some parts to adapt it to my model.  
To obtain the current state vector `x` from the state-space block, I set:

- `C = eye(nx)`  
- `D = zeros(nx, nu)`  

so that the output of the state-space block is exactly the current `x`.  
This `x` is then given to the MPC controller.  

The MPC controller also receives:
- **T**: prediction horizon (here, 10 steps)  
- **Q**: weight on the tracking error  
- **R**: weight on the control signal  
- **r**: reference input  

After the state-space block, I multiply `y = Cx` to obtain the measured outputs, which are sent to a **scope** to visualize tracking performance.  
In addition, process disturbances and measurement noise are modeled using **white noise blocks** as shown below:

<img width="800" alt="Simulink model" src="https://github.com/user-attachments/assets/f414008e-e04f-41c4-823a-efe509c577ea" />

---

For performance evaluation, I calculate **ISE** and **ISU**.  
Since the system has 4 outputs and 2 inputs, I sum the individual ISEs and ISUs to get scalar values for easier comparison:

$ISE = \sum_{k=0}^{N} e^2(k)$

$ISU = \sum_{k=0}^{N} u^2(k)$



Finally, in the **subsystem block**, I compute the **cost function** using `Q`, `R`, the error, and the control signal according to:

![Cost](https://latex.codecogs.com/svg.latex?\color{white}J_{(0-N)}=x_N^TPx_N+\sum_{k=0}^{N-1}(x_k^TQx_k+u_k^TRu_k))
<img width="1085" height="428" alt="image" src="https://github.com/user-attachments/assets/5937ce01-c8a0-4840-a700-b7101376e85b" />


You can find the full Simulink implementation in **`mpccontroller.slx`**.

-- 
if you run the simulink you can see 
for Q=.1 R=.1

<img src="https://github.com/user-attachments/assets/c5e24922-20d8-43fa-b82d-eb8788d7fcc7" width="400" />

<img src="https://github.com/user-attachments/assets/863a3c55-131b-4694-92e6-c9a6604b122a" width="400" />

ost function and error  gradually reduce which show mpc controller done well

<img src="https://github.com/user-attachments/assets/34f1545d-b124-40f2-828d-a8b86f0249a7" width="400" />

<img src="https://github.com/user-attachments/assets/ed5ce072-b5ee-4c66-b9d5-0e21d6c1ce1e" width="400" />


cost function and error  gradually reduce which show mpc controller done well


## sensitivity to parameters

Sensitivity to Q
--
changes value of Q while other parameters steady in R=.1 U=delta_a_max = 2;% 0.262 rad
delta_r_max = 2;% 0.524 rad and N=1

<img width="653" height="594" alt="image" src="https://github.com/user-attachments/assets/1f7ffcde-9220-4776-8336-9c3ddb2d1232" />

as it is obvious increasing Q result in 

the output closely follows the setpoint.

the tracking error is smaller

more aggressive control actions

for better evaluation i make a table to find how error(ISE) and controlsignal (ISU) and cost function change due to changing Q

| R   | Q     | Cost function | ISE     | ISU   |
|-----|-------|---------------|---------|-------|
| 0.1 | 0.01  | 1.464         | 142.400 | 0.3998 |
| 0.1 | 0.025 | 2.669         | 101.092 | 1.417  |
| 0.1 | 0.05  | 4.365         | 82.500  | 2.401  |
| 0.1 | 0.075 | 6.254         | 79.538  | 2.890  |
| 0.1 | 0.1   | 8.190         | 78.870  | 3.031  |
| 0.1 | 0.2   | 15.990        | 78.360  | 3.199  |
| 0.1 | 0.5   | 39.560        | 78.460  | 3.317  |
| 0.1 | 1     | 78.500        | 78.160  | 3.371  |
| 0.1 | 10    | 781.200       | 78.082  | 3.419  |
| 0.1 | 100   | 7809.000      | 78.082  | 3.421  |


sensitivity to R
--
changes value of Q while other parameters steady in Q=.1 U=delta_a_max = 2;% 0.262 rad
delta_r_max = 2;% 0.524 rad and N=1

<img width="652" height="642" alt="image" src="https://github.com/user-attachments/assets/e509fbaf-7416-48be-afa1-3ca42383f685" />

Increasing R (opposite of Q):

The control inputs change more smoothly and conservatively.

Control effort (or actuator usage) is reduced.

Tracking accuracy decreases


| R    | Q   | Cost function | ISE      | ISU    |
|------|-----|---------------|----------|--------|
| 0.01 | 0.1 | 7.850         | 78.1644  | 3.371  |
| 0.05 | 0.1 | 7.996         | 78.3600  | 3.199  |
| 0.1  | 0.1 | 8.190         | 78.870   | 3.031  |
| 0.15 | 0.1 | 8.4245        | 80.000   | 2.788  |
| 0.18 | 0.1 | 8.592         | 81.320   | 2.550  |
| 0.2  | 0.1 | 8.730         | 82.499   | 2.401  |
| 0.25 | 0.1 | 9.183         | 86.615   | 2.087  |
| 0.5  | 0.1 | 11.530        | 109.830  | 1.098  |
| 1    | 0.1 | 14.640        | 142.411  | 0.3998 |
| 10   | 0.1 | 20.210        | 201.571  | 0.0052 |

to compare the effect of them better i plot the cost function ISE and ISU according to R and Q in matlab you can find the code in 

```matlab
R = [.01,.05,.1,.15,.18,.2,.25,.5,1,10]
CostR = [7.85,7.996,8.19,8.4245,8.592,8.73,9.183,11.53,14.64,20.21]
ISER = [78.1644,78.3600,78.87,80.0,81.32,82.499,86.615,109.83,142.411,201.571]
ISUR=[3.371,3.199,3.031,2.788,2.55,2.401,2.087,1.098,.3998,.00523]


% cost with changing R
subplot(3,2,1);
plot(R,CostR, '-o');
xlim([0 1]);
xlabel('R'); ylabel('Cost'); grid on; title('Cost(0-1) ');

subplot(3,2,2);
plot(R, CostR, '-o');
xlabel('R'); ylabel('Cost'); grid on; title('Cost (all)');

%ISE with changing R
subplot(3,2,3);
plot(R, ISER, '-o');
xlim([0 1]);
xlabel('R'); ylabel('ISE'); grid on; title('ISE(0-1) ');

subplot(3,2,4);
plot(R, ISER, '-o');
xlabel('R'); ylabel('ISE'); grid on; title('ISE (all)');

%ISU with changing R
subplot(3,2,5);
plot(R, ISUR, '-o');
xlim([0 1]);
xlabel('R'); ylabel('ISU'); grid on; title('ISU(0-1) ');

subplot(3,2,6);
plot(R, ISUR, '-o');
xlabel('R'); ylabel('ISU'); grid on; title('ISU (all)');

%% data for Q 
Q     = [0.01 0.025 0.05 0.075 0.1 0.2 0.5 1 10 100];
CostQ = [1.464 2.669 4.365 6.254 8.19 15.99 39.56 78.5 781.2 7809];
ISEQ  = [142.4 101.092 82.5 79.538 78.87 78.3596 78.46 78.16 78.08161 78.08209];
ISUQ  = [0.3998 1.417 2.401 2.89 3.031 3.199 3.317 3.371 3.419 3.421];

figure;

% Cost (0-1)
subplot(3,2,1);
plot(Q, CostQ, '-o'); xlim([0 1]);
xlabel('Q'); ylabel('Cost'); grid on; title('Cost (0-1)');

% Cost (all)
subplot(3,2,2);
plot(Q, CostQ, '-o');
xlabel('Q'); ylabel('Cost'); grid on; title('Cost (all)');

% ISE (0-1)
subplot(3,2,3);
plot(Q, ISEQ, '-o'); xlim([0 1]);
xlabel('Q'); ylabel('ISE'); grid on; title('ISE (0-1)');

% ISE (all)
subplot(3,2,4);
plot(Q, ISEQ, '-o');
xlabel('Q'); ylabel('ISE'); grid on; title('ISE (all)');

% ISU (0-1)
subplot(3,2,5);
plot(Q, ISUQ, '-o'); xlim([0 1]);
xlabel('Q'); ylabel('ISU'); grid on; title('ISU (0-1)');

% ISU (all)
subplot(3,2,6);
plot(Q, ISUQ, '-o');
xlabel('Q'); ylabel('ISU'); grid on; title('ISU (all)');
```
<img width="546" height="385" alt="image" src="https://github.com/user-attachments/assets/17106122-08c9-4f12-b286-7c2e5267811d" />

<img width="559" height="419" alt="image" src="https://github.com/user-attachments/assets/528f0738-83dd-455c-bfd7-166581bf56d3" />



changing prediction horizon
--
it is clearly increasing N result in better prediction so decrease ISE and make signal control more aggressive

<img width="624" height="615" alt="image" src="https://github.com/user-attachments/assets/5069f0a1-6946-4e21-bf6a-6e5866681d0c" />

| N   | Cost function | ISE      | ISU   |
|-----|---------------|----------|-------|
| 5   | 12.380        | 121.859  | 1.933 |
| 10  | 8.190         | 78.870   | 3.031 |
| 30  | 7.230         | 67.008   | 3.224 |
| 50  | 6.856         | 65.346   | 3.211 |
| 100 | 6.820         | 65.015   | 3.182 |

This table shows that increasing the prediction horizon N
N from 5 to 10 significantly decreases the ISE (by almost 50%). However, further increasing N
N from 30 to 100 only improves the ISE by about 2 units. Since a larger horizon also increases computational time and complexity, the best trade-off is achieved around N=30
. At this point, the ISE is already close to its minimum, while the computational effort remain.





















    



## 📂 Project Structure
- `src/` → MATLAB source code  
  - `mpc_yalmip.m` → MPC implementation with YALMIP  
  - `mpc_nominal.m` → MPC implementation without YALMIP  
  - `utils/` → helper functions  
- `simulink/` → Simulink models for aircraft and MPC  
- `results/` → Simulation outputs (plots, logs)  
- `docs/` → Additional documentation (e.g., report, notes)  
- `README.md` → Project description  
- `LICENSE` → License file  

---

## ⚙️ Requirements
- MATLAB R202x (recommended: 2022 or later)  
- Simulink  
- [YALMIP](https://yalmip.github.io/) (only for the YALMIP-based MPC)  
- An optimization solver (e.g., `quadprog`, `gurobi`)  

---

## 🚀 How to Run
1. Clone the repository:
   ```bash
   git clone https://github.com/username/aircraft-mpc-controller.git
   cd aircraft-mpc-controller
