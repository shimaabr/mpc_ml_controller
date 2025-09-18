# ✈️ Aircraft MPC Controller (MATLAB & Simulink)

This project implements an MPC controller for the **L-1011 aircraft** using **YALMIP** (optimization toolbox) and **Simulink**.  
- The system results are observed using scopes.  
- The cost function, Integral of Squared Error (ISE), and Integral of Squared Input (ISU) are calculated.  
- The sensitivity to parameters, constraints, and the cost function is evaluated.  

Ultimately, the MPC controller is compared with an **LQR controller**.  

---

## 📖 MPC Controller
Model Predictive Control (MPC) is a feedback control algorithm that uses a model to predict future outputs of the process and solves an optimization problem to select the optimal control.

**Advantages of MPC:**  
1. It is a multivariable controller that can control multiple outputs simultaneously.  
2. It can handle constraints.  
3. It has preview capability.  

**Disadvantages of MPC:**  
- It needs a powerful, fast processor with large memory because it solves an online optimization problem at each step.  

---

## 📐 Theory of MPC
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

## 🔧 Find Model for MPC (L-1011)

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
