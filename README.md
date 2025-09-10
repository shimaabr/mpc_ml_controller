# mpc_ml_controller
# Aircraft MPC Controller (MATLAB & Simulink)

This project implements MPC controller for aircraft L-1011 with yalmip  (optimization toolbox ) a.
Then evaluate the sensitivity to parametrs contraints and cost function.
Then use deep learning to find best weight for cost function.
Ultimately compare mpc controller with LQR and PID controller

All approaches are simulated and compared in **Simulink** to study the impact of them.
# MPC controller 
Model predictive controller is a feedback control algorithm that use a model to make prediction about future output of the process and solve the optimization problem to select the optimal control.
mpc controller adv: 
1- it is a multivariable controller that can control the output simultaneosly.
2-It can handle contraints '
4-it has a preview capacity

 mpc controller disadvanteges
 it need poweful fast processor with a large memory cause solve an online optimization problem at each time .
 #theory of MPC 
 if descriminate dynamic system is 


 x(k+1)=Ax(k)+Bu(k)                                                            
  y(k+1)=Cx(k)+Du(k)
 
  
  x is state space and y is output of system 
  The goal is to implement signal control [u1 u2 ...un] that have best performance and have less effort so we define a cost function: 

  
![formula](https://latex.codecogs.com/svg.latex?\color{white}J_{(0-N)}=x_N^TPx_N+\sum_{k=0}^{N-1}(x_k^TQx_k+u_k^TRu_k))


weight Q is for having less effort and P for best performance 

cause the cost function is quadratic so it is convexx so it has definite minimum


# find model for mpc (L-1011)
i use complib library to find matrix of state space of my system 

complib has more than 120 example of real system which is linearized and is LTI 

I use Ac3 which is defined for a lateral axis model of L-1011 aircraft in cruise flight condition present by C.Edwards and S.K.Spurgeon .
We can extract A B C matrix from complib.
in this modle x y and u represent for 

state:

x=[
    phi=Bank angle;
    r=yaw rate;
    p= roll rate;
    beta=sideslip angle;
    w=washed out filter state;
    ]

input:

u=[
    delta_r=rudder defelection;
    delta_a=aileron defelection;
    ]

output:

y=[
   r-wo=washed out yaw rate;
   p= roll rate;
   beta=sideslip angle;
   phi=Bank angle]


   https://ntrs.nasa.gov/api/citations/19900007452/downloads/19900007452.pdf   
   https://contrails.library.iit.edu/files/original/253c069aca0f1971c8e1b708576252f2c75363ef.pdf  
   

    



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
