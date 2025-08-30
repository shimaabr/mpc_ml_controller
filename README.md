# mpc_ml_controller
# Aircraft MPC Controller (MATLAB & Simulink)

This project implements a **Model Predictive Controller (MPC)** for an aircraft model.  
The controller is developed in two versions:  
1. **Using YALMIP** (optimization toolbox)  
2. **Without YALMIP** (direct formulation)  

Both approaches are simulated and compared in **Simulink** to study the impact of different MPC parameters.  

---

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
