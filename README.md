# Control of Active Suspension System of Vehicle

  ## 📑 Table of Contents
- [📖 Overview](#-Overview)
- [🎯 Purposes](#-purposes)
- [🛑 Constraints](#-constraints)
- [📁 Files Descriptions](#-files-descriptions)
- [⚙️ Model Dynamics](#-model-dynamics)
- [📊 Results](#results)
 [Quarter Car Response](#quarter-car-response)
  - [Classical Controllers](#classical-controllers)
    - [P Controller](#p-controller)
    - [PI Controller](#pi-controller)
    - [PID Controller](#pid-controller)
  - [State Feedback Control SVFC](#state-feedback-control-svfc)
  - [Full-Order Observer](#full-order-observer)
## 📖 Overview
In this project, I aimed to control the vehicle's active suspension system using a quarter-car model. The project includes the design and analysis of classical controllers (P, PI, PID), state feedback control (SVFC), and full-order state observers. Step responses, settling time, overshoot, and steady-state error are analyzed for each controller. The system is illustrated in the figure below.

<img width="479" height="355" alt="image" src="https://github.com/user-attachments/assets/3fa22315-0fc5-485a-88cc-6a213d002a78" />
  
## 🎯 Purposes
- Model the **quarter car system response** under initial conditions, analyzing the behavior of both the sprung and unsprung masses in terms of displacement and velocity.

- Compare the performance of **classical controllers (P, PI, PID)** and a **state feedback controller (SVFC)** on the system in terms of settling time, overshoot, and steady-state error.

- Design and simulate a **full-order state observer** to estimate unmeasured states, and validate the estimation by comparing the estimated and actual states.

## 🛑 Constraints
- **Settling Time:** The controller should ensure that the system settles within **5 seconds**.  
- **Overshoot:** The system overshoot must be less than **20%**.  
- **Steady-State Error:** The steady-state error should be less than **1%** for a unit step input.

## 📁 Files Descriptions

- `QuarterCarState.m` : Defines the quarter car system dynamics as a state-space model. Computes the derivatives of the states given inputs, road disturbances, and vehicle parameters.

- `P_controller_sweep.m` : Sweeps a range of proportional (P) gains, computes step response metrics (settling time, overshoot, steady-state error) for each, and identifies stable configurations.

- `PI_controller_sweep.m` : Sweeps a range of proportional ($$K_p$$) and integral ($$K_i$$) gains for a PI controller, computes step response metrics, and identifies stable configurations.

- `PI_optimal.m` : Function to find the optimal PI controller based on settling time and overshoot constraints.

- `PID_optimal_tuner.m` : Tunes a PID controller using MATLAB's PID Tuner and evaluates performance metrics.

- `Run_file.m` : Main script that executes simulations, sweeps controller gains, computes performance metrics, and generates plots for all controllers and observers.
  
## ⚙️ Model Dynamics

According to the figure, using Newton’s second law, the equations of motion for the two masses can be written as:

1. Vehicle (sprung) mass:
   
$$
M \frac{dv_1}{dt} = -K_1 \big( (x_1 - x_2)\big) - D (\dot{x}_1 - \dot{x}_2) + u
$$


2. Axle (unsprung) mass:
   
$$
m \frac{dv_2}{dt} = -K_1 \big( (x_1 - x_2)\big)  + D (\dot{x}_1 - \dot{x}_2) - u - K_2 (x_2- y_R)
$$

where:  

- \(M\) : Sprung mass (vehicle body)  
- \(m\) : Unsprung mass (axle)  
- \($$K_1$$\) : Suspension stiffness  
- \($$K_2$$\) : Tire stiffness  
- \(D\) : Damping coefficient  
- \($$x_1$$, $$x_2$$\) : Vertical displacement of sprung and unsprung masses  
- \($$\dot{x}_1$$, $$\dot{x}_2$$\) : Vertical velocities   
- \($$y_R$$\) : Road disturbance input  
- \(u\) : Control force applied to the suspension

## 📊 Results

### ⚡ Quarter Car Response:

Shows the displacement and velocity of the sprung and unsprung masses under initial conditions.
  
![initial](https://github.com/user-attachments/assets/6d1c3157-8def-4e89-99d2-d6edbaf20ff5)

### 🎛️ Classical Controllers:

Step responses and performance metrics (settling time, overshoot, steady-state error) for P, PI, and PID controllers. 

#### 🔴 P Controller:

![initial](https://github.com/user-attachments/assets/12dc5d5e-8694-48e5-a10e-07d43eeb3536)

#### 🟢 PI Controller:

![initial](https://github.com/user-attachments/assets/0d93498f-44d2-4a77-8b44-8bf664cb01df)


#### 🔵 PID Controller:

![initial](https://github.com/user-attachments/assets/98525cb2-2bcd-431c-a5d9-fd2dcdf59688)

### 🏎️ State Feedback Control (SVFC):

Step response and performance metrics under the designed pole locations.
  
![initial](https://github.com/user-attachments/assets/03e47851-4192-4365-83cd-319b4d11b626)

### 👁️ Full-Order Observer: 

Comparison of estimated vs actual states to validate observer performance. 

![initial](https://github.com/user-attachments/assets/5752ee32-3881-4b7b-9285-9ea0bf2c4bce)






