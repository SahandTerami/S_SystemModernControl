# Control of Active Suspension System of Vehicle

## 📖 Overview
In this project, I aimed to control the vehicle's active suspension system using a quarter-car model. The project includes the design and analysis of classical controllers (P, PI, PID), state feedback control (SVFC), and full-order state observers. Step responses, settling time, overshoot, and steady-state error are analyzed for each controller. The system is illustrated in the figure below.

<img width="479" height="355" alt="image" src="https://github.com/user-attachments/assets/3fa22315-0fc5-485a-88cc-6a213d002a78" />

## 🎯 Purposes
- Model the **quarter car system response** under initial conditions, analyzing the behavior of both the sprung and unsprung masses in terms of displacement and velocity.

- Compare the performance of **classical controllers (P, PI, PID)** and a **state feedback controller (SVFC)** on the system in terms of settling time, overshoot, and steady-state error.

- Design and simulate a **full-order state observer** to estimate unmeasured states, and validate the estimation by comparing the estimated and actual states.

## 🛠️ Constraints
- **Settling Time:** The controller should ensure that the system settles within **5 seconds**.  
- **Overshoot:** The system overshoot must be less than **20%**.  
- **Steady-State Error:** The steady-state error should be less than **1%** for a unit step input.
  
## 🧠 Model Dynamics

According to the figure, using Newton’s second law, the equations of motion for the two masses can be written as:

1. Vehicle (sprung) mass:
   
$$
M \frac{dv_1}{dt} = -K_1 \big( (x_1 - x_2)\big) - D (\dot{x}_1 - \dot{x}_2) + u
$$


2. Axle (unsprung) mass:
   
$$
m \frac{dv_2}{dt} = -K_1 \big( (x_1 - x_2) + D (\dot{x}_1 - \dot{x}_2) - u - K_2 (x_2- y_R)
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




