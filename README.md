# S_SystemModernControl
In this project, I aimed to control the vehicle's active suspension system using a quarter-car model. The project includes the design and analysis of classical controllers (P, PI, PID), state feedback control (SVFC), and full-order state observers. Step responses, settling time, overshoot, and steady-state error are analyzed for each controller. The system is illustrated in the figure below.

<img width="479" height="355" alt="image" src="https://github.com/user-attachments/assets/3fa22315-0fc5-485a-88cc-6a213d002a78" />

# Model Dynamics:

According to the figure, using Newton’s second law, the equations of motion for the two masses can be written as:

1. Vehicle (sprung) mass:
   
$$
M \frac{dv_1}{dt} = -K_1 \big( (x_1 - x_2) - D (\dot{x}_1 - \dot{x}_2) + u
$$


2. Axle (unsprung) mass:
   
$$
m \frac{dv_2}{dt} = -K_1 \big( (x_1 - x_2) + D (\dot{x}_1 - \dot{x}_2) - u - K_2 (x_2- y_R)
$$

where:  

- \(M\) : Sprung mass (vehicle body)  
- \(m\) : Unsprung mass (axle)  
- \(K_1\) : Suspension stiffness  
- \(K_2\) : Tire stiffness  
- \(D\) : Damping coefficient  
- \(x_1, x_2\) : Vertical displacement of sprung and unsprung masses  
- \(\dot{x}_1, \dot{x}_2\) : Vertical velocities  
- \(x_{10}, x_{20}\) : Initial displacements of sprung and unsprung masses  
- \(y_R\) : Road disturbance input  
- \(u\) : Control force applied to the suspension

