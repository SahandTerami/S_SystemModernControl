# S_SystemModernControl
In this project, I aimed to control the vehicle's active suspension system using a quarter-car model. The project includes the design and analysis of classical controllers (P, PI, PID), state feedback control (SVFC), and full-order state observers. Step responses, settling time, overshoot, and steady-state error are analyzed for each controller. The system is illustrated in the figure below.

<img width="479" height="355" alt="image" src="https://github.com/user-attachments/assets/3fa22315-0fc5-485a-88cc-6a213d002a78" />

# Model Dynamics:

According to the figure, using Newton’s second law, the equations of motion for the two masses can be written as:

Mdtdv1​​=−K1​((x1​−x2​)−(x10​−x20​))−D(x˙1​−x˙2​)+u
