function xdot = QuarterCarState(t, x, Qcar, u, y_er, tspan)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%% Quarter Car Model %%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% States:   x1: Vertical displacement of the sprung mass (vehicle body)
%           x2: Vertical displacement of the unsprung mass (axle)
%           x3: Vertical velocity of the sprung mass
%           x4: Vertical velocity of the unsprung mass

%
% Input:    Qcar.M: Vehicle Mass
%           Qcar.m: Axel Mass
%           Qcar.K1: Suspension System Stiffness
%           Qcar.K2: Tire Stiffness
%           Qcar.D: Linear Damping Coefficient
%
% Output:   xdot = Change in the states

    A = [0                 0                           1                0
         0                 0                           0                1
        -Qcar.K1/Qcar.M    Qcar.K1/Qcar.M             -Qcar.D/Qcar.M    Qcar.D/Qcar.M
         Qcar.K1/Qcar.m   -(Qcar.K1+ Qcar.K2)/Qcar.m   Qcar.D/Qcar.m   -Qcar.D/Qcar.m];

    B = [0
         0
         1/Qcar.M
        -1/Qcar.m];

    B_dist = [0
              0
              0
              Qcar.K2/Qcar.m];

    ut      = interp1(tspan, u, t, 'linear', 'extrap');
    y_er_t  = interp1(tspan, y_er, t, 'linear', 'extrap');

    
    xdot = A*x + B*ut + B_dist*y_er_t;

end