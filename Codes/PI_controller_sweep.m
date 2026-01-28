function [Kp_PI, Ki_PI, y_final_PI, ss_error_perc_PI, STime_PI, Oshoot_PI] = PI_controller_sweep(G, N1, Kp_scale, Ki_scale)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% PI Controller Design %%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%% Sahand Tangerami %%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Inputs:
%   G        : Plant transfer function
%   N1       : Number of gains to test for each Kp and Ki
%   Kp_scale : Scaling factor for Kp (Kp = Kp_scale * i)
%   Ki_scale : Scaling factor for Ki (Ki = Ki_scale * j)
%
% Outputs:
%   Kp_PI, Ki_PI        : Matrices of gains used (NaN if unstable)
%   y_final_PI           : Steady-state value
%   ss_error_perc_PI     : Steady-state error (%)
%   STime_PI             : Settling time (s)
%   Oshoot_PI            : Overshoot (%)


    s = tf('s');

    Kp_PI = NaN(N1,N1);
    Ki_PI = NaN(N1,N1);
    y_final_PI = NaN(N1,N1);
    ss_error_perc_PI = NaN(N1,N1);
    STime_PI = NaN(N1,N1);
    Oshoot_PI = NaN(N1,N1);

    for i = 1:N1
        for j = 1:N1
            Kp_val = Kp_scale * i;
            Ki_val = Ki_scale * j;

            C_PI = Kp_val + Ki_val/s;

            T_PI = feedback(C_PI*G, 1);

            if isstable(T_PI)
                Kp_PI(i,j) = Kp_val;
                Ki_PI(i,j) = Ki_val;

                y_final_PI(i,j) = dcgain(T_PI);
                ss_error_perc_PI(i,j) = 100*(1 - y_final_PI(i,j));

                info_PI = stepinfo(T_PI);
                STime_PI(i,j) = info_PI.SettlingTime;
                Oshoot_PI(i,j) = info_PI.Overshoot;
            end
        end
    end
end
